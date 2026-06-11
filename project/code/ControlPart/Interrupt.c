/**
 * @file    Interrupt.c
 * @brief   轮腿机器人多级定时中断服务实现
 *
 * 中断调度架构（频率由定时器硬件分频决定）：
 *   1ms  → IMU偏航角累积、圈数跟踪（Dirchange）、单腿站立计时
 *   2ms  → 跳跃控制、按键扫描、AI调参数据处理、陀螺仪数据读取、
 *          角速度环PID（内环，响应最快）
 *   4ms  → 卡尔曼滤波、角度环PID（外环）、转向PID模式切换
 *          （turn_mode 0~7，支持逐飞/串级/视觉/GPS/惯导/原地旋转）
 *   8ms  → 单腿高度切换控制(Single_Control)、舵机平衡控制(servo_balance)
 *   16ms → 速度环PID（三环串级的最外层）、惯导实时坐标更新
 *   40ms → 斑马线检测超时处理、偏航角慢漂补偿
 *
 * Created on: 2024年2月
 *      Author: LateRain
 */
#include "zf_common_headfile.h"
#include "Interrupt.h"
#include "PID.h"
#include "servo.h"
#include "menu.h"
#include "imu660.h"
#include "image.h"
#include "small_driver_uart_control.h"
#include "kalman.h"
#include "Math_Advanced.h"
#include "Init.h"

#include "Ins.h"
#include "AI_Pid_Tuner.h"
#include "ins_auto_record.h" /* 自动打点模块 */
#include "ins_pure_pursuit.h" /* Pure Pursuit 算法模块 */

#include "Ins.h" // 惯导系统（yaw_ins）

/* 全局中心控制结构体实例 */
Center_struct Yao;

/* ---- 开机角度校准变量 ----
 * 流程：上电后采集CALIBRATE_SAMPLES个样本的俯仰角，
 * 取平均作为零点偏移，后续角度减去此偏移。
 */
volatile uint8_t calibrate_state = 0;  /* 校准状态：0=未开始, 1=采集中, 2=完成 */
volatile float calibrate_offset = 0;   /* 校准得到的角度偏移（度） */
volatile uint16_t calibrate_count = 0; /* 采集计数 */
volatile float calibrate_sum = 0;      /* 角度累加和 */

/* ---- ADC与电池电压 ---- */
uint16 adc0;
float Battery_voltage;

/* ---- 转向与控制模式 ----
 * steer_control_mode: 0=角度控制(逐飞方案), 1=PWM控制(无速度环)
 * turn_mode:          0=关闭, 1=逐飞PID转向, 2=串级转向,
 *                     3=偏航角度闭环走直线, 4=视觉转向,
 *                     5=GPS转向, 6=原地旋转(Spin3), 7=惯导转向
 * fuzzy_mode:         0=关闭模糊(使用固定KP), 1=开启模糊(动态KP范围)
 */
uint8 steer_control_mode = 0;
uint8 turn_mode = 7; /* 初始化为关闭转向模式，避免启动时 yaw_ins 未初始化导致抽动 */
uint8 fuzzy_mode = 0;

/* ---- 菜单与调试 ---- */
uint8 menu_open = 1;  /* 0=关闭菜单和Flash(保证测试不会误写Flash),
                         1=打开菜单, 2=只打开读取不打开菜单 */
uint8 flag_yawan = 1; /* 偏航辅助(yawan)使能：0=关闭, 1=开启 */
uint8 flag_stop = 1;  /* 停止标志：1=停止(锁定舵机中位), 0=正常运行 */
uint8 ins_open = 1;   /* 惯导转向开关：0=关闭, 1=开启 */
uint8 ins_getdata = 0;

/* ---- 计数器与标志 ---- */
uint16 a11111 = 0;     /* 单腿站立计时器（1ms中断递增） */
uint16 a2222 = 0;      /* 长按计时器（用于跳跃触发） */
bool ff = 0;           /* 长按中间标志 */
uint16 dis_tof_mm = 0; /* TOF距离传感器数据 */

uint16 time_flag = 0;   /* 定时标志（预留） */
uint16_t flag_open = 0; /* 开启标志（预留） */
int16_t flag_main = 0;  /* 主状态标志 */
uint16_t flag_text = 0; /* 文本显示标志 */

/* ---- 偏航角变量 ----
 * angle_Z: 连续累计偏航角（可超过±180°），用于多圈旋转等场景。
 *          计算公式：angle_Z = 360 * Dirchange + yaw
 *          Dirchange 在每次跨过±180°边界时±1，跟踪累计圈数。
 */
volatile float angle_Z = 0;

void control_main(void)
{
    small_driver_get_speed();

    if (motor_value.receive_right_speed_data < -3500 || motor_value.receive_right_speed_data > 3500 || motor_value.receive_left_speed_data < -3500 || motor_value.receive_left_speed_data > 3500 || ins_getdata)
    {
        //            if(!flag_jump_stop)
        flag_main = 1;
        flag_stop = 1;
        Yao.Outp_Gyro_Pitch = 0;
        Yao.Outp_Angle_Pitch = 0;
        Yao.Outp_Speed_Pitch = 0;
        //            motor_value.receive_left_speed_data = 0;
        //            motor_value.receive_right_speed_data = 0;
        small_driver_set_duty(0, 0);
    }

    if (flag_main)
    {
        flag_stop = 1;
        Yao.Outp_Gyro_Pitch = 0;
        Yao.Outp_Angle_Pitch = 0;
        Yao.Outp_Speed_Pitch = 0;
        //            motor_value.receive_left_speed_data = 0;
        //            motor_value.receive_right_speed_data = 0;
        small_driver_set_duty(0, 0);
    }
    else
    {
        if (ins_open == 0 || menu_mode == 1)
        {
            // small_driver_set_duty((int16)(-(Yao.Outp_Gyro_Pitch)),     // 左轮发送占空比
            //                       (int16)(-Yao.Outp_Gyro_Pitch )); // 右轮发送占空比
            //small_driver_set_duty(0, 0);

            small_driver_set_duty((int16)(-(Yao.Outp_Gyro_Pitch + Yao.Outp_Gyro_Yaw)), // 左轮发送占空比
                                  (int16)((Yao.Outp_Gyro_Pitch - Yao.Outp_Gyro_Yaw))); // 右轮发送占空比
            // small_driver_set_duty(500,-500);
        }
        else
        {

            small_driver_set_duty((int16)(-(Yao.Outp_Gyro_Pitch + Yao.Outp_Gyro_Yaw)), // 左轮发送占空比
                                  (int16)((Yao.Outp_Gyro_Pitch - Yao.Outp_Gyro_Yaw))); // 右轮发送占空比
        }
    }

    if (ins_open)
    {

        ins_navigation();
    }
}
/**
 * @brief   1ms中断服务函数
 *
 * 执行任务：
 *   1. TOF距离传感器读取（菜单模式下）
 *   2. IMU偏航角积分累积（gyro_z * 0.001 → 更新yaw）
 *   3. 偏航角归一化到[-180°, 180°]
 *   4. 跨边界圈数检测（Dirchange递增/递减）
 *   5. 连续累计偏航角 angle_Z 计算
 *   6. 单腿站立计时器 a11111 递增
 */
void Interrupt_1ms(void)
{
    /* 1. TOF距离传感器读取 */
    Update_TOF_Sensor();

    /* 2. IMU偏航角积分与归一化 */
    Update_Yaw_Integration();

    /* 3. 跨边界圈数检测与连续偏航角计算 */
    Update_Angle_Z();

    /* 4. 单腿站立计时器更新 */
    Update_Single_Leg_Timer();

    /* 以下为已注释的自动90°转向测试代码 ---- */
    // if (time_flag == 0)
    //  {
    //      Count++;
    //      if (Count >= 100)
    //      {
    //          Target_Yaw += 90;
    //          time_flag = 1;
    //     }

    // }

    // if(key_detect(KEY_2, KEY_SHORT_PRESS))
    //
    //     ff = ~ff;
    // if(ff)
    //     a2222++;
    // else
    // {
    //     a2222 = 0;
    //     Yao.Target_Speed = 0;
    //     Deviation_Value = 0;
    // }
    //
    // if(a2222 >= 3000)
    // {
    //     flag_jump=1;
    //     ff=0;
    //     Deviation_Value = -0.7;
    //     Yao.Target_Speed = 300;
    // }
    // else if(3000 < a2222 && a2222 <= 6000)
    // {
    //     Deviation_Value = 0.7;
    // }
    //// else if(6000 < a2222 && a2222 <= 9000)
    //// {
    ////     Deviation_Value = -0.3;
    //// }
    // else if (a2222 > 6000)
    //     a2222 = 0;
}

float integer = 0;     /* 陀螺仪Y轴累计值（用于标定） */
uint32 num_t = 0;      /* 陀螺仪Y轴采样计数 */
float integer1 = 0;    /* 陀螺仪X轴累计值（用于标定） */
uint32 num_t1 = 0;     /* 陀螺仪X轴采样计数 */
volatile float y1 = 0; /* 俯仰角速度低通滤波值（用于角速度环） */
float ddddd = 0;       /* 预留变量 */

/**
 * @brief   2ms中断服务函数
 *
 * 执行任务：
 *   1. 跳跃控制：flag_jump=1时每周期调用 jump_control() 推进状态机
 *   2. AI调参：flag_ai_open=1时处理串口接收的PID参数
 *   3. 菜单：menu_open=1时运行菜单逻辑；按键检测
 *   4. 长按KEY_2（3秒）：触发跳跃（a2222>=3000时flag_jump翻转）
 *   5. IMU数据读取（date_handle）
 *   6. 开机角度校准（已注释，预留）
 *   7. 俯仰角速度低通滤波 + 角速度环PID（内环）
 *   8. 偏航角速度环PID（内环）
 */
void Interrupt_2ms(void)
{
    jump_text();
    /* 1. 跳跃控制 */
    Update_Jump_Control();

    /* 2. 按键扫描 */
    key_scanner();

    /* 3. AI调参处理 */
    if (flag_ai_open)
    {
        AI_Pid_Tuner_ProcessRx();
    }

    /* 4. IMU数据读取与姿态解算 */
    date_handle();

    /* 6. 串级PID角速度环（最内层） */
    Update_Gyro_PID_Loop();
}

/* ---- 4ms中断用全局变量 ---- */
volatile float aa1 = 0;                                  /* 俯仰角低通滤波值（用于角度环） */
volatile float aa2 = 0;                                  /* 转向角低通滤波值（用于转向PID） */
volatile float dd = 0;                                   /* 偏航偏差低通滤波值 */
float temp_erect_turn[4];                                /* 转向PID临时参数（模糊模式动态调整KP） */
float k11 = 0;                                           /* 急加速补偿系数1 */
float k22 = 0;                                           /* 急加速补偿系数2 */
float kp_roll = 0.9;                                     /* 翻滚KP系数 */
float Target_Yaw = 0;                                    /* 目标偏航角（turn_mode=3走直线模式） */
float V_trans = 0;                                       /* 横向速度（预留） */
uint8 TCount_falg_4ms = 0;                               /* 4ms计数使能标志 */
uint16 TCount_4ms = 0;                                   /* 4ms周期计数器 */
float dt = 0.004f;                                       /* 控制周期（4ms=0.004秒） */
float desired_yaw = 0.0f;                                /* 期望偏航角（视觉/GPS模式） */
float raw_vision_yaw = 0.0f;                             /* 视觉原始偏航角（避锥调整前） */
static float steer_vision_cmd_lpf_alpha = 0.35f;         /* 视觉指令低通滤波系数 */
static float steer_vision_cmd_lpf = 0.0f;                /* 视觉指令低通滤波值 */
volatile float steer_vision_target_yaw_deg = 0.0f;       /* 视觉目标偏航角（度） */
volatile float steer_vision_cone_avoid_delta_deg = 0.0f; /* 视觉避锥角度偏移（度） */

/* ---- GPS转向变量 ---- */
volatile float steer_gps_target_bearing_deg = 0.0f;    /* GPS目标方位角（度） */
volatile float steer_gps_distance_to_wp_m = 0.0f;      /* GPS到目标点距离（米） */
volatile float steer_gps_to_imu_yaw_offset_deg = 0.0f; /* IMU偏航与地理航向的固定偏移 */

/**
 * @brief   4ms中断服务函数
 *
 * 执行任务：
 *   1. 卡尔曼滤波更新姿态（imu963ra_kalman_filter_update）
 *   2. 姿态角偏移补偿（减去offset_angle）
 *   3. 角度死区处理（|pitch|<0.4°时置零）
 *   4. 俯仰角度环PID（外环）—— 输出作为角速度环的目标值
 *   5. 转向PID模式切换与执行（turn_mode 0~7）
 *
 * 转向模式详解：
 *   mode 0: 关闭转向
 *   mode 1: 逐飞PID转向（PID_turn_seekfree，使用erect_turn参数）
 *   mode 2: 串级转向（Cascade_angle_Yaw，支持fuzzy_mode动态KP）
 *   mode 3: 偏航角度闭环走直线（Cascade_angle_Yaw_2，Target_Yaw固定）
 *   mode 4: 视觉转向（Cascade_angle_Yaw_3，desired_yaw=视觉目标+避锥偏移，低通滤波）
 *   mode 5: GPS转向（Cascade_angle_Yaw_4，desired_yaw=GPS方位角+IMU偏移）
 *   mode 6: 原地旋转Spin3（Cascade_angle_Yaw_2，angle_Z为目标，到位后自动切回mode2）
 *   mode 7: 惯导转向（Cascade_angle_Yaw_2，目标=惯导航向）7
 */
void Interrupt_4ms(void)
{
    /* 1. 卡尔曼滤波更新姿态 */
    Update_Kalman_Filter();

    /* 2. 4ms计数器更新 */
    Update_4ms_Counter();

    /* 3. 俯仰角度环PID（外环） */
    Update_Angle_PID_Loop();

    /* 4. 转向控制 */
    Update_Steering_Control();
}

/* ---- 8ms中断用全局变量 ---- */
int16 recordL = 0; /* 左轮记录值（预留） */
int16 recordR = 0; /* 右轮记录值（预留） */
float vv2 = 0;     /* 轮速差滤波值（预留） */

/**
 * @brief   8ms中断服务函数
 *
 * 执行任务：
 *   1. 单腿高度切换控制（Single_Control）：flag_Single=1时运行
 *   2. 舵机平衡控制（servo_balance）：steer_control_mode=1时运行
 *   3. 按键检测（KEY_1触发flag_track）
 *
 * 注意：原速度环PID（Cascade_speed_Pitch）已迁移到16ms中断。
 *       Adapt_Terrain() 调用也已注释（当前仅在main循环中调用）。
 */
void Interrupt_8ms(void)
{
    /* 1. 单腿站立高度切换控制 */
    if (flag_Single)
    {
        Single_Control();
    }

    /* 2. 舵机平衡控制 */
    servo_balance();
}

/* ---- 16ms中断用全局变量 ---- */
volatile float aa11 = 0;        /* 轮速差低通滤波值（用于速度环输入） */
volatile float speed_MOTOR = 0; /* 平均轮速（用于惯导坐标更新） */

/**
 * @brief   16ms中断服务函数
 *
 * 执行任务：
 *   1. 惯导实时坐标更新（get_realtime_coordinate）：ins_open=1时运行
 *   2. 平均轮速计算：左右轮速差/2
 *   3. 轮速低通滤波（系数0.1新+0.9旧）
 *   4. 速度环PID（三环串级最外层）：
 *      输入=滤波后的平均轮速，设定值=0（平衡时目标速度为0），输出限幅±100
 *
 * 注意：速度环输出 Yao.Outp_Speed_Pitch 作为角度环的SetPoint，
 *       形成"速度环→角度环→角速度环"三环串级控制。
 */
void Interrupt_16ms(void)
{
    /* 1. 惯导坐标更新 */
    Update_INS_Coordinate();

    /* 2. 速度环PID（最外环） */
    Update_Speed_PID_Loop();
}

/* ---- 40ms中断用 ---- */
extern uint8 Zebra_Count_Flag; /* 斑马线计数标志（外部定义） */
uint16 TCount_40ms = 0;        /* 40ms周期计数器（用于斑马线超时检测） */

/* ---- 遥测发送控制 ---- */
uint8 telemetry_enable = 0;     /* 遥测使能：0=关闭, 1=开启（通过无线串口发送调试数据） */
uint8 ins_telemetry_enable = 1; /* 惯导遥测使能：0=关闭, 1=开启 */

/**
 * @brief   40ms中断服务函数
 *
 * 执行任务：
 *   1. 斑马线超时检测
 *   2. 偏航角慢漂补偿
 *   3. 遥测数据交替发送（每80ms切换 $T 姿态遥测 / $I 惯导遥测）
 *      $T格式: tick,pitch,roll,yaw,gx,gy,gz,outp_turn,... (20字段)
 *      $I格式: x,y,ins_mode,dis_ins,yaw_ins,n,target,flag_save (8字段)
 *   4. 惯导录点瞬间发送 $W 航点帧
 */
uint16 Count = 0; /* 40ms周期计数器（用于遥测发送切换） */
void Interrupt_40ms(void)
{
    /* 1. 斑马线超时检测 */
    Update_Zebra_Timeout();

    /* 2. 偏航角慢漂补偿 */
    Update_Yaw_Drift_Compensation();

    /* 3. 遥测数据发送 */
    Update_Telemetry_Send();

    /******************************* */
    // Count++;
    // if(Count >= 50)
    // {
    //     Target_Yaw = 90;
    // }
    // if(Count >= 100)
    // {
    //     Target_Yaw = 0;
    //     Count = 0;
    // }
    /******************************** */
    // printf("falg_1: %d ,flag_2: %d ,ins_mode: %d\n", flag_1, flag_2, ins_mode);
}

/* ========================================================================
   内部API函数实现（按中断时间分组）
   ======================================================================== */

/* ---- 1ms中断API函数 ---- */

/**
 * @brief   更新TOF距离传感器读数
 * @note    仅在菜单模式下读取，用于调试和避障
 */
void Update_TOF_Sensor(void)
{
    if (menu_mode)
    {
        dis_tof_mm = tof_dl1b_get_mm();
    }
}

/**
 * @brief   更新偏航角积分
 * @note    gyro_z(度/秒) * 0.001秒 = 每周期角度增量
 */
void Update_Yaw_Integration(void)
{
    imu660ra.eulerAngle.yaw += imu660ra.data_Raw.gyro_z * 0.001;
}

/**
 * @brief   更新偏航角归一化和跨边界圈数检测
 * @note    计算连续累计偏航角 angle_Z（不受±180°限制）
 */
void Update_Angle_Z(void)
{
    /* 偏航角归一化到 [-180°, 180°] 范围 */
    if (imu660ra.eulerAngle.yaw > 180)
        imu660ra.eulerAngle.yaw -= 360;
    if (imu660ra.eulerAngle.yaw < -180)
        imu660ra.eulerAngle.yaw += 360;

    /* 跨边界圈数检测
     *   从+180跳变到-180附近（差<-350）→ 正转一圈，Dirchange++
     *   从-180跳变到+180附近（差>350）→ 反转一圈，Dirchange--
     */
    if ((imu660ra.eulerAngle.yaw - imu660ra.eulerAngle.last_yaw) < -350)
        imu660ra.eulerAngle.Dirchange++;
    else if ((imu660ra.eulerAngle.yaw - imu660ra.eulerAngle.last_yaw) > 350)
        imu660ra.eulerAngle.Dirchange--;

    /* 计算连续累计偏航角 */
    angle_Z = 360 * imu660ra.eulerAngle.Dirchange + imu660ra.eulerAngle.yaw;
    imu660ra.eulerAngle.last_yaw = imu660ra.eulerAngle.yaw;
}

/**
 * @brief   更新单腿站立计时器
 * @note    flag_Single=1时递增，否则清零
 */
void Update_Single_Leg_Timer(void)
{
    if (flag_Single)
        a11111++;
    else
        a11111 = 0;
}

/* ---- 2ms中断API函数 ---- */

void jump_text(void)
{
    if (key_detect(KEY_2, KEY_SHORT_PRESS))
    {

        ff = ~ff;
    }
    if (ff)
        a2222++;
    else
    {
        a2222 = 0;
    }
    if (a2222 >= 3000)
    {
        flag_jump = ~flag_jump;
        ff = 0;
    }
}

/**
 * @brief   更新跳跃控制
 * @note    flag_jump=1时每周期调用 jump_control() 推进状态机
 */
void Update_Jump_Control(void)
{
    if (flag_jump)
    {
        time_j++;
        jump_control();
    }
    else
    {
        time_j = 0;
    }
}

/**
 * @brief   更新角速度环PID（内环）
 * @note    包含俯仰角速度低通滤波和偏航角速度环PID
 */
void Update_Gyro_PID_Loop(void)
{
    /* 俯仰角速度低通滤波 */
    y1 = 0.15f * ((float)-imu660ra_gyro_x) + 0.85f * y1;

    /* 俯仰角速度环PID（内环） */
    Yao.Outp_Gyro_Pitch = -limit(Cascade_gyro_Pitch(&PID_all.Pid_Gyro_Pitch, erect_Gyro_Pitch, y1, -Yao.Outp_Angle_Pitch), 8000.0f);

    /* 偏航角速度环PID（内环） */
    Yao.Outp_Gyro_Yaw = limit(Cascade_gyro_Yaw(&PID_all.Pid_Gyro_Yaw, erect_Gyro_Yaw, imu660rb_gyro_z, -Yao.Outp_turn), 8000.0f);
}

/* ---- 4ms中断API函数 ---- */

/**
 * @brief   更新卡尔曼滤波姿态
 * @note    使用 imu963ra_kalman_filter_update 进行姿态融合
 */
void Update_Kalman_Filter(void)
{
    imu963ra_kalman_filter_update(&imu);

    /* 角度死区处理 */
    if (imu660ra.eulerAngle.pitch > -0.4f && imu660ra.eulerAngle.pitch < 0.4f)
        imu660ra.eulerAngle.pitch = 0;

    imu.roll -= imu660ra.offset_angle.roll;
    imu.pitch -= imu660ra.offset_angle.pitch;

    imu660ra.eulerAngle.roll = imu.pitch; /* 注意：卡尔曼的roll映射到eulerAngle.pitch */
    imu660ra.eulerAngle.pitch = imu.roll; /* 卡尔曼的pitch映射到eulerAngle.roll（坐标系转换） */
}

/**
 * @brief   更新4ms计数器
 * @note    用于调试和时序控制
 */
void Update_4ms_Counter(void)
{
    TCount_4ms++;
}

/**
 * @brief   更新俯仰角度环PID（外环）
 * @note    输出作为角速度环的目标值
 */
void Update_Angle_PID_Loop(void)
{
    Yao.Outp_Angle_Pitch = Cascade_angle_Pitch(&PID_all.Pid_Angle_Pitch, erect_Angle_Pitch, imu660ra.eulerAngle.pitch, Yao.Outp_Speed_Pitch);
    Yao.Outp_Angle_Pitch = -limit(Yao.Outp_Angle_Pitch, 12000.0f);
}

/**
 * @brief   更新转向控制
 * @note    根据 turn_mode 选择不同的转向策略
 */
void Update_Steering_Control(void)
{
    if (turn_mode == 1)
    {
        Yao.Outp_turn = PID_turn_seekfree(&PID_all.Pid_turn, erect_turn, imu660ra.data_Raw.gyro_z, Deviation_Value * 10 + 0.2f);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 2)
    {
        if (fuzzy_mode == 0)
        {
            temp_erect_turn[0] = erect_Angle_Yaw[0];
            temp_erect_turn[1] = erect_Angle_Yaw[1];
            temp_erect_turn[2] = erect_Angle_Yaw[2];
            temp_erect_turn[3] = erect_Angle_Yaw[3];
        }
        else
        {
            temp_erect_turn[0] = Get_P(Y_Meet, Deviation_Value);
            temp_erect_turn[1] = erect_Angle_Yaw[1];
            temp_erect_turn[2] = erect_Angle_Yaw[2];
            temp_erect_turn[3] = erect_Angle_Yaw[3];
        }
        Yao.Outp_turn = Cascade_angle_Yaw(&PID_all.Pid_turn, temp_erect_turn, Deviation_Value * 10, 0);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 3)
    {
        Yao.Outp_turn = Cascade_angle_Yaw_2(&PID_all.Pid_Angle_Yaw, erect_Angle_Yaw_2, imu660ra.eulerAngle.yaw, Target_Yaw);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 4)
    {
        float raw_vision_yaw = steer_vision_target_yaw_deg + steer_vision_cone_avoid_delta_deg;
        raw_vision_yaw = steer_wrap_deg180(raw_vision_yaw);

        // Low-pass target yaw to reduce jitter during cone bypass.
        steer_vision_cmd_lpf = (1.0f - steer_vision_cmd_lpf_alpha) * steer_vision_cmd_lpf + steer_vision_cmd_lpf_alpha * raw_vision_yaw;
        desired_yaw = steer_vision_cmd_lpf;
        Yao.Outp_turn = Cascade_angle_Yaw_3(&PID_all.Pid_turn1, erect_Angle_Yaw_3, imu660ra.eulerAngle.yaw, desired_yaw);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 5)
    {
        desired_yaw = steer_gps_target_bearing_deg + steer_gps_to_imu_yaw_offset_deg;
        Yao.Outp_turn = Cascade_angle_Yaw_4(&PID_all.Pid_turn2, erect_Angle_Yaw_3, imu660ra.eulerAngle.yaw, desired_yaw);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 6)
    {
        // Use continuous yaw angle for multi-turn spin control.
        Yao.Outp_turn = Cascade_angle_Yaw_2(&PID_all.Pid_Angle_Yaw, erect_Angle_Yaw_2, angle_Z, spin3_target_angle);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);

        // Finish only when both angle and yaw rate are within limits.
        if (func_abs(spin3_target_angle - angle_Z) < spin3_angle_ok_deg &&
            func_abs((float)imu660rb_gyro_z) < spin3_gyro_ok_dps)
        {
            spin3_hold_cnt++;
            if (spin3_hold_cnt > 50)
            {
                spin3_active = 0;
                spin3_hold_cnt = 0;
                Yao.Outp_turn = 0;
                turn_mode = 2;
                Target_Yaw = imu660ra.eulerAngle.yaw;
            }
        }
    }
    else if (turn_mode == 7)
    {
        Yao.Outp_turn = Cascade_angle_Yaw_2(&PID_all.Pid_Angle_Yaw, erect_Angle_Yaw_2, imu660ra.eulerAngle.yaw, yaw_ins);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else
    {
        Yao.Outp_turn = 0;
    }
}

/* ---- 16ms中断API函数 ---- */

/**
 * @brief   更新惯导坐标
 * @note    ins_open=1时调用 get_realtime_coordinate 更新实时坐标
 *          ins_mode=4时额外调用 ins_auto_record_update 进行自动打点
 */
void Update_INS_Coordinate(void)
{
    if (ins_open)
    {
        /* 更新实时坐标（所有模式通用） */
        get_realtime_coordinate(speed_MOTOR, 0.016, imu660ra.eulerAngle.yaw);

        /* ins_mode=4: 自动打点模式下，检查是否需要记录航点 */
        if (ins_mode == 4 && g_ins_auto.is_recording)
        {
            // get_realtime_coordinate(speed_MOTOR, 0.016, imu660ra.eulerAngle.yaw);
            ins_auto_record_update(speed_MOTOR, imu660ra.eulerAngle.yaw);
        }
    }
}

/**
 * @brief   更新速度环PID（最外环）
 * @note    输入：编码器平均值，设定值：Yao.Target_Speed，输出：俯仰角目标值
 */
// uint8_t sum;
void Update_Speed_PID_Loop(void)
{
    /* 平均轮速计算 */
    // sum += motor_value.receive_right_speed_data;
    // printf("sum: %d\n", sum);
    speed_MOTOR = (float)(motor_value.receive_right_speed_data - motor_value.receive_left_speed_data) / 2.0f;

    Yao.Encoder_Left = motor_value.receive_right_speed_data;
    Yao.Encoder_Right = -motor_value.receive_left_speed_data;

    /* 速度环PID计算 */
    Yao.Outp_Speed_Pitch = -Cascade_speed_Pitch(&PID_all.Pid_Speed_Pitch, erect_Speed_Pitch,
                                                (float)(Yao.Encoder_Left + Yao.Encoder_Right) / 2,
                                                (float)Yao.Target_Speed);
    Yao.Outp_Speed_Pitch = limit(Yao.Outp_Speed_Pitch, 30.0f);
}

/* ---- 40ms中断API函数 ---- */

/**
 * @brief   更新斑马线超时检测
 * @note    4秒内未检测到新斑马线则触发标志
 */
void Update_Zebra_Timeout(void)
{
    if (Zebra_Count_Flag == 0)
    {
        Zebra_Flag = 0;
        TCount_40ms++;
    }
    else
    {
        TCount_40ms = 0;
    }

    if (TCount_40ms >= 100)
    {
        Zebra_Flag = 1;
        Zebra_Count_Flag = 1;
    }
}

/**
 * @brief   更新偏航角慢漂补偿
 * @note    每周期增加0.0001度，补偿陀螺仪零点漂移
 */
void Update_Yaw_Drift_Compensation(void)
{
    imu660ra.eulerAngle.yaw += 0.001;
}

/**
 * @brief   更新遥测数据发送
 * @note    每40ms发送 $T 姿态遥测和 $I 惯导遥测帧
 * 
 * 惯导两组模式说明:
 *   第一组 (按键打点):
 *     - ins_mode = 0: 单轨录点模式
 *     - ins_mode = 1: 单轨循迹模式
 *     - 数据源: cod_target[] 数组, n (总数), target (已寻)
 *   
 *   第二组 (自动打点):
 *     - ins_mode = 4: 自动定距打点模式
 *     - ins_mode = 5: Pure Pursuit 导航模式
 *     - 数据源: g_ins_auto.waypoints[] 数组, wp_count (总数), wp_current (已寻)
 */
void Update_Telemetry_Send(void)
{
    static uint8 telemetry_toggle = 0;

    /* Flash 调试输出 (每 40ms 发送一次) */
    // ins_auto_debug_output();

    /* 惯导遥测发送 (每 40ms 发送一次) */
    if (ins_telemetry_enable)
    {
        /* $I 惯导遥测帧 */
        /* 格式: $I,模式,实时X,实时Y,目标X,目标Y,目标角度,当前航点索引,总航点数,已寻点数,距离\r\n */
        char ins_buf[128];
        double target_x = 0.0, target_y = 0.0;
        uint16 current_wp_idx = 0, total_wp = 0, found_wp = 0;
        
        /* 根据模式选择数据源 */
        if (ins_mode == 0 || ins_mode == 1)
        {
            /* 第一组: 按键打点模式 (ins_mode 0/1) */
            current_wp_idx = (uint16)target;
            total_wp = (uint16)n;
            found_wp = (uint16)target;
            
            if (target <= n && n > 0)
            {
                target_x = cod_target[target].x;
                target_y = cod_target[target].y;
            }
        }
        else if (ins_mode == 4 || ins_mode == 5)
        {
            /* 第二组: 自动打点模式 (ins_mode 4/5) */
            current_wp_idx = g_ins_auto.wp_current;
            total_wp = g_ins_auto.wp_count;
            found_wp = g_ins_auto.wp_current;
            
            if (g_ins_auto.wp_current < g_ins_auto.wp_count && g_ins_auto.wp_count > 0)
            {
                target_x = g_ins_auto.waypoints[g_ins_auto.wp_current].x;
                target_y = g_ins_auto.waypoints[g_ins_auto.wp_current].y;
            }
        }
        
        int len = sprintf(ins_buf,
                          "$I,%d,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%d,%d,%.1f\r\n",
                          (int)ins_mode,
                          cod_realtime.x,
                          cod_realtime.y,
                          target_x,
                          target_y,
                          yaw_ins,
                          (int)current_wp_idx,
                          (int)total_wp,
                          (int)found_wp,
                          dis_ins);
        if (len > 0 && len < (int)sizeof(ins_buf))
        {
            wireless_uart_send_string(ins_buf);
        }

        /* $W 航点标记帧 (发送最新打点信息) */
        if (ins_mode == 0 && n > 0)
        {
            /* 模式 0: 发送按键打点的最新航点 */
            char wp_buf[64];
            len = sprintf(wp_buf,
                          "$W,%d,%.2f,%.2f,%d\r\n",
                          (int)n,
                          cod_saved[n - 1].x,
                          cod_saved[n - 1].y,
                          (int)ins_mode);
            if (len > 0 && len < (int)sizeof(wp_buf))
            {
                wireless_uart_send_string(wp_buf);
            }
        }
        else if (ins_mode == 4 && g_ins_auto.wp_count > 0)
        {
            /* 模式 4: 发送自动打点的最新航点 */
            char wp_buf[64];
            len = sprintf(wp_buf,
                          "$W,%d,%.2f,%.2f,%d\r\n",
                          (int)g_ins_auto.wp_count,
                          g_ins_auto.waypoints[g_ins_auto.wp_count - 1].x,
                          g_ins_auto.waypoints[g_ins_auto.wp_count - 1].y,
                          (int)ins_mode);
            if (len > 0 && len < (int)sizeof(wp_buf))
            {
                wireless_uart_send_string(wp_buf);
            }
        }

        /* $P Pure Pursuit 详细信息帧 (仅 ins_mode=5 时发送) */
        if (ins_mode == 5 && g_ins_auto.is_navigating)
        {
            /* 获取 Pure Pursuit 状态 */
            PurePursuit_State pp_state = ins_pure_pursuit_get_state();
            
            /* 格式: $P,前瞻距离,曲率,目标转向角,前瞻点索引,横向偏差,角度偏差,当前航点X,当前航点Y\r\n */
            char pp_buf[128];
            double current_wp_x = 0.0, current_wp_y = 0.0;
            
            /* 获取当前目标航点坐标 */
            if (g_ins_auto.wp_current < g_ins_auto.wp_count)
            {
                current_wp_x = g_ins_auto.waypoints[g_ins_auto.wp_current].x;
                current_wp_y = g_ins_auto.waypoints[g_ins_auto.wp_current].y;
            }
            
            len = sprintf(pp_buf,
                          "$P,%.1f,%.4f,%.1f,%d,%.1f,%.1f,%.2f,%.2f\r\n",
                          pp_state.lookahead_distance,
                          pp_state.curvature,
                          pp_state.target_yaw,
                          (int)pp_state.target_wp_index,
                          pp_state.distance_to_path,
                          pp_state.angle_to_path,
                          current_wp_x,
                          current_wp_y);
            if (len > 0 && len < (int)sizeof(pp_buf))
            {
                wireless_uart_send_string(pp_buf);
            }
        }
    }

    if (telemetry_enable)
    {
        /* $T 姿态遥测帧 */
        char telemetry_buf[128];
        int len = sprintf(telemetry_buf,
                          "$T,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%.1f,%d,%.2f,%.1f,%.1f,%d,%d,%.2f,%.2f,%.1f\r\n",
                          (int)TCount_40ms,
                          imu660ra.eulerAngle.pitch,
                          imu660ra.eulerAngle.roll,
                          imu660ra.eulerAngle.yaw,
                          imu660ra.data_Raw.gyro_x,
                          imu660ra.data_Raw.gyro_y,
                          imu660ra.data_Raw.gyro_z,
                          (int)Yao.Outp_turn,
                          (int)Yao.Outp_Gyro_Pitch,
                          Target_Yaw,
                          (int)turn_mode,
                          Deviation_Value,
                          imu.pitch,
                          imu.roll,
                          (int)motor_value.receive_left_speed_data,
                          (int)motor_value.receive_right_speed_data,
                          imu.ax_linear,
                          imu.ay_linear,
                          angle_Z);
        if (len > 0 && len < (int)sizeof(telemetry_buf))
        {
            wireless_uart_send_string(telemetry_buf);
        }
    }
}
/* ---- 自动打点模块外部变量 ---- */
