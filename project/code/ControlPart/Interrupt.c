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
#include "ins_interface.h"
#include "Ins.h"
#include "AI_Pid_Tuner.h"

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
uint8 turn_mode = 7;
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

    if (motor_value.receive_right_speed_data < -3500 
        || motor_value.receive_right_speed_data > 3500 
        ||motor_value.receive_left_speed_data < -3500 
        || motor_value.receive_left_speed_data > 3500
        || ins_getdata)
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
            // small_driver_set_duty(0, 0);
            
            small_driver_set_duty((int16)(-(Yao.Outp_Gyro_Pitch - Yao.Outp_Gyro_Yaw)),  // 左轮发送占空比
                                  (int16)(-(Yao.Outp_Gyro_Pitch + Yao.Outp_Gyro_Yaw))); // 右轮发送占空比
            // small_driver_set_duty(500,-500);
        }
        else
        {
            
            small_driver_set_duty((int16)(-(Yao.Outp_Gyro_Pitch - Yao.Outp_Gyro_Yaw)),  // 左轮发送占空比
                                  (int16)(-(Yao.Outp_Gyro_Pitch + Yao.Outp_Gyro_Yaw))); // 右轮发送占空比
        }
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

    if (menu_mode)
    {
        dis_tof_mm = tof_dl1b_get_mm();
        // ips200_show_uint( 0, 30*8, dis_tof_mm, 5 );
    }
    // EKF_UpData();
    // imu660ra.eulerAngle.pitch = euler_angle.roll - imu660ra.offset_angle.pitch;
    // imu660ra.eulerAngle.roll  = euler_angle.pitch  - imu660ra.offset_angle.roll;

    /* 偏航角积分：gyro_z(度/秒) * 0.001秒 = 每周期角度增量 */
    imu660ra.eulerAngle.yaw += imu660ra.data_Raw.gyro_z * 0.001;

    // Buzzer_Control();

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

    /* 计算连续累计偏航角（不受±180°限制） */
    angle_Z = 360 * imu660ra.eulerAngle.Dirchange + imu660ra.eulerAngle.yaw;
    imu660ra.eulerAngle.last_yaw = imu660ra.eulerAngle.yaw;

    // if(imu660ra.eulerAngle.pitch > 0)
    //     imu660ra.eulerAngle.pitch -= 180;
    // else if(imu660ra.eulerAngle.pitch < 0)
    //     imu660ra.eulerAngle.pitch += 180;

    /* 单腿站立计时器：flag_Single=1时递增，否则清零 */
    // a11111++;

    // control_main();

    if (flag_Single)
    {
        a11111++;
    }
    else
        a11111 = 0;

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
    /* 1. 跳跃控制：每周期推进跳跃状态机 */
    if (flag_jump)
    {
        time_j++;
        jump_control();
    }
    else
        time_j = 0;

    key_scanner();

    /* 2. AI调参：处理串口接收的PID参数更新 */
    if (flag_ai_open)
    {
        AI_Pid_Tuner_ProcessRx();
    }

    /* 3. 菜单处理 */
    // if (menu_open == 1)
    //     //menu();
    // else
    menu_mode = 1;

    // if (menu_mode && key_detect(KEY_1, KEY_SHORT_PRESS))
    //    flag_track = 1;

    // servo_set_angle(LF, 180);
    // servo_set_angle(RF, 180);
    // servo_set_angle(LB, 0);
    // servo_set_angle(RB, 0);
    // servo_set_angle(LF, 265.7f);
    // servo_set_angle(RF, 265.7f);
    // servo_set_angle(LB, 42.2f);
    // servo_set_angle(RB, 42.2f);

    /* 4. 长按KEY_2触发跳跃（约3秒=3000*2ms） */
    // if (menu_mode && key_detect(KEY_2, KEY_SHORT_PRESS))
    // {
    //     ff = ~ff;
    // }
    // if (ff)
    //     a2222++;
    // else
    // {
    //     a2222 = 0;
    // }
    // if (a2222 >= 3000)
    // {
    //     flag_jump = ~flag_jump;
    //     ff = 0;
    // }

    // pwm_set_duty(ATOM0_CH0_P21_2, 1500);
    // pwm_set_duty(ATOM0_CH1_P21_3, 1500);
    // pwm_set_duty(ATOM0_CH2_P21_4, 1500);
    // pwm_set_duty(ATOM0_CH3_P21_5, 1500);

    /* 5. IMU数据读取与姿态解算 */
    // get_eulerAngle();
    date_handle();

    /* 6. 开机角度校准（已注释，预留功能） */
    /*if (calibrate_state == 1)
    {
        calibrate_sum += imu660ra.eulerAngle.pitch;
        calibrate_count++;

        if (calibrate_count >= CALIBRATE_SAMPLES)
        {
            calibrate_offset = calibrate_sum / calibrate_count;
            calibrate_state = 2;  // 标记完成
        }
         ips200_show_float( 30 , 80 , calibrate_offset , 3 , 3 );
    }*/

    /* 7. 调试模式：IMU积分标志控制Z轴陀螺仪积分（用于零偏测试） */
    if (menu_mode)
    {
        // if(num_t >= 3000)
        // date_handle();
        if (IMU_JF_Flag)
        {
            Z_Yaw += imu660ra.data_Raw.gyro_z / 500;
        }
        else
        {
            Z_Yaw = 0;
        }

        // else
        //     num_t++;
    }

    // imu660ra_get_gyro();

    // if(num_t <= 10000)
    // {
    //     num_t++;
    //     integer += (float)imu660ra_gyro_y-2.5f;
    // }
    // ips200_show_float( 0, 0*8, integer, 5,5 );
    // ips200_show_float( 0, 1*8, (integer/num_t), 5,5 );
    // if(num_t1 <= 10000)
    // {
    //     num_t1++;
    //     integer1 += (float)imu660ra_gyro_x-1.62f;
    // }
    // ips200_show_float( 0, 3*8, integer1, 5,5 );
    // ips200_show_float( 0, 4*8, (integer1/num_t1), 5,5 );

    /* 8. 串级PID角速度环（最内层）----
     * 俯仰：陀螺仪X轴低通滤波(0.15新+0.85旧) → 角速度PID → 限幅±8000
     * 偏航：陀螺仪Z轴 → 角速度PID → 限幅±8000
     * 注意：俯仰角速度环的SetPoint来自角度环输出的负值（-Yao.Outp_Angle_Pitch），
     *       负号用于匹配传感器方向与电机输出方向。
     */
    y1 = 0.15f * ((float)-imu660ra_gyro_x) + 0.85f * y1;
    Yao.Outp_Gyro_Pitch = -limit(Cascade_gyro_Pitch(&PID_all.Pid_Gyro_Pitch, erect_Gyro_Pitch, y1, -Yao.Outp_Angle_Pitch), 8000.0f);
    Yao.Outp_Gyro_Yaw = limit(Cascade_gyro_Yaw(&PID_all.Pid_Gyro_Yaw, erect_Gyro_Yaw, imu660rb_gyro_z, Yao.Outp_turn), 8000.0f);
    // if(fabs(Yao.Outp_Gyro_Pitch) < 80) Yao.Outp_Gyro_Pitch = 0;
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
 *   mode 7: 惯导转向（Cascade_angle_Yaw_2，目标=惯导航向）
 */
void Interrupt_4ms(void)
{
    // V_trans = (float)(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data);

    /* 1. 卡尔曼滤波：融合陀螺仪与加速度计，估计roll/pitch */
    imu963ra_kalman_filter_update(&imu);

    // imu963ra_menc15a_kalman_filter_Update(&vel_kf, 0, imu.ay_linear);

    /* 2. 偏移补偿：减去上电标定的零点偏移 */
    imu.roll -= imu660ra.offset_angle.roll;
    imu.pitch -= imu660ra.offset_angle.pitch;
    imu660ra.eulerAngle.roll = imu.pitch; /* 注意：卡尔曼的roll映射到eulerAngle.pitch */
    imu660ra.eulerAngle.pitch = imu.roll; /* 卡尔曼的pitch映射到eulerAngle.roll（坐标系转换） */

    /* 3. 角度死区：|pitch|<0.4°时强制置零，避免微抖动 */
    if (imu660ra.eulerAngle.pitch < 0.4 && imu660ra.eulerAngle.pitch > -0.4)
        imu660ra.eulerAngle.pitch = 0;

    /* 4ms计数器 */
    if (TCount_falg_4ms)
        TCount_4ms++;
    else
        TCount_4ms = 0;

    /* 4. 俯仰角度环PID（外环）----
     * 输入：俯仰角低通滤波值 aa1（滤波系数0.1新+0.9旧）
     * 设定值：Yao.Outp_Speed_Pitch（来自速度环输出）
     * 输出限幅：±12000
     */
    /*float pitch_corrected = imu660ra.eulerAngle.pitch;
    if (calibrate_state == 2)  // 校准完成后才补偿
    {
        pitch_corrected -= calibrate_offset;
    }*/
    aa1 = 0.1f * imu660ra.eulerAngle.pitch + 0.9f * aa1;
    if (steer_control_mode == 0)
    {
        Yao.Outp_Angle_Pitch = Cascade_angle_Pitch(&PID_all.Pid_Angle_Pitch, erect_Angle_Pitch, aa1, 0);
        Yao.Outp_Angle_Pitch = -limit(Yao.Outp_Angle_Pitch, 12000.0f);
    }
    else
    {
        Yao.Outp_Angle_Pitch = Cascade_angle_Pitch(&PID_all.Pid_Angle_Pitch, erect_Angle_Pitch, aa1, 0);
        Yao.Outp_Angle_Pitch = -limit(Yao.Outp_Angle_Pitch, 12000.0f);
    }

    /* 偏航偏差低通滤波 */
    dd = 0.1f * Deviation_Value + 0.9f * dd;

    /* 5. 转向控制 ---- */
    if (turn_mode == 1)
    {
        /* 模式1：逐飞PID转向（含陀螺仪前馈） */
        Yao.Outp_turn = PID_turn_seekfree(&PID_all.Pid_turn, erect_turn, imu660ra.data_Raw.gyro_z, Deviation_Value * 10 + 0.2f);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 2)
    {
        /* 模式2：串级转向
         * fuzzy_mode=0: 使用固定KP(erect_Angle_Yaw)
         * fuzzy_mode=1: 使用模糊动态KP(Get_P根据偏差大小调整)
         */
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

        // if(flag_Single_HighState == 1)
        //     temp_erect_turn[0] = 100;
        Yao.Outp_turn = Cascade_angle_Yaw(&PID_all.Pid_turn, temp_erect_turn, Deviation_Value * 10 /*+ kp_roll * stab_roll*/, 0);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 3)
    {
        /* 模式3：偏航角度闭环走直线（Target_Yaw保持固定方向） */
        Yao.Outp_turn = Cascade_angle_Yaw_2(&PID_all.Pid_Angle_Yaw, erect_Angle_Yaw_2, imu660ra.eulerAngle.yaw, Target_Yaw);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
        // printf("Target_Yaw: %.2f, Current_Yaw: %.2f\n", Target_Yaw, imu660ra.eulerAngle.yaw);
    }
    else if (turn_mode == 4)
    {
        /* 模式4：视觉转向
         * 目标角度 = 视觉目标 + 避锥偏移量
         * 经过低通滤波(a=0.35)平滑目标值，减少避锥瞬间的抖动
         */
        raw_vision_yaw = steer_vision_target_yaw_deg + steer_vision_cone_avoid_delta_deg;
        raw_vision_yaw = steer_wrap_deg180(raw_vision_yaw);

        /* 低通滤波平滑目标偏航角 */
        steer_vision_cmd_lpf = (1.0f - steer_vision_cmd_lpf_alpha) * steer_vision_cmd_lpf + steer_vision_cmd_lpf_alpha * raw_vision_yaw;
        desired_yaw = steer_vision_cmd_lpf;
        Yao.Outp_turn = Cascade_angle_Yaw_3(&PID_all.Pid_turn1, erect_Angle_Yaw_3, imu660ra.eulerAngle.yaw, desired_yaw);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
        // Target_Yaw = steer_target_yaw_deg;  // mirror to existing debug variable
    }
    else if (turn_mode == 5)
    {
        /* 模式5：GPS转向
         * 目标角度 = GPS方位角 + IMU偏航偏移量
         */
        desired_yaw = steer_gps_target_bearing_deg + steer_gps_to_imu_yaw_offset_deg;
        Yao.Outp_turn = Cascade_angle_Yaw_4(&PID_all.Pid_turn2, erect_Angle_Yaw_3, imu660ra.eulerAngle.yaw, desired_yaw);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 6)
    {
        /* 模式6：原地旋转（Spin3）
         * 使用连续累计角度 angle_Z（不受±180°限制）作为当前角度，
         * 目标角度 spin3_target_angle（起始角±1080°=3圈）。
         *
         * 到位条件：角度误差 < spin3_angle_ok_deg 且
         *           陀螺Z轴角速度 < spin3_gyro_ok_dps
         * 保持 spin3_hold_ticks 周期后 → 切回mode2，锁定当前偏航为Target_Yaw
         */
        Yao.Outp_turn = Cascade_angle_Yaw_2(&PID_all.Pid_Angle_Yaw, erect_Angle_Yaw_2, angle_Z, spin3_target_angle);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);

        /* 检查到位条件（角度+角速度同时满足） */
        if (func_abs(spin3_target_angle - angle_Z) < spin3_angle_ok_deg &&
            func_abs((float)imu660rb_gyro_z) < spin3_gyro_ok_dps)
        {
            spin3_hold_cnt++;
        }
        else
        {
            spin3_hold_cnt = 0;
        }

        /* 到位保持足够周期后退出旋转 */
        if (spin3_hold_cnt >= spin3_hold_ticks)
        {
            spin3_active = 0;
            spin3_hold_cnt = 0;
            Yao.Outp_turn = 0;
            turn_mode = 2;                        /* 切回串级转向模式 */
            Target_Yaw = imu660ra.eulerAngle.yaw; /* 锁定当前偏航为目标 */
        }
    }
    else if (turn_mode == 7)
    {
        /* 模式7：惯导转向
         * ins_open=1时使用惯导航向角 yaw_ins（归一化到[-180,180]）作为目标
         */
        if (ins_open)
        {
            float yaw_ins_deg = (float)yaw_ins;
            if (yaw_ins_deg > 180.0f)
                yaw_ins_deg -= 360.0f;
            if (yaw_ins_deg < -180.0f)
                yaw_ins_deg += 360.0f;
            /* 确保 setpoint 在当前 yaw 的 ±180° 范围内，取最短转角路径 */
            {
                float diff = yaw_ins_deg - imu660ra.eulerAngle.yaw;
                if (diff > 180.0f)
                    yaw_ins_deg -= 360.0f;
                else if (diff < -180.0f)
                    yaw_ins_deg += 360.0f;
            }
            // aa2 = 0.95f * yaw_ins_deg + 0.05f * aa2;  /* 惯导目标角低通滤波 */
            Yao.Outp_turn = Cascade_angle_Yaw_2(&PID_all.Pid_Angle_Yaw, erect_Angle_Yaw_2, imu660ra.eulerAngle.yaw, yaw_ins_deg);
            Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
        }
        else
        {
            Yao.Outp_turn = 0;
        }
    }
    else
    {
        /* mode 0 或其他：转向输出为零 */
        Yao.Outp_turn = 0;
    }
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
    /*
     // 电池电压
     adc0 = adc_mean_filter_convert( ADC0_CH6_A6 , 5 );
     Battery_voltage = adc0 / 114.8936;

     if(menu_mode)
     {
     Yao.Encoder_Left  = motor_value.receive_right_speed_data;
     Yao.Encoder_Right = -motor_value.receive_left_speed_data;
     }
     else
     {
         Yao.Encoder_Left = 0;
         Yao.Encoder_Right = 0;
     }
     Yao.Outp_Speed_Pitch = -Cascade_speed_Pitch(&PID_all.Pid_Speed_Pitch, erect_Speed_Pitch, (float)(Yao.Encoder_Left+Yao.Encoder_Right)/2, (float)Yao.Target_Speed);
     Yao.Outp_Speed_Pitch = limit( Yao.Outp_Speed_Pitch, 30.0f );
     */
    // vv2 = 0.05f * (float)(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data) + 0.95 * vv2;
    // Yao.Outp_Speed_Pitch = -Cascade_speed_Pitch(&PID_all.Pid_Speed_Pitch, erect_Speed_Pitch, vv2, (float)Yao.Target_Speed);
    // Yao.Outp_Speed_Pitch = limit( Yao.Outp_Speed_Pitch, 100.0f );

    // 跳跃控制部分
    // if(flag_jump)
    // {
    //     time_j++;
    //     jump_control();
    // }
    // else
    //     time_j = 0;

    /**************************************************************/

    /* 1. 单腿站立高度切换控制 */
    if (flag_Single)
    {
        Single_Control();
    }

    // if (menu_mode == 1 /*&& flag_jump == 0 && Element_State != Jump_State */ && steer_control_mode == 0)
    // Adapt_Terrain();
    // else

    /* 2. 舵机平衡控制（PWM模式下直接使用轮速差） */

    servo_balance();

    /******************************************************************** */

    /* 3. 按键检测：短按KEY_1触发巡线模式 */
    /*if (menu_open == 1)
       // menu();
    else
        menu_mode = 1;*/

    // if (menu_mode && key_detect(KEY_1, KEY_SHORT_PRESS))
    //     flag_track = 1;

    // if(Element_State == Jump_State)
    // {
    //     if(flag_jump == 0)
    //     servo_set_angle(RF, 210);servo_set_angle(RB, 0);
    //     servo_set_angle(LF, 210);servo_set_angle(LB, 0);
    // }

    // small_driver_set_duty(0, 0);
    // processImage();
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

    // 电池电压
    // adc0 = adc_mean_filter_convert( ADC0_CH6_A6 , 5 );
    // Battery_voltage = adc0 / 114.8936;

    // if(menu_mode)
    // {
    //     if(func_abs(Yao.Encoder_Left-motor_value.receive_right_speed_data) <= 100)
    //         Yao.Encoder_Left  = 0.5f * motor_value.receive_right_speed_data + 0.5f * Yao.Encoder_Left;
    //     if(func_abs(Yao.Encoder_Right-motor_value.receive_left_speed_data) <= 100)
    //         Yao.Encoder_Right = -(0.5f * motor_value.receive_left_speed_data + 0.5f * Yao.Encoder_Right);

    //     Yao.Encoder_Left  = motor_value.receive_right_speed_data;
    //     if(motor_value.receive_left_speed_data < 0)
    //         Yao.Encoder_Right = -motor_value.receive_left_speed_data+22;
    //     else if(motor_value.receive_left_speed_data > 0)
    //         Yao.Encoder_Right = -motor_value.receive_left_speed_data-23;
    //     else
    //         Yao.Encoder_Right = -motor_value.receive_left_speed_data;

    //     Yao.Encoder_Left  = motor_value.receive_right_speed_data;
    //     Yao.Encoder_Right = -motor_value.receive_left_speed_data;
    // }
    // else
    // {
    //     Yao.Encoder_Left = 0;
    //     Yao.Encoder_Right = 0;
    // }

    /* 1. 惯导实时坐标更新（16ms周期，当前偏航角） */
    if (ins_open)
        get_realtime_coordinate(speed_MOTOR, 0.016, imu660ra.eulerAngle.yaw);

    /* 2. 平均轮速计算（负号用于匹配方向：左轮反转） */
    speed_MOTOR = (float)(-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data) / 2.0f;

    /* 3. 轮速低通滤波 */
    aa11 = 0.1f * (((float)(-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data)) / 2.0f) + 0.9f * aa11;

    /* 4. 速度环PID（最外层）
     * 设定值=0：平衡时目标速度为0（静止平衡）
     * 输出限幅±100：防止速度环输出过大导致角度环饱和
     */
    Yao.Outp_Speed_Pitch = -Cascade_speed_Pitch(&PID_all.Pid_Speed_Pitch, erect_Speed_Pitch, aa11, 0);
    Yao.Outp_Speed_Pitch = limit(Yao.Outp_Speed_Pitch, 100.0f);
}

/* ---- 40ms中断用 ---- */
extern uint8 Zebra_Count_Flag; /* 斑马线计数标志（外部定义） */
uint16 TCount_40ms = 0;        /* 40ms周期计数器（用于斑马线超时检测） */

/* ---- 遥测发送控制 ---- */
uint8 telemetry_enable = 1;     /* 遥测使能：0=关闭, 1=开启（通过无线串口发送调试数据） */
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
void Interrupt_40ms(void)
{
    /* 1. 斑马线超时检测：4秒内未检测到新斑马线则触发标志 */
    if (Zebra_Count_Flag == 0)
    {
        Zebra_Flag = 0;
        TCount_40ms++;
    }
    else
        TCount_40ms = 0;

    if (TCount_40ms >= 100)
    {
        Zebra_Flag = 1;
        Zebra_Count_Flag = 1;
    }

    // yaokong_data_deal();

    /* 2. 偏航角慢漂补偿 */
    imu660ra.eulerAngle.yaw += 0.0001;

    /* 3. 遥测数据交替发送：
     *    telemetry_toggle=0 → $T 姿态遥测
     *    telemetry_toggle=1 → $I 惯导遥测
     *    各占一半带宽，每种每80ms发送一次
     */
    if (telemetry_enable || ins_telemetry_enable)
    {
        static uint8 telemetry_toggle = 0;
        telemetry_toggle++;
        if (telemetry_toggle >= 2)
        {
            telemetry_toggle = 0;

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
            else if (ins_telemetry_enable)
            {
                /* $I 惯导遥测帧（$T关闭时独占发送带宽） */
                char ins_buf[100];
                int len = sprintf(ins_buf,
                                  "$I,%.1f,%.1f,%d,%.1f,%.1f,%d,%d,%d\r\n",
                                  cod_realtime.x,
                                  cod_realtime.y,
                                  (int)ins_mode,
                                  dis_ins,
                                  yaw_ins,
                                  (int)n,
                                  (int)target,
                                  (int)flag_save);
                if (len > 0 && len < (int)sizeof(ins_buf))
                {
                    wireless_uart_send_string(ins_buf);
                }
            }
        }
        else if (telemetry_enable && ins_telemetry_enable)
        {
            /* toggle=1 时发送 $I 惯导遥测帧（与 $T 交替） */
            char ins_buf[100];
            int len = sprintf(ins_buf,
                              "$I,%.1f,%.1f,%d,%.1f,%.1f,%d,%d,%d\r\n",
                              cod_realtime.x,
                              cod_realtime.y,
                              (int)ins_mode,
                              dis_ins,
                              yaw_ins,
                              (int)n,
                              (int)target,
                              (int)flag_save);
            if (len > 0 && len < (int)sizeof(ins_buf))
            {
                wireless_uart_send_string(ins_buf);
            }
        }
    }

    /* 4. 惯导录点瞬间 → 发送 $W 航点帧 */
    if (ins_telemetry_enable && ins_getdata)
    {
        char wp_buf[50];
        int wp_len = sprintf(wp_buf,
                             "$W,%d,%.1f,%.1f,%d\r\n",
                             (int)(n - 1),                /* 最新航点索引 */
                             cod_saved[(uint8)(n - 1)].x, /* 航点X */
                             cod_saved[(uint8)(n - 1)].y, /* 航点Y */
                             (int)n                       /* 当前航点总数 */
        );
        if (wp_len > 0 && wp_len < (int)sizeof(wp_buf))
        {
            wireless_uart_send_string(wp_buf);
        }
        ins_getdata = 0; /* 单次发送，清除标志 */
    }
}
