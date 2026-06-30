/**
 * @file    Interrupt.c
 * @brief   平衡车多级定时中断服务实现
 *
 * 中断层级概览（频率递减、耗时递增）：
 *   1ms  ： IMU偏航角积分、累加圈计数、Dirchange、到达站计时、到达时
 *   2ms  ： 跳跃控制、按键扫描、AI数据接收、IMU数据读取、
 *          角速度环PID（内环）、偏航响应输出
 *   4ms  ： 卡尔曼滤波、角度环PID（外环）、转向PID模式切换
 *          （turn_mode 0~7，支持视觉/遥控/惯导/GPS/原地/惯导转向）
 *   8ms  ： 腿部高度切换控制(Single_Control)、舵机平衡环(servo_balance)
 *   16ms ： 速度环PID（外环计算）、惯导实时坐标更新
 *   40ms ： 斑马线逻辑、 超时检测、偏航角漂移补偿
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

#include "Ins.h" // 惯导模块头文件，提供yaw_ins等接口

#include "gps_nav.h"

/* 控制核心数据结构，各中断共享 */
Center_struct Yao;

/* ---- 姿态角度校准相关 ----
 * 流程：连续采集CALIBRATE_SAMPLES个姿态角采样，
 * 取平均值作为零偏，后续姿态角减去此偏移。
 */
volatile uint8_t calibrate_state = 0;  /* 校准状态：0=未开始, 1=采集中, 2=完成 */
volatile float calibrate_offset = 0;   /* 校准得到的角度偏移（度） */
volatile uint16_t calibrate_count = 0; /* 采样计数器 */
volatile float calibrate_sum = 0;      /* 角度累加和 */

/* ---- ADC电池电压 ---- */
uint16 adc0;
float Battery_voltage;

/* ---- 转向控制模式  ----
 * steer_control_mode: 0=角度控制(平衡环), 1=PWM直驱(角速度环)
 * turn_mode:          0=关闭, 1=逐飞PID转向, 2=通用转向,
 *                     3=偏航角度锁环(直行), 4=视觉转向,
 *                     5=GPS转向, 6=原地旋转(Spin3), 7=惯导转向
 * fuzzy_mode:         0=关闭模糊(使用固定KP), 1=开启模糊(动态KP随偏差范围)
 */
uint8 steer_control_mode = 0;
uint8 turn_mode = 7;
uint8 fuzzy_mode = 0;

/* ---- 菜单与标志  ---- */
uint8 menu_open = 1;  /* 0=关闭菜单和Flash(保证不会误写Flash),
                         1=打开菜单, 2=只打开读取不打开菜单 */
uint8 flag_yawan = 1; /* 偏航补偿(yawan)使能：0=关闭, 1=开启 */
uint8 flag_stop = 1;  /* 停止标志：1=停止(电机输出零), 0=正常运行 */
uint8 ins_open = 1;   /* 惯导转向开关：0=关闭, 1=开启 */
uint8 ins_getdata = 0;

/* ---- 计时与传感器  ---- */
uint16 a11111 = 0;     /* 到达站计时器，1ms中断中的累加器 */
uint16 a2222 = 0;      /* 按键长按计时器，用于触发跳跃等 */
bool ff = 0;           /* 按键中间标志 */
uint16 dis_tof_mm = 0; /* TOF测距传感器距离(mm) */

uint16 time_flag = 0;   /* 延时标志，预留扩展 */
uint16_t flag_open = 0; /* 使能标志，预留扩展 */
int16_t flag_main = 0;  /* 主状态标志 */
uint16_t flag_text = 0; /* 文本显示标志 */

/* ---- 偏航角相关 ----
 * angle_Z: 连续累加偏航角（可超越±180度），用于多圈旋转等场景
 *          计算公式：angle_Z = 360 * Dirchange + yaw
 *          Dirchange 在每次跨过 ±180度边界时 ±1，表示累加圈计数
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
        flag_main = 1;
        flag_stop = 1;
        Yao.Outp_Gyro_Pitch = 0;
        Yao.Outp_Angle_Pitch = 0;
        Yao.Outp_Speed_Pitch = 0;
        small_driver_set_duty(0, 0);
    }

    if (flag_main)
    {
        flag_stop = 1;
        Yao.Outp_Gyro_Pitch = 0;
        Yao.Outp_Angle_Pitch = 0;
        Yao.Outp_Speed_Pitch = 0;
        small_driver_set_duty(0, 0);
    }
    else
    {
        if (ins_open == 0 || menu_mode == 1)
        {
            small_driver_set_duty((int16)(-(Yao.Outp_Gyro_Pitch - Yao.Outp_Gyro_Yaw)),  // 左轮占空比
                                  (int16)(-(Yao.Outp_Gyro_Pitch + Yao.Outp_Gyro_Yaw))); // 右轮占空比
        }
        else
        {
            small_driver_set_duty((int16)(-(Yao.Outp_Gyro_Pitch - Yao.Outp_Gyro_Yaw)),  // 左轮占空比
                                  (int16)(-(Yao.Outp_Gyro_Pitch + Yao.Outp_Gyro_Yaw))); // 右轮占空比
        }
    }
}
/**
 * @brief   1ms中断服务函数
 *
 * 执行顺序：
 *   1. TOF测距传感器读取（菜单模式下）
 *   2. IMU偏航角积分（累加gyro_z * 0.001 得到增量yaw）
 *   3. 偏航角归一化到[-180度, 180度]
 *   4. 跨越±180度边界检测（Dirchange累加/递减）
 *   5. 更新连续累加偏航角 angle_Z 变量
 *   6. 到达站计时器 a11111 累加
 */
void Interrupt_1ms(void)
{
    if (menu_mode)
    {
        dis_tof_mm = tof_dl1b_get_mm();
    }

    /* 偏航角积分：gyro_z(度/秒) * 0.001秒 = 每次中断角度增量 */
    imu660ra.eulerAngle.yaw += imu660ra.data_Raw.gyro_z * 0.001;

    /* 偏航角归一化到 [-180度, 180度] 范围 */
    if (imu660ra.eulerAngle.yaw > 180)
        imu660ra.eulerAngle.yaw -= 360;
    if (imu660ra.eulerAngle.yaw < -180)
        imu660ra.eulerAngle.yaw += 360;

    /* 跨越±180度边界检测
     *   从+180跳变到-180（差值<-350），说明正转一圈，Dirchange++
     *   从-180跳变到+180（差值>350），说明反转一圈，Dirchange--
     */
    if ((imu660ra.eulerAngle.yaw - imu660ra.eulerAngle.last_yaw) < -350)
        imu660ra.eulerAngle.Dirchange++;
    else if ((imu660ra.eulerAngle.yaw - imu660ra.eulerAngle.last_yaw) > 350)
        imu660ra.eulerAngle.Dirchange--;

    /* 更新连续累加偏航角（支持超越±180度范围） */
    angle_Z = 360 * imu660ra.eulerAngle.Dirchange + imu660ra.eulerAngle.yaw;
    imu660ra.eulerAngle.last_yaw = imu660ra.eulerAngle.yaw;

    /* 到达站计时器（flag_Single=1时才累加，否则清零） */
    if (flag_Single)
    {
        a11111++;
    }
    else
        a11111 = 0;
}

float integer = 0;     /* 陀螺仪Y轴累加值，用于标定零偏 */
uint32 num_t = 0;      /* 陀螺仪Y轴采样计数器 */
float integer1 = 0;    /* 陀螺仪X轴累加值，用于标定零偏 */
uint32 num_t1 = 0;     /* 陀螺仪X轴采样计数器 */
volatile float y1 = 0; /* 角速度低通滤波值，用于角速度环输入 */
float ddddd = 0;       /* 预留变量 */

/**
 * @brief   2ms中断服务函数
 *
 * 执行顺序：
 *   1. 跳跃控制：flag_jump=1时每次调用 jump_control() 平滑状态
 *   2. AI数据：flag_ai_open=1时调用接收函数进行PID参数更新
 *   3. 菜单：menu_open=1时执行菜单逻辑和参数修改
 *   4. 按键KEY_2（长按3秒）触发跳跃：a2222>=3000时flag_jump翻转
 *   5. IMU数据读取：date_handle()
 *   6. 姿态角度校准（注释掉，预留功能）
 *   7. 角速度低通滤波 + 角速度环PID（内环）
 *   8. 偏航角速度环PID（内环）
 */
void Interrupt_2ms(void)
{
    /* 1. 跳跃控制：每次调用平滑过渡跳跃状态 */
    if (flag_jump)
    {
        time_j++;
        jump_control();
    }
    else
        time_j = 0;

    key_scanner();

    /* 2. AI数据：调用接收函数进行PID参数更新 */
    if (flag_ai_open)
    {
        AI_Pid_Tuner_ProcessRx();
    }

    /* 3. 菜单逻辑 */
    menu_mode = 1;

    /* 5. IMU数据读取，更新姿态角 */
    date_handle();

    /* 7. 调试模式：IMU数据直读标志和Z轴偏航角积分，计算偏航偏移 */
    if (menu_mode)
    {
        if (IMU_JF_Flag)
        {
            Z_Yaw += imu660ra.data_Raw.gyro_z / 500;
        }
        else
        {
            Z_Yaw = 0;
        }
    }

    /* 8. 角速度环PID（内环计算）----
     * 正向通道：X轴角速度 低通滤波(0.15新+0.85旧) -> 角速度PID -> 限幅至8000
     * 偏航通道：Z轴角速度 -> 角速度PID -> 限幅至8000
     * 注意：角速度环的SetPoint取自角度环外环的输出值 -Yao.Outp_Angle_Pitch等，
     *       构成串级控制，匹配传感器轴与电机方向
     */
    y1 = 0.15f * ((float)-imu660ra_gyro_x) + 0.85f * y1;
    Yao.Outp_Gyro_Pitch = -limit(Cascade_gyro_Pitch(&PID_all.Pid_Gyro_Pitch, erect_Gyro_Pitch, y1, -Yao.Outp_Angle_Pitch), 8000.0f);
    Yao.Outp_Gyro_Yaw = limit(Cascade_gyro_Yaw(&PID_all.Pid_Gyro_Yaw, erect_Gyro_Yaw, imu660rb_gyro_z, Yao.Outp_turn), 8000.0f);
}

/* ---- 4ms中断全局变量 ---- */
volatile float aa1 = 0;                                  /* 姿态角低通滤波值，用于角度环输入 */
volatile float aa2 = 0;                                  /* 转向角低通滤波值，用于转向PID输入 */
volatile float dd = 0;                                   /* 偏差偏移低通滤波值 */
float temp_erect_turn[4];                                /* 转向PID临时参数（模式切换、动态KP） */
float k11 = 0;                                           /* 车轮差速系数1 */
float k22 = 0;                                           /* 车轮差速系数2 */
float kp_roll = 0.9;                                     /* 横滚KP系数 */
float Target_Yaw = 0;                                    /* 目标偏航角（turn_mode=3直行模式用） */
float V_trans = 0;                                       /* 横向速度，预留扩展 */
uint8 TCount_falg_4ms = 0;                               /* 4ms计数使能标志 */
uint16 TCount_4ms = 0;                                   /* 4ms计数器累加器 */
float dt = 0.004f;                                       /* 控制周期（4ms=0.004秒） */
float desired_yaw = 0.0f;                                /* 期望偏航角，视觉/GPS模式用 */
float raw_vision_yaw = 0.0f;                             /* 视觉原始偏航角，锥桶避障前 */
static float steer_vision_cmd_lpf_alpha = 0.35f;         /* 视觉指令低通滤波系数 */
static float steer_vision_cmd_lpf = 0.0f;                /* 视觉指令低通滤波值 */
volatile float steer_vision_target_yaw_deg = 0.0f;       /* 视觉目标偏航角（度） */
volatile float steer_vision_cone_avoid_delta_deg = 0.0f; /* 视觉锥桶角度偏移（度） */

/* ---- GPS转向相关变量已移至 gps_steer_pp ---- */

/**
 * @brief   4ms中断服务函数
 *
 * 执行顺序：
 *   1. 卡尔曼滤波更新姿态（imu963ra_kalman_filter_update）
 *   2. 姿态偏移补偿（减去offset_angle）
 *   3. 角度死区处理（|pitch|<0.4度时清零）
 *   4. 直立角度环PID（外环计算），输出作为 角速度环 的目标值
 *   5. 转向PID模式切换执行（turn_mode 0~7）
 *
 * 转向模式说明：
 *   mode 0: 关闭转向
 *   mode 1: 逐飞PID转向（PID_turn_seekfree，使用erect_turn参数）
 *   mode 2: 通用转向（Cascade_angle_Yaw，支持fuzzy_mode动态KP）
 *   mode 3: 偏航角度锁环直行（Cascade_angle_Yaw_2，Target_Yaw固定值）
 *   mode 4: 视觉转向（Cascade_angle_Yaw_3，desired_yaw=视觉目标+锥桶偏移，经低通滤波）
 *   mode 5: GPS转向（Cascade_angle_Yaw_4，desired_yaw=GPS方位角+IMU偏移）
 *   mode 6: 原地旋转Spin3（Cascade_angle_Yaw_2，angle_Z为当前角，到位后自动切换mode2）
 *   mode 7: 惯导转向（Cascade_angle_Yaw_2，目标=惯导输出）
 */
void Interrupt_4ms(void)
{
    /* 1. 卡尔曼滤波更新，融合加速度计和陀螺仪（roll/pitch） */
    imu963ra_kalman_filter_update(&imu);

    /* 2. 偏移补偿，减去开机标定的零偏 */
    imu.roll -= imu660ra.offset_angle.roll;
    imu.pitch -= imu660ra.offset_angle.pitch;
    imu660ra.eulerAngle.roll = imu.pitch; /* 注意：卡尔曼roll映射到eulerAngle.pitch */
    imu660ra.eulerAngle.pitch = imu.roll; /* 卡尔曼pitch映射到eulerAngle.roll（坐标系转换） */

    /* 3. 角度死区处理：|pitch|<0.4度时强制清零，消除微小抖动 */
    if (imu660ra.eulerAngle.pitch < 0.4 && imu660ra.eulerAngle.pitch > -0.4)
        imu660ra.eulerAngle.pitch = 0;

    /* 4ms计数器 */
    if (TCount_falg_4ms)
        TCount_4ms++;
    else
        TCount_4ms = 0;

    /* 4. 直立角度环PID（外环计算）----
     * 输入：姿态角低通滤波值 aa1（滤波系数0.1新+0.9旧）
     * 设定值：Yao.Outp_Speed_Pitch（来自速度环输出）
     * 输出限幅至 12000
     */
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

    /* 偏差偏移低通滤波 */
    dd = 0.1f * Deviation_Value + 0.9f * dd;

    /* 5. 转向控制  ---- */
    if (turn_mode == 1)
    {
        /* 模式1：逐飞PID转向（基于偏航角速度控制） */
        Yao.Outp_turn = PID_turn_seekfree(&PID_all.Pid_turn, erect_turn, imu660ra.data_Raw.gyro_z, Deviation_Value * 10 + 0.2f);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 2)
    {
        /* 模式2：通用转向
         * fuzzy_mode=0: 使用固定KP(erect_Angle_Yaw)
         * fuzzy_mode=1: 使用模糊动态KP(Get_P，根据偏差大小调整）
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
        Yao.Outp_turn = Cascade_angle_Yaw(&PID_all.Pid_turn, temp_erect_turn, Deviation_Value * 10, 0);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 3)
    {
        /* 模式3：偏航角度锁环直行，Target_Yaw为固定目标值 */
        Yao.Outp_turn = Cascade_angle_Yaw_2(&PID_all.Pid_Angle_Yaw, erect_Angle_Yaw_2, imu660ra.eulerAngle.yaw, Target_Yaw);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 4)
    {
        /* 模式4：视觉转向
         * 目标角度 = 视觉目标 + 锥桶偏移量
         * 经过低通滤波(a=0.35)平滑目标值，减少锥桶滤波的抖动
         */
        raw_vision_yaw = steer_vision_target_yaw_deg + steer_vision_cone_avoid_delta_deg;
        raw_vision_yaw = steer_wrap_deg180(raw_vision_yaw);

        /* 低通滤波平滑目标偏航角 */
        steer_vision_cmd_lpf = (1.0f - steer_vision_cmd_lpf_alpha) * steer_vision_cmd_lpf + steer_vision_cmd_lpf_alpha * raw_vision_yaw;
        desired_yaw = steer_vision_cmd_lpf;
        Yao.Outp_turn = Cascade_angle_Yaw_3(&PID_all.Pid_turn1, erect_Angle_Yaw_3, imu660ra.eulerAngle.yaw, desired_yaw);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 5)
    {
        /* 模式5：GPS转向
         * 从 gps_steer_pp 的 read_idx 侧读取导航输出
         * 期望偏航角 = GPS方位角 + IMU偏移修正
         */
        const gps_steer_output_t *gps_steer = GPS_STEER_READ();
        desired_yaw = gps_steer->target_bearing_deg + gps_steer->imu_yaw_offset_deg;
        Yao.Outp_turn = Cascade_angle_Yaw_4(&PID_all.Pid_turn2, erect_Angle_Yaw_3, imu660ra.eulerAngle.yaw, desired_yaw);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 6)
    {
        /* 模式6：原地旋转Spin3
         * 使用连续累加角度 angle_Z（支持超越±180度范围）作为当前角度
         * 目标角度 spin3_target_angle（初始值如1080度=3圈）
         *
         * 到位判定：连续角度误差 < spin3_angle_ok_deg 且
         *           Z轴角速度 < spin3_gyro_ok_dps
         * 持续 spin3_hold_ticks 个周期后，切换mode2，以当前偏航为Target_Yaw
         */
        Yao.Outp_turn = Cascade_angle_Yaw_2(&PID_all.Pid_Angle_Yaw, erect_Angle_Yaw_2, angle_Z, spin3_target_angle);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);

        /* 检测到位判定（角度误差 + 角速度同时满足） */
        if (func_abs(spin3_target_angle - angle_Z) < spin3_angle_ok_deg &&
            func_abs((float)imu660rb_gyro_z) < spin3_gyro_ok_dps)
        {
            spin3_hold_cnt++;
        }
        else
        {
            spin3_hold_cnt = 0;
        }

        /* 到位持续足够周期后退出原地旋转 */
        if (spin3_hold_cnt >= spin3_hold_ticks)
        {
            spin3_active = 0;
            spin3_hold_cnt = 0;
            Yao.Outp_turn = 0;
            turn_mode = 2;                        /* 切换回通用转向模式 */
            Target_Yaw = imu660ra.eulerAngle.yaw; /* 以当前偏航为目标 */
        }
    }
    else if (turn_mode == 7)
    {
        /* 模式7：惯导转向
         * ins_open=1时使用惯导输出 yaw_ins（归一化到[-180,180]）作为目标
         */
        if (ins_open)
        {
            float yaw_ins_deg = (float)yaw_ins;
            if (yaw_ins_deg > 180.0f)
                yaw_ins_deg -= 360.0f;
            if (yaw_ins_deg < -180.0f)
                yaw_ins_deg += 360.0f;
            /* 确保 setpoint 在当前 yaw 的 ±180度 范围内（取最短路径） */
            {
                float diff = yaw_ins_deg - imu660ra.eulerAngle.yaw;
                if (diff > 180.0f)
                    yaw_ins_deg -= 360.0f;
                else if (diff < -180.0f)
                    yaw_ins_deg += 360.0f;
            }
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
        /* mode 0：关闭转向输出 */
        Yao.Outp_turn = 0;
    }
}

/* ---- 8ms中断全局变量 ---- */
int16 recordL = 0; /* 左轮记录值，预留扩展 */
int16 recordR = 0; /* 右轮记录值，预留扩展 */
float vv2 = 0;     /* 差速低通滤波值，预留扩展 */

/**
 * @brief   8ms中断服务函数
 *
 * 执行顺序：
 *   1. 腿部高度切换控制（Single_Control，flag_Single=1时执行）
 *   2. 舵机平衡控制（servo_balance，steer_control_mode=1时执行）
 *
 * 注意：原速度环PID（Cascade_speed_Pitch）已迁移至16ms中断。
 *       Adapt_Terrain() 函数也已注释，当前在main循环中调用。
 */
void Interrupt_8ms(void)
{
    /* 1. 到达站腿部高度切换控制 */
    if (flag_Single)
    {
        Single_Control();
    }

    /* 2. 舵机平衡控制（PWM模式，直接使用差速控制） */
    servo_balance();
}

/* ---- 16ms中断全局变量 ---- */
volatile float aa11 = 0;        /* 差速低通滤波值，用于速度环输入 */
volatile float speed_MOTOR = 0; /* 平均速度，用于惯导坐标更新 */

/**
 * @brief   16ms中断服务函数
 *
 * 执行顺序：
 *   1. 惯导实时坐标更新（get_realtime_coordinate，ins_open=1时执行）
 *   2. 平均速度计算：左右轮差速/2
 *   3. 速度低通滤波（系数0.1新+0.9旧）
 *   4. 速度环PID（外环计算）：
 *      输入=滤波后平均速度，设定值=0（平衡时目标速度为0，防止冲跑），输出限幅至 100
 *
 * 注意：速度环输出 Yao.Outp_Speed_Pitch 作为角度环的SetPoint，
 *       形成"速度环->角度环->角速度环"三环串级控制。
 */
void Interrupt_16ms(void)
{
    /* 1. 惯导实时坐标更新（16ms周期，传入当前偏航角） */
    if (ins_open)
        get_realtime_coordinate(speed_MOTOR, 0.016, imu660ra.eulerAngle.yaw);

    /* 2. 平均速度计算（左右轮匹配方向、取反转向） */
    speed_MOTOR = (float)(-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data) / 2.0f;

    /* 3. 速度低通滤波 */
    aa11 = 0.1f * (((float)(-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data)) / 2.0f) + 0.9f * aa11;

    /* 4. 速度环PID（外环计算）
     * 设定值=0（平衡时目标速度为0，防止冲跑）
     * 输出限幅至 100（防止速度环输出过大导致角度环发散）
     */
    Yao.Outp_Speed_Pitch = -Cascade_speed_Pitch(&PID_all.Pid_Speed_Pitch, erect_Speed_Pitch, aa11, 0);
    Yao.Outp_Speed_Pitch = limit(Yao.Outp_Speed_Pitch, 100.0f);
}

/* ---- 40ms中断 ---- */
extern uint8 Zebra_Count_Flag; /* 斑马线逻辑标志（外部定义） */
uint16 TCount_40ms = 0;        /* 40ms计数器累加器，用于斑马线超时检测 */

/* ---- 遥测发送控制 ---- */
uint8 telemetry_enable = 1;     /* 遥测使能：0=关闭, 1=开启（通过无线串口发送调试数据） */
uint8 ins_telemetry_enable = 1; /* 惯导遥测使能：0=关闭, 1=开启 */

/**
 * @brief   40ms中断服务函数
 *
 * 执行顺序：
 *   1. 斑马线超时检测
 *   2. 偏航角漂移补偿
 *   3. 遥测数据交替发送（每80ms切换 $T 姿态遥测 / $I 惯导遥测）
 *      $T格式: tick,pitch,roll,yaw,gx,gy,gz,outp_turn,... (20字段)
 *      $I格式: x,y,ins_mode,dis_ins,yaw_ins,n,target,flag_save (8字段)
 *   4. 惯导录点滤波发送 $W 航点帧
 */
void Interrupt_40ms(void)
{
    /* 1. 斑马线超时检测：4秒未检测到新斑马线则触发 */
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

    /* 2. 偏航角漂移补偿 */
    imu660ra.eulerAngle.yaw += 0.0001;

    /* 3. 遥测数据交替发送：
     *    telemetry_toggle=0 时发送 $T 姿态遥测
     *    telemetry_toggle=1 时发送 $I 惯导遥测
     *    两者轮流发送，每个周期 80ms 发送一次
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
                /* $I 惯导遥测帧（$T关闭时，独占发送窗口） */
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
            /* toggle=1 时发送 $I 惯导遥测帧（$T 间隔中） */
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

    /* 4. 惯导录点滤波发送，发送 $W 航点帧 */
    if (ins_telemetry_enable && ins_getdata)
    {
        char wp_buf[50];
        int wp_len = sprintf(wp_buf,
                             "$W,%d,%.1f,%.1f,%d\r\n",
                             (int)(n - 1),                /* 更新后的航点索引 */
                             cod_saved[(uint8)(n - 1)].x, /* 航点X */
                             cod_saved[(uint8)(n - 1)].y, /* 航点Y */
                             (int)n                       /* 当前航点总数 */
        );
        if (wp_len > 0 && wp_len < (int)sizeof(wp_buf))
        {
            wireless_uart_send_string(wp_buf);
        }
        ins_getdata = 0; /* 发送完毕，清除标志 */
    }
}
