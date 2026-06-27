/**
 * @file    PID.c
 * @brief   轮腿机器人PID控制模块实现
 *
 * 包含以下核心功能：
 *   1. 五连杆闭环逆运动学解算（inverseKinematics）
 *   2. 地形自适应主控制循环（Adapt_Terrain）
 *   3. 串级PID控制（外环角度/速度 → 内环角速度）
 *   4. 增量式PID控制（位移、高度、翻滚）
 *   5. 转向PID（含陀螺仪前馈）
 *   6. 原地旋转控制（Spin3）
 *
 * 控制架构：
 *   俯仰方向：速度环 → 角度环 → 角速度环（三环串级）
 *   翻滚方向：角度环 → 角速度环（双环串级）
 *   偏航方向：角度环 + 角速度环（并联/独立）
 *   位移方向：增量式PID（X前后、Y高度、翻滚稳定）
 */

#include "zf_common_headfile.h"
#include "PID.h"
#include "servo.h"
#include "Interrupt.h"
#include "imu660.h"
#include "image.h"
#include "small_driver_uart_control.h"
#include "Math_Advanced.h"

/* 全局PID控制器和逆运动学参数实例 */
PID_ERECT PID_all;
IKparam IKParam;

/* ---- 角度归一化函数 ----
 * 将角度归一化到 [-180°, 180°] 范围
 * 用于偏航角误差计算，避免 ±360° 的跳变
 */
static inline float normalize_angle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/* ---- Spin3 原地旋转控制变量 ----
 * 功能：让机器人绕偏航轴旋转指定圈数（如1080°=3圈）后锁定。
 * 流程：Spin3_Start(dir) 激活 → 每周期检查是否到位 → 到位后保持spin3_hold_ticks周期 → 完成
 */
volatile uint8 spin3_active = 0;          /* 旋转激活标志（1=正在旋转） */
volatile int8 spin3_dir = 1;              /* 旋转方向：+1逆时针, -1顺时针 */
volatile float spin3_start_angle = 0.0f;  /* 旋转起始时的偏航角 */
volatile float spin3_target_angle = 0.0f; /* 旋转目标偏航角（起始角 ± 1080°） */
volatile uint16 spin3_hold_cnt = 0;       /* 到位后保持计数器 */
volatile float spin3_angle_ok_deg = 3.0f; /* 角度到位容差（度） */
volatile float spin3_gyro_ok_dps = 8.0f;  /* 陀螺仪静止判断容差（度/秒） */
volatile uint16 spin3_hold_ticks = 40;    /* 到位后需保持的控制周期数 */

/******************************************
 * 角度偏移标定
 *   offset_angle.pitch = 2    —— 调大车往前倾，调小往后仰
 *   offset_angle.roll  = -1.8
 *******************************************/

/************************************************************************
 * PID参数数组
 *
 * 数组布局说明：
 *   角度/角速度/速度环（4元素）： {KP, KP2(非线性增益), KD, 积分限幅}
 *   增量式PID（3元素）：        {KP, KI, KD}
 *   转向PID（4元素）：          {KP, KP2(非线性项), KD, 陀螺仪前馈增益}
 *
 * 滤波系数 a：
 *   a越大 → 响应越慢但抗干扰越强（更平滑）
 *   a越小 → 响应越快但噪声敏感
 *   典型值：角速度环0.9，角度环0.5~0.9
 *
 * 备用参数组注释（已注释掉）记录了历史调试过的参数值。
 ************************************************************************/

/* ---- 俯仰角平衡参数 ----
 * 历史值参考：erect_Gyro_Pitch  = {23.6, 0, 17, 0}
 *            erect_Angle_Pitch = {18, 0, 12.9, 0}
 *            erect_Speed_Pitch = {0.01, 0.000009, 0.004, 0}
 */
// float erect_Gyro_Pitch[4]   = {  23.6  ,  0  ,  17  ,  0  };
// float erect_Angle_Pitch[4]  = {  18  ,  0  ,  12.9  ,  0  };
// float erect_Speed_Pitch[4]  = {  0.01  ,  0.000009  ,  0.004  ,  0  };
float erect_Gyro_Pitch[4] = {0.9, 0, 0, 0};   /* 俯仰角速度环: {KP, KP2, KD, 积分限幅} */
float erect_Angle_Pitch[4] = {280, 0, 50, 0}; /* 俯仰角度环:   {KP, KP2, KD, 积分限幅} */
float erect_Speed_Pitch[4] = {0, 0, 0, 0};    /* 俯仰速度环:   {KP, KP2, KD, 积分限幅} */

/* ---- 翻滚角平衡参数 ----
 * 用于左右方向的平衡控制，当前置零（未启用翻滚主动平衡）。
 */
float erect_Gyro_Roll[4] = {0, 0, 0, 0};  /* 翻滚角速度环 */
float erect_Angle_Roll[4] = {0, 0, 0, 0}; /* 翻滚角度环 */

/* ---- 转向PID参数 ----
 * 参数含义：{KP(比例), KP2(误差*|误差| 非线性增益), KD(微分), 陀螺仪前馈增益}
 * 非线性项 KP2 * err * |err| 使大误差时输出更强，小误差时更柔和。
 */
float erect_turn[4] = {0, 0, 0, 0}; /* 转向PID */

/* ---- 偏航控制参数 ---- */
float erect_Gyro_Yaw[4] = {1, 0, 0, 0};     /* 偏航角速度环 */
float erect_Angle_Yaw[4] = {650, 0, 0, 0};  /* 偏航角度环1（主） */
float erect_Angle_Yaw_2[4] = {90, 0, 0, 0}; /* 偏航角度环2（备用，历史值300,0,0,0） */
float erect_Angle_Yaw_3[4] = {0, 0, 0, 0};  /* 偏航角度环3（视觉融合用） */
float erect_Angle_Yaw_4[4] = {0, 0, 0, 0};  /* 偏航角度环4（惯导融合用） */

/* ---- 运动学增量式PID参数 ----
 * erect_Inc_X: X方向前后位移控制，值越大响应越强。
 * erect_Inc_Y: Y方向高度控制，通过调整连杆Y坐标实现高度变化。
 * erect_Inc_Roll: 翻滚稳定控制，通过左右腿Y坐标差实现侧倾补偿。
 */
// 速度环，改变机械零点
float erect_Inc_X[4] = {0.004, 0, 0, 0}; /* X位移: {KP, KI, KD, 积分限幅} */ // 2.7
float erect_Inc_Y[3] = {1.2, 0, 0};                                          /* Y高度:  {KP, KI, KD} */
float erect_Inc_Roll[3] = {0.20, 0, 0.1};                                    /* 翻滚:   {KP, KI, KD} */
float erect_yawan[3] = {0, 0, 0};                                            /* 偏航辅助: {KP, KI, KD} */
float erect_Km[3] = {1.1, 0.1, 0.05};                                        /* 运动学混合系数（预留） */

/* ---- SZR转向调节参数 ---- */
float erect_SZR[4] = {0, 0, 0, 0}; /* SZR: {KP, KP2, KD, 积分限幅} */

/******** High模式参数组（已注释，用于高速/高动态场景） ********/
// float erect_Gyro_Pitch_High[4]   = {  1.5  ,  0  ,  2  ,  0  };
// float erect_Angle_Pitch_High[4]  = {  450  ,  0  ,  500  ,  0  };
// float erect_Gyro_Roll_High[4]    = {  0  ,  0  ,  0  ,  0  };
// float erect_Angle_Roll_High[4]   = {  0  ,  0  ,  0  ,  0  };
// float erect_turn_High[4]         = {  40.16  ,  10.78  ,  8.8  ,  -5.1  };
// float erect_Gyro_Yaw_High[4]     = {  10  ,  0  ,  20  ,  0  };
// float erect_Angle_Yaw_High[4]    = {  0  ,  0  ,  0  ,  0  };
// float erect_Inc_X_High[3]        = {  0.024, 0, 0.028 };
// float erect_Inc_Y_High[3]        = {  0.01, 0, 0 };
// float erect_Inc_Roll_High[3]     = {  -0.2, 0, 0 };
// float erect_Km_High[3]           = {  1.1,  0.1,  0.05  };
// float erect_SZR_High[4]          = {  0.3, 0, 0.6, 0 };
/*********** High模式参数组结束 **********/

/**
 * @brief   启动原地旋转（Spin3）
 * @param   dir     旋转方向：>=0逆时针, <0顺时针
 *
 * 设定目标偏航角为当前角度 ± 1080°（3圈），
 * 并将控制模式切换为 turn_mode=6（固定角度旋转模式）。
 */
void Spin3_Start(int8 dir)
{
    // if (spin3_active) return;

    spin3_dir = (dir >= 0) ? 1 : -1;
    spin3_start_angle = angle_Z;
    spin3_target_angle = spin3_start_angle + spin3_dir * 1080.0f;
    spin3_hold_cnt = 0;
    spin3_active = 1;

    /* 进入固定角度旋转模式 */
    flag_stop = 0;
    // Deviation_Value = 0;
    // desired_yaw = 0;
    Yao.Target_Speed = 0;
    turn_mode = 6;
}

/**
 * @brief   将角度归一化到 [-180°, 180°] 范围
 * @param   x   输入角度（度）
 * @return  归一化后的角度
 *
 * 避免偏航角在 ±180° 边界处产生不连续性，
 * 使PID计算中的角度误差始终取最短路径。
 */
float steer_wrap_deg180(float x)
{
    while (x > 180.0f)
        x -= 360.0f;
    while (x < -180.0f)
        x += 360.0f;
    return x;
}

/**
 * @brief   将 [-180°, 180°] 范围的目标角展开到连续角度坐标系
 * @param   target_deg              目标角（[-180, 180] 范围）
 * @param   current_continuous_deg  当前连续累计角度（可超过 ±180°）
 * @return  展开后的目标角，与 current_continuous_deg 处于同一连续坐标系
 *
 * 原理: 在 target_deg 上加减整数个 360°，使其最接近 current_continuous_deg。
 *       配合 angle_Z（连续累计偏航角）使用，消除 ±180° 边界跳变问题。
 */
float unwrap_to_continuous(float target_deg, float current_continuous_deg)
{
    float diff = target_deg - current_continuous_deg;
    while (diff > 180.0f)
        diff -= 360.0f;
    while (diff < -180.0f)
        diff += 360.0f;
    return current_continuous_deg + diff;
}

/************************************* 五连杆逆运动学逆解 *************************************/
/**
 * @brief   闭环五连杆模型逆运动学求解
 *
 * 模型描述（车体面朝左侧时）：
 *   - 后方为左腿，前方为右腿
 *   - L1=左腿上连杆, L2=左腿下连杆
 *   - L3=右腿上连杆, L4=右腿下连杆
 *   - L5=上方横连杆（连接左右腿）
 *   - alpha=上方舵机角, beta=下方舵机角（弧度）
 *
 * 求解原理：
 *   已知末端坐标(X,Y)，通过几何约束方程求解alpha和beta。
 *   每组方程有两个解（atan的±sqrt分支），需根据机械约束选取有效解：
 *   - alpha >= π/2（上方舵机需在上半区域）
 *   - 0 <= beta <= π/2（下方舵机需在第一象限）
 *
 * 结果转换为角度后直接输出到四个舵机（LF/RF/LB/RB）。
 */
float servoLeftFront, servoLeftRear, servoRightFront, servoRightRear;
void inverseKinematics()
{
    float alpha1, alpha2, beta1, beta2;

    /* ---- 左腿逆解 ---- */
    float aLeft = 2 * IKParam.XLeft * L1;
    float bLeft = 2 * IKParam.YLeft * L1;
    float cLeft = IKParam.XLeft * IKParam.XLeft + IKParam.YLeft * IKParam.YLeft + L1 * L1 - L2 * L2;
    float dLeft = 2 * L4 * (IKParam.XLeft - L5);
    float eLeft = 2 * L4 * IKParam.YLeft;
    float fLeft = ((IKParam.XLeft - L5) * (IKParam.XLeft - L5) + L4 * L4 + IKParam.YLeft * IKParam.YLeft - L3 * L3);

    /* 求解alpha的两个候选值（atan2分支） */
    alpha1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
    alpha2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
    /* 求解beta的两个候选值 */
    beta1 = 2 * atan((eLeft + sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));
    beta2 = 2 * atan((eLeft - sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));

    /* alpha归一化到[0, 2π) */
    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * M_PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * M_PI);

    /* 选解：alpha取>=π/2的解，beta取[0, π/2]内的解 */
    if (alpha1 >= M_PI / 2)
        IKParam.alphaLeft = alpha1;
    else
        IKParam.alphaLeft = alpha2;
    if (beta1 >= 0 && beta1 <= M_PI / 2)
        IKParam.betaLeft = beta1;
    else
        IKParam.betaLeft = beta2;

    /* ---- 右腿逆解（结构与左腿对称）---- */
    float aRight = 2 * IKParam.XRight * L1;
    float bRight = 2 * IKParam.YRight * L1;
    float cRight = IKParam.XRight * IKParam.XRight + IKParam.YRight * IKParam.YRight + L1 * L1 - L2 * L2;
    float dRight = 2 * L4 * (IKParam.XRight - L5);
    float eRight = 2 * L4 * IKParam.YRight;
    float fRight = ((IKParam.XRight - L5) * (IKParam.XRight - L5) + L4 * L4 + IKParam.YRight * IKParam.YRight - L3 * L3);

    IKParam.alphaRight = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    IKParam.betaRight = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

    /* 重新计算以完成选解逻辑 */
    alpha1 = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    alpha2 = 2 * atan((bRight - sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    beta1 = 2 * atan((eRight + sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));
    beta2 = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * M_PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * M_PI);

    if (alpha1 >= M_PI / 2)
        IKParam.alphaRight = alpha1;
    else
        IKParam.alphaRight = alpha2;
    if (beta1 >= 0 && beta1 <= M_PI / 2)
        IKParam.betaRight = beta1;
    else
        IKParam.betaRight = beta2;

    /* 弧度转换为角度，输出到四个舵机 */
    servoLeftFront = RAD_TO_ANGLE(IKParam.alphaLeft);
    servoLeftRear = RAD_TO_ANGLE(IKParam.betaLeft);
    servoRightFront = RAD_TO_ANGLE(IKParam.alphaRight);
    servoRightRear = RAD_TO_ANGLE(IKParam.betaRight);

    // if(servoLeftFront > 360)
    //    servoLeftFront -= 360;
    // if(servoLeftRear > 360)
    //    servoLeftRear -= 360;
    // if(servoRightFront > 360)
    //    servoRightFront -= 360;
    // if(servoRightRear > 360)
    //    servoRightRear -= 360;

    servo_set_angle(LF, servoLeftFront);
    servo_set_angle(RF, servoRightFront);
    servo_set_angle(LB, servoLeftRear);
    servo_set_angle(RB, servoRightRear);
    

    // servo_set_angle(RF, 180); servo_set_angle(RB, 0);
    // servo_set_angle(LF, 225); servo_set_angle(LB, -45);
}

/************************************* 五连杆逆运动学逆解 *************************************/

/**
 * @brief   地形自适应主控制函数
 *
 * 每控制周期执行一次，完成以下步骤：
 *   1. 计算左右轮速差 → 增量式PID → 得到X方向位移调整量
 *   2. 计算当前高度误差 → 增量式PID → 得到Y方向高度调整量
 *   3. 计算SZR转向调节量
 *   4. 计算翻滚稳定（stab_roll）和偏航辅助（yawan）
 *   5. 根据flag_stop/flag_jump等状态标志，设定IKParam目标坐标
 *   6. 坐标限幅处理后，调用逆运动学解算并输出舵机角度
 *
 * 单腿状态(Single_state)下：X范围[-3, 3]
 * 双腿状态(normal)下：  X范围[-1, 4.5], Y范围[3, 14]
 */
float X = 0, X_l = 0, Y = 5;    /* Y初始值5cm（需先确定舵机中位） */
float temp_kx_x[4] = {0};       /* X位移PID参数临时缓冲 */
volatile float vv1 = 0;         /* 左右轮速差低通滤波值 */
volatile float dd2 = 0;         /* 轮速和变化率（急加速检测） */
volatile float roll_filter = 0; /* 翻滚角低通滤波值 */
float stab_roll = 0;            /* 翻滚稳定补偿量 */
float kpp = -0.3;               /* 预留比例系数 */
float SZR = 0;                  /* SZR转向调节输出 */
float yawan = 0;                /* 偏航辅助输出 */
float dif_now = 0;              /* 当前周期轮速和绝对值 */
float dif_last = 0;             /* 上一周期轮速和绝对值 */

void Adapt_Terrain(void)
{
    //    static float a = 0.1;
    //    get_eulerAngle();

    // 平滑计算顶点坐标，后期可替换为PID
    vv1 = 0.05f * (float)(-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data) + 0.95 * vv1;
    dif_now = func_abs((float)(motor_value.receive_left_speed_data + motor_value.receive_right_speed_data));
    dd2 = func_abs(dif_now - dif_last);
    dif_last = dif_now;
    if (turn_mode == 0)
        X = PID_Increase_X(&PID_all.Pid_Inc_X, temp_kx_x, vv1, 0);
    else
        X = PID_Increase_X(&PID_all.Pid_Inc_X, temp_kx_x, vv1 + k11 * dd2, (float)Target_Speed);
    Y = Y - PID_Increase_Y(&PID_all.Pid_Inc_Y, erect_Inc_Y, Y, Yao.Target_height);
    //    X = a * X + (1-a) * X_l;
    SZR = PID_SZR_is_GOD(&PID_all.Pid_SZR, erect_SZR, (Deviation_Value * 10 + 0.2f), 0);

    X_l = X;

    roll_filter = 0.5f * roll_filter + 0.5f * imu660ra.eulerAngle.roll;
    if (flag_Single_HighState == 1)
        stab_roll += PID_Increase_Roll(&PID_all.Pid_Inc_Roll, erect_Inc_Roll, imu660ra.eulerAngle.roll, 0);
    else
        stab_roll = 0;

    stab_roll = limit(stab_roll, Single_Height - 3);
    if (flag_yawan == 1)
        yawan = PID_GOGOGO(&PID_all.Pid_GOGOGO, erect_yawan, Deviation_Value, 0);
    else
        yawan = 0;

    //    stab_roll = stab_roll + erect_Km[2] * (0 - imu660ra.eulerAngle.roll);

    //    if(Yao.Target_Speed == 0)
    //        X = 0;
    if (flag_main == 0 || flag_main == 2)
    {
        if (flag_jump == 0)
        {
            if (flag_jump_2 == 1)
            {
                IKParam.XLeft = 1.75;
                IKParam.XRight = 1.75;
            }
            else
            {
                IKParam.XLeft = 1.75 - X - SZR;
                IKParam.XRight = 1.75 - X + SZR;
            }
            //            IKParam.XLeft  = 1.75-X - SZR;
            //            IKParam.XRight = 1.75-X + SZR;

            //            IKParam.YLeft  = Y - stab_roll;
            //            IKParam.YRight = Y + stab_roll;

            if (stab_roll >= 0)
            {
                IKParam.YLeft = Y;
                IKParam.YRight = Y - stab_roll;
            }
            else
            {
                IKParam.YLeft = Y + stab_roll;
                IKParam.YRight = Y;
            }

            if (yawan > 0)
                IKParam.YLeft += yawan;
            else if (yawan < 0)
                IKParam.YRight -= yawan;
        }
        else
        {
            if (flag_jump_1 == 1)
            {
                IKParam.XLeft = 1.75 - X - SZR;
                IKParam.XRight = 1.75 - X + SZR;
            }
        }
    }
    else
    {
        IKParam.XLeft = 1.75;
        IKParam.YLeft = Y;
        IKParam.XRight = 1.75;
        IKParam.YRight = Y;
    }

    if (Single_state == 1 && flag_Single_HighState == 0) // 减速
    {
        //        TCount_falg_4ms= 1;
        temp_kx_x[0] = erect_Inc_X[0] * 2;
        temp_kx_x[1] = 0;
        temp_kx_x[2] = 0;
        temp_kx_x[3] = 0;
        //        if(func_abs(Y - Yao.Target_height) <= 0.1)
        //        {
        //            IKParam.XLeft  -= 0.1;
        //            IKParam.XRight -= 0.1;
        IKParam.XLeft = func_limit_ab(IKParam.XLeft, -3, 3);
        IKParam.XRight = func_limit_ab(IKParam.XRight, -3, 3);
        //            if(TCount_4ms >= 15)
        //                TCount_falg_4ms = 0;
        //        }
        //        IKParam.YLeft  = 6;
        //        IKParam.YRight = 6;
    }
    else
    {
        temp_kx_x[0] = erect_Inc_X[0];
        temp_kx_x[1] = 0;
        temp_kx_x[2] = 0;
        temp_kx_x[3] = 0;
        IKParam.XLeft = Limit_Float(IKParam.XLeft, -1.0f, 4.5f);
        IKParam.XRight = Limit_Float(IKParam.XRight, -1.0f, 4.5f);
        IKParam.YLeft = Limit_Float(IKParam.YLeft, 3.0f, 14.0f);
        IKParam.YRight = Limit_Float(IKParam.YRight, 3.0f, 14.0f);
    } // 收腿:2.6, 14.8, 7
    //    IKParam.XLeft  = -1;
    //    IKParam.XRight = -1;
    //    IKParam.YLeft  = 3;
    //    IKParam.YRight = 3;

    // 运动学逆解算并输出舵机
    if (flag_jump_1 == 1 || flag_jump == 0)
        inverseKinematics();
}

/************************************* 转向PID（含陀螺仪前馈） ************************************/
/**
 * @brief   转向PID控制器（含陀螺仪前馈）
 * @param   pid_info    PID状态结构体指针
 * @param   PID_Parm    参数数组 {KP, KP2(非线性增益), KD, 陀螺仪前馈增益}
 * @param   gyro        当前陀螺仪角速度（用于前馈补偿）
 * @param   err         目标误差
 * @return  控制输出值
 *
 * 输出 = KP*err + KP2*err*|err| + KD*dErr + 前馈增益*gyro
 *
 * 非线性项 KP2*err*|err|：
 *   大误差 → 输出增强（强力纠正）
 *   小误差 → 输出柔和（避免超调振荡）
 *
 * 陀螺仪前馈：直接补偿当前角速度，提高响应速度。
 * 低通滤波系数 a=0.5，中等平滑度。
 */
float PID_turn_seekfree(PID_INFO *pid_info, float *PID_Parm, float gyro, float err)
{
    float V_out;
    float a = 0.5;

    pid_info->iError = err;

    /* 低通滤波，使波形更平滑，滤除高频干扰，防止微分突变 */
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError;

    V_out = PID_Parm[0] * pid_info->iError +
            PID_Parm[1] * pid_info->iError * func_abs(pid_info->iError) +
            PID_Parm[2] * (pid_info->iError - pid_info->LastError) +
            PID_Parm[3] * gyro;
    pid_info->LastError = pid_info->iError;

    return V_out;
}
/************************************* 转向PID（含陀螺仪前馈） ************************************/

/************************************* 串级PID：速度环 → 角度环 → 角速度环 ************************************/

/**
 * @brief   俯仰速度环PID（串级最外层）
 * @param   pid_info    PID状态结构体指针
 * @param   PID_Parm    参数数组 {KP, KI, KD, 积分限幅}
 * @param   NowPoint    当前速度
 * @param   SetPoint    目标速度
 * @return  角度环设定值（作为串级PID中角度环的目标输入）
 *
 * 滤波系数 a=0.5（中等平滑）。
 * 输出供角度环作为SetPoint使用，形成三环串级结构。
 */
float Cascade_speed_Pitch(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.5;

    /* 1. 计算速度误差 */
    pid_info->iError = (NowPoint - SetPoint);

    /* 2. 低通滤波，使波形更平滑，滤除高频干扰，防止速度突变 */
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError;
    pid_info->SumError += pid_info->iError;

    /* 3. 积分限幅 */
    // if(PID_Parm[3])
    // {
    pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    // }

    /* 4. 计算输出 */
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

/**
 * @brief   俯仰角度环PID（串级中间层/外环）
 * @param   pid_info    PID状态结构体指针
 * @param   PID_Parm    参数数组 {KP, KI, KD, 积分限幅}
 * @param   NowPoint    当前角度
 * @param   SetPoint    目标角度
 * @return  角速度设定值（供角速度环使用）
 *
 * 滤波系数 a=0.9（强滤波，角度信号变化较慢）。
 * 输出为角速度环的SetPoint。
 */
float Cascade_angle_Pitch(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    static float temp_param[4] = {0};
    (void)temp_param; /* 备用参数（对应已注释的High模式切换），消除未使用警告 */
    float V_out;
    float a = 0.9;

    // if(flag_Single)
    // {
    //     temp_param[0] = erect_Angle_Pitch_High[0];
    //     temp_param[1] = erect_Angle_Pitch_High[1];
    //     temp_param[2] = erect_Angle_Pitch_High[2];
    //     temp_param[3] = erect_Angle_Pitch_High[3];
    // }
    // else
    // {
    //     temp_param[0] = PID_Parm[0];
    //     temp_param[1] = PID_Parm[1];
    //     temp_param[2] = PID_Parm[2];
    //     temp_param[3] = PID_Parm[3];
    // }

    /* 1. 计算角度误差 */
    pid_info->iError = (NowPoint - SetPoint);

    /* 2. 低通滤波 */
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError;
    pid_info->SumError += pid_info->iError;

    /* 3. 积分限幅 */
    if (PID_Parm[3])
    {
        pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    }

    /* 4. 计算输出 */
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

/**
 * @brief   翻滚角度环PID（角度外环）
 * @param   pid_info    PID状态结构体指针
 * @param   PID_Parm    参数数组 {KP, KI, KD, 积分限幅}
 * @param   NowPoint    当前翻滚角
 * @param   SetPoint    目标翻滚角（通常为0，表示水平）
 * @return  角速度设定值（供翻滚角速度环使用）
 *
 * 滤波系数 a=0.5（中等平滑）。
 */
float Cascade_angle_Roll(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.5;

    /* 1. 计算角度误差 */
    pid_info->iError = (NowPoint - SetPoint);

    /* 2. 低通滤波 */
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError;
    pid_info->SumError += pid_info->iError;

    /* 3. 积分限幅 */
    if (PID_Parm[3])
    {
        pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    }

    /* 4. 计算输出 */
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

/**
 * @brief   偏航角度环PID（PD控制，无积分项）
 * @param   pid_info    PID状态结构体指针
 * @param   PID_Parm    参数数组 {KP, 0, KD, 0}
 * @param   NowPoint    当前偏航角
 * @param   SetPoint    目标偏航角
 * @return  偏航控制输出
 *
 * 滤波系数 a=0.7。
 * 注意：积分项已注释，实际为PD控制（无I），避免偏航方向积分饱和。
 */
float Cascade_angle_Yaw(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.7;

    /* 1. 计算角度误差（归一化到 [-180°, 180°]） */
    pid_info->iError = normalize_angle(NowPoint - SetPoint);

    /* 2. 低通滤波 */
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError;
    pid_info->SumError += pid_info->iError;

    /* 3. 积分限幅（已注释，偏航方向不使用积分项） */
    // if(PID_Parm[3])
    // {
    //     pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    // }

    /* 4. 计算输出（仅PD，不含积分） */
    V_out = PID_Parm[KP] * pid_info->iError +
            // PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

/**
 * @brief   偏航角度环PID（备用通道2）
 *
 * 与 Cascade_angle_Yaw 结构相同，使用 erect_Angle_Yaw_2 参数组。
 * 预留用于不同控制源（如视觉/惯导）的独立PID通道。
 */
float Cascade_angle_Yaw_2(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.7;

    /* 1. 计算角度误差（归一化到 [-180°, 180°]） */
    pid_info->iError = normalize_angle(NowPoint - SetPoint);

    /* 2. 低通滤波 */
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError;
    pid_info->SumError += pid_info->iError;

    /* 3. 积分限幅 */
    if (PID_Parm[3])
    {
        pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    }

    /* 4. 计算输出（PID） */
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

/**
 * @brief   偏航角度环PID（视觉融合通道3）
 *
 * 与 Cascade_angle_Yaw 结构相同，使用 erect_Angle_Yaw_3 参数组。
 * 预留用于视觉里程计/视觉定位的偏航修正。
 */
float Cascade_angle_Yaw_3(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.7;

    /* 1. 计算角度误差（归一化到 [-180°, 180°]） */
    pid_info->iError = normalize_angle(NowPoint - SetPoint);

    /* 2. 低通滤波 */
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError;
    pid_info->SumError += pid_info->iError;

    /* 3. 积分限幅 */
    if (PID_Parm[3])
    {
        pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    }

    /* 4. 计算输出（PID） */
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

/**
 * @brief   偏航角度环PID（惯导融合通道4）
 *
 * 与 Cascade_angle_Yaw 结构相同，使用 erect_Angle_Yaw_4 参数组。
 * 预留用于IMU惯导融合的偏航修正。
 */
float Cascade_angle_Yaw_4(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.7;

    /* 1. 计算角度误差（归一化到 [-180°, 180°]） */
    pid_info->iError = normalize_angle(NowPoint - SetPoint);

    /* 2. 低通滤波 */
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError;
    pid_info->SumError += pid_info->iError;

    /* 3. 积分限幅（已注释） */
    // if(PID_Parm[3])
    // {
    //     pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    // }

    /* 4. 计算输出（仅PD） */
    V_out = PID_Parm[KP] * pid_info->iError +
            // PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

/**
 * @brief   俯仰角速度环PID（串级最内层）
 * @param   pid_info    PID状态结构体指针
 * @param   PID_Parm    参数数组 {KP, KI, KD, 积分限幅}
 * @param   NowPoint    当前角速度（陀螺仪）
 * @param   SetPoint    目标角速度（来自角度环输出）
 * @return  电机控制输出
 *
 * 滤波系数 a=0.9（强滤波，陀螺仪数据噪声较大）。
 * 角速度环响应最快，直接控制电机力矩输出。
 */
float Cascade_gyro_Pitch(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    static float temp_param[4] = {0};
    float V_out;
    float a = 0.9;

    // if(flag_Single)
    // {
    //     temp_param[0] = erect_Gyro_Pitch_High[0];
    //     temp_param[1] = erect_Gyro_Pitch_High[1];
    //     temp_param[2] = erect_Gyro_Pitch_High[2];
    //     temp_param[3] = erect_Gyro_Pitch_High[3];
    // }
    // else
    // {
    //     temp_param[0] = PID_Parm[0];
    //     temp_param[1] = PID_Parm[1];
    //     temp_param[2] = PID_Parm[2];
    //     temp_param[3] = PID_Parm[3];
    // }

    /* 1. 计算角速度误差 */
    pid_info->iError = (NowPoint - SetPoint);

    /* 2. 低通滤波 */
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError;
    pid_info->SumError += pid_info->iError;

    /* 3. 积分限幅 */
    if (temp_param[3])
    {
        pid_info->SumError = limit(pid_info->SumError, temp_param[3]);
    }

    /* 4. 计算输出 */
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

/**
 * @brief   翻滚角速度环PID（串级内层）
 * @param   pid_info    PID状态结构体指针
 * @param   PID_Parm    参数数组 {KP, KI, KD, 积分限幅}
 * @param   NowPoint    当前翻滚角速度
 * @param   SetPoint    目标翻滚角速度（来自翻滚角度环输出）
 * @return  控制输出
 *
 * 无低通滤波（陀螺仪原始值直接参与计算），响应最快。
 */
float Cascade_gyro_Roll(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float G_out;

    pid_info->iError = (NowPoint - SetPoint);
    pid_info->SumError += pid_info->iError;

    if (PID_Parm[3])
    {
        pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    }
    G_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return G_out;
}

/**
 * @brief   偏航角速度环PID
 * @param   pid_info    PID状态结构体指针
 * @param   PID_Parm    参数数组 {KP, KI, KD, 积分限幅}
 * @param   NowPoint    当前偏航角速度
 * @param   SetPoint    目标偏航角速度
 * @return  偏航控制输出
 *
 * 滤波系数 a=0.9（强滤波）。
 */
float Cascade_gyro_Yaw(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.9;

    /* 1. 计算角速度误差 */
    pid_info->iError = (NowPoint - SetPoint);

    /* 2. 低通滤波 */
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError;
    pid_info->SumError += pid_info->iError;

    /* 3. 积分限幅 */
    if (PID_Parm[3])
    {
        pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    }

    /* 4. 计算输出 */
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

/******************************************************************** 串级PID结束 ********************************************************************/

/**
 * @brief   X方向增量式PID（前后位移控制）
 * @param   pid_info    PID状态结构体指针
 * @param   PID_Parm    参数数组 {KP, KI, KD, 积分限幅}
 * @param   NowPoint    当前值（轮速差滤波值 vv1）
 * @param   SetPoint    目标值（目标速度）
 * @return  X方向位移调整量
 *
 * 滤波系数 a=0.9（强滤波）。
 * 输出叠加到 IKParam.XLeft/XRight 上，通过逆运动学转换为舵机角度。
 */
float PID_Increase_X(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    static float temp_param[3] = {0};
    (void)temp_param; /* 备用参数（对应已注释的High模式），消除未使用警告 */
    float V_out;
    float a = 0.9;

    // if(flag_Single)
    // {
    //     temp_param[0] = erect_Inc_X_High[0];
    //     temp_param[1] = erect_Inc_X_High[1];
    //     temp_param[2] = erect_Inc_X_High[2];
    // }
    // else
    // {
    //     temp_param[0] = PID_Parm[0];
    //     temp_param[1] = PID_Parm[1];
    //     temp_param[2] = PID_Parm[2];
    // }

    /* 1. 计算误差 */
    pid_info->iError = (NowPoint - SetPoint);

    /* 2. 低通滤波 */
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError;
    pid_info->SumError += pid_info->iError;

    /* 3. 积分限幅 */
    if (PID_Parm[3])
    {
        pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    }

    /* 4. 计算输出 */
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

/**
 * @brief   Y方向增量式PID（高度控制）
 * @param   pid_info    PID状态结构体指针
 * @param   PID_Parm    参数数组 {KP, KI, KD}
 * @param   NowPoint    当前高度（Y坐标）
 * @param   SetPoint    目标高度（Yao.Target_height）
 * @return  Y方向高度调整量
 *
 * 滤波系数 a=0.5（中等平滑）。
 * 输出叠加到逆运动学的Y坐标上，改变腿的高度。
 */
float PID_Increase_Y(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.5;

    /* 1. 计算高度误差 */
    pid_info->iError = (NowPoint - SetPoint);

    /* 2. 低通滤波 */
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError;
    pid_info->SumError += pid_info->iError;

    // /* 3. 积分限幅（已注释，无积分限幅保护） */
    // if(PID_Parm[3])
    // {
    //     pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    // }

    /* 4. 计算输出 */
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

/**
 * @brief   翻滚增量式PID（侧向稳定，使用二阶微分）
 * @param   pid_info    PID状态结构体指针
 * @param   PID_Parm    参数数组 {KP, KI, KD}
 * @param   NowPoint    当前翻滚角
 * @param   SetPoint    目标翻滚角（通常为0）
 * @return  翻滚补偿增量
 *
 * 与标准PID不同，该函数使用特殊的增量形式：
 *   KP项 = KP * (iError - LastError)        —— 比例增量
 *   KI项 = KI * iError                       —— 积分项（当前位置式）
 *   KD项 = KD * (iError - 2*LastError + PrevError) —— 二阶微分增量
 *
 * 使用 Pid_Inc_Roll 的 PrevError 字段存储上上次误差。
 */
float PID_Increase_Roll(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float Increase;

    pid_info->iError = NowPoint - SetPoint;

    Increase = PID_Parm[KP] * (pid_info->iError - pid_info->LastError) +
               PID_Parm[KI] * pid_info->iError +
               PID_Parm[KD] * (pid_info->iError - 2 * pid_info->LastError + pid_info->PrevError);

    pid_info->PrevError = pid_info->LastError;
    pid_info->LastError = pid_info->iError;
    pid_info->LastData = NowPoint;

    return Increase;
}

/**
 * @brief   SZR转向调节PID
 * @param   pid_info    PID状态结构体指针
 * @param   PID_Parm    参数数组 {KP, KP2, KD, 积分限幅}
 * @param   NowPoint    当前值（偏航偏差*10 + 0.2）
 * @param   SetPoint    目标值（通常为0）
 * @return  转向调节量（叠加到IKParam的X坐标上，左右反向）
 *
 * 滤波系数 a=0.9（强滤波）。
 * SZR输出使左右腿X坐标反向偏移：左腿-X+偏移，右腿-X-偏移，
 * 产生差动转向效果。
 */
float PID_SZR_is_GOD(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.9;

    /* 1. 计算误差 */
    pid_info->iError = (NowPoint - SetPoint);

    /* 2. 低通滤波 */
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError;
    pid_info->SumError += pid_info->iError;

    /* 3. 积分限幅 */
    if (PID_Parm[3])
    {
        pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    }

    /* 4. 计算输出 */
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

/**
 * @brief   偏航辅助PID（yawan调节）
 * @param   pid_info    PID状态结构体指针
 * @param   PID_Parm    参数数组 {KP, KI, KD, 积分限幅}
 * @param   NowPoint    当前偏航偏差
 * @param   SetPoint    目标值（通常为0）
 * @return  偏航辅助调节量（叠加到Y坐标上，左右腿反向）
 *
 * 滤波系数 a=0.9（强滤波）。
 * 输出通过左右腿Y坐标的差动实现偏航辅助力矩。
 */
float PID_GOGOGO(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.9;

    /* 1. 计算误差 */
    pid_info->iError = (NowPoint - SetPoint);

    /* 2. 低通滤波 */
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError;
    pid_info->SumError += pid_info->iError;

    /* 3. 积分限幅 */
    if (PID_Parm[3])
    {
        pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    }

    /* 4. 计算输出 */
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

/**
 * @brief   通用增量式PID（备用）
 * @param   pid_info    PID状态结构体指针
 * @param   PID_Parm    参数数组 {KP, KI, KD, 积分限幅}
 * @param   NowPoint    当前测量值
 * @param   SetPoint    目标设定值
 * @return  控制输出
 *
 * 滤波系数 a=0.9（强滤波）。
 * 通用增量式PID，可用于各类辅助控制回路。
 */
float PID_Increase(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.9;

    /* 1. 计算误差 */
    pid_info->iError = (NowPoint - SetPoint);

    /* 2. 低通滤波 */
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError;
    pid_info->SumError += pid_info->iError;

    /* 3. 积分限幅（已注释） */
    // if(PID_Parm[3])
    // {
    //     pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    // }

    /* 4. 计算输出 */
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

/**
 * @brief   PID状态初始化（清零）
 * @param   pid_info    待初始化的PID状态结构体指针
 *
 * 将所有误差项（当前误差、累计误差、前次误差、前前次误差、上次数据）清零。
 * 在控制启动或模式切换时调用，避免历史状态影响新控制回路。
 */
void pid_para_init(PID_INFO *pid_info)
{
    pid_info->iError = 0;
    pid_info->SumError = 0;
    pid_info->PrevError = 0;
    pid_info->LastError = 0;
    pid_info->LastData = 0;
}
