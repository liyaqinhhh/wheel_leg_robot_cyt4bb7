/*
 * servo.c
 *
 *  Created on: 2025年2月18日
 *      Author: LateRain
 */
#include "zf_common_headfile.h"
#include <math.h>
#include "servo.h"
#include "PID.h"
#include "Interrupt.h"
#include "small_driver_uart_control.h"
#include "image.h"
#include "Math_Advanced.h"

//uint32 pwmLF = 1510;
//uint32 pwmLB = 1450;
//uint32 pwmRF = 1420;
//uint32 pwmRB = 1520;

uint32 pwmLF = 4350;
uint32 pwmLB = 4550;
uint32 pwmRF = 4610;
uint32 pwmRB = 4560;
// 移植：PWM 引脚映射（TC264 ATOM 通道 → CYT4BB7 TCPWM 通道）
// LF: ATOM0_CH2_P21_4 → TCPWM_CH41_P12_5  （P21_4 在 CYT4BB7 不存在，改用 P12_5）
// LB: ATOM0_CH3_P21_5 → TCPWM_CH34_P21_5  （P21_5 直接对应）
// RF: ATOM0_CH1_P21_3 → TCPWM_CH39_P21_3  （P21_3 直接对应）
// RB: ATOM0_CH0_P21_2 → TCPWM_CH40_P21_2  （P21_2 直接对应）
pwm_channel_enum get_pwm_channel(leg_enum leg) {

    switch(leg)
    {
        case LF:return TCPWM_CH36_P21_6;break;
        case LB:return TCPWM_CH34_P21_5;break;
        case RF:return TCPWM_CH28_P19_3;break;
        case RB:return TCPWM_CH27_P19_2;break;
        default:return TCPWM_CH36_P21_6;break; // 移植：补全 default 避免无返回值警告
    }
}

void servo_init(void)
{
    // 频率100HZ不能更改
    pwm_init( get_pwm_channel(LF) , 300 , pwmLF );
    pwm_init( get_pwm_channel(LB) , 300 , pwmLB );
    pwm_init( get_pwm_channel(RF) , 300 , pwmRF );
    pwm_init( get_pwm_channel(RB) , 300 , pwmRB );
}

// 舵机参数配置结构体
typedef struct {
    uint16_t mid_pwm;      // 舵臂水平时的PWM值（单位us）
    float pwm_per_degree;  // 每度对应的PWM变化量（考虑方向）
    float min_angle;       // 有效角度最小值
    float max_angle;       // 有效角度最大值
} ServoConfig;

// 各腿舵机参数（需要根据实测校准）
static ServoConfig servo_cfg[] = {
    /* LF */ { 4530, -30.0f,  90.0f, 270.0f }, // 前左腿：PWM增大角度减小
    /* LB */ { 4350,  -30.0f, -90.0f,  90.0f }, // 后左腿
    /* RF */ { 4260,  30.0f,  90.0f, 270.0f }, // 前右腿：PWM增大角度增大
    /* RB */ { 4560, 30.0f, -90.0f,  90.0f }  // 后右腿
};

void servo_set_angle(leg_enum leg, float angle) {

    servo_cfg[LF].mid_pwm = (uint16)pwmLF;
    servo_cfg[LB].mid_pwm = (uint16)pwmLB;
    servo_cfg[RF].mid_pwm = (uint16)pwmRF;
    servo_cfg[RB].mid_pwm = (uint16)pwmRB;

    ServoConfig* cfg = &servo_cfg[leg];
    float mapped_angle;

    // 1. 角度周期映射（处理多圈旋转）
    mapped_angle = fmodf(angle, 360.0f);
    if(mapped_angle < 0) mapped_angle += 360.0f;

    // 2. 前腿特殊处理（保持90-270度坐标系）
    if(leg == LF || leg == RF) {
        if(mapped_angle < 90.0f) mapped_angle += 360.0f; // 处理-270度等效90度
    } else { // 后腿处理
        if(mapped_angle > 180.0f) mapped_angle -= 360.0f; // 转换为-180~180
    }

    // 3. 有效性检查
    if(mapped_angle < cfg->min_angle || mapped_angle > cfg->max_angle) {
        return; // 超出机械限制，不执行动作
    }

    // 4. 计算目标PWM
    float angle_offset;
    if(leg == LF || leg == RF) {
        angle_offset = mapped_angle - 180.0f; // 前腿以180度为中位
    } else {
        angle_offset = mapped_angle;          // 后腿以0度为中位
    }

    uint16 pwm = (uint16)(cfg->mid_pwm + angle_offset * cfg->pwm_per_degree);

    // 5. PWM安全限幅
    pwm = (pwm < 1500) ? 1500 : (pwm > 7200) ? 7200 : pwm;

    // 6. 设置PWM输出（需要实现pwm_set_duty函数）
    pwm_set_duty(get_pwm_channel(leg), pwm);
}

 uint8 T1=75;          // 上升阶段持续时间
 uint8 T2=45;         // 收腿阶段持续时间
 uint8 T3=30;          // 放腿阶段持续时间
#define     tt      1           // 缓冲阶段持续时间
//#define     T1      13          // 上升阶段持续时间
//#define     T2      20          // 收腿阶段持续时间
//#define     T3      20           // 放腿阶段持续时间
//#define     tt      1           // 缓冲阶段持续时间
//#define     MAXTIME     500     //  进程最大退出时间




 extern uint8 temp_flag_jump;
 uint8 flag_jump = 0;
 uint8 flag_jump_1 = 0;
 uint16 time_j = 0;
 uint16 time_temp = 0;
 int16 temp_d_f = 150;
 int16 temp_d_b = 30;
 float temp_y = 0;
 float redord_gyro[4] = {0};
 float redord_angle[4] = {0};
 void jump_control(void)
 {
     static uint8 flag_record = 0;

     // 记录起跳前角度，并计算目标角度
     if(flag_record == 0)
     {
         flag_record = 1;
         flag_jump_1 = 0;

         for(uint8 i = 0; i <= 4; i++)
         {
//             redord_gyro[i] = erect_Gyro_Pitch[i];
//             redord_gyro[i] = erect_Angle_Pitch[i];
             erect_Gyro_Pitch[i] = erect_Gyro_Pitch[i]/3;
             erect_Angle_Pitch[i] = erect_Angle_Pitch[i]/3;
         }


         IKParam.XLeft  = 1.75 * 2 - IKParam.XLeft;
         IKParam.XRight = 1.75 * 2 - IKParam.XRight;

         IKParam.XLeft  = Limit_Float(IKParam.XLeft,-1.0f,4.5f);
         IKParam.XRight = Limit_Float(IKParam.XRight,-1.0f,4.5f);
         IKParam.YLeft  = Limit_Float(IKParam.YLeft,3.0f,11.0f);
         IKParam.YRight = Limit_Float(IKParam.YRight,3.0f,11.0f);
     }

     if(time_j <= T1)
     {// 上升阶段
//         IKParam.XLeft  = 1.75;
//         IKParam.YLeft  = 14.8;
//         IKParam.XRight = 1.75;
//         IKParam.YRight = 14.8;
         servo_set_angle(RF, 91);servo_set_angle(RB, 89);
         servo_set_angle(LF, 91);servo_set_angle(LB, 89);
     }
     else if(T1 <= time_j && time_j < (T1+T2))
     {// 收腿阶段
//         IKParam.XLeft  = 1.75;
//         IKParam.YLeft  = 2.6;
//         IKParam.XRight = 1.75;
//         IKParam.YRight = 2.6;
         servo_set_angle(RF, 210);servo_set_angle(RB, -30);
         servo_set_angle(LF, 210);servo_set_angle(LB, -30);
     }
     else if((T1+T2) <= time_j && time_j < (T1+T2+T3))
     {// 放腿阶段
         flag_jump_1 = 1;
         IKParam.XLeft  = 2.5;
         IKParam.YLeft  = 8;
         IKParam.XRight = 2.5;
         IKParam.YRight = 8;
//         servo_set_angle(RF, 150);servo_set_angle(RB, 30);
//         servo_set_angle(LF, 150);servo_set_angle(LB, 30);
     }
     else if((T1+T2+T3) <= time_j)
     {// 缓冲阶段42/dpt*8 ms
         time_temp++;
         if(time_temp == tt)
         {
             time_temp = 0;
             temp_y += 0.05;
//             temp_d_f += 1;
//             temp_d_b -= 1;
         }
         IKParam.YLeft  -= temp_y;
         IKParam.YRight -= temp_y;
         if( IKParam.YLeft <= 3.1 )
         {
             for(uint8 i = 0; i <= 4; i++)
             {
//                 redord_gyro[i] = erect_Gyro_Pitch[i];
//                 redord_gyro[i] = erect_Angle_Pitch[i];
                 erect_Gyro_Pitch[i] = erect_Gyro_Pitch[i]*3;
                 erect_Angle_Pitch[i] = erect_Angle_Pitch[i]*3;
             }
             flag_jump_1 = 0;
             flag_record = 0;
             time_j = 0;
             time_temp = 0;

             temp_y = 0;

             flag_jump = 0;
             temp_flag_jump = 1;

         }

//         servo_set_angle(RF, temp_d_f);servo_set_angle(RB, temp_d_b);
//         servo_set_angle(LF, temp_d_f);servo_set_angle(LB, temp_d_b);
//
//         if( (temp_d_f - 201 >= 1) && (temp_d_b +21 <= 1) )
//         {
//             for(uint8 i = 0; i <= 4; i++)
//             {
////                 redord_gyro[i] = erect_Gyro_Pitch[i];
////                 redord_gyro[i] = erect_Angle_Pitch[i];
//                 erect_Gyro_Pitch[i] = erect_Gyro_Pitch[i]*3;
//                 erect_Angle_Pitch[i] = erect_Angle_Pitch[i]*3;
//             }
//             flag_jump_1 = 0;
//             flag_record = 0;
//             time_j = 0;
//             time_temp = 0;
//             temp_d_f = 150;
//             temp_d_b = 20;
//             flag_jump = 0;
//             temp_flag_jump = 1;
//
//         }
     }
//    if(time_j >= MAXTIME)
//        flag_jump = 0;
}

// 负数下降，正数上升
void servo_set_pwm(leg_enum leg, int16 er_pwm)
{
    servo_cfg[LF].mid_pwm = (uint16)pwmLF;
    servo_cfg[LB].mid_pwm = (uint16)pwmLB;
    servo_cfg[RF].mid_pwm = (uint16)pwmRF;
    servo_cfg[RB].mid_pwm = (uint16)pwmRB;

    ServoConfig* cfg = &servo_cfg[leg];

    if(leg == LF || leg == RB)
        er_pwm = -er_pwm;

    uint16 pwm = (uint16)(cfg->mid_pwm + er_pwm);

    pwm = (pwm < 1500) ? 1500 : (pwm > 7200) ? 7200 : pwm;

    pwm_set_duty(get_pwm_channel(leg), pwm);
}

float vv3 = 0;
void servo_balance(void)
{
    static int16 er_X_l = 0;
    static int16 er_X = 0;
    static float a = 0.05;

//    vv3 = 0.1 * (motor_value.receive_left_speed_data-motor_value.receive_right_speed_data) + 0.9f * vv3;

    er_X = (int16)PID_Increase( &PID_all.Pid_Inc_X, erect_Inc_X, (float)(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data), 500.0);
    er_X = (int16)(a * er_X + (1-a) * er_X_l);
    er_X = (int16)limit((float)er_X, 1800.0f);
    er_X_l = er_X;

    servo_set_pwm(RF,er_X);servo_set_pwm(RB,-er_X);
    servo_set_pwm(LF,er_X);servo_set_pwm(LB,-er_X);
}

float temp_ofst = 0;
float temp_height = 0;
float Single_Height = 6;
void Single_Control(void)
{
    static int flag = 0;
    if(flag_Single_HighState == 0 /*&& a11111 >= 1500*/)
    { // 低位切换高位
        if(flag == 0)
        {
            flag = 1;

//            Target_Yaw = imu660ra.eulerAngle.yaw;
//            Yao.Outp_turn = 0;
//            turn_mode = 3;

            temp_height = Yao.Target_height;
//            temp_ofst = imu660ra.offset_angle.pitch;
        }
//        imu660ra.offset_angle.pitch  =   -2.38;
        Yao.Target_height += 0.5;
        Yao.Target_height = Yao.Target_height >= Single_Height ? Single_Height:Yao.Target_height;
        if(func_abs(Y - Yao.Target_height) <= 0.1 && !TCount_falg_4ms && func_abs((float)(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data) - Yao.Target_Speed) <= 200)
        {
//            Yao.Target_height = 6.0;
            a11111=0;
            flag_Single_HighState = 1;
        }
    }
    else if(flag_Single_HighState == 2)
    { // 高位切换低位
//        imu660ra.offset_angle.pitch  =   temp_ofst;
        Yao.Target_height = temp_height;

//        Yao.Outp_turn = 0;
//        turn_mode = 2;

        if(func_abs(Y - Yao.Target_height) <= 0.1)
            flag_Single_HighState = 3;
        flag = 0;
    }

}

