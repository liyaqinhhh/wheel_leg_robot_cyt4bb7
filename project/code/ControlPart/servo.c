/*
 * servo.c
 * 舵机控制模块 - 负责四条腿的舵机角度控制、跳跃动作和平衡调节
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

// 四条腿舵机的PWM中位值（舵臂水平位置时的PWM值，单位：0.1us）
// LF=前左腿, LB=后左腿, RF=前右腿, RB=后右腿
// 旧值（TC264平台）：pwmLF=1510, pwmLB=1450, pwmRF=1420, pwmRB=1520
// uint32 pwmLF = 4350;
// uint32 pwmLB = 4550;
// uint32 pwmRF = 4610;
// uint32 pwmRB = 4560;
uint32 pwmLF = 5010;
uint32 pwmLB = 3890;
uint32 pwmRF = 3950;
uint32 pwmRB = 5220;
// 移植：PWM 引脚映射（TC264 ATOM 通道 → CYT4BB7 TCPWM 通道）
// LF: ATOM0_CH2_P21_4 → TCPWM_CH41_P12_5  （P21_4 在 CYT4BB7 不存在，改用 P12_5）
// LB: ATOM0_CH3_P21_5 → TCPWM_CH34_P21_5  （P21_5 直接对应）
// RF: ATOM0_CH1_P21_3 → TCPWM_CH39_P21_3  （P21_3 直接对应）
// RB: ATOM0_CH0_P21_2 → TCPWM_CH40_P21_2  （P21_2 直接对应）
/**
 * @brief 根据腿编号获取对应的PWM通道
 * @param leg 腿编号 (LF/LB/RF/RB)
 * @return 对应的PWM通道枚举值
 */
pwm_channel_enum get_pwm_channel(leg_enum leg) {
    switch(leg)
    {
        case LF: return TCPWM_CH12_P05_3;  // 前左腿 -> P01_1引脚 CH2
        case LB: return TCPWM_CH21_P08_2;  // 后左腿 -> P19_3引脚 CH1
        case RF: return TCPWM_CH20_P08_1;  // 前右腿 -> P01_0引脚 CH3
        case RB: return TCPWM_CH11_P05_2;  // 后右腿 -> P00_3引脚 CH4
        default: return TCPWM_CH12_P05_3;  // 默认返回前左腿通道
        // case LF:return TCPWM_CH12_P01_0;break;
        // case LB:return TCPWM_CH13_P00_3;break;
        // case RF:return TCPWM_CH11_P01_1;break;
        // case RB:return TCPWM_CH28_P19_3;break;
        // default:return TCPWM_CH12_P01_0;break; // 移植：补全 default 避免无返回值警告
    }
}

/**
 * @brief 舵机初始化 - 配置四路PWM输出
 * @note  PWM周期=300(对应50Hz/100Hz)，初始占空比为各腿中位值
 */
void servo_init(void)
{
    // 参数说明：pwm_init(通道, 周期, 初始占空比)
    // 周期300对应舵机控制频率，不能随意更改
    pwm_init( get_pwm_channel(LF) , 300 , pwmLF );  // 前左腿
    pwm_init( get_pwm_channel(LB) , 300 , pwmLB );  // 后左腿
    pwm_init( get_pwm_channel(RF) , 300 , pwmRF );  // 前右腿
    pwm_init( get_pwm_channel(RB) , 300 , pwmRB );  // 后右腿
    //printf("舵机初始化完成\n");
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
    /* LF */ { 5010, 30.0f,  90.0f, 270.0f }, // 前左腿：PWM增大角度减小
    /* LB */ { 3890,  30.0f, -90.0f,  90.0f }, // 后左腿
    /* RF */ { 3950,  30.0f,  90.0f, 270.0f }, // 前右腿：PWM增大角度增大
    /* RB */ { 5220, 30.0f, -90.0f,  90.0f }  // 后右腿
};

/**
 * @brief 设置指定腿舵机的目标角度
 * @param leg 腿编号 (LF/LB/RF/RB)
 * @param angle 目标角度（单位：度）
 *              前腿有效范围：90~270度（180度为中位）
 *              后腿有效范围：-90~90度（0度为中位）
 * @note  角度会自动进行周期映射和限幅处理
 */
void servo_set_angle(leg_enum leg, float angle) {
    // 同步更新中位PWM值（允许运行时动态调整）
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

// ========== 跳跃控制时间参数（单位：控制周期） ==========
uint8 T1 = 75;   // 上升阶段：蓄力准备起跳
uint8 T2 = 45;   // 收腿阶段：快速收缩腿部
uint8 T3 = 30;   // 放腿阶段：展开腿部落地
#define tt  1     // 缓冲阶段：缓慢恢复站立
//#define     T1      13          // 上升阶段持续时间
//#define     T2      20          // 收腿阶段持续时间
//#define     T3      20           // 放腿阶段持续时间
//#define     tt      1           // 缓冲阶段持续时间
//#define     MAXTIME     500     //  进程最大退出时间




// ========== 跳跃控制全局变量 ==========
extern uint8 temp_flag_jump;    // 外部跳跃完成标志（由其他模块读取）
uint8 flag_jump = 0;            // 跳跃使能标志：1=正在执行跳跃
uint8 flag_jump_1 = 0;          // 跳跃阶段标志：1=放腿阶段（用于IK控制切换）
uint16 time_j = 0;              // 跳跃过程计时器
uint16 time_temp = 0;           // 缓冲阶段子计时器
int16 temp_d_f = 150;           // 前腿临时PWM值（调试用）
int16 temp_d_b = 30;            // 后腿临时PWM值（调试用）
float temp_y = 0;               // 缓冲阶段Y轴递减量
float redord_gyro[4] = {0};     // 陀螺仪数据备份（未使用）
float redord_angle[4] = {0};    // 角度数据备份（未使用）

/**
 * @brief 跳跃动作控制 - 四阶段状态机
 * @note  阶段流程：上升(T1) -> 收腿(T2) -> 放腿(T3) -> 缓冲(tt)
 *        执行期间会临时修改IK参数和陀螺仪增益
 *        跳跃完成后自动复位所有状态
 */
void jump_control(void)
 {
     static uint8 flag_record = 0;  // 初始化标志：确保只执行一次

     // ===== 跳跃初始化（仅首次进入时执行） =====
     if(flag_record == 0)
     {
         flag_record = 1;
         flag_jump_1 = 0;

         // 降低陀螺仪/角度增益至1/3，防止跳跃瞬间姿态数据突变导致失控
         for(uint8 i = 0; i <= 4; i++)
         {
             erect_Gyro_Pitch[i] = erect_Gyro_Pitch[i] / 3;
             erect_Angle_Pitch[i] = erect_Angle_Pitch[i] / 3;
         }

         // 计算X轴目标位置：以1.75*2为中心进行镜像翻转
         IKParam.XLeft  = 1.75 * 2 - IKParam.XLeft;
         IKParam.XRight = 1.75 * 2 - IKParam.XRight;

         // 限制IK参数在安全范围内
         IKParam.XLeft  = Limit_Float(IKParam.XLeft, -1.0f, 4.5f);
         IKParam.XRight = Limit_Float(IKParam.XRight, -1.0f, 4.5f);
         IKParam.YLeft  = Limit_Float(IKParam.YLeft,  3.0f, 11.0f);
         IKParam.YRight = Limit_Float(IKParam.YRight, 3.0f, 11.0f);
     }

     // ===== 阶段1：上升（蓄力） =====
     if(time_j <= T1)
     {
         // 舵机打到接近中位，腿部伸展准备起跳
         servo_set_angle(RF, 91);  servo_set_angle(RB, 89);
         servo_set_angle(LF, 91);  servo_set_angle(LB, 89);
     }
     // ===== 阶段2：收腿 =====
     else if(T1 <= time_j && time_j < (T1 + T2))
     {
         // 快速收缩腿部，前腿向后摆，后腿向前摆
         servo_set_angle(RF, 210); servo_set_angle(RB, -30);
         servo_set_angle(LF, 210); servo_set_angle(LB, -30);
     }
     // ===== 阶段3：放腿（起跳） =====
     else if((T1 + T2) <= time_j && time_j < (T1 + T2 + T3))
     {
         flag_jump_1 = 1;  // 通知IK模块切换到跳跃放腿模式
         // 设置IK目标位置，腿部展开落地
         IKParam.XLeft  = 2.5;
         IKParam.YLeft  = 8;
         IKParam.XRight = 2.5;
         IKParam.YRight = 8;
     }
     // ===== 阶段4：缓冲（恢复站立） =====
     else if((T1 + T2 + T3) <= time_j)
     {
         time_temp++;
         // 每tt个周期递增Y轴下降量，实现缓慢下蹲效果
         if(time_temp == tt)
         {
             time_temp = 0;
             temp_y += 0.05;  // Y轴下降速度递增
         }
         // 持续降低腿部高度
         IKParam.YLeft  -= temp_y;
         IKParam.YRight -= temp_y;

         // 腿部降到目标高度后，恢复所有状态
         if(IKParam.YLeft <= 3.1)
         {
             // 恢复陀螺仪/角度增益至原始值（乘3还原）
             for(uint8 i = 0; i <= 4; i++)
             {
                 erect_Gyro_Pitch[i] = erect_Gyro_Pitch[i] * 3;
                 erect_Angle_Pitch[i] = erect_Angle_Pitch[i] * 3;
             }
             // 复位所有跳跃状态
             flag_jump_1 = 0;       // 清除放腿阶段标志
             flag_record = 0;       // 允许下次跳跃重新初始化
             time_j = 0;            // 清零计时器
             time_temp = 0;         // 清零子计时器
             temp_y = 0;            // 清零下降量
             flag_jump = 0;         // 跳跃动作完成
             temp_flag_jump = 1;    // 通知外部模块跳跃已结束
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

/**
 * @brief 直接设置舵机PWM偏移量（相对于中位）
 * @param leg 腿编号 (LF/LB/RF/RB)
 * @param er_pwm PWM偏移量：正数=腿上升，负数=腿下降
 * @note  LF和RB腿方向取反（机械安装差异）
 */
void servo_set_pwm(leg_enum leg, int16 er_pwm)
{
    // 同步更新中位PWM值
    servo_cfg[LF].mid_pwm = (uint16)pwmLF;
    servo_cfg[LB].mid_pwm = (uint16)pwmLB;
    servo_cfg[RF].mid_pwm = (uint16)pwmRF;
    servo_cfg[RB].mid_pwm = (uint16)pwmRB;

    ServoConfig* cfg = &servo_cfg[leg];

    // 前左腿和后右腿安装方向相反，需要取反
    if(leg == LF || leg == RB)
        er_pwm = -er_pwm;

    // 计算最终PWM值并限幅
    uint16 pwm = (uint16)(cfg->mid_pwm + er_pwm);
    pwm = (pwm < 1500) ? 1500 : (pwm > 7200) ? 7200 : pwm;

    pwm_set_duty(get_pwm_channel(leg), pwm);
}

float vv3 = 0;  // 差速滤波值（未使用）

/**
 * @brief 舵机平衡控制 - 通过左右轮速差调节舵机实现横向平衡
 * @note  使用增量PID计算平衡补偿量，低通滤波后输出到四路舵机
 *        对角腿同向运动：RF/LF同向，RB/LB反向
 */
void servo_balance(void)
{
    static int16 er_X_l = 0;   // 上一次的平衡补偿量（用于低通滤波）
    static int16 er_X = 0;     // 当前平衡补偿量
    static float a = 0.05;     // 低通滤波系数（0.05=5%新值+95%旧值）

    // 增量PID：输入=左轮速-右轮速，输出=平衡补偿PWM
    er_X = (int16)PID_Increase(&PID_all.Pid_Inc_X, erect_Inc_X,
               (float)(-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data), 150);
     //printf("速度: %f\n", (float)(-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data));   
    // 低通滤波，平滑输出抖动
    er_X = (int16)(a * er_X + (1 - a) * er_X_l);

    // 限幅保护，防止舵机过冲
    er_X = (int16)limit((float)er_X, 1800.0f);
    er_X_l = er_X;

    // 输出到舵机：对角腿反向运动产生扭腰效果
    servo_set_pwm(RF, -er_X);   servo_set_pwm(RB, er_X);
    servo_set_pwm(LF, -er_X);   servo_set_pwm(LB, er_X);
    //printf("平衡补偿PWM: %d\n", er_X);
}

// ========== 单腿高度切换控制变量 ==========
float temp_ofst = 0;       // 俯仰角偏移备份（未使用）
float temp_height = 0;     // 切换前的目标高度备份
float Single_Height = 6;   // 单腿站立目标高度（cm）

/**
 * @brief 单腿高度切换控制 - 实现高低位姿态切换
 * @note  状态机由外部变量 flag_Single_HighState 驱动：
 *        0 = 低位→高位切换中
 *        1 = 高位就绪
 *        2 = 高位→低位切换中
 *        3 = 低位就绪
 *
 *        切换条件：高度误差<0.1cm 且 速度误差<200
 */
void Single_Control(void)
{
    static int flag = 0;  // 初始化标志

    // ===== 状态0：低位→高位切换 =====
    if(flag_Single_HighState == 0)
    {
        // 首次进入：备份当前高度
        if(flag == 0)
        {
            flag = 1;
            temp_height = Yao.Target_height;  // 备份原始目标高度
        }

        // 每周期递增0.5cm，逐步抬升到目标高度
        Yao.Target_height += 0.5;
        Yao.Target_height = Yao.Target_height >= Single_Height ? Single_Height : Yao.Target_height;

        // 切换完成条件：高度稳定 + 速度稳定
        if(func_abs(Y - Yao.Target_height) <= 0.1 &&
           !TCount_falg_4ms &&
           func_abs((float)(motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) - Yao.Target_Speed) <= 200)
        {
            a11111 = 0;
            flag_Single_HighState = 1;  // 标记高位就绪
        }
    }
    // ===== 状态2：高位→低位切换 =====
    else if(flag_Single_HighState == 2)
    {
        Yao.Target_height = temp_height;  // 恢复原始目标高度

        // 高度恢复到位后标记低位就绪
        if(func_abs(Y - Yao.Target_height) <= 0.1)
            flag_Single_HighState = 3;

        flag = 0;  // 允许下次切换重新初始化
    }
}

