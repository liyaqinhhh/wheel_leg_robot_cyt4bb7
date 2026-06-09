/*
 * servo.h
 * 舵机控制模块头文件 - 四条腿舵机角度/PWM控制、跳跃动作和平衡调节
 *
 *  Created on: 2025年2月18日
 *      Author: LateRain
 */

#ifndef CODE_CONTROLPART_SERVO_H_
#define CODE_CONTROLPART_SERVO_H_

// @brief  腿编号枚举（用于标识四条腿的舵机通道）
typedef enum
{
    LF = 0,  // 前左腿 (Left Front)
    LB = 1,  // 后左腿 (Left Back)
    RF = 2,  // 前右腿 (Right Front)
    RB = 3,  // 后右腿 (Right Back)
}leg_enum;

extern uint32 pwm;             // 通用PWM占空比变量
extern uint32 pwmLF;           // 前左腿舵机中位PWM值（舵臂水平位置）
extern uint32 pwmLB;           // 后左腿舵机中位PWM值
extern uint32 pwmRF;           // 前右腿舵机中位PWM值
extern uint32 pwmRB;           // 后右腿舵机中位PWM值
extern uint8 flag_jump;        // 跳跃使能标志：1=正在执行跳跃动作
extern uint8 flag_jump_1;      // 跳跃阶段标志：1=放腿阶段（用于IK控制切换）
extern uint16 time_j;          // 跳跃过程计时器（控制周期数）
extern uint8 flag_jump_stop;   // 跳跃停止标志
extern uint8 T1;               // 上升阶段持续时间（蓄力准备起跳，单位：控制周期）
extern uint8 T2;               // 收腿阶段持续时间（快速收缩腿部，单位：控制周期）
extern uint8 T3;               // 放腿阶段持续时间（展开腿部落地，单位：控制周期）
extern float Single_Height;    // 单腿站立目标高度（单位：cm）

// @brief  舵机初始化 - 配置四路PWM输出通道和初始占空比
void servo_init(void);

// @brief  设置指定腿舵机的目标角度（经角度映射和限幅后输出PWM）
// @param  leg    腿编号 (LF/LB/RF/RB)
// @param  angle  目标角度（度），前腿范围90~270，后腿范围-90~90
void servo_set_angle( leg_enum leg, float angle );

// @brief  跳跃动作控制 - 四阶段状态机（上升->收腿->放腿->缓冲）
void jump_control(void);

// @brief  直接设置舵机PWM偏移量（相对于中位值的偏移）
// @param  leg      腿编号 (LF/LB/RF/RB)
// @param  er_pwm   PWM偏移量，正数=腿上升，负数=腿下降（LF和RB方向取反）
void servo_set_pwm(leg_enum leg, int16 er_pwm);

// @brief  舵机平衡控制 - 通过左右轮速差增量PID计算补偿量，低通滤波后输出到四路舵机
void servo_balance(void);

// @brief  单腿高度切换控制 - 实现高低位姿态切换的状态机
void Single_Control(void);

#endif /* CODE_CONTROLPART_SERVO_H_ */
