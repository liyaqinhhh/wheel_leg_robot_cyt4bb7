/**
 * @file    Interrupt.h
 * @brief   平衡车定时中断服务模块头文件
 *
 * 包含多级定时中断服务函数（1ms/2ms/4ms/8ms/16ms/40ms），
 * 每级中断负责不同频率的控制任务（IMU更新、PID计算、舵机控制等）。
 *
 * 中断优先级层级：
 *   1ms  ： IMU偏航角积分、累加圈计数、到达站计时
 *   2ms  ： 跳跃控制、按键扫描、AI数据、IMU数据读取、角速度环PID（内环）
 *   4ms  ： 卡尔曼滤波、角度环PID（外环）、转向PID模式切换
 *   8ms  ： 腿部高度控制、舵机平衡环
 *   16ms ： 速度环PID（外环计算）、惯导坐标更新
 *   40ms ： 斑马线检测超时、偏航角漂移补偿
 */

#ifndef CODE_CONTROLPART_INTERRUPT_H_
#define CODE_CONTROLPART_INTERRUPT_H_

/* 到达站计时器（1ms中断中的累加器） */
extern uint16 a11111;

/**
 * 控制核心平衡结构体
 *
 * 存储各级中断计算出的PID输出值和传感器数据，
 * 由运动控制模块（Adapt_Terrain）、舵机平衡等读取使用。
 */
typedef struct
{
    float Outp_Gyro_Pitch;      /* 正向角速度环PID输出 */
    float Outp_Angle_Pitch;     /* 正向角度环PID输出 */
    float Outp_Speed_Pitch;     /* 正向速度环PID输出 */

    float Outp_Gyro_Roll;       /* 横滚角速度环PID输出 */
    float Outp_Angle_Roll;      /* 横滚角度环PID输出 */

    float Outp_Gyro_Yaw;        /* 偏航角速度环PID输出 */
    float Outp_Angle_Yaw;       /* 偏航角度环PID输出 */

    float Outp_turn;            /* 转向PID输出（叠加到左右轮差速） */

    int16 Encoder_Left;         /* 左轮编码器速度 */
    int16 Encoder_Right;        /* 右轮编码器速度 */

    int Target_Speed;           /* 目标速度（设定值） */
    float Target_height;        /* 目标高度（设定值，单位cm） */
} Center_struct;

extern Center_struct Yao;       /* 全局控制核心平衡结构体实例 */

/* ---- AI控制 ---- */
extern uint16_t flag_ai_open;   /* AI控制模式开关 */

/* ---- 系统标志 ---- */
extern int16_t flag_main;       /* 主状态标志 */
extern uint16_t flag_text;      /* 文本显示标志 */

/* ---- 转向控制 ---- */
extern uint8 steer_control_mode; /* 转向控制模式：0=角度控制(平衡环), 1=PWM直驱(角速度环) */
extern uint8 turn_mode;          /* 转向模式：
                                  *   0=关闭, 1=逐飞PID转向,
                                  *   2=通用转向, 3=偏航角度锁环直行,
                                  *   4=视觉转向, 5=GPS转向,
                                  *   6=原地旋转(Spin3), 7=惯导转向 */
extern uint8 fuzzy_mode;         /* 模糊控制模式：0=关闭(使用固定KP), 1=开启(动态KP随偏差范围) */
extern float kp_roll;            /* 横滚KP系数 */
extern float k11;                /* 车轮差速系数1 */
extern float k22;                /* 车轮差速系数2 */
extern float Target_Yaw;         /* 目标偏航角（度） */

/* ---- 电池与传感器 ---- */
extern float Battery_voltage;    /* 电池电压（V） */
extern uint16 dis_tof_mm;        /* TOF测距传感器距离（mm） */
extern volatile float angle_Z;   /* 连续累加偏航角（度），可超越±180度，用于多圈旋转 */

/* ---- 惯导 ---- */
extern uint8 ins_open;           /* 惯导系统开关：0=关闭, 1=开启 */


/* ---- 菜单 ---- */
extern uint8 menu_open;          /* 菜单模式：0=关闭, 1=打开菜单和Flash, 2=只读取不打开菜单 */
extern uint8 ins_getdata;          /* 惯导数据获取标志：0=未获取, 1=已获取 */
/* ---- 标志位 ---- */
extern uint8 flag_stop;          /* 停止标志：1=停止, 0=运行 */
extern uint8 flag_yawan;         /* 偏航补偿使能：0=关闭, 1=开启 */
extern uint8 telemetry_enable;   /* 遥测使能：0=关闭, 1=开启（40ms中断通过无线串口发送调试数据） */
extern uint8 ins_telemetry_enable; /* 惯导遥测使能：0=关闭, 1=开启 */

/* ---- 计时器变量 ---- */
extern uint16 TCount_4ms;        /* 4ms中断计数器（受TCount_falg_4ms控制） */
extern uint16 TCount_40ms;       /* 40ms中断计数器（用于斑马线超时检测） */
extern uint8 TCount_falg_4ms;    /* 4ms计数使能标志 */

/* ---- GPS转向 (供4ms中断统一调用) ---- */
#include "gps_nav.h"

/* ---- 视觉转向 ---- */
extern float desired_yaw;        /* 期望偏航角（度） */

/* ---- 姿态角度校准 ---- */
extern volatile uint8_t calibrate_state;   /* 校准状态：0=未开始, 1=采集中, 2=完成 */
extern volatile float calibrate_offset;    /* 校准角度偏移量（度） */
extern volatile uint16_t calibrate_count;  /* 校准采样计数器 */
extern volatile float calibrate_sum;       /* 校准角度累加和 */
#define CALIBRATE_SAMPLES 500              /* 姿态角度校准采样数量500次（@2ms周期=1秒） */

/* ---- 中断服务函数 ---- */

// @brief  1ms定时中断服务 - IMU偏航角积分、累加圈计数、到达站计时
void Interrupt_1ms(void);

// @brief  2ms定时中断服务 - 跳跃控制、按键扫描、AI数据、IMU数据读取、角速度环PID（内环）
void Interrupt_2ms(void);

// @brief  4ms定时中断服务 - 卡尔曼滤波、角度环PID（外环）、转向PID模式切换
void Interrupt_4ms(void);

// @brief  8ms定时中断服务 - 腿部高度控制、舵机平衡环
void Interrupt_8ms(void);

// @brief  16ms定时中断服务 - 速度环PID（外环计算）、惯导坐标更新
void Interrupt_16ms(void);

// @brief  40ms定时中断服务 - 斑马线检测超时、偏航角漂移补偿
void Interrupt_40ms(void);

#endif /* CODE_CONTROLPART_INTERRUPT_H_ */
