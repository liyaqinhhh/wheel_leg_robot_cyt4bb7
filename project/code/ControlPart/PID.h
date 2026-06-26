/**
 * @file    PID.h
 * @brief   轮腿机器人PID控制模块头文件
 *
 * 包含串级PID（外环角度→内环角速度）、增量式PID、转向PID、
 * 五连杆逆运动学解算等控制算法所需的类型定义与接口声明。
 *
 * PID参数数组布局（4元素数组）：
 *   [0]=KP(比例), [1]=KP2(二次比例/非线性增益), [2]=KD(微分), [3]=积分限幅
 * 增量式PID参数布局（3元素数组）：
 *   [0]=KP, [1]=KI, [2]=KD
 * 转向PID参数布局（4元素数组）：
 *   [0]=KP, [1]=KP2(非线性项), [2]=KD, [3]=陀螺仪前馈增益
 */

#include "zf_common_headfile.h"

#ifndef _FLY_MOTOR_PID_h
#define _FLY_MOTOR_PID_h

/* 标准PID参数数组索引 */
#define KP 0    /* 比例系数索引 */
#define KI 1    /* 积分系数索引 */
#define KD 2    /* 微分系数索引 */

/* 五连杆机构尺寸参数（单位：cm）
 *
 * 坐标系约定：车体面朝左侧时，
 *   左侧（后方）为左腿，右侧（前方）为右腿。
 *   L1/L2 为左腿参数，L3/L4 为右腿参数，L5 为横连杆。
 */
#define L1  6      /* 左腿上连杆长度 */
#define L2  9      /* 左腿下连杆长度 */
#define L3  9      /* 右腿上连杆长度 */
#define L4  6      /* 右腿下连杆长度 */
#define L5  3.5    /* 上方横连杆长度 */

/*****************---------结构体--------*****************/

/** PID控制器状态信息 */
typedef struct
{
    float iError;                 /* 当前误差 */
    float LastError;              /* 上一次误差（用于微分计算） */
    float PrevError;              /* 上上次误差（用于二阶微分） */
    float LastData;               /* 上一次输入数据 */
    float iErrorHistory[5];       /* 历史误差记录（预留数组） */
    float SumError;               /* 积分累计误差 */
} PID_INFO;

/** 全局PID控制器集合
 *
 * 串级控制架构（俯仰/翻滚）：
 *   外环=角度环（Pid_Angle_*） → 内环=角速度环（Pid_Gyro_*）
 *   最外层速度环（Pid_Speed_Pitch）叠加在角度环之上。
 *
 * 偏航（Yaw）方向使用独立的角速度+角度双环。
 * 位移/高度/翻滚稳定使用增量式PID（Pid_Inc_*）。
 */
typedef struct
{
    PID_INFO Pid_Gyro_Pitch;      /* 俯仰角速度环（内环） */
    PID_INFO Pid_Angle_Pitch;     /* 俯仰角度环（外环） */
    PID_INFO Pid_Speed_Pitch;     /* 俯仰速度环（最外层） */

    PID_INFO Pid_Gyro_Roll;       /* 翻滚角速度环（内环） */
    PID_INFO Pid_Angle_Roll;      /* 翻滚角度环（外环） */

    PID_INFO Pid_Gyro_Yaw;        /* 偏航角速度环 */
    PID_INFO Pid_Angle_Yaw;       /* 偏航角度环 */

    PID_INFO Pid_Inc_X;           /* X方向增量式PID（前后位移控制） */
    PID_INFO Pid_Inc_Y;           /* Y方向增量式PID（高度控制） */
    PID_INFO Pid_Inc_Roll;        /* 翻滚增量式PID（侧向稳定） */

    PID_INFO Pid_SZR;             /* 数字舵机/转向调节PID */
    PID_INFO Pid_GOGOGO;          /* 偏航辅助PID（yawan调节） */

    PID_INFO Pid_turn;            /* 转向PID */
    PID_INFO Pid_turn1;           /* 转向PID备用通道1 */
    PID_INFO Pid_turn2;           /* 转向PID备用通道2 */
} PID_ERECT;

/** 逆运动学参数
 *
 * 坐标系约定：车体面朝左侧时，后方为左腿，前方为右腿。
 * alpha为上方舵机弧度，beta为下方舵机弧度。
 * X/Y为连杆末端在笛卡尔坐标系中的目标坐标（单位：cm）。
 */
typedef struct
{
    float alphaLeft, betaLeft;    /* 左腿上/下舵机弧度 */
    float alphaRight, betaRight;  /* 右腿上/下舵机弧度 */
    float XLeft, YLeft;           /* 左腿末端目标坐标（前后, 上下） */
    float XRight, YRight;         /* 右腿末端目标坐标（前后, 上下） */
} IKparam;

extern PID_ERECT PID_all;       /* 全局PID控制器实例 */
extern IKparam IKParam;         /* 全局逆运动学参数实例 */
/*****************---------结构体--------*****************/


/*****************---------PID参数声明---------*****************/
/* 俯仰角平衡参数 */
extern float erect_Gyro_Pitch[4];     /* 俯仰角速度环: {KP, KP2, KD, 积分限幅} */
extern float erect_Angle_Pitch[4];    /* 俯仰角度环:   {KP, KP2, KD, 积分限幅} */
extern float erect_Speed_Pitch[4];    /* 俯仰速度环:   {KP, KP2, KD, 积分限幅} */
/* 翻滚角平衡参数 */
extern float erect_Gyro_Roll[4];      /* 翻滚角速度环: {KP, KP2, KD, 积分限幅} */
extern float erect_Angle_Roll[4];     /* 翻滚角度环:   {KP, KP2, KD, 积分限幅} */
/* 转向参数 */
extern float erect_turn[4];           /* 转向PID: {KP, KP2, KD, 陀螺仪前馈增益} */

/* 偏航控制参数 */
extern float erect_Gyro_Yaw[4];       /* 偏航角速度环: {KP, KP2, KD, 积分限幅} */
extern float erect_Angle_Yaw[4];      /* 偏航角度环1:  {KP, KP2, KD, 积分限幅} */
extern float erect_Angle_Yaw_2[4];    /* 偏航角度环2:  {KP, KP2, KD, 积分限幅}（备用） */
extern float erect_Angle_Yaw_3[4];    /* 偏航角度环3:  {KP, KP2, KD, 积分限幅}（视觉） */
extern float erect_Angle_Yaw_4[4];    /* 偏航角度环4:  {KP, KP2, KD, 积分限幅}（惯导） */
/* 运动学参数 */
extern float erect_Inc_X[4];          /* X位移增量式PID: {KP, KI, KD, 积分限幅} */
extern float erect_Inc_Y[3];          /* Y位移增量式PID: {KP, KI, KD} */
extern float erect_Inc_Roll[3];       /* 翻滚增量式PID: {KP, KI, KD} */
extern float erect_yawan[3];          /* 偏航辅助PID:   {KP, KI, KD} */

extern float erect_SZR[4];            /* SZR转向调节: {KP, KP2, KD, 积分限幅} */

/* 逆运动学输出变量 */
extern IKparam IKParam;
extern float alpha1, alpha2, beta1, beta2;
extern float servoLeftFront, servoLeftRear, servoRightFront, servoRightRear;
extern float X, Y;                    /* 连杆末端笛卡尔坐标 */
extern float stab_roll;               /* 翻滚稳定偏移量 */
extern volatile float vv1;            /* 左右轮速差滤波值 */
extern volatile float dd2;            /* 轮速和变化率 */

/* Spin3原地旋转控制变量 */
extern volatile uint8 spin3_active;         /* 旋转激活标志 */
extern volatile int8 spin3_dir;             /* 旋转方向（+1/-1） */
extern volatile float spin3_start_angle;    /* 旋转起始偏航角 */
extern volatile float spin3_target_angle;   /* 旋转目标偏航角 */
extern volatile uint16 spin3_hold_cnt;      /* 旋转到位保持计数 */
extern volatile float spin3_angle_ok_deg;   /* 角度到位容差（度） */
extern volatile float spin3_gyro_ok_dps;    /* 陀螺静止容差（度/秒） */
extern volatile uint16 spin3_hold_ticks;    /* 到位保持所需周期数 */
/*****************---------PID参数声明---------*****************/


/* ---- 逆运动学 ---- */
void inverseKinematics(void);

/* ---- 地形自适应（主控制循环） ---- */
void Adapt_Terrain(void);

/* ---- 原地旋转 ---- */
void Spin3_Start(int8 dir);
float steer_wrap_deg180(float x);     /* 角度归一化到[-180, 180] */
float unwrap_to_continuous(float target_deg, float current_continuous_deg); /* 目标角展开到连续坐标系 */

/* ---- 串级PID：速度环（最外层） ---- */
float Cascade_speed_Pitch(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint);

/* ---- 串级PID：角度环（外环） ---- */
float Cascade_angle_Pitch(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint);
float Cascade_angle_Roll (PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint);
float Cascade_angle_Yaw  (PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint);
float Cascade_angle_Yaw_2(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint);
float Cascade_angle_Yaw_3(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint);
float Cascade_angle_Yaw_4(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint);

/* ---- 串级PID：角速度环（内环） ---- */
float Cascade_gyro_Pitch(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint);
float Cascade_gyro_Roll (PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint);
float Cascade_gyro_Yaw  (PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint);

/* ---- 增量式PID ---- */
float PID_Increase_X   (PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint);
float PID_Increase_Y   (PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint);
float PID_Increase_Roll(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint);
float PID_Increase     (PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint);

/* ---- 辅助PID ---- */
float PID_SZR_is_GOD(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint);
float PID_GOGOGO     (PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint);

/* ---- 转向PID（含陀螺仪前馈） ---- */
float PID_turn_seekfree(PID_INFO *pid_info, float *PID_Parm, float gyro, float err);

/* ---- PID状态初始化 ---- */
void pid_para_init(PID_INFO *pid_info);
/*****************---------函数声明---------*****************/

#endif
