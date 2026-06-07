
#include "zf_common_headfile.h"

#ifndef _FLY_MOTOR_PID_h
#define _FLY_MOTOR_PID_h

#define KP 0
#define KI 1
#define KD 2

#define L1  6
#define L2  9
#define L3  9
#define L4  6
#define L5  3.5

/*****************---------结构体--------*****************/
typedef struct
{
    float iError;                 // 误差
    float LastError;              // 上次误差
    float PrevError;              // 上上次误差
    float LastData;               // 上次数据
    float iErrorHistory[5];       // 历史误差
    float SumError;               // 累计误差
} PID_INFO;
typedef struct
{
    PID_INFO Pid_Gyro_Pitch;
    PID_INFO Pid_Angle_Pitch;
    PID_INFO Pid_Speed_Pitch;

    PID_INFO Pid_Gyro_Roll;
    PID_INFO Pid_Angle_Roll;

    PID_INFO Pid_Gyro_Yaw;
    PID_INFO Pid_Angle_Yaw;

    PID_INFO Pid_Inc_X;
    PID_INFO Pid_Inc_Y;
    PID_INFO Pid_Inc_Roll;

    PID_INFO Pid_SZR;
    PID_INFO Pid_GOGOGO;

    PID_INFO Pid_turn;
    PID_INFO Pid_turn1;// [待确认]
    PID_INFO Pid_turn2;// [待确认]
} PID_ERECT;
typedef struct{
    float alphaLeft, betaLeft;
    float alphaRight, betaRight;
    float XLeft,YLeft;
    float XRight, YRight;
}IKparam;

extern PID_ERECT PID_all;       // PID
extern IKparam IKParam;         // 运动学结构体
/*****************---------结构体--------*****************/


/*****************---------PID参数---------*****************/
// 俯仰角平衡
extern float erect_Gyro_Pitch[4];
extern float erect_Angle_Pitch[4];
extern float erect_Speed_Pitch[4];
// 翻滚角平衡
extern float erect_Gyro_Roll[4];
extern float erect_Angle_Roll[4];
// 转向参数
extern float erect_turn[4];

extern float erect_Gyro_Yaw[4];
extern float erect_Angle_Yaw[4];
extern float erect_Angle_Yaw_2[4];
extern float erect_Angle_Yaw_3[4];
extern float erect_Angle_Yaw_4[4];
// 运动学参数
extern float erect_Inc_X[4];
extern float erect_Inc_Y[3];
extern float erect_Inc_Roll[3];
extern float erect_yawan[3];
//extern float steer_fusion_turn[3];

extern float erect_SZR[4];

extern IKparam IKParam;
extern float alpha1,alpha2,beta1,beta2;
extern float servoLeftFront,servoLeftRear,servoRightFront,servoRightRear;
extern float X,Y;
extern float stab_roll;
extern volatile float vv1;
extern volatile float dd2;
extern volatile uint8 spin3_active;
extern volatile int8 spin3_dir;
extern volatile float spin3_start_angle;
extern volatile float spin3_target_angle;
extern volatile uint16 spin3_hold_cnt;
extern volatile float spin3_angle_ok_deg;
extern volatile float spin3_gyro_ok_dps;
extern volatile uint16 spin3_hold_ticks;
/*****************---------PID参数---------*****************/


void Adapt_Terrain(void);
void Spin3_Start(int8 dir);

float Cascade_speed_Pitch( PID_INFO *pid_info , float * PID_Parm , float NowPoint , float SetPoint );

float Cascade_angle_Pitch( PID_INFO *pid_info , float * PID_Parm , float NowPoint , float SetPoint );
float Cascade_angle_Roll( PID_INFO *pid_info , float * PID_Parm , float NowPoint , float SetPoint );
float Cascade_angle_Yaw( PID_INFO *pid_info , float * PID_Parm , float NowPoint , float SetPoint );
float Cascade_angle_Yaw_2( PID_INFO *pid_info , float * PID_Parm , float NowPoint , float SetPoint );

float Cascade_gyro_Pitch( PID_INFO *pid_info , float * PID_Parm , float NowPoint , float SetPoint );
float Cascade_gyro_Roll( PID_INFO *pid_info , float * PID_Parm , float NowPoint , float SetPoint );
float Cascade_gyro_Yaw( PID_INFO *pid_info , float * PID_Parm , float NowPoint , float SetPoint );

float PID_Increase_X( PID_INFO *pid_info , float *PID_Parm , float NowPoint , float SetPoint );
float PID_Increase_Y( PID_INFO *pid_info , float *PID_Parm , float NowPoint , float SetPoint );
float PID_Increase_Roll( PID_INFO *pid_info , float *PID_Parm , float NowPoint , float SetPoint );
float PID_SZR_is_GOD( PID_INFO *pid_info , float *PID_Parm , float NowPoint , float SetPoint );
float PID_GOGOGO( PID_INFO *pid_info , float *PID_Parm , float NowPoint , float SetPoint );
float PID_Increase( PID_INFO *pid_info , float *PID_Parm , float NowPoint , float SetPoint );

float PID_turn_seekfree( PID_INFO *pid_info , float * PID_Parm , float gyro , float err );
// Steering fusion control (implemented in PID.c)
float steer_wrap_deg180(float x);

void pid_para_init( PID_INFO *pid_info );
/*****************---------函数---------*****************/

#endif


