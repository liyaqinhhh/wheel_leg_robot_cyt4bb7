#include "zf_common_headfile.h"
#include "PID.h"
#include "servo.h"
#include "Interrupt.h"
#include "imu660.h"
#include "image.h"
#include "small_driver_uart_control.h"
#include "Math_Advanced.h"

PID_ERECT PID_all;
IKparam IKParam;

volatile uint8 spin3_active = 0;
volatile int8 spin3_dir = 1;
volatile float spin3_start_angle = 0.0f;
volatile float spin3_target_angle = 0.0f;
volatile uint16 spin3_hold_cnt = 0;
volatile float spin3_angle_ok_deg = 3.0f;
volatile float spin3_gyro_ok_dps = 8.0f;
volatile uint16 spin3_hold_ticks = 40;

/******************************************
// �?�?�?
        offset_angle.pitch =   2 ;    调尝车往剝坝，调大往坎坝;
        offset_angle.roll  =  -1.8 ;
*******************************************/
// 俯仰角平�?
/*
�?5�?3.6-0-17
     18-0-12.9
     0.01-0.000009-0.004-0


*/
// float erect_Gyro_Pitch[4]   = {  23.6  ,  0  ,  17  ,  0  };   //
// float erect_Angle_Pitch[4]  = {  18  ,  0  ,  12.9  ,  0  };   //
// float erect_Speed_Pitch[4]  = {  0.01  ,  0.000009  ,  0.004  ,  0  };   //
float erect_Gyro_Pitch[4] = {1.0, 0, 0, 0};  //  17.522  ,  0  ,  11  ,  0
float erect_Angle_Pitch[4] = {400, 0, 0, 0}; //  17.49  ,  0  ,  9  ,  0
float erect_Speed_Pitch[4] = {0, 0, 0, 0};   //
// 翻滚角平�?
float erect_Gyro_Roll[4] = {0, 0, 0, 0};  //
float erect_Angle_Roll[4] = {0, 0, 0, 0}; //
// 转坑�?                 kp,kp2,kd,gkd,
float erect_turn[4] = {0, 0, 0, 0};

float erect_Gyro_Yaw[4] = {1, 0, 0, 0};      //
float erect_Angle_Yaw[4] = {650, 0, 0, 0};   //
float erect_Angle_Yaw_2[4] = {3, 0, 0, 0}; //300,0,0,0
float erect_Angle_Yaw_3[4] = {0, 0, 0, 0}; //�Ӿ�
float erect_Angle_Yaw_4[4] = {0, 0, 0, 0}; //����
// 违动学坂�?           Kp_x, Kp_y, Kp_roll
float erect_Inc_X[4] = {1.1, 0, 0, 0}; // 0.0045, 0, 0.0049
float erect_Inc_Y[3] = {1, 0, 0};
float erect_Inc_Roll[3] = {0.15, 0, 0};
float erect_yawan[3] = {0, 0, 0};
float erect_Km[3] = {1.1, 0.1, 0.05};

float erect_SZR[4] = {0, 0, 0, 0};
/********High********/
// float erect_Gyro_Pitch_High[4]   = {  1.5  ,  0  ,  2  ,  0  };   //  17.522  ,  0  ,  11  ,  0
// float erect_Angle_Pitch_High[4]  = {  450  ,  0  ,  500  ,  0  };   //  17.49  ,  0  ,  9  ,  0
// float erect_Speed_Pitch[4]  = {  0  , 0  ,  0  ,  0  };   //
//  翻滚角平�?
// float erect_Gyro_Roll[4]    = {  0  ,  0  ,  0  ,  0  };   //
// float erect_Angle_Roll[4]   = {  0  ,  0  ,  0  ,  0  };   //
//  转坑�?                 kp,kp2,kd,gkd
// float erect_turn[4]         = {  40.16  ,  10.78  ,  8.8  ,  -5.1  };
// float erect_Gyro_Yaw[4]     = {  10  ,  0  ,  20  ,  0  };   //
// float erect_Angle_Yaw[4]    = {  0  ,  0  ,  0  ,  0  };   //
//  违动学坂�?           Kp_x, Kp_y, Kp_roll
// float erect_Inc_X_High[3] = { 0.024, 0, 0.028 };  // 0.0045, 0, 0.0049
// float erect_Inc_Y[3] = { 0.01, 0, 0 };
// float erect_Inc_Roll[3] = { -0.2, 0, 0 };
// float erect_Inc_Roll[3] = { 0, 0, 0 };
// float erect_Km[3] = {  1.1,  0.1,  0.05  };

// float erect_SZR[4] = { 0.3, 0, 0.6, 0 };
/*********High**********/




void Spin3_Start(int8 dir)
{
    //if (spin3_active) return;

    spin3_dir = (dir >= 0) ? 1 : -1;
    spin3_start_angle = angle_Z;
    spin3_target_angle = spin3_start_angle + spin3_dir * 1080.0f;
    spin3_hold_cnt = 0;
    spin3_active = 1;

    // Enter fixed-angle spin mode.
    flag_stop =0;
    //Deviation_Value = 0;
    //desired_yaw = 0;
    Yao.Target_Speed = 0;
    turn_mode = 6;
}

float steer_wrap_deg180(float x)
{
    // Keep angle in [-180, 180] to avoid yaw discontinuity.
    while (x > 180.0f)
        x -= 360.0f;
    while (x < -180.0f)
        x += 360.0f;
    return x;
}

// ������Ч�Ժ�ѡ���ģʽȷ������ת��Դ��






/*************************************轮足违动学逆解*************************************/
//----------------------------------------------------------------
//  @brief      闭环五连杆模型违动学逆解�?
//  @param      L1   面朝左侧时，左臂上连杆长�?
//  @param      L2   面朝左侧时，左臂下连杆长�?
//  @param      L3   面朝左侧时，坳臂上连杆长�?
//  @param      L4   面朝左侧时，坳臂下连杆长�?
//  @param      L5   面朝左侧时，上方横连杆长�?
//  @return     void

//  @note       模型为尝车面朝左侧，故靠剝方为左腿，坎方为坳腿，注愝左坳alpha和beta对应的舵机角�?
//----------------------------------------------------------------
float servoLeftFront, servoLeftRear, servoRightFront, servoRightRear;
void inverseKinematics()
{
    float alpha1, alpha2, beta1, beta2;

    float aLeft = 2 * IKParam.XLeft * L1;
    float bLeft = 2 * IKParam.YLeft * L1;
    float cLeft = IKParam.XLeft * IKParam.XLeft + IKParam.YLeft * IKParam.YLeft + L1 * L1 - L2 * L2;
    float dLeft = 2 * L4 * (IKParam.XLeft - L5);
    float eLeft = 2 * L4 * IKParam.YLeft;
    float fLeft = ((IKParam.XLeft - L5) * (IKParam.XLeft - L5) + L4 * L4 + IKParam.YLeft * IKParam.YLeft - L3 * L3);

    alpha1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
    alpha2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
    beta1 = 2 * atan((eLeft + sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));
    beta2 = 2 * atan((eLeft - sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));

    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * M_PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * M_PI);

    if (alpha1 >= M_PI / 2)
        IKParam.alphaLeft = alpha1;
    else
        IKParam.alphaLeft = alpha2;
    if (beta1 >= 0 && beta1 <= M_PI / 2)
        IKParam.betaLeft = beta1;
    else
        IKParam.betaLeft = beta2;

    float aRight = 2 * IKParam.XRight * L1;
    float bRight = 2 * IKParam.YRight * L1;
    float cRight = IKParam.XRight * IKParam.XRight + IKParam.YRight * IKParam.YRight + L1 * L1 - L2 * L2;
    float dRight = 2 * L4 * (IKParam.XRight - L5);
    float eRight = 2 * L4 * IKParam.YRight;
    float fRight = ((IKParam.XRight - L5) * (IKParam.XRight - L5) + L4 * L4 + IKParam.YRight * IKParam.YRight - L3 * L3);

    IKParam.alphaRight = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    IKParam.betaRight = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

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

    // 弧度转角�?
    servoLeftFront = RAD_TO_ANGLE(IKParam.alphaLeft);
    servoLeftRear = RAD_TO_ANGLE(IKParam.betaLeft);
    servoRightFront = RAD_TO_ANGLE(IKParam.alphaRight);
    servoRightRear = RAD_TO_ANGLE(IKParam.betaRight);
    //  if(servoLeftFront > 360)
    //    servoLeftFront -= 360;
    //  if(servoLeftRear > 360)
    //      servoLeftRear -= 360;
    //  if(servoRightFront > 360)
    //      servoRightFront -= 360;
    //  if(servoRightRear > 360)
    //      servoRightRear -= 360;

    //  pwm_set_duty(ATOM0_CH2_P21_4, 5000);
    servo_set_angle(LF, servoLeftFront);
    servo_set_angle(RF, servoRightFront);
    servo_set_angle(LB, servoLeftRear);
    servo_set_angle(RB, servoRightRear);
    //  servo_set_angle(RF, 225);servo_set_angle(RB, -45);
    //  servo_set_angle(LF, 225);servo_set_angle(LB, -45);

    //
    //  setServoAngle(servoLeftFront,servoLeftRear,servoRightFront,servoRightRear);
}

float X = 0, X_l = 0, Y = 5; // 确定Y值覝先确定舵机中�?
float temp_kx_x[4] = {0};
volatile float vv1 = 0;
volatile float dd2 = 0;
volatile float roll_filter = 0;
float stab_roll = 0;
float kpp = -0.3;
float SZR = 0;
float yawan = 0;
float dif_now = 0;
float dif_last = 0;
// 最低姿思高�?.5，中�?
void Adapt_Terrain(void)
{
    //    static float a = 0.1;
    //    get_eulerAngle();

    // 平滑计算顶点坝标，坎期坯替杢为PID
    vv1 = 0.05f * (float)(motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) + 0.95 * vv1;
    dif_now = func_abs((float)(motor_value.receive_left_speed_data + motor_value.receive_right_speed_data));
    dd2 = func_abs(dif_now - dif_last);
    dif_last = dif_now;
    if (turn_mode == 0)
        X = PID_Increase_X(&PID_all.Pid_Inc_X, temp_kx_x, vv1, 0);
    else
        X = PID_Increase_X(&PID_all.Pid_Inc_X, temp_kx_x, vv1 + k11 * dd2, (float)Yao.Target_Speed);
    Y = Y - PID_Increase_Y(&PID_all.Pid_Inc_Y, erect_Inc_Y, Y, Yao.Target_height);
    //    X = a * X + (1-a) * X_l;
    SZR = PID_SZR_is_GOD(&PID_all.Pid_SZR, erect_SZR, (Deviation_Value * 10 + 0.2f), 0);

    X_l = X;

    roll_filter = 0.5f * roll_filter + 0.5f * imu660ra.eulerAngle.roll;
    //???
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
    if (flag_stop == 0)
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
                IKParam.YLeft = Y - stab_roll;
                IKParam.YRight = Y;
            }
            else
            {
                IKParam.YLeft = Y;
                IKParam.YRight = Y + stab_roll;
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

    if (Single_state == 1 && flag_Single_HighState == 0) // 凝�?
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

    // 违动学逆解算并输出舵机
    if (flag_jump_1 == 1 || flag_jump == 0)
        inverseKinematics();
}

/*************************************轮足违动学逆解*************************************/
/********************************逝飞转坑PID********************************/
float PID_turn_seekfree(PID_INFO *pid_info, float *PID_Parm, float gyro, float err)
{
    float V_out;
    float a = 0.5;

    pid_info->iError = err;

    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError; // 使得波形更加平滑，滤除高频干扰，防止速度窝坘�?

    V_out = PID_Parm[0] * pid_info->iError +
            PID_Parm[1] * pid_info->iError * func_abs(pid_info->iError) +
            PID_Parm[2] * (pid_info->iError - pid_info->LastError) +
            PID_Parm[3] * gyro;
    pid_info->LastError = pid_info->iError;

    return V_out;
}
/********************************逝飞转坑PID********************************/
/*************************************串级（外-速度 次外-角度 �?角速度�?************************************/
/*****************---------�?速度---------*****************/
float Cascade_speed_Pitch(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.5;

    // 1.计算速度坝差
    pid_info->iError = (NowPoint - SetPoint);

    // 2.对速度坝差进行低通滤�?
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError; // 使得波形更加平滑，滤除高频干扰，防止速度窝坘�?
    pid_info->SumError += pid_info->iError;

    // 3.积分陝幅
    //    if( PID_Parm[3] )
    //    {
    pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    //    }

    // 4.计算输出
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

/*****************---------次外-角度---------*****************/
float Cascade_angle_Pitch(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    static float temp_param[4] = {0};
    (void)temp_param; // 备用坂数（对应代砝已注释），消除未使用警�?
    float V_out;
    float a = 0.9;

    //    if(flag_Single)
    //    {
    //        temp_param[0] = erect_Angle_Pitch_High[0];
    //        temp_param[1] = erect_Angle_Pitch_High[1];
    //        temp_param[2] = erect_Angle_Pitch_High[2];
    //        temp_param[3] = erect_Angle_Pitch_High[3];
    //    }
    //    else
    //    {
    //        temp_param[0] = PID_Parm[0];
    //        temp_param[1] = PID_Parm[1];
    //        temp_param[2] = PID_Parm[2];
    //        temp_param[3] = PID_Parm[3];
    //    }

    // 1.计算速度坝差
    pid_info->iError = (NowPoint - SetPoint);

    // 2.低通滤�?
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError; // 使得波形更加平滑，滤除高频干扰，防止窝坘�?
    pid_info->SumError += pid_info->iError;

    // 3.积分陝幅
    if (PID_Parm[3])
    {
        pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    }

    // 4.计算输出
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

float Cascade_angle_Roll(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.5;

    // 1.计算速度坝差
    pid_info->iError = (NowPoint - SetPoint);

    // 2.低通滤�?
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError; // 使得波形更加平滑，滤除高频干扰，防止窝坘�?
    pid_info->SumError += pid_info->iError;

    // 3.积分陝幅
    if (PID_Parm[3])
    {
        pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    }

    // 4.计算输出
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

float Cascade_angle_Yaw(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.7;

    // 1.计算速度坝差
    pid_info->iError = (NowPoint - SetPoint);

    // 2.低通滤�?
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError; // 使得波形更加平滑，滤除高频干扰，防止窝坘�?
    pid_info->SumError += pid_info->iError;

    // 3.积分陝幅
    //    if( PID_Parm[3] )
    //    {
    //        pid_info->SumError = limit( pid_info->SumError , PID_Parm[3] );
    //    }

    // 4.计算输出
    V_out = PID_Parm[KP] * pid_info->iError +
            //            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

float Cascade_angle_Yaw_2(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.7;

    // 1.计算速度坝差
    pid_info->iError = (NowPoint - SetPoint);

    // 2.低通滤�?
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError; // 使得波形更加平滑，滤除高频干扰，防止窝坘�?
    pid_info->SumError += pid_info->iError;

    // 3.积分陝幅
    //    if( PID_Parm[3] )
    //    {
    //        pid_info->SumError = limit( pid_info->SumError , PID_Parm[3] );
    //    }

    // 4.计算输出
    V_out = PID_Parm[KP] * pid_info->iError +
            //            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

float Cascade_angle_Yaw_3(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.7;

    // 1.计算速度坝差
    pid_info->iError = (NowPoint - SetPoint);

    // 2.低通滤�?
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError; // 使得波形更加平滑，滤除高频干扰，防止窝坘�?
    pid_info->SumError += pid_info->iError;

    // 3.积分陝幅
    //    if( PID_Parm[3] )
    //    {
    //        pid_info->SumError = limit( pid_info->SumError , PID_Parm[3] );
    //    }

    // 4.计算输出
    V_out = PID_Parm[KP] * pid_info->iError +
            //            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

float Cascade_angle_Yaw_4(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.7;

    // 1.计算速度坝差
    pid_info->iError = (NowPoint - SetPoint);

    // 2.低通滤�?
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError; // 使得波形更加平滑，滤除高频干扰，防止窝坘�?
    pid_info->SumError += pid_info->iError;

    // 3.积分陝幅
    //    if( PID_Parm[3] )
    //    {
    //        pid_info->SumError = limit( pid_info->SumError , PID_Parm[3] );
    //    }

    // 4.计算输出
    V_out = PID_Parm[KP] * pid_info->iError +
            //            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}
/*****************---------�?角速度---------*****************/
float Cascade_gyro_Pitch(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    static float temp_param[4] = {0};
    float V_out;
    float a = 0.9;

    //    if(flag_Single)
    //    {
    //        temp_param[0] = erect_Gyro_Pitch_High[0];
    //        temp_param[1] = erect_Gyro_Pitch_High[1];
    //        temp_param[2] = erect_Gyro_Pitch_High[2];
    //        temp_param[3] = erect_Gyro_Pitch_High[3];
    //    }
    //    else
    //    {
    //        temp_param[0] = PID_Parm[0];
    //        temp_param[1] = PID_Parm[1];
    //        temp_param[2] = PID_Parm[2];
    //        temp_param[3] = PID_Parm[3];
    //    }

    // 1.计算速度坝差
    pid_info->iError = (NowPoint - SetPoint);

    // 2.低通滤�?
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError; // 使得波形更加平滑，滤除高频干扰，防止窝坘�?
    pid_info->SumError += pid_info->iError;

    // 3.积分陝幅
    if (temp_param[3])
    {
        pid_info->SumError = limit(pid_info->SumError, temp_param[3]);
    }

    // 4.计算输出
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

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

float Cascade_gyro_Yaw(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.9;

    // 1.计算速度坝差
    pid_info->iError = (NowPoint - SetPoint);

    // 2.低通滤�?
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError; // 使得波形更加平滑，滤除高频干扰，防止窝坘�?
    pid_info->SumError += pid_info->iError;

    // 3.积分陝幅
    if (PID_Parm[3])
    {
        pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    }

    // 4.计算输出
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}

/********************************************************************串级PID********************************************************************/
// 增針弝PID
float PID_Increase_X(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    static float temp_param[3] = {0};
    (void)temp_param; // 备用坂数（对应代砝已注释），消除未使用警�?
    float V_out;
    float a = 0.9;

    //    if(flag_Single)
    //    {
    //        temp_param[0] = erect_Inc_X_High[0];
    //        temp_param[1] = erect_Inc_X_High[1];
    //        temp_param[2] = erect_Inc_X_High[2];
    //    }
    //    else
    //    {
    //        temp_param[0] = PID_Parm[0];
    //        temp_param[1] = PID_Parm[1];
    //        temp_param[2] = PID_Parm[2];
    //    }

    // 1.计算速度坝差
    pid_info->iError = (NowPoint - SetPoint);

    // 2.低通滤�?
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError; // 使得波形更加平滑，滤除高频干扰，防止窝坘�?
    pid_info->SumError += pid_info->iError;

    // 3.积分陝幅
    if (PID_Parm[3])
    {
        pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    }

    // 4.计算输出
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}
float PID_Increase_Y(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.5;

    // 1.计算速度坝差
    pid_info->iError = (NowPoint - SetPoint);

    // 2.低通滤�?
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError; // 使得波形更加平滑，滤除高频干扰，防止窝坘�?
    pid_info->SumError += pid_info->iError;

    //    // 3.积分陝幅
    //    if( PID_Parm[3] )
    //    {
    //        pid_info->SumError = limit( pid_info->SumError , PID_Parm[3] );
    //    }

    // 4.计算输出
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}
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
// 增針弝PID
float PID_SZR_is_GOD(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.9;

    // 1.计算速度坝差
    pid_info->iError = (NowPoint - SetPoint);

    // 2.低通滤�?
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError; // 使得波形更加平滑，滤除高频干扰，防止窝坘�?
    pid_info->SumError += pid_info->iError;

    // 3.积分陝幅
    if (PID_Parm[3])
    {
        pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    }

    // 4.计算输出
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}
float PID_GOGOGO(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.9;

    // 1.计算速度坝差
    pid_info->iError = (NowPoint - SetPoint);

    // 2.低通滤�?
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError; // 使得波形更加平滑，滤除高频干扰，防止窝坘�?
    pid_info->SumError += pid_info->iError;

    // 3.积分陝幅
    if (PID_Parm[3])
    {
        pid_info->SumError = limit(pid_info->SumError, PID_Parm[3]);
    }

    // 4.计算输出
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}
float PID_Increase(PID_INFO *pid_info, float *PID_Parm, float NowPoint, float SetPoint)
{
    float V_out;
    float a = 0.9;

    // 1.计算速度坝差
    pid_info->iError = (NowPoint - SetPoint);

    // 2.低通滤�?
    pid_info->iError = (1 - a) * pid_info->iError + a * pid_info->LastError; // 使得波形更加平滑，滤除高频干扰，防止窝坘�?
    pid_info->SumError += pid_info->iError;

    // 3.积分陝幅
    //    if( PID_Parm[3] )
    //    {
    //        pid_info->SumError = limit( pid_info->SumError , PID_Parm[3] );
    //    }

    // 4.计算输出
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * (pid_info->iError - pid_info->LastError);
    pid_info->LastError = pid_info->iError;

    return V_out;
}
/*****************---------PID坂数初始�?--------*****************/
void pid_para_init(PID_INFO *pid_info)
{
    pid_info->iError = 0;
    pid_info->SumError = 0;
    pid_info->PrevError = 0;
    pid_info->LastError = 0;
    pid_info->LastData = 0;


}
