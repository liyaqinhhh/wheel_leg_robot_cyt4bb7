/*
 * Interrupt.c
 *
 *  Created on: 2024�???2�???�???
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

Center_struct Yao;

// 开机角度校准变量
volatile uint8_t calibrate_state = 0;   // 0:未开始 1:采集中 2:完成
volatile float calibrate_offset = 0;    // 校准得到的角度偏移
volatile uint16_t calibrate_count = 0;  // 采集计数
volatile float calibrate_sum = 0;       // 角度累加和

uint16 adc0;
float Battery_voltage;

uint8 steer_control_mode = 0; // 0:瑙掑害鎺у埗锛岀伅鍝ュ紑�???
                              // 1:PWM鎺у埗锛岄€愰鏂规锛屾棤閫熷害�???
uint8 turn_mode = 2;          // 0:鍏抽�??
                              // 1:閫愰鍙孭D杞�??
                              // 2:涓茬骇杞�??
                              // 3:yaw瑙掗棴鐜蛋鐩寸�??
uint8 fuzzy_mode = 0;         // 0:鍏抽棴妯＄硦锛屼娇鐢↘P
                              // 1:寮€鍚ā绯婏紝浣跨敤KP浣滀负鏈€灏忥紝KI浣滀负鏈€澶у幓妯＄硦�???
uint8 menu_open = 1;          // 0:鍏抽棴鑿滃崟鍜宖lash浠ヤ繚璇佹祴璇曚笉浼氳浼lash
                              // 1:鎵撳紑鑿滃崟
                              // 2:鍙墦寮€璇诲彇锛屼笉鎵撳紑鑿滃�??
uint8 flag_yawan = 1;         // 0
                              // 1:鎵撳紑yawan
uint8 flag_stop = 1;
uint16 a11111 = 0;
uint16 a2222 = 0;
bool ff = 0;
uint16 dis_tof_mm = 0;

uint16 time_flag = 0;//??????



volatile float angle_Z = 0;
// Steering fusion implementation moved to PID.c (keep ISR call unchanged).
void Interrupt_1ms(void)
{

    // static uint8 Count = 0;
    // static uint8 Cnt = 0;

    if (menu_mode)
    {
        dis_tof_mm = tof_dl1b_get_mm();
        //        ips200_show_uint( 0, 30*8, dis_tof_mm, 5 );
    }
    //    EKF_UpData();
    //    imu660ra.eulerAngle.pitch = euler_angle.roll - imu660ra.offset_angle.pitch;
    //    imu660ra.eulerAngle.roll  = euler_angle.pitch  - imu660ra.offset_angle.roll;
    imu660ra.eulerAngle.yaw += imu660ra.data_Raw.gyro_z * 0.001;

    //    Buzzer_Control();
    if (imu660ra.eulerAngle.yaw > 180)
        imu660ra.eulerAngle.yaw -= 360;
    if (imu660ra.eulerAngle.yaw < -180)
        imu660ra.eulerAngle.yaw += 360;

    if((imu660ra.eulerAngle.yaw-imu660ra.eulerAngle.last_yaw) < -350) imu660ra.eulerAngle.Dirchange++;
    else if ((imu660ra.eulerAngle.yaw-imu660ra.eulerAngle.last_yaw) > 350) imu660ra.eulerAngle.Dirchange--;

    angle_Z=360*imu660ra.eulerAngle.Dirchange+imu660ra.eulerAngle.yaw;//180*eulerAngle.Dirchange+eulerAngle.yaw
    imu660ra.eulerAngle.last_yaw=imu660ra.eulerAngle.yaw;

    //    if(imu660ra.eulerAngle.pitch > 0)
    //        imu660ra.eulerAngle.pitch -= 180;
    //    else if(imu660ra.eulerAngle.pitch < 0)
    //        imu660ra.eulerAngle.pitch += 180;
    // a11111++;
    if (flag_Single )
    {
        a11111++;
    }
    else
        a11111 = 0;



    //????90?

   /* if (time_flag == 0)
    {
        Count++;
        if (Count >= 5000)
        {
            Count = 0;
            if(Cnt == 0)
            Target_Yaw += 90;
            Cnt++;
            if (Cnt != 0)
            {   
                if (Cnt%2 == 0)
                    Target_Yaw += 90;
                if (Cnt%2 == 1)
                    Target_Yaw -= 90;
            }
           time_flag = 1;         
        }
    }
    if(imu660ra.eulerAngle.yaw == Target_Yaw && time_flag == 1)
     time_flag = 0;*/
            
    //    if(key_detect(KEY_2, KEY_SHORT_PRESS))
    //
    //        ff = ~ff;
    //    if(ff)
    //        a2222++;
    //    else
    //    {
    //        a2222 = 0;
    //        Yao.Target_Speed = 0;
    //        Deviation_Value = 0;
    //    }
    //
    //    if(a2222 >= 3000)
    //    {
    //        flag_jump=1;
    //        ff=0;
    //        Deviation_Value = -0.7;
    //        Yao.Target_Speed = 300;
    //    }
    //    else if(3000 < a2222 && a2222 <= 6000)
    //    {
    //        Deviation_Value = 0.7;
    //    }
    ////    else if(6000 < a2222 && a2222 <= 9000)
    ////    {
    ////        Deviation_Value = -0.3;
    ////    }
    //    else if (a2222 > 6000)
    //        a2222 = 0;
}
float integer = 0;
uint32 num_t = 0;
float integer1 = 0;
uint32 num_t1 = 0;
volatile float y1 = 0;
float ddddd = 0;
void Interrupt_2ms(void)
{
    // 璺宠穬鎺у埗閮ㄥ垎
    if (flag_jump)
    {
        time_j++;
        jump_control();
    }
    else
        time_j = 0;
    key_scanner();
    /*if (menu_open == 1)
        menu();
    else
        menu_mode = 1;*/

    if (menu_mode && key_detect(KEY_1, KEY_SHORT_PRESS))
        flag_track = 1;

    //    servo_set_angle(LF, 180);
    //    servo_set_angle(RF, 180);
    //    servo_set_angle(LB, 0);
    //    servo_set_angle(RB, 0);
    //    servo_set_angle(LF, 265.7f);
    //    servo_set_angle(RF, 265.7f);
    //    servo_set_angle(LB, 42.2f);
    //    servo_set_angle(RB, 42.2f);
    //    float A_RF,A_LF,A_RB,A_LB;
    if (menu_mode && key_detect(KEY_2, KEY_SHORT_PRESS))
    {
        ff = ~ff;
    }
    if (ff)
        a2222++;
    else
    {
        a2222 = 0;
    }
    if (a2222 >= 3000)
    {
        flag_jump = ~flag_jump;
        ff = 0;
    }

    //    pwm_set_duty(ATOM0_CH0_P21_2, 1500);
    //    pwm_set_duty(ATOM0_CH1_P21_3, 1500);
    //    pwm_set_duty(ATOM0_CH2_P21_4, 1500);
    //    pwm_set_duty(ATOM0_CH3_P21_5, 1500);
     //date_handle();
    //    get_eulerAngle();
    // 闄€铻轰华鏁版嵁
    date_handle();

    // 开机角度校准逻辑
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
    if (menu_mode)
    {
        //        if(num_t >= 3000)
        //date_handle();
        if (IMU_JF_Flag)
        {
            Z_Yaw += imu660ra.data_Raw.gyro_z / 500;
        }
        else
        {
            Z_Yaw = 0;
        }

        //        else
        //            num_t++;
    }
    //    imu660ra_get_gyro();

    //    if(num_t <= 10000)
    //    {
    //        num_t++;
    //        integer += (float)imu660ra_gyro_y-2.5f;
    //    }
    //    ips200_show_float( 0, 0*8, integer, 5,5 );
    //    ips200_show_float( 0, 1*8, (integer/num_t), 5,5 );
    //    if(num_t1 <= 10000)
    //    {
    //        num_t1++;
    //        integer1 += (float)imu660ra_gyro_x-1.62f;
    //    }
    //    ips200_show_float( 0, 3*8, integer1, 5,5 );
    //    ips200_show_float( 0, 4*8, (integer1/num_t1), 5,5 );
    // 涓茬骇瑙掗€熷害�???
    y1 = 0.1f * ((float)-imu660ra_gyro_x) + 0.9 * y1;
    Yao.Outp_Gyro_Pitch = -limit(Cascade_gyro_Pitch(&PID_all.Pid_Gyro_Pitch, erect_Gyro_Pitch, y1, -Yao.Outp_Angle_Pitch), 8000.0f);
    Yao.Outp_Gyro_Yaw = limit(Cascade_gyro_Yaw(&PID_all.Pid_Gyro_Yaw, erect_Gyro_Yaw, imu660rb_gyro_z, -Yao.Outp_turn), 8000.0f);

} // 2ms缁撴�??

volatile float aa1 = 0;
volatile float dd = 0;
float temp_erect_turn[4];
float k11 = 0;
float k22 = 0;
float kp_roll = 0.9;
float Target_Yaw = 0;
float V_trans = 0;
uint8 TCount_falg_4ms = 0;
uint16 TCount_4ms = 0;
float dt = 0.004f;     
float desired_yaw = 0.0f;
float raw_vision_yaw = 0.0f;
static float steer_vision_cmd_lpf_alpha = 0.35f;
static float steer_vision_cmd_lpf = 0.0f;
volatile float steer_vision_target_yaw_deg = 0.0f;       //????
volatile float steer_vision_cone_avoid_delta_deg = 0.0f; //????


volatile float steer_gps_target_bearing_deg = 0.0f; //????
volatile float steer_gps_distance_to_wp_m = 0.0f;   //??
volatile float steer_gps_to_imu_yaw_offset_deg = 0.0f;
void Interrupt_4ms(void)
{
    //    V_trans = (float)(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data);
    imu963ra_kalman_filter_update(&imu);
    //    imu963ra_menc15a_kalman_filter_Update(&vel_kf, 0, imu.ay_linear);
    imu.roll -= imu660ra.offset_angle.roll;
    imu.pitch -= imu660ra.offset_angle.pitch;
    imu660ra.eulerAngle.roll = imu.pitch;
    imu660ra.eulerAngle.pitch = imu.roll;

    //if(imu660ra.eulerAngle.pitch<0.5&&imu660ra.eulerAngle.pitch>-0.5)
    //    imu660ra.eulerAngle.pitch = 0;

    if (TCount_falg_4ms)
        TCount_4ms++;
    else
        TCount_4ms = 0;

    //角速度环
    /*float pitch_corrected = imu660ra.eulerAngle.pitch;
    if (calibrate_state == 2)  // 校准完成后才补偿
    {
        pitch_corrected -= calibrate_offset;
    }*/
    aa1 = 0.1f * imu660ra.eulerAngle.pitch + 0.9f * aa1;
    if (steer_control_mode == 0)
    {
        Yao.Outp_Angle_Pitch = Cascade_angle_Pitch(&PID_all.Pid_Angle_Pitch, erect_Angle_Pitch, aa1, Yao.Outp_Speed_Pitch);
        Yao.Outp_Angle_Pitch = -limit(Yao.Outp_Angle_Pitch, 12000.0f);
    }
    else
    {
        Yao.Outp_Angle_Pitch = Cascade_angle_Pitch(&PID_all.Pid_Angle_Pitch, erect_Angle_Pitch, aa1, Yao.Outp_Speed_Pitch);
        Yao.Outp_Angle_Pitch = -limit(Yao.Outp_Angle_Pitch, 12000.0f);
    }

    dd = 0.1f * Deviation_Value + 0.9f * dd;

    //get_realtime_coordinate((float)(motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) / 2.0f,0.004f,imu660ra.eulerAngle.yaw);
    // 杞悜鐜?
    if (turn_mode == 1)
    {
        Yao.Outp_turn = PID_turn_seekfree(&PID_all.Pid_turn, erect_turn, imu660ra.data_Raw.gyro_z, Deviation_Value * 10 + 0.2f);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 2)
    {
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

        //        if(flag_Single_HighState == 1)
        //            temp_erect_turn[0] = 100;
        Yao.Outp_turn = Cascade_angle_Yaw(&PID_all.Pid_turn, temp_erect_turn, Deviation_Value * 10 /*+ kp_roll * stab_roll*/, 0);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 3)
    {
        Yao.Outp_turn = Cascade_angle_Yaw_2(&PID_all.Pid_Angle_Yaw, erect_Angle_Yaw_2, imu660ra.eulerAngle.yaw, Target_Yaw);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }


    else if (turn_mode == 4)
    {
        
        raw_vision_yaw = steer_vision_target_yaw_deg + steer_vision_cone_avoid_delta_deg;
        raw_vision_yaw = steer_wrap_deg180(raw_vision_yaw);

            // Low-pass target yaw to reduce jitter during cone bypass.
        steer_vision_cmd_lpf = (1.0f - steer_vision_cmd_lpf_alpha) * steer_vision_cmd_lpf + steer_vision_cmd_lpf_alpha * raw_vision_yaw;
        desired_yaw = steer_vision_cmd_lpf;
        Yao.Outp_turn = Cascade_angle_Yaw_3(&PID_all.Pid_turn1, erect_Angle_Yaw_3, imu660ra.eulerAngle.yaw, desired_yaw);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
        // Target_Yaw = steer_target_yaw_deg;  // mirror to existing debug variable
    }
    else if (turn_mode == 5)
    {
        desired_yaw = steer_gps_target_bearing_deg + steer_gps_to_imu_yaw_offset_deg;
        Yao.Outp_turn = Cascade_angle_Yaw_4(&PID_all.Pid_turn2, erect_Angle_Yaw_3, imu660ra.eulerAngle.yaw, desired_yaw);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else if (turn_mode == 6)
    {
        // Use continuous yaw angle for multi-turn spin control.
        Yao.Outp_turn = Cascade_angle_Yaw_2(&PID_all.Pid_Angle_Yaw, erect_Angle_Yaw_2, angle_Z, spin3_target_angle);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);

        // Finish only when both angle and yaw rate are within limits.
        if (func_abs(spin3_target_angle - angle_Z) < spin3_angle_ok_deg &&
            func_abs((float)imu660rb_gyro_z) < spin3_gyro_ok_dps)
        {
            spin3_hold_cnt++;
        }
        else
        {
            spin3_hold_cnt = 0;
        }

        if (spin3_hold_cnt >= spin3_hold_ticks)
        {
            spin3_active = 0;
            spin3_hold_cnt = 0;
            Yao.Outp_turn = 0;
            turn_mode = 2;
            Target_Yaw = imu660ra.eulerAngle.yaw;
        }
    }
    else
    {
        Yao.Outp_turn = 0;
    }

} // 4ms缁撴�??

int16 recordL = 0;
int16 recordR = 0;
float vv2 = 0;
void Interrupt_8ms(void)
{ /*
     // 鐢垫睜鐢靛帇
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
    //    vv2 = 0.05f * (float)(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data) + 0.95 * vv2;
    //    Yao.Outp_Speed_Pitch = -Cascade_speed_Pitch(&PID_all.Pid_Speed_Pitch, erect_Speed_Pitch, vv2, (float)Yao.Target_Speed);
    //    Yao.Outp_Speed_Pitch = limit( Yao.Outp_Speed_Pitch, 100.0f );
    //    // 璺宠穬鎺у埗閮ㄥ垎
    //    if(flag_jump)
    //    {
    //        time_j++;
    //        jump_control();
    //    }
    //    else
    //        time_j = 0;
    // 鍗曡竟妗ユ帶鍒堕儴鍒?



/************************************************************** */

    if (flag_Single)
    {
        Single_Control();
    }
    //if (menu_mode == 1 /*&& flag_jump == 0 && Element_State != Jump_State */ && steer_control_mode == 0)
        //Adapt_Terrain();
    //else 
    if (menu_mode == 1 && flag_jump == 0 && steer_control_mode == 1)
        servo_balance();

/******************************************************************** */






    /*if (menu_open == 1)
       // menu();
    else

        menu_mode = 1;*/

    if (menu_mode && key_detect(KEY_1, KEY_SHORT_PRESS))
        flag_track = 1;

    //    if(Element_State == Jump_State)
    //    {
    //        if(flag_jump == 0)
    //        servo_set_angle(RF, 210);servo_set_angle(RB, 0);
    //        servo_set_angle(LF, 210);servo_set_angle(LB, 0);
    //    }

    //    small_driver_set_duty(0, 0);
    //    processImage();

} // 8ms缁撴�??
volatile float aa11 = 0;
void Interrupt_16ms(void)
{

    // 鐢垫睜鐢靛帇
    //    adc0 = adc_mean_filter_convert( ADC0_CH6_A6 , 5 );
    //    Battery_voltage = adc0 / 114.8936;

    //    if(menu_mode)
    //    {
    //        if(func_abs(Yao.Encoder_Left-motor_value.receive_right_speed_data) <= 100)
    //            Yao.Encoder_Left  = 0.5f * motor_value.receive_right_speed_data + 0.5f * Yao.Encoder_Left;
    //        if(func_abs(Yao.Encoder_Right-motor_value.receive_left_speed_data) <= 100)
    //            Yao.Encoder_Right = -(0.5f * motor_value.receive_left_speed_data + 0.5f * Yao.Encoder_Right);

    //        Yao.Encoder_Left  = motor_value.receive_right_speed_data;
    //        if(motor_value.receive_left_speed_data < 0)
    //            Yao.Encoder_Right = -motor_value.receive_left_speed_data+22;
    //        else if(motor_value.receive_left_speed_data > 0)
    //            Yao.Encoder_Right = -motor_value.receive_left_speed_data-23;
    //        else
    //            Yao.Encoder_Right = -motor_value.receive_left_speed_data;

    //        Yao.Encoder_Left  = motor_value.receive_right_speed_data;
    //        Yao.Encoder_Right = -motor_value.receive_left_speed_data;
    //    }
    //    else
    //    {
    //        Yao.Encoder_Left = 0;
    //        Yao.Encoder_Right = 0;
    //    }
    aa11 = 0.1f * (((float)(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data))/2.0f) + 0.9f * aa11;
    Yao.Outp_Speed_Pitch = -Cascade_speed_Pitch(&PID_all.Pid_Speed_Pitch, erect_Speed_Pitch, aa11, 0);
    Yao.Outp_Speed_Pitch = limit(Yao.Outp_Speed_Pitch, 100.0f);

} // 16ms缁撴�??


extern uint8 Zebra_Count_Flag;
uint16 TCount_40ms = 0;
void Interrupt_40ms(void)
{
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

    //    yaokong_data_deal();

    //    if(IMU_JF_Flag)
    imu660ra.eulerAngle.yaw += 0.000;

} // 40ms缁撴�??
