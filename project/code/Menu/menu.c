/*
 * menu.c
 *
 *  Created on: 2024年7月5日
 *      Author: LateRain
 */
#include "zf_common_headfile.h"
#include "menu.h"
#include "PID.h"
#include "Interrupt.h"
#include "imu660.h"
#include "image.h"
#include "ips.h"
#include "servo.h"
#include "kalman.h"
#include "Math_Advanced.h"
#include "Init.h"
#include "small_driver_uart_control.h"
#include "Ins.h"

uint8 menu_mode = 0;
int8 page = 0;
int8 cursor = 0;
int8 unit = 2;

// 摄像头参数（菜单调节用）
uint16 Image_Gain    = 32;    // 摄像头增益，默认32（对应 MT9V03X_GAIN_DEF）
uint16 Image_EpTime  = 4000;  // 摄像头曝光时间，默认4000
uint8  Change_Control = 0;    // 写入后触发摄像头重初始化

//-------------------------------------------------------------------------------------------------------------------
//  @brief      显示实时调试信息: 电机速度、陀螺仪输出、偏航角、俯仰角、航点数
//  @param      void
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void IPS200_Show1(void)
{
    ips200_show_int( 0, 0, motor_value.receive_left_speed_data, 5 );
    ips200_show_int( 60, 0, motor_value.receive_right_speed_data, 5 );
    ips200_show_float( 0 , 16 ,  Yao.Outp_Gyro_Yaw , 3 , 3 );
    ips200_show_float( 60 , 16 ,  Yao.Outp_Gyro_Pitch  , 3 , 3 );
    //ips200_show_float( 120 , 16 ,  Yao.Outp_Gyro_Pitch  , 3 , 3 );

    ips200_show_string(0 , 48 , "Yaw:");
    ips200_show_float( 60 , 48 , imu660ra.eulerAngle.yaw  , 3 , 3 );
    
    ips200_show_string(0 , 64 , "Pitch:");
    ips200_show_float( 60 , 64 , imu660ra.eulerAngle.pitch , 3 , 3 );
    
    ips200_show_string(0 , 80 , "n:");
    ips200_show_float( 60 , 80 , n , 3 , 3 );
    ips200_show_float( 0 , 128 , (float)flag_save , 3 , 3 );
    ips200_show_float( 60 , 128 , (float)target, 3 , 3 );

    // ips200_show_string(0 , 80 , "n:");
    // ips200_show_float( 60 , 80 , imu660ra_gyro_z , 3 , 3 );


//    ips200_show_float( 0 , 96 , IKParam.XLeft  , 3 , 3 );
//    ips200_show_float( 60 , 96 , IKParam.XRight  , 3 , 3 );
//
//    ips200_show_float( 60 , 80 , stab_roll , 3 , 3 );
//    ips200_show_float( 0 , 112 , IKParam.YLeft  , 3 , 3 );
//    ips200_show_float( 60 , 112 , IKParam.YRight  , 3 , 3 );


}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      将所有 PID/控制参数写入 Flash 或从 Flash 读出
//  @param      way     WRITE=写入Flash, READ=从Flash读出
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void store_or_read_DATA(int way)
{
    if( way == WRITE ) // 写入内存
    {


        // 第一页：俯仰角、翻滚角、gyro/angle前后平衡参数、速度、高度、显示标志
        flash_union_buffer[0].float_type = imu660ra.offset_angle.pitch;
        flash_union_buffer[1].float_type = imu660ra.offset_angle.roll;
        flash_union_buffer[2].float_type = erect_Gyro_Pitch[0];
        flash_union_buffer[3].float_type = erect_Gyro_Pitch[1];
        flash_union_buffer[4].float_type = erect_Gyro_Pitch[2];
        flash_union_buffer[5].float_type = erect_Gyro_Pitch[3];
        flash_union_buffer[6].float_type = erect_Angle_Pitch[0];
        flash_union_buffer[7].float_type = erect_Angle_Pitch[1];
        flash_union_buffer[8].float_type = erect_Angle_Pitch[2];
        flash_union_buffer[9].float_type = erect_Angle_Pitch[3];
        flash_union_buffer[10].int32_type = Yao.Target_Speed;
        flash_union_buffer[11].float_type = Yao.Target_height;
        flash_union_buffer[12].uint8_type = Show_Flag;

        // 第二页：yaw相关参数、模糊模式、P值
        flash_union_buffer[13].float_type = erect_Gyro_Yaw[0];
        flash_union_buffer[14].float_type = erect_Gyro_Yaw[1];
        flash_union_buffer[15].float_type = erect_Gyro_Yaw[2];
        flash_union_buffer[16].float_type = erect_Gyro_Yaw[3];
        flash_union_buffer[17].float_type = erect_Angle_Yaw[0];
        flash_union_buffer[18].float_type = erect_Angle_Yaw[1];
        flash_union_buffer[19].float_type = erect_Angle_Yaw[2];
        flash_union_buffer[20].float_type = erect_Angle_Yaw[3];
        flash_union_buffer[21].float_type = erect_Angle_Yaw_2[0];
        flash_union_buffer[22].float_type = erect_Angle_Yaw_2[1];
        flash_union_buffer[23].float_type = erect_Angle_Yaw_2[2];
        flash_union_buffer[24].float_type = erect_Angle_Yaw_2[3];
        flash_union_buffer[26].float_type = Target_Yaw ;
        flash_union_buffer[26].float_type = P_Value_L[0];
        flash_union_buffer[27].float_type = P_Value_L[1];
        flash_union_buffer[28].float_type = P_Value_L[2];
        flash_union_buffer[29].float_type = P_Value_L[3];
        flash_union_buffer[30].float_type = P_Value_L[4];
        flash_union_buffer[31].float_type = P_Value_L[5];
        flash_union_buffer[32].float_type = P_Value_L[6];

        // 第三页：电机PWM、增量参数、yaw角度
        flash_union_buffer[33].uint32_type = pwmLF;
        flash_union_buffer[34].uint32_type = pwmLB;
        flash_union_buffer[35].uint32_type = pwmRF;
        flash_union_buffer[36].uint32_type = pwmRB;
        flash_union_buffer[37].float_type = erect_Inc_X[0];
        flash_union_buffer[38].float_type = erect_Inc_X[1];
        flash_union_buffer[39].float_type = erect_Inc_X[2];
        flash_union_buffer[40].float_type = erect_Inc_Y[0];
        flash_union_buffer[41].float_type = erect_Inc_Y[1];
        flash_union_buffer[42].float_type = erect_Inc_Y[2];
        flash_union_buffer[43].float_type = erect_Inc_Roll[0];
        flash_union_buffer[44].float_type = erect_Inc_Roll[1];
        flash_union_buffer[45].float_type = erect_Inc_Roll[2];
        flash_union_buffer[46].float_type = erect_yawan[0];
        flash_union_buffer[47].float_type = erect_yawan[1];
        flash_union_buffer[48].float_type = erect_yawan[2];

        // 第四页：控制标志、速度参数
//         flash_union_buffer[49].uint8_type = Thres_Filiter_Flag_1;
//         flash_union_buffer[50].uint8_type = Jump_Control_Flag;
//         flash_union_buffer[51].uint8_type = Single_Control_Flag;
// //        flash_union_buffer[52].uint8_type = Zebra_Flag;
//         flash_union_buffer[53].uint8_type = Ramp_Flag;
//         flash_union_buffer[54].uint8_type = Small_S_Flag;
//         flash_union_buffer[55].float_type = Stretch_Coefficient;
//         flash_union_buffer[56].uint16_type = Max_Speed;
//         flash_union_buffer[57].uint16_type = Min_Speed;
//         flash_union_buffer[58].uint16_type = Jump_speed;
//         flash_union_buffer[59].uint16_type = Single_speed_1;
//         flash_union_buffer[60].uint16_type = Single_speed_2;
//         flash_union_buffer[61].uint16_type = Cross_Speed;
//         flash_union_buffer[62].uint16_type = Cross_State_4_Speed;
//         flash_union_buffer[63].uint16_type = L_Circle_State_1_Speed;
//         flash_union_buffer[64].uint16_type = L_Circle_State_2_Speed;
//         flash_union_buffer[65].uint16_type = L_Circle_State_3_Speed;
//         flash_union_buffer[66].uint16_type = L_Circle_State_4_Speed;
//         flash_union_buffer[67].uint16_type = L_Circle_State_5_Speed;
//         flash_union_buffer[68].uint16_type = R_Circle_State_1_Speed;
//         flash_union_buffer[69].uint16_type = R_Circle_State_2_Speed;
//         flash_union_buffer[70].uint16_type = R_Circle_State_3_Speed;
//         flash_union_buffer[71].uint16_type = R_Circle_State_4_Speed;
//         flash_union_buffer[72].uint16_type = R_Circle_State_5_Speed;
//         flash_union_buffer[73].uint16_type = Small_S_Speed;
//         flash_union_buffer[74].uint16_type = Ramp_State_2_Speed;

//         // 第五页：跳跃线、控制参数、bypass参数
//         flash_union_buffer[75].uint8_type = jump_line;
//         flash_union_buffer[76].uint8_type = jump_line_slowdown;
//         flash_union_buffer[77].uint8_type = T1;
//         flash_union_buffer[78].uint8_type = T2;
//         flash_union_buffer[79].uint8_type = T3;
//         flash_union_buffer[80].uint8_type = Bypass_Line;
//         flash_union_buffer[81].float_type = yaw_bypass;
//         flash_union_buffer[82].uint16_type = Bypass_Count1;
//         flash_union_buffer[83].uint16_type = Bypass_Count2;
//         flash_union_buffer[84].uint16_type = Bypass_Count3;

//         // in addition
//         flash_union_buffer[85].uint8_type = Error_Line;
//         flash_union_buffer[86].uint8_type = Single_jg_line;
//         flash_union_buffer[87].uint16_type = Image_count_Single_End;
//         flash_union_buffer[88].uint16_type = Image_Gain;
//         flash_union_buffer[89].uint16_type = Image_EpTime;
//         flash_union_buffer[90].uint16_type = ramp_judge_dis;
//         flash_union_buffer[91].uint16_type = ramp_end_dis;
//         flash_union_buffer[92].uint16_type = ramp_up_delay;

        flash_erase_page(0, 1);
        flash_write_page_from_buffer(0, 1, 93);
        flash_buffer_clear();
    }
    if( way == READ ) // 读出
    {
        flash_buffer_clear();
        flash_read_page_to_buffer(0, 1, 93);

        // 第一页：俯仰角、翻滚角、gyro/angle前后平衡参数、速度、高度、显示标志
        imu660ra.offset_angle.pitch = flash_union_buffer[0].float_type;
        imu660ra.offset_angle.roll = flash_union_buffer[1].float_type;
        erect_Gyro_Pitch[0] = flash_union_buffer[2].float_type;
        erect_Gyro_Pitch[1] = flash_union_buffer[3].float_type;
        erect_Gyro_Pitch[2] = flash_union_buffer[4].float_type;
        erect_Gyro_Pitch[3] = flash_union_buffer[5].float_type;
        erect_Angle_Pitch[0] = flash_union_buffer[6].float_type;
        erect_Angle_Pitch[1] = flash_union_buffer[7].float_type;
        erect_Angle_Pitch[2] = flash_union_buffer[8].float_type;
        erect_Angle_Pitch[3] = flash_union_buffer[9].float_type;
        Yao.Target_Speed = flash_union_buffer[10].int32_type;
        Yao.Target_height = flash_union_buffer[11].float_type;
        Show_Flag = flash_union_buffer[12].uint8_type;

        // 第二页：yaw相关参数、模糊模式、P值
        erect_Gyro_Yaw[0] = flash_union_buffer[13].float_type;
        erect_Gyro_Yaw[1] = flash_union_buffer[14].float_type;
        erect_Gyro_Yaw[2] = flash_union_buffer[15].float_type;
        erect_Gyro_Yaw[3] = flash_union_buffer[16].float_type;
        erect_Angle_Yaw[0] = flash_union_buffer[17].float_type;
        erect_Angle_Yaw[1] = flash_union_buffer[18].float_type;
        erect_Angle_Yaw[2] = flash_union_buffer[19].float_type;
        erect_Angle_Yaw[3] = flash_union_buffer[20].float_type;
        erect_Angle_Yaw_2[0] = flash_union_buffer[21].float_type;
        erect_Angle_Yaw_2[1] = flash_union_buffer[22].float_type;
        erect_Angle_Yaw_2[2] = flash_union_buffer[23].float_type;
        erect_Angle_Yaw_2[3] = flash_union_buffer[24].float_type;
        Target_Yaw  = flash_union_buffer[25].float_type;
        P_Value_L[0] = flash_union_buffer[26].float_type;
        P_Value_L[1] = flash_union_buffer[27].float_type;
        P_Value_L[2] = flash_union_buffer[28].float_type;
        P_Value_L[3] = flash_union_buffer[29].float_type;
        P_Value_L[4] = flash_union_buffer[30].float_type;
        P_Value_L[5] = flash_union_buffer[31].float_type;
        P_Value_L[6] = flash_union_buffer[32].float_type;

        // 第三页：电机PWM、增量参数、yaw角度
        pwmLF = flash_union_buffer[33].uint32_type;
        pwmLB = flash_union_buffer[34].uint32_type;
        pwmRF = flash_union_buffer[35].uint32_type;
        pwmRB = flash_union_buffer[36].uint32_type;
        erect_Inc_X[0] = flash_union_buffer[37].float_type;
        erect_Inc_X[1] = flash_union_buffer[38].float_type;
        erect_Inc_X[2] = flash_union_buffer[39].float_type;
        erect_Inc_Y[0] = flash_union_buffer[40].float_type;
        erect_Inc_Y[1] = flash_union_buffer[41].float_type;
        erect_Inc_Y[2] = flash_union_buffer[42].float_type;
        erect_Inc_Roll[0] = flash_union_buffer[43].float_type;
        erect_Inc_Roll[1] = flash_union_buffer[44].float_type;
        erect_Inc_Roll[2] = flash_union_buffer[45].float_type;
        erect_yawan[0] = flash_union_buffer[46].float_type;
        erect_yawan[1] = flash_union_buffer[47].float_type;
        erect_yawan[2] = flash_union_buffer[48].float_type;

        //第四页：控制标志、速度参数
//         Thres_Filiter_Flag_1 = flash_union_buffer[49].uint8_type;
//         Jump_Control_Flag = flash_union_buffer[50].uint8_type;
//         Single_Control_Flag = flash_union_buffer[51].uint8_type;
// //        Zebra_Flag = flash_union_buffer[52].uint8_type;
//         Ramp_Flag = flash_union_buffer[53].uint8_type;
//         Small_S_Flag = flash_union_buffer[54].uint8_type;
//         Stretch_Coefficient = flash_union_buffer[55].float_type;
//         Max_Speed = flash_union_buffer[56].uint16_type;
//         Min_Speed = flash_union_buffer[57].uint16_type;
//         Jump_speed = flash_union_buffer[58].uint16_type;
//         Single_speed_1 = flash_union_buffer[59].uint16_type;
//         Single_speed_2 = flash_union_buffer[60].uint16_type;
//         Cross_Speed = flash_union_buffer[61].uint16_type;
//         Cross_State_4_Speed = flash_union_buffer[62].uint16_type;
//         L_Circle_State_1_Speed = flash_union_buffer[63].uint16_type;
//         L_Circle_State_2_Speed = flash_union_buffer[64].uint16_type;
//         L_Circle_State_3_Speed = flash_union_buffer[65].uint16_type;
//         L_Circle_State_4_Speed = flash_union_buffer[66].uint16_type;
//         L_Circle_State_5_Speed = flash_union_buffer[67].uint16_type;
//         R_Circle_State_1_Speed = flash_union_buffer[68].uint16_type;
//         R_Circle_State_2_Speed = flash_union_buffer[69].uint16_type;
//         R_Circle_State_3_Speed = flash_union_buffer[70].uint16_type;
//         R_Circle_State_4_Speed = flash_union_buffer[71].uint16_type;
//         R_Circle_State_5_Speed = flash_union_buffer[72].uint16_type;
//         Small_S_Speed = flash_union_buffer[73].uint16_type;
//         Ramp_State_2_Speed = flash_union_buffer[74].uint16_type;

//         // 第五页：跳跃线、控制参数、bypass参数
//         jump_line = flash_union_buffer[75].uint8_type;
//         jump_line_slowdown = flash_union_buffer[76].uint8_type;
//         T1 = flash_union_buffer[77].uint8_type;
//         T2 = flash_union_buffer[78].uint8_type;
//         T3 = flash_union_buffer[79].uint8_type;
//         Bypass_Line = flash_union_buffer[80].uint8_type;
//         yaw_bypass = flash_union_buffer[81].float_type;
//         Bypass_Count1 = flash_union_buffer[82].uint16_type;
//         Bypass_Count2 = flash_union_buffer[83].uint16_type;
//         Bypass_Count3 = flash_union_buffer[84].uint16_type;

//         // in addition
//         Error_Line = flash_union_buffer[85].uint8_type;
//         Single_jg_line = flash_union_buffer[86].uint8_type;
//         Image_count_Single_End = flash_union_buffer[87].uint16_type;
//         Image_Gain = flash_union_buffer[88].uint16_type;
//         Image_EpTime = flash_union_buffer[89].uint16_type;
//         ramp_judge_dis = flash_union_buffer[90].uint16_type;
//         ramp_end_dis = flash_union_buffer[91].uint16_type;
//         ramp_up_delay = flash_union_buffer[92].uint16_type;

        flash_buffer_clear();
    }
}

float data_change = 1;
uint8 key  = 0;
uint8 key1 = 0;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      切换参数调节步进单位 (从100到0.00001共8档)
//  @param      void
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void switch_unit()
{
    ips200_show_string( 0, 33*8, "switch:" );
    ips200_show_float( 0, 34*8, 321.12345, 3, 5 );

        ips200_show_string( 0, 35*8, "         " );
        if(key_detect(KEY_1, KEY_SHORT_PRESS))
            unit += 1;
        if(key_detect(KEY_3, KEY_SHORT_PRESS))
            unit -= 1;

        if( unit <= -1 )
            unit = 7;
        else if( unit >= 8 )
            unit = 0;

        switch(unit)
        {
            case 0:ips200_show_string( 0, 35*8, "^" );data_change = 100;break;
            case 1:ips200_show_string( 6, 35*8, "^" );data_change = 10;break;
            case 2:ips200_show_string( 12, 35*8, "^" );data_change = 1;break;
            case 3:ips200_show_string( 24, 35*8, "^" );data_change = 0.1;break;
            case 4:ips200_show_string( 30, 35*8, "^" );data_change = 0.01;break;
            case 5:ips200_show_string( 36, 35*8, "^" );data_change = 0.001;break;
            case 6:ips200_show_string( 42, 35*8, "^" );data_change = 0.0001;break;
            case 7:ips200_show_string( 48, 35*8, "^" );data_change = 0.00001;break;
            default:break;
        }
        if(key_detect(KEY_2, KEY_SHORT_PRESS))
        {
            ips200_clear();
            key1 = 0;
        }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      通过按键调节 float 类型变量值 (KEY_1加/KEY_3减, KEY_2长按切换单位)
//  @param      data    指向待调节的 float 变量
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void data_operate(float *data)
{
        if(key1)
            switch_unit();
        else
        {
            if(key_detect(KEY_1, KEY_SHORT_PRESS))
                *data += data_change;
            if(key_detect(KEY_3, KEY_SHORT_PRESS))
                *data -= data_change;

            if(key_detect(KEY_2, KEY_ONCE_LONG_PRESS))
                key1 = 1;
            if(key_detect(KEY_2, KEY_SHORT_PRESS))
                key = 0;
        }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      通过按键调节 uint8 类型变量值 (KEY_1加/KEY_3减, KEY_2长按切换单位)
//  @param      data    指向待调节的 uint8 变量
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void data_operate_uint8(uint8 *data)
{
        if(key1)
            switch_unit();
        else
        {
            if(key_detect(KEY_1, KEY_SHORT_PRESS))
                *data += data_change;
            if(key_detect(KEY_3, KEY_SHORT_PRESS))
                *data -= data_change;

            if(key_detect(KEY_2, KEY_ONCE_LONG_PRESS))
                key1 = 1;
            if(key_detect(KEY_2, KEY_SHORT_PRESS))
                key = 0;
        }
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      通过按键调节 uint16 类型变量值 (KEY_1加/KEY_3减, KEY_2长按切换单位)
//  @param      data    指向待调节的 uint16 变量
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void data_operate_uint16(uint16 *data)
{
        if(key1)
            switch_unit();
        else
        {
            if(key_detect(KEY_1, KEY_SHORT_PRESS))
                *data += data_change;
            if(key_detect(KEY_3, KEY_SHORT_PRESS))
                *data -= data_change;

            if(key_detect(KEY_2, KEY_ONCE_LONG_PRESS))
                key1 = 1;
            if(key_detect(KEY_2, KEY_SHORT_PRESS))
                key = 0;
        }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      通过按键调节 uint32 类型变量值 (KEY_1加/KEY_3减, KEY_2长按切换单位)
//  @param      data    指向待调节的 uint32 变量
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void data_operate_uint32(uint32 *data)
{
        if(key1)
            switch_unit();
        else
        {
            if(key_detect(KEY_1, KEY_SHORT_PRESS))
                *data += data_change;
            if(key_detect(KEY_3, KEY_SHORT_PRESS))
                *data -= data_change;

            if(key_detect(KEY_2, KEY_ONCE_LONG_PRESS))
                key1 = 1;
            if(key_detect(KEY_2, KEY_SHORT_PRESS))
                key = 0;
        }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      通过按键调节 int 类型变量值 (KEY_1加/KEY_3减, KEY_2长按切换单位)
//  @param      data    指向待调节的 int 变量
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void data_operate_int(int *data)
{
        if(key1)
            switch_unit();
        else
        {
            if(key_detect(KEY_1, KEY_SHORT_PRESS))
                *data += data_change;
            if(key_detect(KEY_3, KEY_SHORT_PRESS))
                *data -= data_change;

            if(key_detect(KEY_2, KEY_ONCE_LONG_PRESS))
                key1 = 1;
            if(key_detect(KEY_2, KEY_SHORT_PRESS))
                key = 0;
        }
}
uint8 flag_track = 0;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      菜单主函数: 多页参数显示与按键调节, KEY_4保存并退出/进入运行模式
//  @param      void
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void menu(void)
{
    if( menu_mode == 0 )            // menu_mode == 0,进入菜单
    {
        flag_track = 0;
        if(page > 4)
            page = 0;
        else if(page < 0)
            page = 4;
        switch(page)
        {
            case 0 :                // 第一面，平衡相关
                ips200_show_string( 20, 0*8, "page: 111111111" );
                //零点
                ips200_show_string( 20, 1*8, "pitch:" );
                ips200_show_float( 100, 1*8, imu660ra.offset_angle.pitch, 3, 6 );
                ips200_show_string( 20, 2*8, "roll :" );
                ips200_show_float( 100, 2*8, imu660ra.offset_angle.roll, 3, 6 );
                //串级PID
                ips200_show_string( 20, 3*8, "pit-Gyro-P :" );
                ips200_show_float( 100, 3*8, erect_Gyro_Pitch[0], 3, 5 );
                ips200_show_string( 20, 4*8, "pit-Gyro-I :" );
                ips200_show_float( 100, 4*8, erect_Gyro_Pitch[1], 3, 5 );
                ips200_show_string( 20, 5*8, "pit-Gyro-D:" );
                ips200_show_float( 100, 5*8, erect_Gyro_Pitch[2], 3, 5 );
                ips200_show_string( 20, 6*8, "pit-Gyro-IL:" );
                ips200_show_float( 100, 6*8, erect_Gyro_Pitch[3], 3, 5 );
                ips200_show_string( 20, 7*8, "pit-Angle-P:" );
                ips200_show_float( 100, 7*8, erect_Angle_Pitch[0],4, 5 );
                ips200_show_string( 20, 8*8, "pit-Angle-I   :" );
                ips200_show_float( 100, 8*8, erect_Angle_Pitch[1],3, 5 );
                ips200_show_string( 20, 9*8, "pit-Angle-D:" );
                ips200_show_float( 100, 9*8, erect_Angle_Pitch[2],4, 5 );
                ips200_show_string( 20, 10*8,"pit-Angle-IL:" );
                ips200_show_float( 100, 10*8,erect_Angle_Pitch[3], 3, 5 );
                ips200_show_string( 20, 11*8,"speed:" );
                ips200_show_int(   100, 11*8, Yao.Target_Speed, 5 );
                ips200_show_string( 20 , 12*8,"height:" );
                ips200_show_float(  100, 12*8, Yao.Target_height, 3, 6 );
                ips200_show_string( 20 , 13*8,"IPS:" );
                ips200_show_uint(  100 , 13*8, Show_Flag, 3 );


                // 光标显示
                if(!key)
                {
                    if(key_detect(KEY_3, KEY_SHORT_PRESS))
                    {
                        cursor++;
                        ips200_show_string( 0, ((cursor-1)*8), "   " );
                        if(cursor > 13)
                            cursor = 0;
                    }
                    if(key_detect(KEY_1, KEY_SHORT_PRESS))
                    {
                        cursor--;
                        ips200_show_string( 0, ((cursor+1)*8), "   " );
                        if(cursor <= -1)
                            cursor = 13;
                    }
                    if(key_detect(KEY_2, KEY_SHORT_PRESS))
                        key = 1;
                    ips200_show_string( 0, (cursor*8), ">>>" );
                }
                else
                    ips200_show_string( 0, (cursor*8), "---" );

                // 根据光标位置选择对应变量操作
                if(key)
                {
                    switch(cursor)
                    {
                        case 0:     // 最上一行用来翻面
                            if(key_detect(KEY_3, KEY_SHORT_PRESS)){
                                ips200_clear();
                                page++;}
                            if(key_detect(KEY_1, KEY_SHORT_PRESS)){
                                ips200_clear();
                                page--;}
                            if(key_detect(KEY_2, KEY_SHORT_PRESS))
                                key = 0;break;

                        case 1:     // 俯仰角
                            data_operate(&imu660ra.offset_angle.pitch);break;
                        case 2:     // 翻滚角
                            data_operate(&imu660ra.offset_angle.roll);break;
                        case 3:     // gyro前后平衡参数
                            data_operate(&erect_Gyro_Pitch[0]);break;
                        case 4:
                            data_operate(&erect_Gyro_Pitch[1]);break;
                        case 5:
                            data_operate(&erect_Gyro_Pitch[2]);break;
                        case 6:
                            data_operate(&erect_Gyro_Pitch[3]);break;
                        case 7:     // angle前后平衡参数
                            data_operate(&erect_Angle_Pitch[0]);break;
                        case 8:
                            data_operate(&erect_Angle_Pitch[1]);break;
                        case 9:
                            data_operate(&erect_Angle_Pitch[2]);break;
                        case 10:
                            data_operate(&erect_Angle_Pitch[3]);break;
                        case 11:     // speed前后平衡参数
                            data_operate_int(&Yao.Target_Speed);break;
                        case 12:
                            data_operate(&Yao.Target_height);break;
                        case 13:
                            data_operate_uint8(&Show_Flag);break;
                        default:break;
                    }// page1-cursor end
                }break;// page1 end
                case 1:              //第二面，转向参数
                    ips200_show_string( 20, 0*8, "page: 222222222" );
                    ips200_show_string( 20, 1*8, "yaw-gyro-P:" );
                    ips200_show_float( 100, 1*8, erect_Gyro_Yaw[0], 3, 6 );
                    ips200_show_string( 20, 2*8,"yaw-gyro-I:" );
                    ips200_show_float( 100, 2*8, erect_Gyro_Yaw[1], 3, 6 );
                    ips200_show_string( 20, 3*8,"yaw-gyro-D:" );
                    ips200_show_float( 100, 3*8, erect_Gyro_Yaw[2], 3, 6 );
                    ips200_show_string( 20, 4*8,"yaw-gyro-IL:" );
                    ips200_show_float( 100, 4*8, erect_Gyro_Yaw[3], 3, 6 );
                    ips200_show_string( 20, 5*8, "turn-image-P:" );
                    ips200_show_float( 100, 5*8, erect_Angle_Yaw[0],3, 6 );
                    ips200_show_string( 20, 6*8, "turn-image-I   :" );
                    ips200_show_float( 100, 6*8, erect_Angle_Yaw[1],3, 6 );
                    ips200_show_string( 20, 7*8, "turn-image-D:" );
                    ips200_show_float( 100, 7*8, erect_Angle_Yaw[2],3, 6 );
                    ips200_show_string( 20, 8*8, "turn-image-IL:" );
                    ips200_show_float( 100, 8*8, erect_Angle_Yaw[3], 3, 6 );
                    ips200_show_string( 20, 9*8, "yaw_Angle_P:" );
                    ips200_show_float( 100, 9*8, erect_Angle_Yaw_2[0],3, 6 );
                    ips200_show_string( 20, 10*8, "yaw_Angle-I:" );
                    ips200_show_float( 100, 10*8, erect_Angle_Yaw_2[1],3, 6 );
                    ips200_show_string( 20, 11*8, "yaw_Angle-D:" );
                    ips200_show_float( 100, 11*8, erect_Angle_Yaw_2[2],3, 6 );
                    ips200_show_string( 20, 12*8, "yaw_Angle-IL:" );
                    ips200_show_float( 100, 12*8, erect_Angle_Yaw_2[3], 3, 6 );
                    ips200_show_string( 20, 13*8,"Target_Yaw" );
                    ips200_show_float( 100, 14*8, Target_Yaw  , 4, 2 );
                    ips200_show_string( 20, 14*8,"Fuzzy-P1:" );
                    ips200_show_float( 100, 14*8, P_Value_L[0], 4, 2 );
                    ips200_show_string( 20, 15*8,"Fuzzy-P2:" );
                    ips200_show_float( 100, 15*8, P_Value_L[1], 4, 2 );
                    ips200_show_string( 20, 16*8,"Fuzzy-P3:" );
                    ips200_show_float( 100, 16*8, P_Value_L[2], 4, 2 );
                    ips200_show_string( 20, 17*8,"Fuzzy-P4:" );
                    ips200_show_float( 100, 17*8, P_Value_L[3], 4, 2 );
                    ips200_show_string( 20, 18*8,"Fuzzy-P5:" );
                    ips200_show_float( 100, 18*8, P_Value_L[4], 4, 2 );
                    ips200_show_string( 20, 19*8,"Fuzzy-P6:" );
                    ips200_show_float( 100, 19*8, P_Value_L[5], 4, 2 );
                    ips200_show_string( 20, 20*8,"Fuzzy-P7:" );
                    ips200_show_float( 100, 20*8, P_Value_L[6], 4, 2 );


                    if(!key)
                    {
                        if(key_detect(KEY_3, KEY_SHORT_PRESS))
                        {
                            cursor++;
                            ips200_show_string( 0, ((cursor-1)*8), "   " );
                            if(cursor > 20)
                                cursor = 0;
                        }
                        if(key_detect(KEY_1, KEY_SHORT_PRESS))
                        {
                            cursor--;
                            ips200_show_string( 0, ((cursor+1)*8), "   " );
                            if(cursor <= -1)
                                cursor = 20;
                        }
                        if(key_detect(KEY_2, KEY_SHORT_PRESS))
                            key = 1;
                        ips200_show_string( 0, (cursor*8), ">>>" );
                    }
                    else
                        ips200_show_string( 0, (cursor*8), "---" );

                    if(key)
                    {
                        switch(cursor)
                        {
                            case 0:     // 翻面
                                if(key_detect(KEY_3, KEY_SHORT_PRESS)){
                                    ips200_clear();
                                    page++;}
                                if(key_detect(KEY_1, KEY_SHORT_PRESS)){
                                    ips200_clear();
                                    page--;}
                                if(key_detect(KEY_2, KEY_SHORT_PRESS))
                                    key = 0;break;

                            case 1:
                                data_operate(&erect_Gyro_Yaw[0]);break;
                            case 2:
                                data_operate(&erect_Gyro_Yaw[1]);break;
                            case 3:
                                data_operate(&erect_Gyro_Yaw[2]);break;
                            case 4:
                                data_operate(&erect_Gyro_Yaw[3]);break;
                            case 5:
                                data_operate(&erect_Angle_Yaw[0]);break;
                            case 6:
                                data_operate(&erect_Angle_Yaw[1]);break;
                            case 7:
                                data_operate(&erect_Angle_Yaw[2]);break;
                            case 8:
                                data_operate(&erect_Angle_Yaw[3]);break;
                            case 9:
                                data_operate(&erect_Angle_Yaw_2[0]);break;
                            case 10:
                                data_operate(&erect_Angle_Yaw_2[1]);break;
                            case 11:
                                data_operate(&erect_Angle_Yaw_2[2]);break;
                            case 12:
                                data_operate(&erect_Angle_Yaw_2[3]);break;
                            case 13:
                                data_operate(&Target_Yaw);break;
                            case 14:
                                data_operate(&P_Value_L[0]);break;
                            case 15:
                                data_operate(&P_Value_L[1]);break;
                            case 16:
                                data_operate(&P_Value_L[2]);break;
                            case 17:
                                data_operate(&P_Value_L[3]);break;
                            case 18:
                                data_operate(&P_Value_L[4]);break;
                            case 19:
                                data_operate(&P_Value_L[5]);break;
                            case 20:
                                data_operate(&P_Value_L[6]);break;
                            default:break;
                        }// page2-cursor end
                    }break;// page2 end
                        case 2:              //第三面,舵机
                            ips200_show_string( 20, 0*8, "page: 333333333" );
                            ips200_show_string( 20, 1*8,"pwmLF:" );
                            ips200_show_uint(  100, 1*8, pwmLF, 5 );
                            ips200_show_string( 20, 2*8,"pwmLB:" );
                            ips200_show_uint(  100, 2*8, pwmLB, 5 );
                            ips200_show_string( 20, 3*8,"pwmRF:" );
                            ips200_show_uint(  100, 3*8, pwmRF, 5 );
                            ips200_show_string( 20, 4*8,"pwmRB:" );
                            ips200_show_uint(  100, 4*8, pwmRB, 5 );
                            ips200_show_string( 20, 5*8, "Km-X-P :" );
                            ips200_show_float( 100, 5*8, erect_Inc_X[0], 3, 5 );
                            ips200_show_string( 20, 6*8, "Km-X-i :" );
                            ips200_show_float( 100, 6*8, erect_Inc_X[1], 3, 5 );
                            ips200_show_string( 20, 7*8, "Km-X-d:" );
                            ips200_show_float( 100, 7*8, erect_Inc_X[2], 3, 5 );
                            ips200_show_string( 20, 8*8, "Km-Y-P:" );
                            ips200_show_float( 100, 8*8, erect_Inc_Y[0], 3, 5 );
                            ips200_show_string( 20, 9*8, "Km-Y-i:" );
                            ips200_show_float( 100, 9*8, erect_Inc_Y[1],3, 5 );
                            ips200_show_string( 20, 10*8, "Km-Y-d:" );
                            ips200_show_float( 100, 10*8, erect_Inc_Y[2],3, 5 );
                            ips200_show_string( 20, 11*8, "Km-ROLL-P:" );
                            ips200_show_float( 100, 11*8, erect_Inc_Roll[0],3, 5 );
                            ips200_show_string( 20, 12*8,"Km-ROLL-i:" );
                            ips200_show_float( 100, 12*8, erect_Inc_Roll[1], 3, 5 );
                            ips200_show_string( 20, 13*8,"Km-ROLL-d:" );
                            ips200_show_float( 100, 13*8, erect_Inc_Roll[2], 3, 5 );
                            ips200_show_string( 20, 14*8,"yawan-p:" );
                            ips200_show_float( 100, 14*8, erect_yawan[0],3, 5 );
                            ips200_show_string( 20, 15*8,"yawan-i:" );
                            ips200_show_float( 100, 15*8, erect_yawan[1], 3, 5 );
                            ips200_show_string( 20, 16*8,"yawan-d:" );
                            ips200_show_float( 100, 16*8, erect_yawan[2], 3, 5 );




                            if(!key)
                            {
                                if(key_detect(KEY_3, KEY_SHORT_PRESS))
                                {
                                    cursor++;
                                    ips200_show_string( 0, ((cursor-1)*8), "   " );
                                    if(cursor > 16)
                                        cursor = 0;
                                }
                                if(key_detect(KEY_1, KEY_SHORT_PRESS))
                                {
                                    cursor--;
                                    ips200_show_string( 0, ((cursor+1)*8), "   " );
                                    if(cursor <= -1)
                                        cursor = 16;
                                }
                                if(key_detect(KEY_2, KEY_SHORT_PRESS))
                                    key = 1;
                                ips200_show_string( 0, (cursor*8), ">>>" );
                            }
                            else
                                ips200_show_string( 0, (cursor*8), "---" );

                            if(key)
                            {
                                switch(cursor)
                                {
                                    case 0:     // 翻面
                                        if(key_detect(KEY_3, KEY_SHORT_PRESS)){
                                            ips200_clear();
                                            page++;}
                                        if(key_detect(KEY_1, KEY_SHORT_PRESS)){
                                            ips200_clear();
                                            page--;}
                                        if(key_detect(KEY_2, KEY_SHORT_PRESS))
                                            key = 0;break;

                                    case 1:     // 俯仰角
                                        data_operate_uint32(&pwmLF);break;
                                    case 2:     // 翻滚角
                                        data_operate_uint32(&pwmLB);break;
                                    case 3:     // gyro前后平衡参数
                                        data_operate_uint32(&pwmRF);break;
                                    case 4:
                                        data_operate_uint32(&pwmRB);break;
                                    case 5:
                                        data_operate(&erect_Inc_X[0]);break;
                                    case 6:
                                        data_operate(&erect_Inc_X[1]);break;
                                    case 7:     // angle前后平衡参数
                                        data_operate(&erect_Inc_X[2]);break;
                                    case 8:
                                        data_operate(&erect_Inc_Y[0]);break;
                                    case 9:
                                        data_operate(&erect_Inc_Y[1]);break;
                                    case 10:
                                        data_operate(&erect_Inc_Y[2]);break;
                                    case 11:     // angle前后平衡参数
                                        data_operate(&erect_Inc_Roll[0]);break;
                                    case 12:
                                        data_operate(&erect_Inc_Roll[1]);break;
                                    case 13:
                                        data_operate(&erect_Inc_Roll[2]);break;
                                    case 14:
                                        data_operate(&erect_yawan[0]);break;
                                    case 15:
                                        data_operate(&erect_yawan[1]);break;
                                    case 16:
                                        data_operate(&erect_yawan[2]);break;
                                    default:break;
                                }// page3-cursor end
                            }break;// page3 end
                        case 3:     //第四面 图像参数
                            ips200_show_string( 20, 0*8, "page: 444444444" );
                            ips200_show_string( 20, 1*8, "Thres_Filiter:" );
                            ips200_show_uint(  100, 1*8,  Thres_Filiter_Flag_1, 3);
                            ips200_show_string( 20, 2*8, "Jump_Flag:" );
                            ips200_show_uint(  100, 2*8,  Jump_Control_Flag, 3);
                            ips200_show_string( 20, 3*8, "Single_Flag:" );
                            ips200_show_uint(  100, 3*8,  Single_Control_Flag, 3);
                            ips200_show_string( 20, 4*8, "Zebra_Flag:" );
                            ips200_show_uint(  100, 4*8,  Zebra_Flag, 3);
                            ips200_show_string( 20, 5*8, "Ramp_Flag:" );
                            ips200_show_uint(  100, 5*8,  Ramp_Flag, 3);
                            ips200_show_string( 20, 6*8, "Small_S_Flag:" );
                            ips200_show_uint(  100, 6*8,  Small_S_Flag, 3);

                            ips200_show_string( 20, 7*8, "Stretch:" );
                            ips200_show_float( 100, 7*8, Stretch_Coefficient, 3,6);

                            ips200_show_string( 20, 8*8, "Max_Speed:" );
                            ips200_show_uint(  100, 8*8, Max_Speed, 5 );
                            ips200_show_string( 20, 9*8, "Min_Speed:" );
                            ips200_show_uint(  100, 9*8, Min_Speed,5 );
                            ips200_show_string( 20, 10*8,"Jump_speed:" );
                            ips200_show_uint(  100, 10*8, Jump_speed, 5 );
                            ips200_show_string( 20, 11*8,"Sin_sped_1:" );
                            ips200_show_uint(  100, 11*8, Single_speed_1, 5);
                            ips200_show_string( 20, 12*8,"Sin_sped_2:" );
                            ips200_show_uint(  100, 12*8, Single_speed_2, 5 );
                            ips200_show_string( 20, 13*8,"Cross_Speed:" );
                            ips200_show_uint(  100, 13*8, Cross_Speed, 5 );
                            ips200_show_string( 20, 14*8,"Cro_S_Speed:" );
                            ips200_show_uint(  100, 14*8, Cross_State_4_Speed, 5 );
                            ips200_show_string( 20, 15*8,"L_Cir_1:" );
                            ips200_show_uint(  100, 15*8, L_Circle_State_1_Speed, 5);
                            ips200_show_string( 20, 16*8,"L_Cir_2:" );
                            ips200_show_uint(  100, 16*8, L_Circle_State_2_Speed, 5 );
                            ips200_show_string( 20, 17*8,"L_Cir_3:" );
                            ips200_show_uint(  100, 17*8, L_Circle_State_3_Speed, 5 );
                            ips200_show_string( 20, 18*8,"L_Cir_4:" );
                            ips200_show_uint(  100, 18*8, L_Circle_State_4_Speed, 5 );
                            ips200_show_string( 20, 19*8,"L_Cir_5:" );
                            ips200_show_uint(  100, 19*8, L_Circle_State_5_Speed, 5 );
                            ips200_show_string( 20, 20*8,"R_Cir_1:" );
                            ips200_show_uint(  100, 20*8, R_Circle_State_1_Speed, 5);
                            ips200_show_string( 20, 21*8,"R_Cir_2:" );
                            ips200_show_uint(  100, 21*8, R_Circle_State_2_Speed, 5 );
                            ips200_show_string( 20, 22*8,"R_Cir_3:" );
                            ips200_show_uint(  100, 22*8, R_Circle_State_3_Speed, 5 );
                            ips200_show_string( 20, 23*8,"R_Cir_4:" );
                            ips200_show_uint(  100, 23*8, R_Circle_State_4_Speed, 5 );
                            ips200_show_string( 20, 24*8,"R_Cir_5:" );
                            ips200_show_uint(  100, 24*8, R_Circle_State_5_Speed, 5 );
                            ips200_show_string( 20, 25*8,"Small_S_Sped:" );
                            ips200_show_uint(  100, 25*8, Small_S_Speed, 5 );
                            ips200_show_string( 20, 26*8,"Ramp_Speed:" );
                            ips200_show_uint(  100, 26*8, Ramp_State_2_Speed, 5 );
                            ips200_show_string( 20, 27*8,"Error_Line:" );
                            ips200_show_uint(  100, 27*8, Error_Line, 5 );
                            ips200_show_string( 20, 28*8,"Image_Gain:" );
                            ips200_show_uint(  100, 28*8, Image_Gain, 5 );
                            ips200_show_string( 20, 29*8,"Image_EpTime:" );
                            ips200_show_uint(  100, 29*8, Image_EpTime, 5 );
                            ips200_show_string( 20, 30*8,"Change_Control:" );
                            ips200_show_uint(  100, 30*8, Change_Control, 5 );

                            if(!key)
                            {
                                if(key_detect(KEY_3, KEY_SHORT_PRESS))
                                {
                                    cursor++;
                                    ips200_show_string( 0, ((cursor-1)*8), "   " );
                                    if(cursor > 30)
                                        cursor = 0;
                                }
                                if(key_detect(KEY_1, KEY_SHORT_PRESS))
                                {
                                    cursor--;
                                    ips200_show_string( 0, ((cursor+1)*8), "   " );
                                    if(cursor <= -1)
                                        cursor = 30;
                                }
                                if(key_detect(KEY_2, KEY_SHORT_PRESS))
                                    key = 1;
                                ips200_show_string( 0, (cursor*8), ">>>" );
                            }
                            else
                                ips200_show_string( 0, (cursor*8), "---" );

                            // 根据光标位置选择对应变量操作
                            if(key)
                            {
                                switch(cursor)
                                {
                                    case 0:     // 最上一行用来翻面
                                        if(key_detect(KEY_3, KEY_SHORT_PRESS)){
                                            ips200_clear();
                                            page++;}
                                        if(key_detect(KEY_1, KEY_SHORT_PRESS)){
                                            ips200_clear();
                                            page--;}
                                        if(key_detect(KEY_2, KEY_SHORT_PRESS))
                                            key = 0;break;

                                    case 1:
                                        data_operate_uint8(&Thres_Filiter_Flag_1);break;
                                    case 2:
                                        data_operate_uint8(&Jump_Control_Flag);break;
                                    case 3:
                                        data_operate_uint8(&Single_Control_Flag);break;
                                    case 4:
                                        data_operate_uint8(&Zebra_Flag);break;
                                    case 5:
                                        data_operate_uint8(&Ramp_Flag);break;
                                    case 6:
                                        data_operate_uint8(&Small_S_Flag);break;
                                    case 7:
                                        data_operate(&Stretch_Coefficient);break;
                                    case 8:
                                        data_operate_uint16(&Max_Speed);break;
                                    case 9:
                                        data_operate_uint16(&Min_Speed);break;
                                    case 10:
                                        data_operate_uint16(&Jump_speed);break;
                                    case 11:
                                        data_operate_uint16(&Single_speed_1);break;
                                    case 12:
                                        data_operate_uint16(&Single_speed_2);break;
                                    case 13:
                                        data_operate_uint16(&Cross_Speed);break;
                                    case 14:
                                        data_operate_uint16(&Cross_State_4_Speed);break;
                                    case 15:
                                        data_operate_uint16(&L_Circle_State_1_Speed);break;
                                    case 16:
                                        data_operate_uint16(&L_Circle_State_2_Speed);break;
                                    case 17:
                                        data_operate_uint16(&L_Circle_State_3_Speed);break;
                                    case 18:
                                        data_operate_uint16(&L_Circle_State_4_Speed);break;
                                    case 19:
                                        data_operate_uint16(&L_Circle_State_5_Speed);break;
                                    case 20:
                                        data_operate_uint16(&R_Circle_State_1_Speed);break;
                                    case 21:
                                        data_operate_uint16(&R_Circle_State_2_Speed);break;
                                    case 22:
                                        data_operate_uint16(&R_Circle_State_3_Speed);break;
                                    case 23:
                                        data_operate_uint16(&R_Circle_State_4_Speed);break;
                                    case 24:
                                        data_operate_uint16(&R_Circle_State_5_Speed);break;
                                    case 25:
                                        data_operate_uint16(&Small_S_Speed);break;
                                    case 26:
                                        data_operate_uint16(&Ramp_State_2_Speed);break;
                                    case 27:
                                        data_operate_uint8(&Error_Line);break;
                                    case 28:
                                        data_operate_uint16(&Image_Gain);break;
                                    case 29:
                                        data_operate_uint16(&Image_EpTime);break;
                                    case 30:
                                        data_operate_uint8(&Change_Control);break;
                                    default:break;
                                }// page4-cursor end
                            }break;
                        case 4:     //第五面 跳跃与单边桥
                            ips200_show_string( 20, 0*8, "page: 555555555" );
                            ips200_show_string( 20, 1*8,"Jump_Line:" );
                            ips200_show_uint(  100, 1*8, jump_line, 3 );
                            ips200_show_string( 20, 2*8,"jump_slowdown:" );
                            ips200_show_uint(  100, 2*8, jump_line_slowdown, 3 );
                            ips200_show_string( 20, 3*8,"T1:" );
                            ips200_show_uint(  100, 3*8, T1, 5 );
                            ips200_show_string( 20, 4*8,"T2:" );
                            ips200_show_uint(  100, 4*8, T2, 5 );
                            ips200_show_string( 20, 5*8,"T3:" );
                            ips200_show_uint(  100, 5*8, T3, 5 );

                            ips200_show_string( 20, 6*8,"Bypass_Line:" );
                            ips200_show_uint(  100, 6*8, Bypass_Line, 4 );
                            ips200_show_string( 20, 7*8, "Yaw_inc:" );
                            ips200_show_float( 100, 7*8, yaw_bypass, 3,6);
                            ips200_show_string( 20, 8*8,"bps_count1:" );
                            ips200_show_uint(  100, 8*8, Bypass_Count1, 4 );
                            ips200_show_string( 20, 9*8,"bps_count2:" );
                            ips200_show_uint(  100, 9*8, Bypass_Count2, 4 );
                            ips200_show_string( 20, 10*8,"bps_count3:" );
                            ips200_show_uint(  100, 10*8, Bypass_Count3, 4 );
                            ips200_show_string( 20, 11*8,"Single_line:" );
                            ips200_show_uint(  100, 11*8, Single_jg_line, 4 );
                            ips200_show_string( 20, 12*8,"Single_Cout:" );
                            ips200_show_uint(  100, 12*8, Image_count_Single_End, 4 );
                            ips200_show_string( 20, 13*8,"ramp_judge_dis:" );
                            ips200_show_uint(  100, 13*8, ramp_judge_dis, 4 );
                            ips200_show_string( 20, 14*8,"ramp_end_dis:" );
                            ips200_show_uint(  100, 14*8, ramp_end_dis, 4 );
                            ips200_show_string( 20, 15*8,"ramp_up_delay:" );
                            ips200_show_uint(  100, 15*8, ramp_up_delay, 4 );


                            if(!key)
                            {
                                if(key_detect(KEY_3, KEY_SHORT_PRESS))
                                {
                                    cursor++;
                                    ips200_show_string( 0, ((cursor-1)*8), "   " );
                                    if(cursor > 15)
                                        cursor = 0;
                                }
                                if(key_detect(KEY_1, KEY_SHORT_PRESS))
                                {
                                    cursor--;
                                    ips200_show_string( 0, ((cursor+1)*8), "   " );
                                    if(cursor <= -1)
                                        cursor = 15;
                                }
                                if(key_detect(KEY_2, KEY_SHORT_PRESS))
                                    key = 1;
                                ips200_show_string( 0, (cursor*8), ">>>" );
                            }
                            else
                                ips200_show_string( 0, (cursor*8), "---" );

                            // 根据光标位置选择对应变量操作
                            if(key)
                            {
                                switch(cursor)
                                {
                                    case 0:     // 最上一行用来翻面
                                        if(key_detect(KEY_3, KEY_SHORT_PRESS)){
                                            ips200_clear();
                                            page++;}
                                        if(key_detect(KEY_1, KEY_SHORT_PRESS)){
                                            ips200_clear();
                                            page--;}
                                        if(key_detect(KEY_2, KEY_SHORT_PRESS))
                                            key = 0;break;

                                    case 1:
                                        data_operate_uint8(&jump_line);break;
                                    case 2:
                                        data_operate_uint8(&jump_line_slowdown);break;
                                    case 3:
                                        data_operate_uint8(&T1);break;
                                    case 4:
                                        data_operate_uint8(&T2);break;
                                    case 5:
                                        data_operate_uint8(&T3);break;
                                    case 6:
                                        data_operate_uint8(&Bypass_Line);break;
                                    case 7:
                                        data_operate(&yaw_bypass);break;
                                    case 8:
                                        data_operate_uint16(&Bypass_Count1);break;
                                    case 9:
                                        data_operate_uint16(&Bypass_Count2);break;
                                    case 10:
                                        data_operate_uint16(&Bypass_Count3);break;
                                    case 11:
                                        data_operate_uint8(&Single_jg_line);break;
                                    case 12:
                                        data_operate_uint16(&Image_count_Single_End);break;
                                    case 13:
                                        data_operate_uint16(&ramp_judge_dis);break;
                                    case 14:
                                        data_operate_uint16(&ramp_end_dis);break;
                                    case 15:
                                        data_operate_uint16(&ramp_up_delay);break;

                                    default:break;
                                }// page4-cursor end
                            }break;


           default:break;
        }// page allend
        if(key_detect(KEY_4, KEY_SHORT_PRESS))
        {
            // 存入数据
            store_or_read_DATA(WRITE);
            // 读出
            store_or_read_DATA(READ);
            if(Change_Control)
                mt9v03x_init();

            ips200_clear();
            menu_mode = 1;
        }

    }//menu_mode == 0 end
    if( menu_mode == 1 )
    {   // 退出菜单进入正常寻迹模式
        // 菜单很消耗算力，因为有大量的屏幕显示与switch语句
        // 不要开着菜单跑车。

//        /********************************************通用数据显示********************************************/
//        //euler角,角速度
//        ips200_show_string(0 , 0*8 , "roll:");
//        ips200_show_float( 0 , 1*8 , imu660ra.eulerAngle.roll  , 3 , 3 );
//        ips200_show_float( 0 , 2*8 , imu660ra.data_Raw.gyro_x , 3 , 3 );
////        ips200_show_float( 160 , 3*8 , Jarvis.dynamic_roll_angle , 4 , 3 );
//        ips200_show_string(0 , 4*8 , "pitch:");
//        ips200_show_float( 0  , 5*8 , imu660ra.eulerAngle.pitch , 3 , 3 );
//        ips200_show_float( 0 , 6*8 , imu660ra.data_Raw.gyro_y , 3 , 3 );
////        ips200_show_float( 0 , 7*8 , Jarvis.dynamic_pitch_angle , 4 , 3 );
//
//        ips200_show_int( 0, 10*8, Yao.Encoder_Left, 5 );
//        ips200_show_int( 0, 11*8, Yao.Encoder_Right, 5 );
////        ips200_show_int( 0, 12*8, motor_value.receive_left_speed_data, 5 );
////        ips200_show_int( 0, 13*8, motor_value.receive_right_speed_data, 5 );
//
//        ips200_show_float( 0 , 38*8 , Battery_voltage , 4 , 3 );
//        /********************************************通用数据显示********************************************/
//        if(key_detect(KEY_2, KEY_SHORT_PRESS))
//        {   // 退出所有元素
//            Element_Break_Flag = 1;
//            ips200_clear();
//        }
        if(key_detect(KEY_3, KEY_SHORT_PRESS))
        {   // 倒地保护后重新发车
            flag_stop = !flag_stop;
            ips200_clear();
        }
        if(key_detect(KEY_4, KEY_SHORT_PRESS))
        {   // 需要调参，回到菜单页面
            menu_mode = 0;
            ips200_clear();
        }

    }//menu_mode == 1 end

}
//----------------------------------------------------------------
//  @brief      使用逐飞库自封装的按键检测，自清零。
//  @param      key_n     目标按键，定义在.h
//  @param      state     目标状态，定义在.h
//  @return     int       0为未检测到，1为检测到目标状态
//  @note       write by laterain, 2025.2.19
//              KEY_LONG_PRESS和KEY_ONCE_LONG_PRESS不能同时检测
//----------------------------------------------------------------
int key_detect(key_index_enum key_n, key_state_enum state)
{
    static uint8 flag = 0;
    switch(state)
    {
        case KEY_RELEASE:break;
        case KEY_SHORT_PRESS:
            if(key_get_state(key_n) == KEY_SHORT_PRESS)
            {
                key_clear_state(key_n);
                return 1;
            }break;
        case KEY_LONG_PRESS:
            if(key_get_state(key_n) == KEY_LONG_PRESS)
            {
                key_clear_state(key_n);
                return 1;
            }break;
        case KEY_ONCE_LONG_PRESS:
            if(key_get_state(key_n) == KEY_LONG_PRESS && !flag)
            {
                flag = 1;
                key_clear_state(key_n);
                return 1;
            }
            if(key_get_state(key_n) == KEY_RELEASE)
                flag = 0;break;
        default:break;
    }

    return 0;

}


