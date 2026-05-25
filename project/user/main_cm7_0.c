/*********************************************************************************************************************
 * CYT4BB 开源库（CYT4BB Opensource Library），一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 CYT4BB 开源库的一部分
 *
 * CYT4BB 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅 <https://www.gnu.org/licenses/>
 *
 * 文件名称：main_cm7_0
 * 公司名称：成都逐飞科技有限公司
 * 版本信息：查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境：IAR 9.40.1
 * 适用平台：CYT4BB
 * 店铺链接：https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2024-1-4       pudding            first version
 ********************************************************************************************************************/

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

// **************************** 代码区域 ****************************

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化 <务必保留>
    debug_init();                  // 调试串口信息初始化

    Init_All(); // 所有外设初始化（从 TC264 cpu0_main.c 移植）
    // servo_init();
    /*ips200_set_font(IPS200_6X8_FONT);                   // 设置字体大小为 6 * 8像素
        ips200_set_color(RGB565_BLACK, RGB565_WHITE);       // 设置颜色为彩色
        ips200_set_dir(IPS200_PORTAIT);                     // 设置为竖屏显示
        ips200_init(IPS200_TYPE_SPI);                 // 双排并口款式*/
    // uint32 i = 0;

    imu660ra.offset_angle.pitch = -5.33; // �??5�??.1
    imu660ra.offset_angle.roll = -13.0; // you'i'h'h'h
    int16_t flag_main = 0;
    while (true)
    {
        IPS200_Show1();
        // 此处编写需要循环执行的代码
        // small_driver_get_speed();
        // ips200_show_string( 20, 0*8, "page: 333333333" );youihhhh
        /*key_scanner();
         menu();*/
        /* if (key_detect(KEY_2, KEY_SHORT_PRESS))
         i++;
         ips200_show_uint(  100 , 13*8, i, 3 );*/
        // imu660rb_get_gyro();
        // imu660rb_get_acc();

        //            small_driver_set_duty(0,1000);
        //            if(turn_mode == 0)
        //            {
         Yao.Outp_Gyro_Pitch = 0;
            Yao.Outp_Angle_Pitch = 0;
            Yao.Outp_Speed_Pitch = 0;
        if (motor_value.receive_right_speed_data < -3000 || motor_value.receive_right_speed_data > 3000 || motor_value.receive_left_speed_data < -3000 || motor_value.receive_left_speed_data > 3000 || -Yao.Outp_Gyro_Pitch > 7000 || Yao.Outp_Gyro_Pitch > 7000)
        {
            //            if(!flag_jump_stop)
            flag_main = 1;
            flag_stop = 1;
            Yao.Outp_Gyro_Pitch = 0;
            Yao.Outp_Angle_Pitch = 0;
            Yao.Outp_Speed_Pitch = 0;
            //            motor_value.receive_left_speed_data = 0;
            //            motor_value.receive_right_speed_data = 0;
            small_driver_set_duty(0, 0);
        }

        if (flag_main)
        {
            flag_stop = 1;
            Yao.Outp_Gyro_Pitch = 0;
            Yao.Outp_Angle_Pitch = 0;
            Yao.Outp_Speed_Pitch = 0;
            //            motor_value.receive_left_speed_data = 0;
            //            motor_value.receive_right_speed_data = 0;
            small_driver_set_duty(0, 0);
        }
        else
        {
            small_driver_set_duty((int16)(Yao.Outp_Gyro_Pitch),   // 左轮发送占空比
                                  (int16)(-Yao.Outp_Gyro_Pitch)); // 右轮发送占空比
        }
        printf("A: %f,%f,%f,%d\n", imu660ra.eulerAngle.pitch, imu660ra.eulerAngle.roll, imu660ra.eulerAngle.yaw, imu660ra_acc_y);
        }

        // 向前:leftduty(+),rightduty(-)
        // small_driver_set_duty(500,0);

        // printf("A: %f,%f,%d,%d\n", imu660ra.eulerAngle.pitch, imu660ra.eulerAngle.roll, flag_stop, imu660ra_gyro_x );
        //            }
        //            else
        //            {
        //                if(Yao.Outp_Gyro_Yaw > 0)
        //                small_driver_set_duty( (int16)(Yao.Outp_Gyro_Pitch + Yao.Outp_Gyro_Yaw),      //左轮发送占空比
        //                                       (int16)(Yao.Outp_Gyro_Pitch)  );   //右轮发送占空比
        //                else
        //                    small_driver_set_duty( (int16)(Yao.Outp_Gyro_Pitch),      //左轮发送占空比
        //                                           (int16)(Yao.Outp_Gyro_Pitch - Yao.Outp_Gyro_Yaw)  );   //右轮发送占空比
        //            }

        // small_driver_set_duty(500,-500);
        // ins_navigation();
    }

    // **************************** 代码区域 ****************************
