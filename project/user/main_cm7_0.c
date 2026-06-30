/*********************************************************************************************************************
 * CYT4BB 开源库（CYT4BB Opensource Library），一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 CYT4BB 开源库的一部分
 *
 * CYT4BB 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 以 GPL 的第3版（即 GPL3.0）或任何更新的版本发布/修改。
 *
 * 本开源库的发布是希望它能发挥作用，但未做任何保证
 * 也没有适销性或特定用途的隐含保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅 <https://www.gnu.org/licenses/>
 *
 * 文件名称：main_cm7_0
 * 公司名称：成都逐飞科技有限公司
 * 版本信息：查看 libraries/doc 文件夹内的 version 文件 版本说明
 * 修改日期：IAR 9.40.1
 * 开发平台：CYT4BB
 * 官方店铺：https://seekfree.taobao.com/
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
#include "zf_device_gnss.h"
#include "gps_nav.h"
#include "gps_waypoint.h"
#include "remote_control.h"
#include "yaokong.h"
#include "zf_device_lora3a22.h"

// **************************** 主函数入口 ****************************

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化 <必须放在第一行>
    debug_init();                  // 调试串口信息初始化

    Init_All(); // 全部外设初始化（从 TC264 cpu0_main.c 移植）


    imu660ra.offset_angle.pitch = -5.33; // 俯仰角零偏校准值
    imu660ra.offset_angle.roll = -13.0;  // 横滚角零偏校准值
    int16_t flag_main = 0;
 ;   
    while (true)
    {
        // GPS 导航处理：gnss_flag 由 ISR 置位，主循环中处理
        if(gnss_flag)
        {
            gnss_flag = 0;
            gps_nav_proc();
        }

        // GPS 航点采集控制：menu_open==0 时响应按键（避免菜单冲突）
        if(menu_open == 0)
        {
            // KEY_1 短按：采集当前 GPS 坐标作为航点
            if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
            {
                key_clear_state(KEY_1);
                if(gnss.state == 1 && gnss.satellite_used >= 4)
                {
                    if(gps_wp_add(gnss.latitude, gnss.longitude))
                    {
                        gps_wp_save_to_flash();  // 采集后自动保存到 Flash
                    }
                }
            }

            // KEY_2 短按：启动/停止导航切换
            if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
            {
                key_clear_state(KEY_2);
                if(gps_nav_get_state() == GPS_NAV_IDLE)
                {
                    gps_nav_start();  // IDLE -> CALIBRATING
                }
                else
                {
                    gps_nav_stop();   // 任意导航态 -> IDLE
                }
            }
        }

        printf ("joystick[1] = %d\r\n",lora3a22_uart_transfer.joystick[1]);

        //IPS200_Show1();
        IPS200_ShowGPS();   // GPS导航屏幕调试显示 (10Hz局部刷新)
        // 此处可编写需要循环执行的代码
        // 注意: 所有硬件中断和 PID 计算都在中断和遥控模块中运行
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
            small_driver_set_duty((int16)(Yao.Outp_Gyro_Pitch),   // 左轮占空比
                                  (int16)(-Yao.Outp_Gyro_Pitch)); // 右轮占空比
        }
     //   printf("A: %f,%f,%f,%d\n", imu660ra.eulerAngle.pitch, imu660ra.eulerAngle.roll, imu660ra.eulerAngle.yaw, imu660ra_acc_y);
        }

        static uint8 dbg = 0;
        if(++dbg >= 10) 
        { dbg = 0;
            printf("GPS: s=%d, sat=%d, lat=%.6f, lng=%.6f\r\n",
            gnss.state, gnss.satellite_used, gnss.latitude, gnss.longitude);
        }

        if(gps_wp_get_count() > 0)
         {
            gps_waypoint_t *wp = gps_wp_current();
            float bear = get_two_points_azimuth(gnss.latitude, gnss.longitude, wp->lat, wp->lng);
            float dist = get_two_points_distance(gnss.latitude, gnss.longitude, wp->lat, wp->lng);
            printf("CALC: bear=%.1f, dist=%.2f\r\n", bear, dist);
         }
    }

    // **************************** 主函数结束 ****************************
