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
 * 店铺链接：https://seekfree.taobao.com/*/

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
#include "AI_Pid_Tuner.h"
#include "kalman.h"

// **************************** 代码区域 ****************************

/**
 * @brief 外部变量声明
 *
 * y1: 俯仰角速度低通滤波值（来自Interrupt.c的2ms中断）
 *     用于AI调参时发送实时角速度数据到上位机
 */
extern volatile float y1;

int main(void)
{

  clock_init(SYSTEM_CLOCK_250M); /* 时钟配置：250MHz主频 <务必保留，首先执行> */
  debug_init();                  /* 调试串口初始化：用于printf调试输出 */

  /* ===== 第二阶段：外设初始化 ===== */

  Init_All(); /* 所有外设初始化（从 TC264 cpu0_main.c 移植）
              /* 包括：IMU、编码器电机、IPS屏幕、舵机、按键等
               */
  // servo_init(); /* 舵机单独初始化（已在Init_All中调用，此处注释） */

  /* IPS200屏幕初始化示例（已注释，保留参考） */
  /*ips200_set_font(IPS200_6X8_FONT);                   // 设置字体大小为 6 * 8像素
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);       // 设置颜色为彩色
    ips200_set_dir(IPS200_PORTAIT);                     // 设置为竖屏显示
    ips200_init(IPS200_TYPE_PARALLEL8);                 // 双排并口款式*/

  // uint32 i = 0;
  // debug_init();
  // extern int16_t flag_main;
  // gpio_init(P19_0,GPO,1,GPO_PUSH_PULL);
  /* ===== 第四阶段：主循环 ===== */
  // uint8_t sum;
  //  pit_ms_init(PIT_CH0, 1);
  //  imu660rb_init();
  // servo_init();
  //  ips200_set_font(IPS200_6X8_FONT);             // 璁剧疆瀛椾綋澶у皬涓?6 * 8鍍忕�?
  //  ips200_set_color(RGB565_BLACK, RGB565_WHITE); // 璁剧疆棰滆壊涓哄僵鑹?
  //  ips200_set_dir(IPS200_PORTAIT);               // 璁剧疆涓虹珫灞忔樉绀?
  //  ips200_init(IPS200_TYPE_SPI);           // 鍙屾帓骞跺彛娆惧�?
  //  servo_init();
  //  small_driver_uart_init();
  while (true)
  {
    /* 0. Roll 零点独立调参模式：flag_main_test==1 时运行调参菜单 */
    if (flag_main_test)
    {
      roll_tune_menu();
      continue;
    }

    /* 1. 中断任务调度（将原 ISR 中的定时任务迁至主函数轮询执行）
     * 每次调用检查各时间级标志位，有积压则执行一次并递减
     */
    Run_Interrupt_Tasks();
    // small_driver_set_duty(-500,500);
    // Interrupt_40ms();
    //  imu660rb_get_gyro();
    //  imu660rb_get_acc();
    // printf("imu660rb_gyro_z: %d, imu660rb_gyro_x: %d,imu660rb_gyro_y: %d,imu660rb_acc_z: %d\n", imu660rb_gyro_z, imu660rb_gyro_x, imu660rb_gyro_y, imu660rb_acc_z);
    /* AI调参示例：发送实时俯仰角速度数据到上位机 */

    //    while(1)
    // {
    //     if(imu660rb_init())
    //     {
    //        printf("\r\n imu660rb init error.");                                 // imu660rb 初始化失败
    //     }
    //     else
    //     {
    //        break;
    //     }
    //     //gpio_toggle_level(LED1);                                                // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
    // }
    // 此处编写用户代码 例如外设初始化代码等
  }
}
// **************************** 代码区域 ****************************
