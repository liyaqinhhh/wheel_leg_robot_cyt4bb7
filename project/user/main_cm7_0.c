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

/**
 * @brief   主函数入口
 *
 * 系统启动流程：
 *   1. 时钟初始化（250MHz）—— 必须首先执行
 *   2. 调试串口初始化 —— 用于printf输出
 *   3. 所有外设初始化（Init_All）—— 包括IMU、电机、屏幕、舵机等
 *   4. IMU角度偏移校准 —— 手动设置pitch/roll零点偏移
 *   5. 主循环：屏幕显示 + 控制输出 + 惯导导航
 *
 * 主循环任务（每次循环执行）：
 *   - IPS200_Show1(): 刷新屏幕显示（姿态角、PID参数、菜单等）
 *   - control_main(): 电机控制输出（将PID结果发送到电机驱动板）
 *   - ins_navigation(): 惯导导航（ins_open=1时执行航点跟踪）
 *
 * @return  理论上永不返回（while(true)死循环）
 */
int main(void)
{
    /* ===== 第一阶段：系统基础初始化 ===== */
    
    clock_init(SYSTEM_CLOCK_250M); /* 时钟配置：250MHz主频 <务必保留，首先执行> */
    debug_init();                  /* 调试串口初始化：用于printf调试输出 */

    /* ===== 第二阶段：外设初始化 ===== */
    
    Init_All(); /* 所有外设初始化（从 TC264 cpu0_main.c 移植）
                 * 包括：IMU、编码器电机、IPS屏幕、舵机、按键等
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

  
    
    
    /* ===== 第四阶段：主循环 ===== */
    uint8_t sum;
    while (true)
    {
        /* 1. 屏幕显示刷新
         * 显示内容：姿态角、PID参数、菜单界面、电池电压等
         */
        IPS200_Show1();
        
        /* 2. 电机控制输出
         * 将PID计算结果（Yao.Outp_Gyro_Pitch/Yaw）发送到电机驱动板
         * 包含安全保护：速度超限或flag_main=1时停止输出
         */
        control_main();
         
          // sum +=motor_value.receive_right_speed_data;
          // printf("sum: %d\n", sum);
       

     }
}
// **************************** 代码区域 ****************************
