#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"

// 移植：UART_2 已被 GPS 模块占用，改用 UART_3
// 移植：P33 端口在 CYT4BB7 不存在，改用 P17
#define SMALL_DRIVER_UART (UART_2)          // 无刷驱动通信使用的串口通道

#define SMALL_DRIVER_BAUDRATE (460800)      // 无刷驱动串口波特率

#define SMALL_DRIVER_RX (UART2_TX_P10_1) // 无刷驱动 串口接收引脚

#define SMALL_DRIVER_TX (UART2_RX_P10_0) // 无刷驱动 串口发送引脚

// @brief  无刷驱动通信数据结构体（包含收发缓冲区、校验位和电机速度数据）
typedef struct
{
    uint8 send_data_buffer[7]; // 发送缓冲数组

    uint8 receive_data_buffer[7]; // 接收缓冲数组

    uint8 receive_data_count; // 接收计数

    uint8 sum_check_data; // 校验位

    int16 receive_left_speed_data; // 接收到的左侧电机速度数据

    int16 receive_right_speed_data; // 接收到的右侧电机速度数据

} small_device_value_struct;

extern small_device_value_struct motor_value;  // 无刷驱动全局数据实例
extern uint16_t num;                           // 通用计数变量1
extern uint16_t num_1;                         // 通用计数变量2

// @brief  无刷驱动 串口接收回调函数
void uart_control_callback(void); // 无刷驱动 串口接收回调函数

// @brief  无刷驱动 设置电机占空比
void small_driver_set_duty(int16 left_duty, int16 right_duty); // 无刷驱动 设置电机占空比

// @brief  无刷驱动 获取速度信息
void small_driver_get_speed(void); // 无刷驱动 获取速度信息

// @brief  无刷驱动 串口通讯初始化
void small_driver_uart_init(void); // 无刷驱动 串口通讯初始化

#endif
