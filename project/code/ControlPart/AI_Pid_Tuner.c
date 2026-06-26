#include "AI_Pid_Tuner.h"
#include "Interrupt.h"
#include "PID.h"
#include "imu660.h"
#include "zf_device_wireless_uart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// AI调参功能开关变量定义
uint16_t flag_ai_open = 0;

// 串口接收缓冲区
static char rx_buf[256];
static uint16_t rx_idx = 0;

// 外部变量声明
extern float speed_MOTOR;
extern float wheel_distance_cm; 
extern int16_t flag_main;
extern float erect_Gyro_Pitch[4];
extern float erect_Angle_Pitch[4];
extern float erect_Speed_Pitch[4];
extern PID_ERECT PID_all;

// PID积分项复位函数声明
extern void pid_para_init(PID_INFO *pid);

/**
 * @brief 初始化AI调参模块
 */
void AI_Pid_Tuner_Init(void)
{
    flag_ai_open = 0;
    rx_idx = 0;
    memset(rx_buf, 0, sizeof(rx_buf));
}

/**
 * @brief 发送实时数据到PC端（JSON格式）
 * @note 数据格式: {"pitch":%.2f,"speed_out":%.2f,"angle_out":%.1f,"gyro_out":%.1f,"motor_speed":%.1f,"flag_main":%d}
 */
void AI_Pid_Tuner_SendData(void)
{
    char tx_buf[256];
    int len;

    // 构建JSON格式数据
    len = snprintf(tx_buf, sizeof(tx_buf),
                   "{\"pitch\":%.2f,\"speed_out\":%.2f,\"angle_out\":%.1f,\"gyro_out\":%.1f,\"motor_speed\":%.1f,\"flag_main\":%d}\n",
                   imu660ra.eulerAngle.pitch,
                   Yao.Outp_Speed_Pitch,
                   Yao.Outp_Angle_Pitch,
                   Yao.Outp_Gyro_Pitch,
                   speed_MOTOR,
                   flag_main);

    // 通过无线串口发送
    if (len > 0 && len < sizeof(tx_buf))
    {
        wireless_uart_send_string(tx_buf);
    }
}

/**
 * @brief 处理接收到的PID参数
 * @note 接收格式: P:kp_a,ki_a,kd_a,kp_g,ki_g,kd_g,kp_s,ki_s,kd_s,offset_roll
 */
void AI_Pid_Tuner_ProcessRx(void)
{
    uint8_t ch;
    uint32_t read_len;

    // 读取串口数据
    while (1)
    {
        read_len = wireless_uart_read_buffer(&ch, 1);
        if (read_len == 0)
        {
            break;
        }

        // 检测换行符，表示一帧数据结束
        if (ch == '\n' || ch == '\r')
        {
            if (rx_idx > 0)
            {
                rx_buf[rx_idx] = '\0';

                // 解析参数
                float kp_angle, ki_angle, kd_angle;
                float kp_gyro, ki_gyro, kd_gyro;
                float kp_speed, ki_speed, kd_speed;
                float offset_roll;

                // 解析格式: P:kp_a,ki_a,kd_a,kp_g,ki_g,kd_g,kp_s,ki_s,kd_s,offset_roll
                if (sscanf(rx_buf, "P:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                           &kp_angle, &ki_angle, &kd_angle,
                           &kp_gyro, &ki_gyro, &kd_gyro,
                           &kp_speed, &ki_speed, &kd_speed,
                           &offset_roll) == 10)
                {
                    // 更新角度环参数
                    erect_Angle_Pitch[0] = kp_angle;
                    erect_Angle_Pitch[1] = ki_angle;
                    erect_Angle_Pitch[2] = kd_angle;

                    // 更新角速度环参数
                    erect_Gyro_Pitch[0] = kp_gyro;
                    erect_Gyro_Pitch[1] = ki_gyro;
                    erect_Gyro_Pitch[2] = kd_gyro;

                    // 更新速度环参数
                    erect_Speed_Pitch[0] = kp_speed;
                    erect_Speed_Pitch[1] = ki_speed;
                    erect_Speed_Pitch[2] = kd_speed;

                    // 更新机械零点
                    imu660ra.offset_angle.roll = offset_roll;

                    // 复位PID积分项，避免积分累积导致冲击
                    pid_para_init(&PID_all.Pid_Angle_Pitch);
                    pid_para_init(&PID_all.Pid_Gyro_Pitch);
                    pid_para_init(&PID_all.Pid_Speed_Pitch);

                    // 发送确认信息
                    wireless_uart_send_string("{\"status\":\"ok\",\"msg\":\"params updated\"}\n");
                }
                else
                {
                    // 解析失败，发送错误信息
                    wireless_uart_send_string("{\"status\":\"error\",\"msg\":\"parse failed\"}\n");
                }

                // 重置接收缓冲区
                rx_idx = 0;
                memset(rx_buf, 0, sizeof(rx_buf));
            }
        }
        else
        {
            // 存储接收到的字节
            if (rx_idx < sizeof(rx_buf) - 1)
            {
                rx_buf[rx_idx++] = ch;
            }
            else
            {
                // 缓冲区溢出，重置
                rx_idx = 0;
                memset(rx_buf, 0, sizeof(rx_buf));
            }
        }
    }
}
