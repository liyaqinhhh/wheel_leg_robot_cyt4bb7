/*
 * yaokong.c
 *
 *  Created on: 2024年9月7日
 *      Author: LateRain
 */
#include "zf_common_headfile.h"
#include "yaokong.h"
#include "Interrupt.h"
#include "image.h"
#include "menu.h"

float speed_yk;

// lora3a22 模块在 CYT4BB7 工程中未启用，函数体注释保留
uint8 yaokong_data_deal(void) {
//    // 前后
//    if (lora3a22_uart_transfer.joystick[1] > 200)
//        Yao.Target_Speed = 1000;
//    else if (lora3a22_uart_transfer.joystick[1] < -200)
//        Yao.Target_Speed = (int)(0.2f * lora3a22_uart_transfer.joystick[1]);
//    else
//        Yao.Target_Speed = 0;
//
//    // 左右
//    if (lora3a22_uart_transfer.joystick[2] > 100)
//        Deviation_Value = (float)( -lora3a22_uart_transfer.joystick[2]/1800.0f);
//    else if (lora3a22_uart_transfer.joystick[2] < -100)
//        Deviation_Value = (float)( -lora3a22_uart_transfer.joystick[2]/1800.0f);
//    else
//        Deviation_Value = 0;
//
//    Deviation_Value = func_limit_ab( Deviation_Value , -1.0f, 1.0f );
//    lora3a22_finsh_flag = 0;
//    return 1;
    return 0;
}



