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
#include "zf_device_lora3a22.h"

float speed_yk;

void yaokong_map_joystick(int16_t joystick_1, int16_t joystick_2)
{
    /* 前后速度：死区 ±200，满速约 1000 */
    if (joystick_1 > 200)
        Yao.Target_Speed = 1000;
    else if (joystick_1 < -200)
        Yao.Target_Speed = (int)(0.2f * joystick_1);
    else
        Yao.Target_Speed = 0;

    /* 左右转向：死区 ±100，限幅 ±1.0 */
    if (joystick_2 > 100 || joystick_2 < -100)
        Deviation_Value = -((float)joystick_2 / 1800.0f);
    else
        Deviation_Value = 0.0f;

    Deviation_Value = func_limit_ab(Deviation_Value, -1.0f, 1.0f);
}

uint8 yaokong_data_deal(void)
{
    /* 保留老接口，兼容旧调用；实际遥控路径走 yaokong_map_joystick() */
    if (lora3a22_finsh_flag)
    {
        yaokong_map_joystick((int16_t)lora3a22_uart_transfer.joystick[1],
                             (int16_t)lora3a22_uart_transfer.joystick[2]);
    }

    lora3a22_finsh_flag = 0;
    return 1;
}



