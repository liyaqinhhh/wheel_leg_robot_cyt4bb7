/*
 * yaokong.c
 *
 *  Created on: 2024Äê9ÔÂ7ÈÕ
 *      Author: LateRain
 */

#ifndef CODE_PERIPHERAL_YAOKONG_H_
#define CODE_PERIPHERAL_YAOKONG_H_

#include "zf_common_headfile.h"

extern int velocity;
extern float speed_yk;

void yaokong_map_joystick(int16_t joystick_1, int16_t joystick_2);
uint8 yaokong_data_deal(void);

#endif /* CODE_PERIPHERAL_YAOKONG_H_ */
