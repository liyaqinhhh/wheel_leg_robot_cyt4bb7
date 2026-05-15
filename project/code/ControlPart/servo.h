/*
 * servo.h
 *
 *  Created on: 2025年2月18日
 *      Author: LateRain
 */

#ifndef CODE_CONTROLPART_SERVO_H_
#define CODE_CONTROLPART_SERVO_H_

typedef enum
{
    LF = 0,
    LB = 1,
    RF = 2,
    RB = 3,
}leg_enum;

extern uint32 pwm;
extern uint32 pwmLF;
extern uint32 pwmLB;
extern uint32 pwmRF;
extern uint32 pwmRB;
extern uint8 flag_jump;
extern uint8 flag_jump_1;
extern uint16 time_j;
extern uint8 flag_jump_stop;
extern uint8 T1;          // 上升阶段持续时间
extern uint8 T2;         // 收腿阶段持续时间
extern uint8 T3;          // 放腿阶段持续时间
extern float Single_Height;

void servo_init(void);
void servo_set_angle( leg_enum leg, float angle );
void jump_control(void);
void servo_set_pwm(leg_enum leg, int16 er_pwm);
void servo_balance(void);
void Single_Control(void);

#endif /* CODE_CONTROLPART_SERVO_H_ */
