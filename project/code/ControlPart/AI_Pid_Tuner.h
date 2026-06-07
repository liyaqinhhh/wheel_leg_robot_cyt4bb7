#ifndef AI_PID_TUNER_H
#define AI_PID_TUNER_H

#include "zf_common_headfile.h"

// 外部变量声明
extern uint16_t flag_ai_open;

// 函数声明
void AI_Pid_Tuner_Init(void);
void AI_Pid_Tuner_SendData(void);
void AI_Pid_Tuner_ProcessRx(void);

#endif
