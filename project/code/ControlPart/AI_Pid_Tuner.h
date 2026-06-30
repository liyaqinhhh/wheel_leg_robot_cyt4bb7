#ifndef AI_PID_TUNER_H
#define AI_PID_TUNER_H

#include "zf_common_headfile.h"

// 外部变量声明
// @brief  AI调参开关标志: 1=开启 0=关闭
extern uint16_t flag_ai_open;

// 函数声明
// @brief  AI PID调参模块初始化，配置通信接口和初始参数
void AI_Pid_Tuner_Init(void);
// @brief  向上位机发送当前PID运行数据（误差、输出等）
void AI_Pid_Tuner_SendData(void);
// @brief  处理上位机下发的PID参数调整指令
void AI_Pid_Tuner_ProcessRx(void);

#endif
