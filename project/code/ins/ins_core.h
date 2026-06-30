/*
 * ins_core.h
 * INS 核心数据结构与 API 声明
 * 功能：XY 坐标累加 + ZUPT 零速检测，姿态复用 kalman.c
 */

#ifndef _INS_CORE_H_
#define _INS_CORE_H_

#include "zf_common_typedef.h"

 //------------------------------------------- 参数宏（待实测后修改）-------------------------------------------
#define INS_PI               3.14159265358979f
#define INS_DEG2RAD          0.017453292519943f
#define INS_RAD2DEG          57.2957795130823f

#define INS_WHEELBASE_M      0.0f   // 【待实测】轮距/腿间距（米）
#define INS_TICK_TO_METER    0.0f   // 【待实测】编码器 tick 转米系数（m/tick）

#define INS_ZUPT_SPEED_THRESH   0.05f   // ZUPT 速度阈值（m/s），低于此值视为静止
#define INS_ZUPT_GYRO_THRESH    0.05f   // ZUPT 角速度阈值（rad/s），低于此值视为静止

 //------------------------------------------- 数据结构 --------------------------------------------------------
typedef struct
{
    float x;          // X 坐标（米）
    float y;          // Y 坐标（米）
    float yaw;        // 偏航角（弧度）
    float pitch;      // 俯仰角（弧度），从 kalman.c 读取
    float roll;       // 横滚角（弧度），从 kalman.c 读取
} INS_State;

typedef struct
{
    float v_mps;           // 机体前进速度（m/s），由编码器计算
    float gyro_z_rad_s;    // 陀螺仪 Z 轴角速度（rad/s），用于 ZUPT 判断
} INS_Input;

 //------------------------------------------- API 声明 --------------------------------------------------------
// @brief  INS核心模块初始化，复位坐标和姿态状态
void ins_core_init(void);
// @brief  INS核心更新函数，根据输入速度和陀螺仪数据进行航位推算
// @param  input  传感器输入（速度m/s + 陀螺仪Z轴角速度rad/s）
// @param  dt_s   时间步长（秒）
void ins_core_update(const INS_Input *input, float dt_s);
// @brief  重置INS状态到指定位置和朝向
// @param  x      X坐标（米）
// @param  y      Y坐标（米）
// @param  theta  朝向角（弧度）
void ins_core_reset(float x, float y, float theta);
// @brief  获取当前INS状态的只读指针
// @return 指向内部INS_State结构体的const指针
const INS_State* ins_core_get_state(void);

#endif /* _INS_CORE_H_ */
