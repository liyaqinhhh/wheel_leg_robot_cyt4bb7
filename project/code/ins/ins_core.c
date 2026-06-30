/*
 * ins_core.c
 * INS 核心：XY 坐标累加 + ZUPT 零速检测
 * 姿态直接复用 kalman.c 的 imu.roll/pitch/yaw（度）
 */

#include "ins_core.h"
#include "kalman.h"
#include <math.h>
#include <string.h>

 //------------------------------------------- 内部变量 --------------------------------------------------------
static INS_State s_state = {0};
static float s_pos_x = 0.0f;
static float s_pos_y = 0.0f;
static uint8 s_initialized = 0u;

 //------------------------------------------- 内部函数 --------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ZUPT 零速检测
//  @param      input       INS 输入数据
//  @return     1-静止 0-运动
//-------------------------------------------------------------------------------------------------------------------
static uint8 ins_is_stationary(const INS_Input *input)
{
    if (input == NULL)
        return 1u;

    if (fabsf(input->v_mps) < INS_ZUPT_SPEED_THRESH &&
        fabsf(input->gyro_z_rad_s) < INS_ZUPT_GYRO_THRESH)
    {
        return 1u;
    }
    return 0u;
}

 //------------------------------------------- 公共 API --------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  @brief      初始化 INS 核心
//  @param      void
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void ins_core_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_pos_x = 0.0f;
    s_pos_y = 0.0f;
    s_initialized = 1u;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      更新 INS 状态（每 4ms 调用一次）
//  @param      input       INS 输入数据（速度、角速度）
//  @param      dt_s        时间间隔（秒）
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void ins_core_update(const INS_Input *input, float dt_s)
{
    if (!s_initialized)
        ins_core_init();

    if (input == NULL || dt_s <= 0.0f)
        return;

    // 1. 从现有 kalman.c 获取 yaw（度→弧度）
    float yaw_rad = imu.yaw * INS_DEG2RAD;

    // 2. ZUPT：静止时清零速度
    float v = input->v_mps;
    if (ins_is_stationary(input))
    {
        v = 0.0f;
    }

    // 3. 位置推算：dx = v * cos(yaw) * dt, dy = v * sin(yaw) * dt
    s_pos_x += v * cosf(yaw_rad) * dt_s;
    s_pos_y += v * sinf(yaw_rad) * dt_s;

    // 4. 更新状态（姿态直接从 imu 结构体读取）
    s_state.x = s_pos_x;
    s_state.y = s_pos_y;
    s_state.yaw = yaw_rad;
    s_state.pitch = imu.pitch * INS_DEG2RAD;
    s_state.roll = imu.roll * INS_DEG2RAD;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      重置 INS 状态（设置原点）
//  @param      x           初始 X 坐标（米）
//  @param      y           初始 Y 坐标（米）
//  @param      theta       初始航向角（弧度）
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void ins_core_reset(float x, float y, float theta)
{
    if (!s_initialized)
        ins_core_init();

    s_state.yaw = theta;
    s_pos_x = x;
    s_pos_y = y;
    s_state.x = x;
    s_state.y = y;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取当前 INS 状态
//  @param      void
//  @return     const INS_State* 状态指针（只读）
//-------------------------------------------------------------------------------------------------------------------
const INS_State* ins_core_get_state(void)
{
    return &s_state;
}
