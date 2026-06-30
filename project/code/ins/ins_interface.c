/*
 * ins_interface.c
 * INS 对外 API 实现
 *
 * 本文件将底层 ins_core / ins_track 功能封装为高层 API，
 * 供 main 循环或 Interrupt 调用，屏蔽内部实现细节。
 *
 * API 分组:
 *   校准相关:  陀螺仪零偏校准、保存、加载
 *   坐标系管理: 设置原点、设置航向角
 *   轨迹记录:  开始/停止/清除录制
 *   轨迹循迹:  开始/停止循迹、设置循迹速度
 *   状态查询:  获取 INS 状态、轨迹点数、录制/循迹状态
 *   转向输出:  获取循迹转向角
 */

#include "ins_interface.h"
#include "ins_track.h"
#include "Interrupt.h"
#include "imu660.h"
#include "zf_driver_flash.h"
#include <math.h>

 //------------------------------------------- 内部变量 --------------------------------------------------------
static uint8 s_gyro_calib_active = 0u;       // 校准进行中标志
static uint32 s_gyro_calib_count = 0u;       // 已采样次数
static float s_gyro_sum_x = 0.0f;            // 陀螺仪 X 累计
static float s_gyro_sum_y = 0.0f;            // 陀螺仪 Y 累计
static float s_gyro_sum_z = 0.0f;            // 陀螺仪 Z 累计
#define GYRO_CALIB_SAMPLES  500              // 校准采样次数

static uint8 s_saved_turn_mode = 2;          // 循迹前保存的 turn_mode
static float s_follow_speed = 0.35f;         // 循迹速度（m/s）

 //------------------------------------------- Flash 页分配 ----------------------------------------------------
// 页 25：IMU 零偏参数（与轨迹页不冲突）
#define INS_FLASH_IMU_BIAS_PAGE   25u

 //------------------------------------------- 校准相关 --------------------------------------------------------

// @brief  启动陀螺仪零偏校准（清零累计器，设置活跃标志）
void ins_api_start_gyro_calib(void)
{
    s_gyro_calib_active = 1u;
    s_gyro_calib_count = 0u;
    s_gyro_sum_x = 0.0f;
    s_gyro_sum_y = 0.0f;
    s_gyro_sum_z = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      校准处理（在 2ms 中断中调用 date_handle 后调用）
//  @param      void
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void ins_api_calib_process(void)
{
    if (!s_gyro_calib_active)
        return;

    s_gyro_sum_x += imu660ra.data_Raw.gyro_x;
    s_gyro_sum_y += imu660ra.data_Raw.gyro_y;
    s_gyro_sum_z += imu660ra.data_Raw.gyro_z;
    s_gyro_calib_count++;

    if (s_gyro_calib_count >= GYRO_CALIB_SAMPLES)
    {
        // 计算均值作为零偏
        imu660ra.offset_angle.pitch = s_gyro_sum_y / (float)GYRO_CALIB_SAMPLES;
        imu660ra.offset_angle.roll = s_gyro_sum_x / (float)GYRO_CALIB_SAMPLES;
        // yaw 零偏暂不使用（通过 INS 推算）
        s_gyro_calib_active = 0u;
    }
}

// @brief  保存 IMU 零偏到 Flash（页25，2个 float）
void ins_api_save_imu_bias(void)
{
    flash_union_buffer[0].float_type = imu660ra.offset_angle.pitch;
    flash_union_buffer[1].float_type = imu660ra.offset_angle.roll;
    flash_erase_page(0, INS_FLASH_IMU_BIAS_PAGE);
    flash_write_page_from_buffer(0, INS_FLASH_IMU_BIAS_PAGE, 2);
    flash_buffer_clear();
}

// @brief  从 Flash 加载 IMU 零偏（含 NaN 和范围校验）
void ins_api_load_imu_bias(void)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(0, INS_FLASH_IMU_BIAS_PAGE, 2);

    float pitch_bias = flash_union_buffer[0].float_type;
    float roll_bias = flash_union_buffer[1].float_type;
    flash_buffer_clear();

    // 简单有效性检查（零偏不应超过 ±50 度）
    if (fabsf(pitch_bias) < 50.0f && fabsf(roll_bias) < 50.0f &&
        pitch_bias == pitch_bias && roll_bias == roll_bias)  // 排除 NaN
    {
        imu660ra.offset_angle.pitch = pitch_bias;
        imu660ra.offset_angle.roll = roll_bias;
    }
}

 //------------------------------------------- 坐标系管理 ------------------------------------------------------

// @brief  将当前位置设为原点 (0,0)，航向角归零
void ins_api_set_origin(void)
{
    ins_core_reset(0.0f, 0.0f, 0.0f);
}

// @brief  重设航向角（不改变坐标位置）
// @param  yaw_deg  新的航向角（度）
void ins_api_set_heading(float yaw_deg)
{
    const INS_State* state = ins_core_get_state();
    ins_core_reset(state->x, state->y, yaw_deg * INS_DEG2RAD);
}

 //------------------------------------------- 轨迹记录 --------------------------------------------------------

// @brief  开始轨迹录制（转发到 ins_track 模块）
void ins_api_start_record(void)
{
    ins_track_start_save();
}

// @brief  停止轨迹录制
void ins_api_stop_record(void)
{
    ins_track_stop_save();
}

// @brief  清除已录制的轨迹数据
void ins_api_clear_track(void)
{
    ins_track_clear();
}

 //------------------------------------------- 轨迹循迹 --------------------------------------------------------

// @brief  启动 Pure Pursuit 循迹（重置 INS 到原点，切换 turn_mode 到偏航角闭环）
void ins_api_start_follow(void)
{
    if (ins_track_get_point_count() == 0)
        return;

    // 重置 INS 位置到原点
    ins_core_reset(0.0f, 0.0f, 0.0f);

    // 保存当前 turn_mode 并切换到 yaw 角闭环模式
    s_saved_turn_mode = turn_mode;
    turn_mode = 3;
    Target_Yaw = 0.0f;

    // 启动循迹
    ins_track_start_follow();

    // 设置循迹速度
    Yao.Target_Speed = (int)s_follow_speed;
}

// @brief  停止循迹（恢复之前的 turn_mode，速度归零）
void ins_api_stop_follow(void)
{
    ins_track_stop_follow();

    // 恢复原 turn_mode
    turn_mode = s_saved_turn_mode;
    Target_Yaw = 0.0f;
    Yao.Target_Speed = 0;
}

// @brief  设置循迹目标速度
// @param  speed_mps  速度值（m/s）
void ins_api_set_follow_speed(float speed_mps)
{
    s_follow_speed = speed_mps;
}

 //------------------------------------------- 状态查询 --------------------------------------------------------

// @brief  获取 INS 当前状态（位置、速度、航向）
// @return INS_State 只读指针
const INS_State* ins_api_get_state(void)
{
    return ins_core_get_state();
}

// @brief  获取已录制的轨迹点数
// @return 轨迹点数
uint32 ins_api_get_track_points(void)
{
    return ins_track_get_point_count();
}

// @brief  是否正在录制轨迹
// @return 1=录制中，0=未录制
uint8 ins_api_is_recording(void)
{
    return ins_track_is_saving();
}

// @brief  是否正在循迹
// @return 1=循迹中，0=未循迹
uint8 ins_api_is_following(void)
{
    return ins_track_is_following();
}

 //------------------------------------------- 循迹转向输出 ----------------------------------------------------

// @brief  获取循迹输出的转向角（度）
// @return 转向角度，未循迹时返回 0
float ins_api_get_steer_angle_deg(void)
{
    if (!ins_track_is_following())
        return 0.0f;

    return ins_track_get_steer_output_deg();
}
