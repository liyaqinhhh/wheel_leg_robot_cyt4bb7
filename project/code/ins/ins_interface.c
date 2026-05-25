/*
 * ins_interface.c
 * INS 对外 API 实现
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

void ins_api_save_imu_bias(void)
{
    flash_union_buffer[0].float_type = imu660ra.offset_angle.pitch;
    flash_union_buffer[1].float_type = imu660ra.offset_angle.roll;
    flash_erase_page(0, INS_FLASH_IMU_BIAS_PAGE);
    flash_write_page_from_buffer(0, INS_FLASH_IMU_BIAS_PAGE, 2);
    flash_buffer_clear();
}

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

void ins_api_set_origin(void)
{
    ins_core_reset(0.0f, 0.0f, 0.0f);
}

void ins_api_set_heading(float yaw_deg)
{
    const INS_State* state = ins_core_get_state();
    ins_core_reset(state->x, state->y, yaw_deg * INS_DEG2RAD);
}

 //------------------------------------------- 轨迹记录 --------------------------------------------------------

void ins_api_start_record(void)
{
    ins_track_start_save();
}

void ins_api_stop_record(void)
{
    ins_track_stop_save();
}

void ins_api_clear_track(void)
{
    ins_track_clear();
}

 //------------------------------------------- 轨迹循迹 --------------------------------------------------------

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

void ins_api_stop_follow(void)
{
    ins_track_stop_follow();

    // 恢复原 turn_mode
    turn_mode = s_saved_turn_mode;
    Target_Yaw = 0.0f;
    Yao.Target_Speed = 0;
}

void ins_api_set_follow_speed(float speed_mps)
{
    s_follow_speed = speed_mps;
}

 //------------------------------------------- 状态查询 --------------------------------------------------------

const INS_State* ins_api_get_state(void)
{
    return ins_core_get_state();
}

uint32 ins_api_get_track_points(void)
{
    return ins_track_get_point_count();
}

uint8 ins_api_is_recording(void)
{
    return ins_track_is_saving();
}

uint8 ins_api_is_following(void)
{
    return ins_track_is_following();
}

 //------------------------------------------- 循迹转向输出 ----------------------------------------------------

float ins_api_get_steer_angle_deg(void)
{
    if (!ins_track_is_following())
        return 0.0f;

    return ins_track_get_steer_output_deg();
}
