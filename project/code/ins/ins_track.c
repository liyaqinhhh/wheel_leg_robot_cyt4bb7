/*
 * ins_track.c
 * 轨迹存取与 Pure Pursuit 循迹
 * 从源工程 track.c 移植，适配 CYT4BB7 Flash API 和 motor_value 编码器
 */

#include "ins_track.h"
#include "ins_core.h"
#include "zf_driver_flash.h"
#include "small_driver_uart_control.h"
#include <math.h>
#include <string.h>

 //------------------------------------------- 元数据配置 ------------------------------------------------------
#define INS_TRACK_META_MAGIC      0x494E5354u   // "INST"
#define INS_TRACK_META_VERSION    1u
#define INS_TRACK_MAX_COORD_ABS   1000000.0f
#define INS_TRACK_MAX_YAW_RAD     3.5f

 //------------------------------------------- 内部变量 --------------------------------------------------------
static uint32 s_total_points = 0;
static uint8 s_save_flag = 0;
static uint8 s_follow_flag = 0;

// 记录状态
static uint32 s_cur_write_page = INS_TRACK_FLASH_PAGE_BEGIN;
static float s_dist_acc_m = 0.0f;               // 累计行驶距离
static int16 s_flash_point_index = 0;            // 当前页内 float 索引

// 读取缓冲区
static uint32 s_read_buf[INS_TRACK_PAGE_FLOAT_NUM] = {0};

// 循迹状态
static uint16 s_follow_page = INS_TRACK_FLASH_PAGE_BEGIN;
static uint16 s_follow_point_idx = 1;
static uint32 s_follow_abs_index = 1;
static float s_steer_output = 0.0f;
static float s_steer_output_filtered = 0.0f;
static float s_current_speed_dir = 1.0f;        // 1.0=前进，0.0=后退

 //------------------------------------------- 内部函数 --------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取已存储的轨迹点总数 (限制在最大容量内)
//  @param      void
//  @return     uint32   轨迹点计数, 不超过 Flash 最大容量
//-------------------------------------------------------------------------------------------------------------------
static uint32 track_get_stored_count(void)
{
    uint32 max_pts = (uint32)INS_TRACK_FLASH_PAGE_MAX * (uint32)INS_TRACK_POINT_PER_PAGE;
    return (s_total_points > max_pts) ? max_pts : s_total_points;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      检查浮点数值是否有效 (非 NaN 且绝对值不超过限制)
//  @param      value       待检查的浮点数
//  @param      abs_limit   绝对值上限
//  @return     uint8       1=有效, 0=无效 (NaN 或越界)
//-------------------------------------------------------------------------------------------------------------------
static uint8 track_float_is_valid(float value, float abs_limit)
{
    if (value != value) return 0;  // NaN check
    return (fabsf(value) <= abs_limit) ? 1u : 0u;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      保存元数据到 Flash 页 0
//-------------------------------------------------------------------------------------------------------------------
static void track_meta_save(void)
{
    uint32 meta_buf[FLASH_PAGE_LENGTH];
    memset(meta_buf, 0xFF, sizeof(meta_buf));

    meta_buf[0] = INS_TRACK_META_MAGIC;
    meta_buf[1] = INS_TRACK_META_VERSION;
    meta_buf[2] = track_get_stored_count();
    meta_buf[3] = s_cur_write_page;
    meta_buf[4] = (uint32)((s_flash_point_index < 0) ? 0 : s_flash_point_index);

    flash_erase_page(0, INS_TRACK_FLASH_META_PAGE);
    for (uint32 i = 0; i < 5; i++)
        flash_union_buffer[i].uint32_type = meta_buf[i];
    flash_write_page_from_buffer(0, INS_TRACK_FLASH_META_PAGE, 5);
    flash_buffer_clear();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      清除元数据
//-------------------------------------------------------------------------------------------------------------------
static void track_meta_clear(void)
{
    flash_erase_page(0, INS_TRACK_FLASH_META_PAGE);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      从 Flash 页 0 加载元数据
//-------------------------------------------------------------------------------------------------------------------
static void track_meta_load(void)
{
    uint32 max_pts = (uint32)INS_TRACK_FLASH_PAGE_MAX * (uint32)INS_TRACK_POINT_PER_PAGE;

    flash_buffer_clear();
    flash_read_page_to_buffer(0, INS_TRACK_FLASH_META_PAGE, 5);
    uint32 meta_buf[5];
    for (uint32 i = 0; i < 5; i++)
        meta_buf[i] = flash_union_buffer[i].uint32_type;
    flash_buffer_clear();

    if (meta_buf[0] != INS_TRACK_META_MAGIC || meta_buf[1] != INS_TRACK_META_VERSION)
        return;
    if (meta_buf[2] > max_pts)
        return;
    if (meta_buf[3] < INS_TRACK_FLASH_PAGE_BEGIN ||
        meta_buf[3] >= (INS_TRACK_FLASH_PAGE_BEGIN + INS_TRACK_FLASH_PAGE_MAX))
        return;
    if (meta_buf[4] > INS_TRACK_PAGE_FLOAT_NUM || (meta_buf[4] % INS_TRACK_FLOAT_PER_POINT) != 0u)
        return;

    s_total_points = meta_buf[2];
    s_cur_write_page = meta_buf[3];
    s_flash_point_index = (int16)meta_buf[4];
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      将 flash_union_buffer 写入指定页（先擦除再写入）
//-------------------------------------------------------------------------------------------------------------------
static void track_flash_cache_flush(uint32 page_num)
{
    if (page_num < INS_TRACK_FLASH_PAGE_BEGIN ||
        page_num >= (INS_TRACK_FLASH_PAGE_BEGIN + INS_TRACK_FLASH_PAGE_MAX))
        return;

    flash_erase_page(0, page_num);
    flash_write_page_from_buffer(0, page_num, FLASH_PAGE_LENGTH);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      页循环递增
//-------------------------------------------------------------------------------------------------------------------
static void track_flash_next_page(void)
{
    s_cur_write_page++;
    if (s_cur_write_page >= (INS_TRACK_FLASH_PAGE_BEGIN + INS_TRACK_FLASH_PAGE_MAX))
        s_cur_write_page = INS_TRACK_FLASH_PAGE_BEGIN;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Pure Pursuit 转向计算
//-------------------------------------------------------------------------------------------------------------------
static float pure_pursuit_calc_steer(float x, float y, float yaw, Ins_TrackPoint* target)
{
    float dx = target->x - x;
    float dy = target->y - y;

    // 转换到车体坐标系
    float x_local = dx * cosf(yaw) + dy * sinf(yaw);
    float y_local = -dx * sinf(yaw) + dy * cosf(yaw);

    float ld = sqrtf(x_local * x_local + y_local * y_local);
    if (ld < 0.1f) return 0.0f;

    // 曲率 k = 2 * y_local / Ld^2
    float curvature = 2.0f * y_local / (ld * ld);
    float steer_rad = atanf(curvature * INS_TRACK_WHEELBASE);
    float steer_deg = steer_rad * INS_RAD2DEG;

    // 倒车时反转
    if (target->speed_dir < 0.5f)
        steer_deg = -steer_deg;

    // 死区：±2 度内视为直行
    if (steer_deg > -2.0f && steer_deg < 2.0f)
        steer_deg = 0.0f;

    // 限幅
    if (steer_deg > 20.0f) steer_deg = 20.0f;
    if (steer_deg < -20.0f) steer_deg = -20.0f;

    return steer_deg;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      读取指定页数据到 s_read_buf
//-------------------------------------------------------------------------------------------------------------------
static void track_flash_read_page(uint32 page_num)
{
    if (page_num < INS_TRACK_FLASH_PAGE_BEGIN ||
        page_num >= (INS_TRACK_FLASH_PAGE_BEGIN + INS_TRACK_FLASH_PAGE_MAX))
        return;

    flash_buffer_clear();
    flash_read_page_to_buffer(0, page_num, FLASH_PAGE_LENGTH);
    for (uint32 i = 0; i < INS_TRACK_PAGE_FLOAT_NUM; i++)
        s_read_buf[i] = flash_union_buffer[i].uint32_type;
    flash_buffer_clear();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      从 s_read_buf 中解析指定编号的轨迹点
//  @param      point_index     点编号（1-127）
//  @param      out             输出结构体
//  @return     1-成功 0-失败
//-------------------------------------------------------------------------------------------------------------------
static uint8 track_flash_get_point(uint32 point_index, Ins_TrackPoint* out)
{
    if (out == NULL || point_index < 1 || point_index > INS_TRACK_POINT_PER_PAGE)
        return 0;

    uint32 base = (point_index - 1) * INS_TRACK_FLOAT_PER_POINT;
    flash_data_union tmp;

    tmp.uint32_type = s_read_buf[base + 0];
    out->x = tmp.float_type;
    tmp.uint32_type = s_read_buf[base + 1];
    out->y = tmp.float_type;
    tmp.uint32_type = s_read_buf[base + 2];
    out->yaw = tmp.float_type;
    tmp.uint32_type = s_read_buf[base + 3];
    out->speed_dir = tmp.float_type;

    // 检查 Flash 空白值
    if (s_read_buf[base + 0] == 0xFFFFFFFFu || s_read_buf[base + 1] == 0xFFFFFFFFu ||
        s_read_buf[base + 2] == 0xFFFFFFFFu || s_read_buf[base + 3] == 0xFFFFFFFFu)
        return 0;
    if (s_read_buf[base + 0] == 0u && s_read_buf[base + 1] == 0u &&
        s_read_buf[base + 2] == 0u && s_read_buf[base + 3] == 0u)
        return 0;

    // 有效性检查
    if (!track_float_is_valid(out->x, INS_TRACK_MAX_COORD_ABS) ||
        !track_float_is_valid(out->y, INS_TRACK_MAX_COORD_ABS) ||
        !track_float_is_valid(out->yaw, INS_TRACK_MAX_YAW_RAD) ||
        !track_float_is_valid(out->speed_dir, 1.0f))
        return 0;

    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      准备指定绝对编号的轨迹点（自动切换页）
//-------------------------------------------------------------------------------------------------------------------
static uint8 track_follow_prepare_point(uint32 abs_index)
{
    uint32 stored = track_get_stored_count();
    if (abs_index < 1 || abs_index > stored)
        return 0;

    uint32 page_offset = (abs_index - 1U) / (uint32)INS_TRACK_POINT_PER_PAGE;
    uint32 target_page = INS_TRACK_FLASH_PAGE_BEGIN + page_offset;
    uint32 point_in_page = ((abs_index - 1U) % (uint32)INS_TRACK_POINT_PER_PAGE) + 1U;

    if (target_page != s_follow_page)
    {
        track_flash_read_page(target_page);
        s_follow_page = (uint16)target_page;
    }

    s_follow_point_idx = (uint16)point_in_page;
    s_follow_abs_index = abs_index;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      寻找前视点
//-------------------------------------------------------------------------------------------------------------------
static uint8 find_lookahead_point(float x, float y)
{
    uint32 stored = track_get_stored_count();
    if (stored == 0) return 0;

    if (s_follow_abs_index < 1 || s_follow_abs_index > stored)
        s_follow_abs_index = 1;

    if (!track_follow_prepare_point(s_follow_abs_index))
    {
        s_follow_abs_index = 1;
        if (!track_follow_prepare_point(s_follow_abs_index))
            return 0;
    }

    uint32 search_end = stored;
    uint32 last_same_dir_index = s_follow_abs_index;
    Ins_TrackPoint last_same_dir_point = {0};
    track_flash_get_point(s_follow_point_idx, &last_same_dir_point);

    while (s_follow_abs_index <= search_end)
    {
        Ins_TrackPoint pt;
        if (track_flash_get_point(s_follow_point_idx, &pt))
        {
            // 检查方向一致性
            if ((pt.speed_dir >= 0.5f && s_current_speed_dir >= 0.5f) ||
                (pt.speed_dir < 0.5f && s_current_speed_dir < 0.5f))
            {
                last_same_dir_index = s_follow_abs_index;
                last_same_dir_point = pt;

                float dx = pt.x - x;
                float dy = pt.y - y;
                float dist = sqrtf(dx * dx + dy * dy);

                if (dist >= INS_TRACK_LOOKAHEAD_DIST)
                    return 1;  // 找到前视点，数据在 last_same_dir_point 中
            }
            else
            {
                s_follow_abs_index = last_same_dir_index;
                return 1;
            }
        }

        s_follow_abs_index++;
        if (s_follow_abs_index > search_end) break;

        if (!track_follow_prepare_point(s_follow_abs_index))
            continue;
    }

    // 使用最后一个点
    if (track_follow_prepare_point(stored))
    {
        s_follow_abs_index = stored;
        return 1;
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      执行一次 Pure Pursuit 循迹
//-------------------------------------------------------------------------------------------------------------------
static void track_follow(void)
{
    const INS_State* state = ins_core_get_state();
    if (state == NULL)
    {
        ins_track_stop_follow();
        return;
    }

    Ins_TrackPoint target;
    if (!find_lookahead_point(state->x, state->y))
    {
        // 未找到前视点，保持上一次输出
        return;
    }

    // 获取当前前视点
    if (!track_flash_get_point(s_follow_point_idx, &target))
        return;

    // 计算转向
    s_steer_output = pure_pursuit_calc_steer(state->x, state->y, state->yaw, &target);

    // 低通滤波
    s_steer_output_filtered = INS_TRACK_STEER_FILTER_ALPHA * s_steer_output +
                              (1.0f - INS_TRACK_STEER_FILTER_ALPHA) * s_steer_output_filtered;

    // 检查是否到达前视点，推进索引
    float dx = target.x - state->x;
    float dy = target.y - state->y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist < 0.05f)
    {
        uint32 stored = track_get_stored_count();
        if (s_follow_abs_index < stored)
        {
            s_follow_abs_index++;

            // 检查下一个点的方向
            if (track_follow_prepare_point(s_follow_abs_index))
            {
                Ins_TrackPoint next_pt;
                if (track_flash_get_point(s_follow_point_idx, &next_pt))
                {
                    if ((next_pt.speed_dir >= 0.5f && s_current_speed_dir < 0.5f) ||
                        (next_pt.speed_dir < 0.5f && s_current_speed_dir >= 0.5f))
                    {
                        s_current_speed_dir = next_pt.speed_dir;
                    }
                }
            }
        }
        else if (s_follow_abs_index == stored)
        {
            // 到达终点
            ins_track_stop_follow();
            return;
        }
    }
}

 //------------------------------------------- 公共 API --------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  @brief      初始化轨迹模块: 清零状态变量, 从 Flash 加载元数据
//  @param      void
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void ins_track_init(void)
{
    s_cur_write_page = INS_TRACK_FLASH_PAGE_BEGIN;
    s_dist_acc_m = 0.0f;
    s_flash_point_index = 0;
    s_read_buf[0] = 0;  // 清零
    s_follow_page = INS_TRACK_FLASH_PAGE_BEGIN;
    s_follow_point_idx = 1;
    s_follow_abs_index = 1;
    s_steer_output = 0.0f;
    s_steer_output_filtered = 0.0f;
    s_current_speed_dir = 1.0f;
    s_save_flag = 0;
    s_follow_flag = 0;
    s_total_points = 0;

    flash_buffer_clear();
    track_meta_load();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      推入一个轨迹点（每 4ms 调用，自动判断是否需要采样）
//-------------------------------------------------------------------------------------------------------------------
void ins_track_push_point(void)
{
    // 计算当前帧行驶距离
    float v = (float)(motor_value.receive_left_speed_data - motor_value.receive_right_speed_data);
    float ds = fabsf(v * INS_TICK_TO_METER);
    s_dist_acc_m += ds;

    // 未达到采样间距则跳过
    if (s_dist_acc_m < INS_TRACK_SAMPLE_STEP)
        return;
    s_dist_acc_m -= INS_TRACK_SAMPLE_STEP;

    // 缓冲区满则先 flush
    if (s_flash_point_index + (int16)INS_TRACK_FLOAT_PER_POINT > (int16)INS_TRACK_PAGE_FLOAT_NUM)
    {
        track_flash_cache_flush(s_cur_write_page);
        track_meta_save();
        track_flash_next_page();
        s_flash_point_index = 0;
        flash_buffer_clear();
    }

    // 写入轨迹点到 flash_union_buffer
    const INS_State* state = ins_core_get_state();
    flash_data_union tmp;

    tmp.float_type = state->x;
    flash_union_buffer[s_flash_point_index] = tmp;
    tmp.float_type = state->y;
    flash_union_buffer[s_flash_point_index + 1] = tmp;
    tmp.float_type = state->yaw;
    flash_union_buffer[s_flash_point_index + 2] = tmp;

    // 速度方向：编码器速度为正=前进，为负=后退
    float speed_dir = 1.0f;
    if (v < -0.05f) speed_dir = 0.0f;
    tmp.float_type = speed_dir;
    flash_union_buffer[s_flash_point_index + 3] = tmp;

    s_flash_point_index += INS_TRACK_FLOAT_PER_POINT;
    uint32 max_pts = (uint32)INS_TRACK_FLASH_PAGE_MAX * (uint32)INS_TRACK_POINT_PER_PAGE;
    if (s_total_points < max_pts)
        s_total_points++;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开始轨迹录制: 清零录制状态并置位保存标志
//  @param      void
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void ins_track_start_save(void)
{
    s_cur_write_page = INS_TRACK_FLASH_PAGE_BEGIN;
    s_flash_point_index = 0;
    s_dist_acc_m = 0.0f;
    s_total_points = 0;
    s_follow_page = INS_TRACK_FLASH_PAGE_BEGIN;
    s_follow_point_idx = 1;
    s_follow_abs_index = 1;
    s_steer_output = 0.0f;
    s_steer_output_filtered = 0.0f;
    flash_buffer_clear();
    s_follow_flag = 0;
    s_save_flag = 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      停止轨迹录制: flush 剩余缓冲区并保存元数据
//  @param      void
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void ins_track_stop_save(void)
{
    if (s_save_flag == 1)
    {
        s_save_flag = 0;
        // flush 未满页的缓冲区
        if (s_flash_point_index > 0)
            track_flash_cache_flush(s_cur_write_page);
        track_meta_save();
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      清除所有轨迹数据: 擦除 Flash 页并重置 RAM 状态
//  @param      void
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void ins_track_clear(void)
{
    // 清除所有轨迹页
    track_meta_clear();
    for (uint32 page = 0; page < INS_TRACK_FLASH_PAGE_MAX; page++)
        flash_erase_page(0, INS_TRACK_FLASH_PAGE_BEGIN + page);

    s_save_flag = 0;
    s_follow_flag = 0;
    s_cur_write_page = INS_TRACK_FLASH_PAGE_BEGIN;
    s_flash_point_index = 0;
    s_dist_acc_m = 0.0f;
    s_total_points = 0;
    s_follow_page = INS_TRACK_FLASH_PAGE_BEGIN;
    s_follow_point_idx = 1;
    s_follow_abs_index = 1;
    s_steer_output = 0.0f;
    s_steer_output_filtered = 0.0f;
    flash_buffer_clear();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开始 Pure Pursuit 循迹: 加载首个轨迹点并预执行寻点
//  @param      void
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void ins_track_start_follow(void)
{
    if (track_get_stored_count() == 0)
    {
        s_follow_flag = 0;
        return;
    }

    s_follow_page = INS_TRACK_FLASH_PAGE_BEGIN;
    s_follow_point_idx = 1;
    s_follow_abs_index = 1;
    s_steer_output = 0.0f;
    s_steer_output_filtered = 0.0f;
    s_current_speed_dir = 1.0f;

    track_flash_read_page(s_follow_page);

    // 验证第一个点
    Ins_TrackPoint first_point;
    if (!track_flash_get_point(1, &first_point))
    {
        s_follow_flag = 0;
        return;
    }

    s_current_speed_dir = first_point.speed_dir;

    // 预执行一次寻点
    const INS_State* state = ins_core_get_state();
    if (state != NULL)
        find_lookahead_point(state->x, state->y);

    s_save_flag = 0;
    s_follow_flag = 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      停止循迹: 清除循迹标志
//  @param      void
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void ins_track_stop_follow(void)
{
    s_follow_flag = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      循迹处理入口: 在循迹标志有效时执行一次 Pure Pursuit
//  @param      void
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void ins_track_follow_proc(void)
{
    if (s_follow_flag == 1)
        track_follow();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取已存储的轨迹点总数
//  @param      void
//  @return     uint32   轨迹点计数
//-------------------------------------------------------------------------------------------------------------------
uint32 ins_track_get_point_count(void)
{
    return track_get_stored_count();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      查询当前是否正在录制轨迹
//  @param      void
//  @return     uint8   1=正在录制, 0=未录制
//-------------------------------------------------------------------------------------------------------------------
uint8 ins_track_is_saving(void)
{
    return s_save_flag;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      查询当前是否正在循迹
//  @param      void
//  @return     uint8   1=正在循迹, 0=未循迹
//-------------------------------------------------------------------------------------------------------------------
uint8 ins_track_is_following(void)
{
    return s_follow_flag;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取循迹转向输出角度 (经低通滤波)
//  @param      void
//  @return     float   转向角度 (度)
//-------------------------------------------------------------------------------------------------------------------
float ins_track_get_steer_output_deg(void)
{
    return s_steer_output_filtered;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      统一入口：每 4ms 调用，内部判断记录/循迹
//-------------------------------------------------------------------------------------------------------------------
void ins_track_proc(void)
{
    if (s_save_flag == 1)
    {
        ins_track_push_point();
    }
    else if (s_follow_flag == 1)
    {
        ins_track_follow_proc();
    }
}
