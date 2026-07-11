/*
 * ins_auto_record.c
 *
 *  Created on: 2026年6月7日
 *      Author: LateRain
 *
 *  自动打点惯导模块实现 —— 固定距离打点 + Pure Pursuit 循迹
 *  ================================================================
 */

#include "zf_common_headfile.h"
#include "ins_auto_record.h"
#include "ins_pure_pursuit.h"
#include "Interrupt.h"
#include "imu660.h"
#include <math.h>

/* ==================================================================
 *  全局变量
 * ================================================================== */

InsAuto_State g_ins_auto = {0};

/* Flash 操作缓冲区 (extern from zf_driver_flash.h) */
extern flash_data_union flash_union_buffer[FLASH_PAGE_LENGTH];

/* Flash double 读写函数 (定义在 Ins.c) */
extern void writeDoubleToFlash1(double data1, double data2, uint8 z);
extern double readFlash_to_double1(bool a, uint8 x);

/* 平均轮速，供自适应前瞻距离使用 (extern from Interrupt.c) */
extern volatile float speed_MOTOR;

/* exit2 延时恢复速度 (extern from Interrupt.c, 40ms 驱动) */


/* ==================================================================
 *  Flash 读写辅助函数
 * ================================================================== */

/*
 * 将 double 拆分为两个 uint32 (little-endian)
 * 与 Ins.c 的实现保持一致
 */
static void write_double_to_flash(double data, uint16 index)
{
    uint64_t u64 = *(uint64_t *)&data;
    uint32_t hi = (uint32_t)(u64 >> 32);
    uint32_t lo = (uint32_t)(u64 & 0xFFFFFFFF);

    flash_union_buffer[index].uint32_type = lo;
    flash_union_buffer[index + 1].uint32_type = hi;
}

/*
 * 从 Flash 读取 double (两个 uint32 合并)
 */
static double read_double_from_flash(uint16 index)
{
    uint32_t lo = flash_union_buffer[index].uint32_type;
    uint32_t hi = flash_union_buffer[index + 1].uint32_type;
    uint64_t u64 = ((uint64_t)hi << 32) | lo;
    return *(double *)&u64;
}

/* ==================================================================
 *  自动打点核心逻辑
 * ================================================================== */

/*
 * 初始化自动打点模块
 */
void ins_auto_record_init(void)
{
    /* 清空状态 */
    memset(&g_ins_auto, 0, sizeof(g_ins_auto));

    /* 设置默认配置 */
    g_ins_auto.config.record_distance = INS_AUTO_RECORD_DISTANCE;
    g_ins_auto.config.lookahead_distance = INS_AUTO_LOOKAHEAD_DEFAULT;
    g_ins_auto.config.arrival_threshold = INS_AUTO_ARRIVAL_THRESHOLD;
    g_ins_auto.config.auto_record_enable = 1;
    g_ins_auto.config.adaptive_lookahead_enable = INS_AUTO_ADAPTIVE_LD_ENABLE;

    /* subject3: 默认特殊事件速度 */
    g_ins_auto.special_speed2 = INS_AUTO_SPECIAL_SPEED2_DEFAULT;

    /* 初始化 Pure Pursuit 模块 */
    ins_pure_pursuit_init();

    /* 尝试从 Flash 加载数据 */
    // ins_auto_load_from_flash();
}

/*
 * 自动打点更新 (每 16ms 由中断调用)
 *
 * 算法:
 *   1. 累加行驶距离
 *   2. 当累计距离 >= record_distance 时，记录当前坐标
 *   3. 重置距离计数器
 */
void ins_auto_record_update(int speed, float yaw)
{
    /* 检查是否启用自动录制 */
    if (!g_ins_auto.is_recording || !g_ins_auto.config.auto_record_enable)
    {
        return;
    }

    /* 特殊点模式下暂停自动打点 */
    if (g_ins_auto.Special_point)
    {
        return;
    }

    /* 检查航点数组是否已满 */
    if (g_ins_auto.wp_count >= INS_AUTO_MAX_WAYPOINTS)
    {
        return; /* 已满，停止录制 */
    }

    /* 计算本次移动距离 (厘米)
     *   speed(rev/s) × 0.016s × 周长 = WHEEL_CIRCUMFERENCE_CM */
    double move_dist = fabs((double)speed * 0.016 * WHEEL_CIRCUMFERENCE_CM);

    /* 累加距离 */
    g_ins_auto.last_record_distance += move_dist;

    /* 检查是否达到打点阈值 */
    if (g_ins_auto.last_record_distance >= INS_AUTO_RECORD_DISTANCE)
    {
        /* 记录当前坐标 */
        g_ins_auto.waypoints[g_ins_auto.wp_count] = cod_realtime;
        g_ins_auto.wp_count++;

        /* 重置距离计数器 */
        g_ins_auto.last_record_distance = 0.0;

        /* 可选: 在屏幕上显示打点信息 */
        /* ips200_show_string(0, 0, "WP: ");
        ips200_show_uint(40, 0, g_ins_auto.wp_count, 3); */
    }
}

/*
 * 开始自动录制
 */
void ins_auto_record_start(void)
{
    /* 清空现有航点 */
    g_ins_auto.wp_count = 0;
    g_ins_auto.wp_current = 0;

    /* 重置距离计数器 */
    g_ins_auto.last_record_distance = 0.0;

    /* 设置录制标志 */
    g_ins_auto.is_recording = 1;
    g_ins_auto.config.auto_record_enable = 1;

    /* 清空导航状态 */
    g_ins_auto.is_navigating = 0;
    g_ins_auto.nav_finished = 0;
}

/*
 * 停止自动录制
 */
void ins_auto_record_stop(void)
{
    g_ins_auto.is_recording = 0;
    g_ins_auto.config.auto_record_enable = 0;

    /* 可选: 自动保存到 Flash */
    /* ins_auto_save_to_flash(); */
}

/*
 * 清空 Flash 数据
 */
void ins_auto_clear_flash(void)
{
    uint8 i;
    for (i = 0; i < 4; i++)
    { /* 页 60-63，共 4 页 */
        flash_erase_page(0, INS_AUTO_FLASH_PAGE_META + i);
    }

    /* 重置状态 */
    g_ins_auto.wp_count = 0;
    g_ins_auto.wp_current = 0;
}

/* ==================================================================
 *  Pure Pursuit 导航逻辑
 * ================================================================== */

/*
 * Pure Pursuit 导航主函数 (在主循环中每帧调用)
 */
void ins_auto_record_navigation(void)
{
    /* 检查是否正在导航 */

    /* 特殊点自转模式: 暂停常规巡点逻辑 (subject2) */
    if (g_ins_auto.Special_point_get)
    {
        return;
    }

    /* subject3 特殊事件模式: 暂停常规巡点逻辑 */
    if (g_ins_auto.Special_point_get2)
    {
        return;
    }

    /* exit2 延时到时: 恢复 subject3 之前备份的速度 */
    // if (g_exit2_timeout_flag)
    // {
    //     g_exit2_timeout_flag = 0;
    //     Target_Speed = temp_speed2;
    // }

    /* 检查当前航点索引是否有效 */
    if (g_ins_auto.wp_current >= g_ins_auto.wp_count)
    {
        /* 已到达最后一个航点 */
        g_ins_auto.nav_finished = 1;
        ins_auto_nav_stop();
        return;
    }

    /* 1. 计算自适应前瞻距离（速度 + 上一帧曲率融合） */
    static float prev_curvature = 0.0f; /* 跨帧持久 */
    float lookahead_dist = g_ins_auto.config.lookahead_distance;

    if (g_ins_auto.config.adaptive_lookahead_enable)
    {
        float abs_speed = fabsf(speed_MOTOR);
        float abs_curv = fabsf(prev_curvature);

        /* Ld = Ld_base + k_speed * |speed| - k_curv * |curvature|
         * 直道高速: 前瞻增大（轨迹平滑）
         * 弯道急:   前瞻缩小（贴线精准）
         */
        lookahead_dist = g_ins_auto.config.lookahead_distance + INS_AUTO_ADAPTIVE_LD_SPEED_GAIN * abs_speed - INS_AUTO_ADAPTIVE_LD_CURV_GAIN * abs_curv;

        /* 钳位到 [MIN, MAX] */
        if (lookahead_dist < PURE_PURSUIT_MIN_LOOKAHEAD)
            lookahead_dist = PURE_PURSUIT_MIN_LOOKAHEAD;
        if (lookahead_dist > PURE_PURSUIT_MAX_LOOKAHEAD)
            lookahead_dist = PURE_PURSUIT_MAX_LOOKAHEAD;
    }

    /* 2. 使用 Pure Pursuit 计算目标转向角 */
    PurePursuit_State pp_state = ins_pure_pursuit_update(
        g_ins_auto.waypoints,
        g_ins_auto.wp_count,
        g_ins_auto.wp_current,
        cod_realtime,
        imu660ra.eulerAngle.yaw, /* 当前偏航角 */
        lookahead_dist);

    /* 2b. 存储本帧曲率供下一帧自适应计算 */
    if (g_ins_auto.config.adaptive_lookahead_enable)
    {
        prev_curvature = pp_state.curvature;
    }

    yaw_ins = pp_state.target_yaw;
    /* 2. 更新全局变量 yaw_ins (供 Interrupt.c 的 turn_mode==7 使用) */
    /* 对目标偏航角做变化率限幅，间接限制小车转向角速度 */
    {
        static float yaw_ins_smoothed = 0.0f;
        static uint8 yaw_ins_inited = 0;
        float raw_target = pp_state.target_yaw;

        if (!yaw_ins_inited)
        {
            yaw_ins_smoothed = raw_target;
            yaw_ins_inited = 1;
        }

        /* 计算最短路径误差（考虑 ±180° 包裹） */
        float delta = raw_target - yaw_ins_smoothed;
        if (delta > 180.0f)
            delta -= 360.0f;
        if (delta < -180.0f)
            delta += 360.0f;

        /* 固定速率限幅: 每帧最多变化 INS_AUTO_YAW_RATE_MAX_DEG_PER_FRAME 度 */
        if (delta > INS_AUTO_YAW_RATE_MAX_DEG_PER_FRAME)
            delta = INS_AUTO_YAW_RATE_MAX_DEG_PER_FRAME;
        if (delta < -INS_AUTO_YAW_RATE_MAX_DEG_PER_FRAME)
            delta = -INS_AUTO_YAW_RATE_MAX_DEG_PER_FRAME;

        yaw_ins_smoothed += delta;

        /* 归一化到 [-180°, 180°] */
        if (yaw_ins_smoothed > 180.0f)
            yaw_ins_smoothed -= 360.0f;
        if (yaw_ins_smoothed < -180.0f)
            yaw_ins_smoothed += 360.0f;

        yaw_ins = yaw_ins_smoothed;
    }

    /* 3. 计算到当前航点的距离和向量 (用于到达判定) */
    Coordinates current_wp = g_ins_auto.waypoints[g_ins_auto.wp_current];
    double dx = current_wp.x - cod_realtime.x;
    double dy = current_wp.y - cod_realtime.y;
    dis_ins = sqrt(dx * dx + dy * dy);

    /* 4. 双重到达判定: 距离容差 OR 向量点乘过线检测 */
    uint8 wp_reached = 0;

    /* 条件 A: 距离容差 —— 正常进入到达圆内 */
    if (dis_ins < g_ins_auto.config.arrival_threshold && dis_ins > 0.001)
    {
        wp_reached = 1;
    }

    /* 条件 B: 向量点乘过线检测 —— 防急弯"漏点绕圈"
     *
     * 原理: 计算车身前向向量 F(cos_yaw, sin_yaw) 与
     *       指向目标向量 T(dx, dy) 的点乘。
     *       若 F·T < 0，说明目标点已被甩到车身后方 → 已越过 → 强制切换。
     *
     * 安全距离: 只在 PASS_GUARD_MAX_DIST 以内生效，防止误触发。
     */
#if INS_AUTO_PASS_GUARD_ENABLE
    if (!wp_reached && dis_ins < INS_AUTO_PASS_GUARD_MAX_DIST)
    {
        double yaw_rad = ANGLE_TO_RAD(imu660ra.eulerAngle.yaw);
        double forward_x = cos(yaw_rad); /* 车身前向 X 分量 */
        double forward_y = sin(yaw_rad); /* 车身前向 Y 分量 */

        /* F·T = forward_x * dx + forward_y * dy
         * < 0 → 目标在后方 → 已越过 */
        double dot = forward_x * dx + forward_y * dy;

        if (dot < 0.0)
        {
            wp_reached = 1;
        }
    }
#endif

    if (wp_reached)
    {
        /* 到达当前航点，切换到下一个 */
        g_ins_auto.wp_current++;

        /* yaw_ins > 160° 或 < -160° 则舍弃，继续跳到下一个 */
        // while (g_ins_auto.wp_current < g_ins_auto.wp_count && (yaw_ins > 160.0 || yaw_ins < -160.0))
        // {
        //     g_ins_auto.wp_current++;
        // }

        /* 检查是否到达终点 */
        if (g_ins_auto.wp_current >= g_ins_auto.wp_count)
        {
            g_ins_auto.nav_finished = 1;
            ins_auto_nav_stop();
            return;
        }
    }

    /* 5. 可选: 在屏幕上显示导航信息 */
    /* ips200_show_string(0, 16, "Nav: ");
    ips200_show_uint(40, 16, g_ins_auto.wp_current, 3);
    ips200_show_string(80, 16, "/");
    ips200_show_uint(100, 16, g_ins_auto.wp_count, 3); */
}

/*
 * 开始 Pure Pursuit 导航
 */
void ins_auto_nav_start(void)
{
    /* 检查航点数据是否有效 */

    /* 重置导航状态 */
    g_ins_auto.wp_current = 0;
    g_ins_auto.nav_finished = 0;
    flag_2 = 1;

    /* 停止录制 */
    g_ins_auto.is_recording = 0;

    /* 设置 turn_mode 为 7 (惯导转向模式) */

    turn_mode = 7;
}

/*
 * 停止导航
 */
void ins_auto_nav_stop(void)
{

    // Target_Yaw = imu660ra.eulerAngle.yaw; /* 记录当前偏航角作为目标 */
    // turn_mode = 3;                        /* 切换到偏航角度闭环模式，沿当前方向直走 */

    Target_Speed = 0;
    Yao.Outp_turn = 0;
    /* 不归零 Outp_Gyro_Pitch/Outp_Angle_Pitch/Outp_Speed_Pitch:
     * 它们由 Interrupt.c 的 PID 循环持续更新, 清零反而造成短暂失控 */
    /* 不调用 small_driver_set_duty(0,0), control_main 正常驱动电机保持平衡 */
    g_ins_auto.nav_finished = 1;
    // ins_mode = 3; /* 导航完成, 回到打点模式, 防止下一帧再次进入 case 5 */
    flag_main = 2;
    /* 4. 停止录制和导航标志 */
    // flag_2 = 0;
    g_ins_auto.is_recording = 0;

    /* 注意：不清零平衡控制输出，不设置 flag_stop，让小车继续平衡并沿直线走 */
}

/* ==================================================================
 *  配置和辅助函数
 * ================================================================== */

/* ==================================================================
 *  特殊事件点功能实现
 * ================================================================== */

/*
 * 保存特殊点到 Flash
 * - 将当前实时坐标记录为特殊点
 * - 写入页 93 (元数据: sp_count) 和页 94 (坐标)
 */
void ins_auto_special_point_save(void)
{
    if (g_ins_auto.sp_count >= INS_AUTO_MAX_SPECIAL_POINTS)
    {
        return; /* 已满 */
    }

    /* 记录当前坐标 */
    g_ins_auto.special_points[g_ins_auto.sp_count] = cod_realtime;
    g_ins_auto.sp_count++;

    /* 擦除并写入页 93 (元数据: sp_count) */
    if (flash_check(0, INS_AUTO_FLASH_PAGE_SP_META))
        flash_erase_page(0, INS_AUTO_FLASH_PAGE_SP_META);

    flash_buffer_clear();
    flash_union_buffer[0].uint16_type = g_ins_auto.sp_count;
    flash_write_page_from_buffer(0, INS_AUTO_FLASH_PAGE_SP_META, FLASH_PAGE_LENGTH);

    /* 擦除并写入页 94 (特殊点坐标) */
    if (flash_check(0, INS_AUTO_FLASH_PAGE_SP_DATA))
        flash_erase_page(0, INS_AUTO_FLASH_PAGE_SP_DATA);

    flash_buffer_clear();
    for (uint16 i = 0; i < g_ins_auto.sp_count; i++)
    {
        uint8 buffer_idx = i;
        writeDoubleToFlash1(g_ins_auto.special_points[i].x,
                            g_ins_auto.special_points[i].y, buffer_idx);
    }
    flash_write_page_from_buffer(0, INS_AUTO_FLASH_PAGE_SP_DATA, FLASH_PAGE_LENGTH);
}

/*
 * 从 Flash 读取所有特殊点
 * - 在导航启动时（KEY_1 按下）调用
 * - 读取页 93-94 到 g_ins_auto.special_points[]
 */
void ins_auto_special_point_load(void)
{
    /* 读取页 93: 特殊点数量 */
    flash_read_page_to_buffer(0, INS_AUTO_FLASH_PAGE_SP_META, FLASH_PAGE_LENGTH);
    g_ins_auto.sp_count = flash_union_buffer[0].uint16_type;
    flash_buffer_clear();

    /* 校验数量 */
    if (g_ins_auto.sp_count == 0 || g_ins_auto.sp_count > INS_AUTO_MAX_SPECIAL_POINTS)
    {
        g_ins_auto.sp_count = 0;
        return;
    }

    /* 读取页 94: 特殊点坐标 */
    flash_read_page_to_buffer(0, INS_AUTO_FLASH_PAGE_SP_DATA, FLASH_PAGE_LENGTH);
    for (uint16 i = 0; i < g_ins_auto.sp_count; i++)
    {
        uint8 buffer_idx = i;
        g_ins_auto.special_points[i].x = readFlash_to_double1(0, buffer_idx);
        g_ins_auto.special_points[i].y = readFlash_to_double1(1, buffer_idx);
    }
    flash_buffer_clear();

    /* 初始化状态 */
    g_ins_auto.sp_current = 0;
    g_ins_auto.Special_point_get = 0;
    g_ins_auto.rotate_count = 0;
    g_ins_auto.rotate_state = 0;
    g_ins_auto.rotate_timeout = 0;
}

/*
 * 特殊点导航更新 (每 16ms 由中断调用)
 * - 检测到当前特殊点的距离
 * - 20cm 以内: 判定到达，触发自转
 */
float temp_speed = 0;
float temp_speed2 = 0; /* subject3 专用速度备份, 与 subject2 的 temp_speed 解耦 */
void ins_auto_special_point_update(void)
{
    /* 没有特殊点或已全部处理完 → 退出 */
    if (g_ins_auto.sp_count == 0 || g_ins_auto.sp_current >= g_ins_auto.sp_count)
    {
        return;
    }

    /* 已在自转模式中 → 执行自转状态机 */
    if (g_ins_auto.Special_point_get && flag_subject2)
    {
        ins_auto_special_point_rotate();
        return;
    }

    /* 计算到当前特殊点的距离 */
    Coordinates sp = g_ins_auto.special_points[g_ins_auto.sp_current];
    double dx = sp.x - cod_realtime.x;
    double dy = sp.y - cod_realtime.y;
    double dist_to_sp = sqrt(dx * dx + dy * dy);

    /* 20cm 以内: 判定到达，触发自转 */
    if (dist_to_sp < 20.0 && dist_to_sp > 0.001)
    {
        g_ins_auto.Special_point_get = 1;

        /* 设置目标速度为0，停止巡点转向 */
        temp_speed = Target_Speed; /* 保存当前速度 */
        Target_Speed = 0;
        yaw_ins = 0;

        /* 保存当前 yaw 角作为自转起始角 */
        g_ins_auto.temp_yaw = imu660ra.eulerAngle.yaw;

        /* 设置转向输出为 100（正转） */
        Yao.Outp_turn = TURN_SPEED;

        /* 初始化自转状态 */
        g_ins_auto.rotate_count = 0;
        g_ins_auto.rotate_state = 1; /* 等待离开 temp_yaw */
        g_ins_auto.rotate_timeout = 0;
    }
}

/*
 * 特殊点自转状态机 (由 ins_auto_special_point_update 每 16ms 调用)
 *
 * 状态机:
 *   状态 1 (等待离开 temp_yaw):
 *     Outp_turn 已设为 100，小车开始旋转
 *     等待 yaw 角偏离 temp_yaw 超过 20°（表示已经开始转）
 *     → 切换到状态 2
 *
 *   状态 2 (等待再次进入 temp_yaw):
 *     每当 yaw 角经过 temp_yaw（±10°窗口）计一圈
 *     防漏检: 使用穿越方向判断，而不是窗口内停留判断
 *     超时: 5 秒内未能完成 → 强制退出
 *     → 圈数 >= 2 且 yaw 在 temp_yaw±10°内 → 完成
 */
void ins_auto_special_point_rotate(void)
{
    float current_yaw = imu660ra.eulerAngle.yaw;
    float delta;

    /* 超时保护: 5 秒 (5000ms / 16ms = 312 帧) */
    g_ins_auto.rotate_timeout++;
    if (g_ins_auto.rotate_timeout > OUT_Time)
    {
        /* 超时强制退出 */
        g_ins_auto.Special_point_get = 0;
        Yao.Outp_turn = 0;
        Target_Yaw = current_yaw;
        turn_mode = 7;

        /* 跳到下一个特殊点 */
        g_ins_auto.sp_current++;
        return;
    }

    switch (g_ins_auto.rotate_state)
    {
    case 1: /* 等待离开 temp_yaw */
        delta = current_yaw - g_ins_auto.temp_yaw;
        if (delta > 180.0f)
            delta -= 360.0f;
        if (delta < -180.0f)
            delta += 360.0f;

        if (func_abs(delta) > 20.0f)
        {
            /* 已离开 temp_yaw 超过 20°，进入圈数计数状态 */
            g_ins_auto.rotate_state = 2;
        }
        break;

    case 2: /* 等待再次经过 temp_yaw，计数圈数 */
    {
        /* 计算 yaw 角与 temp_yaw 的差值 */
        delta = current_yaw - g_ins_auto.temp_yaw;
        if (delta > 180.0f)
            delta -= 360.0f;
        if (delta < -180.0f)
            delta += 360.0f;

        /* 使用穿越标志防漏检/误检:
         *   用上一帧的 delta 和当前帧的 delta 判断是否穿越了 temp_yaw
         *   delta_prev 和 delta_curr 符号不同(或 delta_curr≈0) → 穿过了 temp_yaw */
        static float delta_prev = 0.0f;
        static uint8 delta_prev_valid = 0;

        if (!delta_prev_valid)
        {
            delta_prev = delta;
            delta_prev_valid = 1;
        }

        /* 检测穿越: delta 从正变负或从负变正（且差值足够大，防止抖动） */
        if ((delta_prev > 10.0f && delta < -10.0f) ||
            (delta_prev < -10.0f && delta > 10.0f) ||
            func_abs(delta) < 5.0f)
        {
            /* 防止同一穿越多次计数: 要求 delta_prev 的绝对值较大 */
            if (func_abs(delta_prev) > 10.0f || func_abs(delta) < 5.0f)
            {
                /* 在 temp_yaw±10°窗口内才计数 */
                if (func_abs(delta) < 10.0f)
                {
                    g_ins_auto.rotate_count++;
                }
            }

            /* 重置 delta_prev 防止连续计数 */
            delta_prev = current_yaw - g_ins_auto.temp_yaw;
            if (delta_prev > 180.0f)
                delta_prev -= 360.0f;
            if (delta_prev < -180.0f)
                delta_prev += 360.0f;
        }

        delta_prev = current_yaw - g_ins_auto.temp_yaw;
        if (delta_prev > 180.0f)
            delta_prev -= 360.0f;
        if (delta_prev < -180.0f)
            delta_prev += 360.0f;

        /* 完成条件: 圈数 >= 2 且 yaw 在 temp_yaw±10°内 */
        if (g_ins_auto.rotate_count >= 2 && func_abs(delta) < 10.0f)
        {
            /* 自转完成 */
            g_ins_auto.Special_point_get = 0;
            Yao.Outp_turn = 0;
            g_ins_auto.rotate_count = 0;
            g_ins_auto.rotate_state = 0;
            g_ins_auto.rotate_timeout = 0;
            delta_prev_valid = 0;
            turn_mode = 7;
            Target_Speed = temp_speed; /* 恢复之前的速度 */

            /* 跳到下一个特殊点 */
            g_ins_auto.sp_current++;

            /* 跳到下一个常规航点 */
            if (g_ins_auto.wp_current < g_ins_auto.wp_count)
            {
                g_ins_auto.wp_current++;
            }

            /* 恢复惯导转向模式 */
            // Target_Yaw = current_yaw;
        }
        break;
    }
    }
}

/* ==================================================================
 *  subject3 特殊事件点功能实现 (flag_subject3 控制)
 *  - 与 subject2 自转模式完全解耦，独立 Flash 区域 (页 95-96)
 *  - Flash 读写仅在 ins_mode=4 KEY_2 / ins_mode=5 KEY_1 统一执行
 *  - 打点阶段: 录制中每次 KEY_3 按下记录一个坐标点到 RAM
 *  - 循迹阶段: 到达进入点(n)后坐标跳变到退出点(n+1)，关闭/恢复惯导
 * ================================================================== */

/*
 * subject3: 保存特殊点到 Flash (页 95-96)
 * - 仅在 ins_mode=4 KEY_2 按下时统一调用（与现有航点保存一起执行）
 */
void ins_auto_special_point_save2(void)
{
    if (g_ins_auto.sp2_count == 0)
    {
        return; /* 无数据，不保存 */
    }

    /* 擦除并写入页 95 (元数据: sp2_count) */
    if (flash_check(0, INS_AUTO_FLASH_PAGE_SP2_META))
        flash_erase_page(0, INS_AUTO_FLASH_PAGE_SP2_META);

    flash_buffer_clear();
    flash_union_buffer[0].uint16_type = g_ins_auto.sp2_count;
    flash_write_page_from_buffer(0, INS_AUTO_FLASH_PAGE_SP2_META, FLASH_PAGE_LENGTH);

    /* 擦除并写入页 96 (特殊点坐标) */
    if (flash_check(0, INS_AUTO_FLASH_PAGE_SP2_DATA))
        flash_erase_page(0, INS_AUTO_FLASH_PAGE_SP2_DATA);

    flash_buffer_clear();
    for (uint16 i = 0; i < g_ins_auto.sp2_count; i++)
    {
        uint8 buffer_idx = i;
        writeDoubleToFlash1(g_ins_auto.special_points2[i].x,
                            g_ins_auto.special_points2[i].y, buffer_idx);
    }
    flash_write_page_from_buffer(0, INS_AUTO_FLASH_PAGE_SP2_DATA, FLASH_PAGE_LENGTH);
}

/*
 * subject3: 从 Flash 读取特殊点 (页 95-96)
 * - 仅在 ins_mode=5 首次进入 (flag_1==1) 时统一调用
 */
void ins_auto_special_point_load2(void)
{
    /* 读取页 95: 特殊点数量 */
    flash_read_page_to_buffer(0, INS_AUTO_FLASH_PAGE_SP2_META, FLASH_PAGE_LENGTH);
    g_ins_auto.sp2_count = flash_union_buffer[0].uint16_type;
    flash_buffer_clear();

    /* 校验数量 */
    if (g_ins_auto.sp2_count == 0 || g_ins_auto.sp2_count > INS_AUTO_MAX_SPECIAL_POINTS2)
    {
        g_ins_auto.sp2_count = 0;
        return;
    }

    /* 读取页 96: 特殊点坐标 */
    flash_read_page_to_buffer(0, INS_AUTO_FLASH_PAGE_SP2_DATA, FLASH_PAGE_LENGTH);
    for (uint16 i = 0; i < g_ins_auto.sp2_count; i++)
    {
        uint8 buffer_idx = i;
        g_ins_auto.special_points2[i].x = readFlash_to_double1(0, buffer_idx);
        g_ins_auto.special_points2[i].y = readFlash_to_double1(1, buffer_idx);
    }
    flash_buffer_clear();

    /* 初始化导航状态 */
    g_ins_auto.sp2_current = 0;
    g_ins_auto.Special_point_get2 = 0;
}

/*
 * subject3: 特殊点导航更新 (每 16ms 调用)
 * - 检测到达进入点 → 触发坐标跳变
 */
void ins_auto_special_point_update2(void)
{
    if (g_ins_auto.sp2_count == 0 || g_ins_auto.sp2_current >= g_ins_auto.sp2_count)
        return;

    if (g_ins_auto.Special_point_get2)
        return;

    Coordinates sp = g_ins_auto.special_points2[g_ins_auto.sp2_current];
    double dx = sp.x - cod_realtime.x;
    double dy = sp.y - cod_realtime.y;
    double dist_to_sp = sqrt(dx * dx + dy * dy);

    /* 20cm 以内: 到达进入点 */
    if (dist_to_sp < 20.0 && dist_to_sp > 0.001)
    {
        ins_auto_special_point_enter2(0);
    }
}

/*
 * subject3: 进入特殊事件模式
 * - Special_point_get2=1 关闭惯导/坐标计算/惯导转向
 * - 将退出点(n+1)坐标替换小车当前位置
 * - sp2_current+=2 指向下一事件进入点
 */
void ins_auto_special_point_enter2(float special_speed)
{
    g_ins_auto.Special_point_get2 = 1;

    /* 关闭惯导转向 */
    yaw_ins = 0;
    Yao.Outp_turn = 0;

    /* 保存当前速度到 subject3 专用备份, 设置特殊事件速度 */
    temp_speed2 = Target_Speed;
    Target_Speed = special_speed;

    /* 用退出点坐标(n+1)替换当前坐标 */
    if (g_ins_auto.sp2_current + 1 < g_ins_auto.sp2_count)
    {
        cod_realtime = g_ins_auto.special_points2[g_ins_auto.sp2_current + 1];
    }

    /* 跳到下一个事件的进入点(n+2) */
    g_ins_auto.sp2_current += 2;
}

/*
 * subject3: 退出特殊事件模式
 * - 恢复惯导转向和坐标计算
 * - 启动 40ms 延时计时，确保小车稳定后再恢复速度
 */
void ins_auto_special_point_exit2(void)
{
    g_ins_auto.Special_point_get2 = 0;
    Target_Speed = 0;
    yaw_ins = 0;
    turn_mode = 7; /* 恢复惯导转向模式 */

    /* 启动 40ms 延时计时: 25 次 × 40ms = 1000ms = 1 秒后到时 */
    g_exit2_delay_enable = 1;
    g_exit2_delay_counter = 0;
    g_exit2_timeout_flag = 0;
}

/* ==================================================================
 *  调试输出函数 (40ms 中断调用)
 * ================================================================== */

/*
 * 通过无线串口发送调试信息
 * 格式: [FLASH_DEBUG] MEM: wp_count=X, is_nav=X, is_rec=X | FLASH: magic=0xXXXXXXXX, wp_count=X | POS: x=X.XX, y=X.XX
 */
void ins_auto_debug_output(void)
{
    /* 清空缓冲区并读取 Flash 元数据页 */
    flash_buffer_clear();
    flash_read_page_to_buffer(0, INS_AUTO_FLASH_PAGE_META, FLASH_PAGE_LENGTH);

    uint32_t magic_read = flash_union_buffer[0].uint32_type;
    uint32_t wp_count_read = flash_union_buffer[1].uint32_type;

    /* 格式化调试信息 */
    char debug_buf[150];
    int len = sprintf(debug_buf,
                      "[FLASH_DEBUG] MEM: wp_count=%d, is_nav=%d, is_rec=%d | FLASH: magic=0x%08X, wp_count=%d | POS: x=%.2f, y=%.2f\r\n",
                      g_ins_auto.wp_count,
                      g_ins_auto.is_navigating,
                      g_ins_auto.is_recording,
                      magic_read,
                      wp_count_read,
                      cod_realtime.x,
                      cod_realtime.y);

    /* 发送到无线串口 */
    if (len > 0 && len < (int)sizeof(debug_buf))
    {
        wireless_uart_send_string(debug_buf);
    }
}

/*
 * 设置配置参数
 */
void ins_auto_set_config(float record_dist, float lookahead, float arrival_thresh)
{
    if (record_dist > 0.0f)
    {
        g_ins_auto.config.record_distance = record_dist;
    }
    if (lookahead > 0.0f)
    {
        g_ins_auto.config.lookahead_distance = lookahead;
        ins_pure_pursuit_set_lookahead(lookahead);
    }
    if (arrival_thresh > 0.0f)
    {
        g_ins_auto.config.arrival_threshold = arrival_thresh;
    }
}

/*
 * 获取当前航点索引
 */
uint16 ins_auto_get_current_wp(void)
{
    return g_ins_auto.wp_current;
}

/*
 * 获取航点总数
 */
uint16 ins_auto_get_total_wp(void)
{
    return g_ins_auto.wp_count;
}
