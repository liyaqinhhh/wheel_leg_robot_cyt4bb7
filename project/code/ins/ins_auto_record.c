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

    /* 检查航点数组是否已满 */
    if (g_ins_auto.wp_count >= INS_AUTO_MAX_WAYPOINTS)
    {
        return; /* 已满，停止录制 */
    }

    /* 计算本次移动距离 */
    double move_dist = fabs((double)speed * 0.016); /* 16ms 时间间隔 */

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
 * 保存航点数据到 Flash
 *
 * 以 Ins.c ins_mode=0 的 Flash 操作为模板
 * Flash 布局:
 *   页 60: 元数据 (magic + wp_count + config)
 *   页 61-63: 航点坐标数据 (3 页，每页 128 个航点，共 384 个容量)
 */
// uint8 ins_auto_save_to_flash(void)
// {
//     uint16 i;

//     /* ========== 1. 保存元数据到页 60 ========== */

//     /* 先擦除旧数据 */
//     if (flash_check(0, INS_AUTO_FLASH_PAGE_META))
//         flash_erase_page(0, INS_AUTO_FLASH_PAGE_META);

//     /* 写入元数据 */
//     if (!flash_check(0, INS_AUTO_FLASH_PAGE_META))
//     {
//         /* 清空缓冲区 */
//         memset(flash_union_buffer, 0xFF, sizeof(flash_union_buffer));

//         /* 写入 Magic */
//         //flash_union_buffer[0].uint32_type = INS_AUTO_MAGIC;

//         /* 写入航点数量 */
//         flash_union_buffer[1].uint32_type = g_ins_auto.wp_count;

//         /* 写入配置参数 */
//         write_double_to_flash(g_ins_auto.config.record_distance, 2);
//         write_double_to_flash(g_ins_auto.config.lookahead_distance, 4);
//         write_double_to_flash(g_ins_auto.config.arrival_threshold, 6);

//         /* 写入 Flash */
//         flash_write_page_from_buffer(0, INS_AUTO_FLASH_PAGE_META, FLASH_PAGE_LENGTH);
//         flash_buffer_clear();
//     }

//     /* ========== 2. 保存航点数据到页 61-63 ========== */

//     /* 计算需要的页数 (每页 128 个航点) */
//     uint16 pages_needed = (g_ins_auto.wp_count + 127) / 128;
//     if (pages_needed > 3) pages_needed = 3;  /* 最多 3 页 */

//     for (uint16 page_idx = 0; page_idx < pages_needed; page_idx++)
//     {
//         uint16 page_num = INS_AUTO_FLASH_PAGE_DATA + page_idx;

//         /* 先擦除旧数据 */
//         if (flash_check(0, page_num))
//             flash_erase_page(0, page_num);

//         /* 写入航点数据 */
//         if (!flash_check(0, page_num))
//         {
//             /* 清空缓冲区 */
//             memset(flash_union_buffer, 0xFF, sizeof(flash_union_buffer));

//             /* 写入该页的航点数据 */
//             uint16 wp_start = page_idx * 128;
//             uint16 wp_end = wp_start + 128;
//             if (wp_end > g_ins_auto.wp_count) {
//                 wp_end = g_ins_auto.wp_count;
//             }

//             uint16 buffer_idx = 0;
//             for (i = wp_start; i < wp_end; i++) {
//                 write_double_to_flash(g_ins_auto.waypoints[i].x, buffer_idx);
//                 write_double_to_flash(g_ins_auto.waypoints[i].y, buffer_idx + 2);
//                 buffer_idx += 4;
//             }

//             /* 写入 Flash */
//             flash_write_page_from_buffer(0, page_num, FLASH_PAGE_LENGTH);
//             flash_buffer_clear();
//         }
//     }

//     return 0;  /* 成功 */
// }

// /*
//  * 从 Flash 加载航点数据
//  *
//  * 以 Ins.c ins_mode=1 的 Flash 操作为模板
//  */
// uint8 ins_auto_load_from_flash(void)
// {
//     uint16 i;

//     /* ========== 1. 加载元数据从页 60 ========== */

//     /* 读取元数据页 */
//     flash_read_page_to_buffer(0, INS_AUTO_FLASH_PAGE_META, FLASH_PAGE_LENGTH);

//     /* 校验 Magic */
//     // if (flash_union_buffer[0].uint32_type != INS_AUTO_MAGIC) {
//     //     flash_buffer_clear();
//     //     return 1;  /* 无有效数据 */
//     // }

//     /* 读取航点数量 */
//     g_ins_auto.wp_count = (uint16)flash_union_buffer[1].uint32_type;

//     /* 限制航点数量在有效范围 */
//     if (g_ins_auto.wp_count > INS_AUTO_MAX_WAYPOINTS) {
//         g_ins_auto.wp_count = INS_AUTO_MAX_WAYPOINTS;
//     }

//     /* 读取配置参数 */
//     g_ins_auto.config.record_distance = (float)read_double_from_flash(2);
//     g_ins_auto.config.lookahead_distance = (float)read_double_from_flash(4);
//     g_ins_auto.config.arrival_threshold = (float)read_double_from_flash(6);

//     flash_buffer_clear();

//     /* ========== 2. 加载航点数据从页 61-63 ========== */

//     if (g_ins_auto.wp_count == 0) {
//         return 0;  /* 无航点数据 */
//     }

//     /* 计算需要的页数 */
//     uint16 pages_needed = (g_ins_auto.wp_count + 127) / 128;
//     if (pages_needed > 3) pages_needed = 3;

//     for (uint16 page_idx = 0; page_idx < pages_needed; page_idx++)
//     {
//         uint16 page_num = INS_AUTO_FLASH_PAGE_DATA + page_idx;

//         /* 读取 Flash 页 */
//         flash_read_page_to_buffer(0, page_num, FLASH_PAGE_LENGTH);

//         /* 读取该页的航点数据 */
//         uint16 wp_start = page_idx * 128;
//         uint16 wp_end = wp_start + 128;
//         if (wp_end > g_ins_auto.wp_count) {
//             wp_end = g_ins_auto.wp_count;
//         }

//         uint16 buffer_idx = 0;
//         for (i = wp_start; i < wp_end; i++) {
//             g_ins_auto.waypoints[i].x = read_double_from_flash(buffer_idx);
//             g_ins_auto.waypoints[i].y = read_double_from_flash(buffer_idx + 2);
//             buffer_idx += 4;
//         }

//         flash_buffer_clear();
//     }

//     return 0;
// }

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

    /* 检查当前航点索引是否有效 */
    if (g_ins_auto.wp_current >= g_ins_auto.wp_count)
    {
        /* 已到达最后一个航点 */
        g_ins_auto.nav_finished = 1;
        ins_auto_nav_stop();
        return;
    }

    /* 1. 使用 Pure Pursuit 计算目标转向角 */
    PurePursuit_State pp_state = ins_pure_pursuit_update(
        g_ins_auto.waypoints,
        g_ins_auto.wp_count,
        g_ins_auto.wp_current,
        cod_realtime,
        imu660ra.eulerAngle.yaw, /* 当前偏航角 */
        g_ins_auto.config.lookahead_distance);

    /* 2. 更新全局变量 yaw_ins (供 Interrupt.c 的 turn_mode==7 使用) */
    yaw_ins = pp_state.target_yaw;

    /* 3. 计算到当前航点的距离 (用于到达判定) */
    Coordinates current_wp = g_ins_auto.waypoints[g_ins_auto.wp_current];
    double dx = current_wp.x - cod_realtime.x;
    double dy = current_wp.y - cod_realtime.y;
    dis_ins = sqrt(dx * dx + dy * dy);

    /* 4. 检查是否到达当前航点 */
    if (dis_ins < g_ins_auto.config.arrival_threshold && dis_ins > 0.001)
    {
        /* 到达当前航点，切换到下一个 */
        g_ins_auto.wp_current++;

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

    Yao.Outp_turn = 0;

    /* 4. 停止录制和导航标志 */
    flag_2 = 0;
    g_ins_auto.is_recording = 0;

    /* 注意：不清零平衡控制输出，不设置 flag_stop，让小车继续平衡并沿直线走 */
}

/* ==================================================================
 *  配置和辅助函数
 * ================================================================== */

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
