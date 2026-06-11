/*
 * ins_auto_record.h
 *
 *  Created on: 2026年6月7日
 *      Author: LateRain
 *
 *  自动打点惯导模块 —— 固定距离打点 + Pure Pursuit 循迹
 *  ========================================================
 *
 *  设计背景:
 *    原 Ins.c 使用按键触发打点，需要人工干预，且最大航点数仅 30 个。
 *    本模块实现:
 *      1. 固定距离自动打点（每 10 单位距离记录一个航点）
 *      2. Pure Pursuit 算法平滑循迹（前瞻距离可配置）
 *      3. 扩展容量至 200 个航点
 *      4. 独立 Flash 存储区域（页 60-70）
 *
 *  与原 Ins.c 的关系:
 *    - 完全独立模块，不修改 Ins.c
 *    - 复用 Ins.c 的 cod_realtime 实时坐标
 *    - 复用 Ins.c 的 get_target() 计算距离/方位
 *    - 输出 dis_ins / yaw_ins 供 Interrupt.c 的 turn_mode==7 使用
 *
 *  调用链路:
 *    Interrupt.c 16ms → ins_auto_record_update() → 自动打点逻辑
 *    main_cm7_0.c     → ins_auto_record_navigation() → Pure Pursuit 循迹
 *    Interrupt.c 4ms  → turn_mode==7 → yaw_ins 作为 PID 目标角
 *
 *  Flash 布局:
 *    页 60: 元数据 (magic + wp_count + config)
 *    页 61-70: 航点坐标数据 (每页 128 个航点，共 10 页 = 1280 个航点容量)
 *    实际使用: 200 个航点占 200×2×2 = 800 个 uint32，约 2 页
 */

#ifndef _INS_AUTO_RECORD_H_
#define _INS_AUTO_RECORD_H_

#include "zf_common_typedef.h"
#include "Ins.h" /* 复用 Coordinates 结构体和 get_target() */

/* ==================================================================
 *  容量常量
 * ================================================================== */

#define INS_AUTO_MAX_WAYPOINTS 4096 /* 最大航点数 (32页，每页128个) */

/* Flash 页分配 */
#define INS_AUTO_FLASH_PAGE_META 60  /* 元数据页 */
#define INS_AUTO_FLASH_PAGE_DATA 61  /* 航点数据起始页 (页 61-92，共 32 页) */
#define INS_AUTO_FLASH_PAGE_COUNT 32 /* 航点数据页数 */

/*
 * Magic Number = 0x4155544F = ASCII "AUTO"
 * 用于校验 Flash 数据有效性
 */
// #define INS_AUTO_MAGIC 0x4155544Fu

/* ==================================================================
 *  配置参数
 * ================================================================== */

/* 自动打点距离阈值 (单位: 编码器脉冲) */
#define INS_AUTO_RECORD_DISTANCE 40.0

/* Pure Pursuit 前瞻距离
 * 建议值: 打点距离的 3-4 倍 = 30-40
 * 前瞻距离越大，轨迹越平滑但转弯精度降低
 * 前瞻距离越小，跟踪精度越高但可能震荡
 */
#define INS_AUTO_LOOKAHEAD_DEFAULT 80.0

/* 到达航点判定阈值 (单位: 编码器脉冲) */
#define INS_AUTO_ARRIVAL_THRESHOLD 15.0

/* Pure Pursuit 最小转弯半径限制 (防止除零) */
#define INS_AUTO_MIN_RADIUS 1

/* ==================================================================
 *  数据结构
 * ================================================================== */

/*
 * 自动打点配置
 */
typedef struct
{
    float record_distance;    /* 打点距离阈值 */
    float lookahead_distance; /* Pure Pursuit 前瞻距离 */
    float arrival_threshold;  /* 到达判定阈值 */
    uint8 auto_record_enable; /* 自动打点使能: 0=禁用, 1=启用 */
} InsAuto_Config;

/*
 * 自动打点状态
 */
typedef struct
{
    Coordinates waypoints[INS_AUTO_MAX_WAYPOINTS]; /* 航点数组 */
    uint16 wp_count;                               /* 已记录航点数 */
    uint16 wp_current;                             /* 当前目标航点索引 (导航模式) */

    /* 自动打点状态 */
    double last_record_distance; /* 上次打点时的累计距离 */
    uint8 is_recording;          /* 是否正在录制: 0=停止, 1=录制中 */

    /* Pure Pursuit 状态 */
    uint8 is_navigating; /* 是否正在导航: 0=停止, 1=导航中 */
    uint8 nav_finished;  /* 导航是否完成: 0=未完成, 1=已完成 */

    /* 配置参数 */
    InsAuto_Config config;

} InsAuto_State;

/* ==================================================================
 *  全局变量 (extern)
 * ================================================================== */

extern InsAuto_State g_ins_auto; /* 全局状态实例 */

/* ==================================================================
 *  API 函数声明
 * ================================================================== */

/*
 * 初始化自动打点模块
 * - 初始化状态结构体
 * - 从 Flash 加载配置和航点数据（如果有）
 * - 在 main() 中调用一次
 */
void ins_auto_record_init(void);

/*
 * 自动打点更新 (每 16ms 由中断调用)
 * - 检查是否达到打点距离阈值
 * - 自动记录当前坐标到航点数组
 * - 在 Interrupt.c 的 16ms 中断中调用
 *
 * 参数:
 *   speed: 当前速度 (编码器单位)
 *   yaw: 当前偏航角 (度, 0~359°)
 */
void ins_auto_record_update(int speed, float yaw);

/*
 * 开始自动录制
 * - 清空现有航点
 * - 设置录制标志
 * - 重置距离计数器
 */
void ins_auto_record_start(void);

/*
 * 停止自动录制
 * - 清除录制标志
 * - 可选: 自动保存到 Flash
 */
void ins_auto_record_stop(void);

/*
 * 保存航点数据到 Flash
 * - 保存元数据到页 60
 * - 保存航点坐标到页 61-70
 * - 返回: 0=成功, 1=失败
 */
uint8 ins_auto_save_to_flash(void);

/*
 * 从 Flash 加载航点数据
 * - 校验 Magic Number
 * - 加载元数据和航点坐标
 * - 返回: 0=成功, 1=失败(无有效数据)
 */
uint8 ins_auto_load_from_flash(void);

/*
 * 清空 Flash 数据
 * - 擦除页 60-70
 * - 重置状态
 */
void ins_auto_clear_flash(void);

/*
 * Pure Pursuit 导航主函数 (在主循环中每帧调用)
 * - 计算前瞻点
 * - 计算转向角
 * - 更新 dis_ins / yaw_ins
 * - 检查航点到达判定
 */
void ins_auto_record_navigation(void);

/*
 * 开始 Pure Pursuit 导航
 * - 从 Flash 加载航点（如果需要）
 * - 设置导航标志
 * - 重置导航状态
 */
void ins_auto_nav_start(void);

/*
 * 停止导航
 * - 清除导航标志
 * - 停车
 */
void ins_auto_nav_stop(void);

/*
 * 设置配置参数
 * - 可在运行时修改打点距离、前瞻距离等
 */
void ins_auto_set_config(float record_dist, float lookahead, float arrival_thresh);

/*
 * 获取当前航点索引和总数
 * - 用于屏幕显示
 */
uint16 ins_auto_get_current_wp(void);
uint16 ins_auto_get_total_wp(void);

/*
 * 调试输出函数 (40ms 中断调用)
 * - 通过无线串口发送 Flash 和导航状态信息
 */
void ins_auto_debug_output(void);

#endif /* _INS_AUTO_RECORD_H_ */
