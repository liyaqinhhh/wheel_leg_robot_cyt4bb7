/*
 * ins_pure_pursuit.h
 *
 *  Created on: 2026年6月7日
 *      Author: LateRain
 *
 *  Pure Pursuit 算法模块 —— 平滑路径跟踪
 *  ==========================================
 *
 *  算法原理:
 *    Pure Pursuit 是一种经典的路径跟踪算法，通过在路径前方寻找一个前瞻点，
 *    计算从当前位置到前瞻点的圆弧轨迹，从而实现平滑转向。
 *
 *    核心公式:
 *      1. 前瞻点选择: 在路径上找到距离当前位置 Ld 的点
 *      2. 转弯半径计算: R = Ld? / (2 × x)
 *         其中 x 是前瞻点在机器人坐标系下的横向偏移
 *      3. 转向角: δ = atan2(2 × L × sin(α), Ld)
 *         其中 L 是轴距，α 是前瞻点相对机器人的角度
 *
 *  优势:
 *    - 计算简单，实时性好
 *    - 轨迹平滑，适合高速运动
 *    - 鲁棒性强，对传感器噪声不敏感
 *
 *  参数调优:
 *    - 前瞻距离 Ld: 越大越平滑但精度降低，越小精度高但可能震荡
 *    - 建议值: 打点距离的 3-4 倍
 *
 *  调用链路:
 *    ins_auto_record.c → ins_pure_pursuit_update() → 计算转向角
 */

#ifndef _INS_PURE_PURSUIT_H_
#define _INS_PURE_PURSUIT_H_

#include "zf_common_typedef.h"
#include "Ins.h"  /* 复用 Coordinates 结构体 */

/* ==================================================================
 *  Pure Pursuit 配置
 * ================================================================== */

/* 默认前瞻距离 (单位: 编码器脉冲) */
#define PURE_PURSUIT_DEFAULT_LOOKAHEAD   60.0

/* 最小前瞻距离 (防止过小导致震荡) */
#define PURE_PURSUIT_MIN_LOOKAHEAD       20.0

/* 最大前瞻距离 (防止过大导致切角) */
#define PURE_PURSUIT_MAX_LOOKAHEAD       50.0

/* 轴距 (单位: 编码器脉冲，需根据实际机器人测量) */
#define PURE_PURSUIT_WHEELBASE           17.7

/* ==================================================================
 *  数据结构
 * ================================================================== */

/*
 * Pure Pursuit 状态
 */
typedef struct {
    float lookahead_distance;   /* 当前前瞻距离 */
    float curvature;            /* 当前曲率 (1/转弯半径) */
    float target_yaw;           /* 目标转向角 (度) */
    uint16 target_wp_index;     /* 前瞻点对应的航点索引 */
    
    /* 调试信息 */
    float distance_to_path;     /* 到路径的横向距离 */
    float angle_to_path;        /* 到路径的角度偏差 */
    
} PurePursuit_State;

/* ==================================================================
 *  API 函数声明
 * ================================================================== */

/*
 * 初始化 Pure Pursuit 模块
 * - 设置默认前瞻距离
 */
void ins_pure_pursuit_init(void);

/*
 * Pure Pursuit 主计算函数
 * - 从航点数组中寻找前瞻点
 * - 计算转向角
 * - 更新全局变量 yaw_ins
 *
 * 参数:
 *   waypoints: 航点数组
 *   wp_count: 航点总数
 *   wp_current: 当前航点索引
 *   current_pos: 当前位置
 *   current_yaw: 当前偏航角 (度)
 *   lookahead_dist: 前瞻距离
 *
 * 返回:
 *   PurePursuit_State 结构体，包含计算结果
 */
PurePursuit_State ins_pure_pursuit_update(
    const Coordinates* waypoints,
    uint16 wp_count,
    uint16 wp_current,
    Coordinates current_pos,
    float current_yaw,
    float lookahead_dist
);

/*
 * 寻找前瞻点
 * - 从当前航点开始，沿路径寻找距离当前位置 Ld 的点
 * - 使用线性插值提高精度
 *
 * 参数:
 *   waypoints: 航点数组
 *   wp_count: 航点总数
 *   wp_start: 起始搜索索引
 *   current_pos: 当前位置
 *   lookahead_dist: 前瞻距离
 *
 * 返回:
 *   前瞻点坐标，以及对应的航点索引（通过输出参数）
 */
Coordinates ins_pure_pursuit_find_lookahead(
    const Coordinates* waypoints,
    uint16 wp_count,
    uint16 wp_start,
    Coordinates current_pos,
    float lookahead_dist,
    uint16* out_wp_index
);

/*
 * 计算从 current_pos 到 target_pos 的转向角
 * - 使用 Pure Pursuit 公式计算转弯半径
 * - 转换为转向角
 *
 * 参数:
 *   current_pos: 当前位置
 *   current_yaw: 当前偏航角 (度)
 *   target_pos: 目标点（前瞻点）
 *   lookahead_dist: 前瞻距离
 *
 * 返回:
 *   目标转向角 (度)
 */
float ins_pure_pursuit_calc_steering(
    Coordinates current_pos,
    float current_yaw,
    Coordinates target_pos,
    float lookahead_dist
);

/*
 * 计算两点间的距离
 */
float ins_pure_pursuit_distance(Coordinates p1, Coordinates p2);

/*
 * 计算从 p1 到 p2 的方位角 (度, 0~360°)
 */
float ins_pure_pursuit_angle(Coordinates p1, Coordinates p2);

/*
 * 角度归一化到 [-180, 180] 范围
 */
float ins_pure_pursuit_normalize_angle(float angle);

/*
 * 设置前瞻距离
 * - 可在运行时动态调整
 * - 会自动限制在 [MIN, MAX] 范围内
 */
void ins_pure_pursuit_set_lookahead(float lookahead);

/*
 * 获取当前 Pure Pursuit 状态（用于调试）
 */
PurePursuit_State ins_pure_pursuit_get_state(void);

#endif /* _INS_PURE_PURSUIT_H_ */
