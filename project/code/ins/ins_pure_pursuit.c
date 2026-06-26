/*
 * ins_pure_pursuit.c
 *
 *  Created on: 2026年6月7日
 *      Author: LateRain
 *
 *  Pure Pursuit 算法实现 —— 平滑路径跟踪
 *  ==========================================
 */

#include "zf_common_headfile.h"
#include "ins_pure_pursuit.h"
#include <math.h>

/* ==================================================================
 *  全局变量
 * ================================================================== */

static PurePursuit_State g_pp_state = {0};

/* ==================================================================
 *  辅助函数实现
 * ================================================================== */

/*
 * 计算两点间的距离
 */
float ins_pure_pursuit_distance(Coordinates p1, Coordinates p2)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return (float)sqrt(dx * dx + dy * dy);
}

/*
 * 计算从 p1 到 p2 的方位角 (度, 0~360°)
 */
float ins_pure_pursuit_angle(Coordinates p1, Coordinates p2)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double angle_rad = atan2(dy, dx);           /* -π ~ π */
    double angle_deg = RAD_TO_ANGLE(angle_rad); /* -180 ~ 180 */

    /* 归一化到 0~360° */
    if (angle_deg < 0)
    {
        angle_deg += 360.0;
    }

    return (float)angle_deg;
}

/*
 * 角度归一化到 [-180, 180] 范围
 */
float ins_pure_pursuit_normalize_angle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/* ==================================================================
 *  Pure Pursuit 核心算法实现
 * ================================================================== */

/*
 * 初始化 Pure Pursuit 模块
 */
void ins_pure_pursuit_init(void)
{
    g_pp_state.lookahead_distance = PURE_PURSUIT_DEFAULT_LOOKAHEAD;
    g_pp_state.curvature = 0.0f;
    g_pp_state.target_yaw = 0.0f;
    g_pp_state.target_wp_index = 0;
    g_pp_state.distance_to_path = 0.0f;
    g_pp_state.angle_to_path = 0.0f;
}

/*
 * 寻找前瞻点
 *
 * 算法:
 *   1. 从当前航点开始，沿路径累加距离
 *   2. 当累计距离 >= 前瞻距离时，在该段路径上进行线性插值
 *   3. 返回插值后的前瞻点坐标
 */
Coordinates ins_pure_pursuit_find_lookahead(
    const Coordinates *waypoints,
    uint16 wp_count,
    uint16 wp_start,
    Coordinates current_pos,
    float lookahead_dist,
    uint16 *out_wp_index)
{
    Coordinates lookahead_point = current_pos;
    uint16 i;
    float accumulated_dist = 0.0f;

    /* 参数检查 */
    if (waypoints == NULL || wp_count == 0 || wp_start >= wp_count)
    {
        if (out_wp_index)
            *out_wp_index = wp_start;
        return current_pos;
    }

    /* 从当前航点开始搜索 */
    for (i = wp_start; i < wp_count - 1; i++)
    {
        float segment_length = ins_pure_pursuit_distance(waypoints[i], waypoints[i + 1]);

        /* 如果这段路径长度足够 */
        if (accumulated_dist + segment_length >= lookahead_dist)
        {
            /* 在这段路径上进行线性插值 */
            float remaining_dist = lookahead_dist - accumulated_dist;
            float ratio = remaining_dist / segment_length;

            /* 限制 ratio 在 [0, 1] 范围内 */
            if (ratio > 1.0f)
                ratio = 1.0f;
            if (ratio < 0.0f)
                ratio = 0.0f;

            /* 线性插值 */
            lookahead_point.x = waypoints[i].x + ratio * (waypoints[i + 1].x - waypoints[i].x);
            lookahead_point.y = waypoints[i].y + ratio * (waypoints[i + 1].y - waypoints[i].y);

            if (out_wp_index)
                *out_wp_index = i + 1;
            return lookahead_point;
        }

        accumulated_dist += segment_length;
    }

    /* 如果路径总长度不足前瞻距离，返回最后一个航点 */
    lookahead_point = waypoints[wp_count - 1];
    if (out_wp_index)
        *out_wp_index = wp_count - 1;

    return lookahead_point;
}

/*
 * 计算转向角
 *
 * Pure Pursuit 核心公式:
 *   1. 计算前瞻点在机器人坐标系下的位置 (x_local, y_local)
 *   2. 转弯半径 R = Ld? / (2 × y_local)
 *   3. 曲率 κ = 1/R = 2 × y_local / Ld?
 *   4. 转向角 δ = atan(κ × L) = atan(2 × y_local × L / Ld?)
 *      其中 L 是轴距
 */
float ins_pure_pursuit_calc_steering(
    Coordinates current_pos,
    float current_yaw,
    Coordinates target_pos,
    float lookahead_dist)
{
    /* 1. 计算前瞻点在全局坐标系下的向量 */
    double dx = target_pos.x - current_pos.x;
    double dy = target_pos.y - current_pos.y;

    /* 2. 转换到机器人坐标系 */
    /* 机器人坐标系: X轴朝前，Y轴朝左 */
    /* 需要将全局坐标系旋转 -current_yaw */
    double yaw_rad = ANGLE_TO_RAD(current_yaw);
    double cos_yaw = cos(yaw_rad);
    double sin_yaw = sin(yaw_rad);

    /* 旋转变换 (逆时针旋转 -yaw) */
    double x_local = dx * cos_yaw + dy * sin_yaw;  /* 前向距离 */
    double y_local = -dx * sin_yaw + dy * cos_yaw; /* 横向距离 (左正右负) */

    /* 3. 计算曲率 */
    /* κ = 2 × y_local / Ld? */
    double lookahead_sq = lookahead_dist * lookahead_dist;
    double curvature = 0.0;

    if (lookahead_sq > 0.001)
    { /* 防止除零 */
        curvature = 2.0 * y_local / lookahead_sq;
    }

    /* 4. 计算目标偏航角 */
    /* 目标偏航角 = 从当前位置指向前瞻点的方位角 */
    /* 使用 atan2 计算方位角，范围 [-π, +π] */
    double target_yaw_rad = atan2(dy, dx);

    /* 转换为度数，保持 [-180°, 180°] 范围，与 imu660ra.eulerAngle.yaw 一致 */
    double target_yaw = RAD_TO_ANGLE(target_yaw_rad);
    if (target_yaw > 180)
        target_yaw -= 360;
    if (target_yaw < -180)
        target_yaw += 360;
    

    return (float)target_yaw;
}

/*
 * Pure Pursuit 主计算函数
 */
PurePursuit_State ins_pure_pursuit_update(
    const Coordinates *waypoints,
    uint16 wp_count,
    uint16 wp_current,
    Coordinates current_pos,
    float current_yaw,
    float lookahead_dist)
{
    /* 参数检查 */
    if (waypoints == NULL || wp_count == 0 || wp_current >= wp_count)
    {
        g_pp_state.target_yaw = current_yaw;
        return g_pp_state;
    }

    /* 限制前瞻距离在合理范围 */
    if (lookahead_dist < PURE_PURSUIT_MIN_LOOKAHEAD)
    {
        lookahead_dist = PURE_PURSUIT_MIN_LOOKAHEAD;
    }
    if (lookahead_dist > PURE_PURSUIT_MAX_LOOKAHEAD)
    {
        lookahead_dist = PURE_PURSUIT_MAX_LOOKAHEAD;
    }

    g_pp_state.lookahead_distance = lookahead_dist;

    /* 1. 寻找前瞻点 */
    Coordinates lookahead_point = ins_pure_pursuit_find_lookahead(
        waypoints, wp_count, wp_current, current_pos, lookahead_dist,
        &g_pp_state.target_wp_index);

    /* 2. 计算转向角 */
    g_pp_state.target_yaw = ins_pure_pursuit_calc_steering(
        current_pos, current_yaw, lookahead_point, lookahead_dist);

    /* 3. 计算调试信息 */
    /* 到路径的横向距离 */
    if (wp_current < wp_count - 1)
    {
        /* 计算到当前路径段的距离 */
        Coordinates path_start = waypoints[wp_current];
        Coordinates path_end = waypoints[wp_current + 1];

        /* 使用点到线段距离公式 */
        double line_vec_x = path_end.x - path_start.x;
        double line_vec_y = path_end.y - path_start.y;
        double point_vec_x = current_pos.x - path_start.x;
        double point_vec_y = current_pos.y - path_start.y;

        double line_len_sq = line_vec_x * line_vec_x + line_vec_y * line_vec_y;

        if (line_len_sq > 0.001)
        {
            /* 投影比例 */
            double t = (point_vec_x * line_vec_x + point_vec_y * line_vec_y) / line_len_sq;
            t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t); /* 限制在 [0, 1] */

            /* 最近点 */
            double nearest_x = path_start.x + t * line_vec_x;
            double nearest_y = path_start.y + t * line_vec_y;

            /* 横向距离 */
            double dist_dx = current_pos.x - nearest_x;
            double dist_dy = current_pos.y - nearest_y;
            g_pp_state.distance_to_path = (float)sqrt(dist_dx * dist_dx + dist_dy * dist_dy);
        }
    }

    /* 到路径的角度偏差 */
    float path_angle = ins_pure_pursuit_angle(waypoints[wp_current], waypoints[wp_current + 1]);
    g_pp_state.angle_to_path = ins_pure_pursuit_normalize_angle(path_angle - current_yaw);

    return g_pp_state;
}

/*
 * 设置前瞻距离
 */
void ins_pure_pursuit_set_lookahead(float lookahead)
{
    if (lookahead < PURE_PURSUIT_MIN_LOOKAHEAD)
    {
        lookahead = PURE_PURSUIT_MIN_LOOKAHEAD;
    }
    if (lookahead > PURE_PURSUIT_MAX_LOOKAHEAD)
    {
        lookahead = PURE_PURSUIT_MAX_LOOKAHEAD;
    }

    g_pp_state.lookahead_distance = lookahead;
}

/*
 * 获取当前 Pure Pursuit 状态
 */
PurePursuit_State ins_pure_pursuit_get_state(void)
{
    return g_pp_state;
}
