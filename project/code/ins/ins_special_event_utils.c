/*
 * ins_special_event_utils.c
 *
 *  Created on: 2026年7月6日
 *      Author: LateRain
 *
 *  subject3 特殊事件工具函数实现
 *  ================================================================
 */

#include "zf_common_headfile.h"
#include "ins_special_event_utils.h"
#include "ins_auto_record.h"
#include "Interrupt.h"
#include "imu660.h"

/* ---- 外部变量 (来自 Interrupt.c) ---- */
extern volatile float speed_MOTOR; /* 平均轮速 */

/* ==================================================================
 *  定向直走状态机
 *
 *  原理:
 *    不依赖 get_realtime_coordinate() 和 ins_auto_record_update()。
 *    直接使用速度环 PID (Target_Speed) + 偏航角闭环 (turn_mode=3)
 *    实现沿当前偏航角方向直走指定距离。
 *
 *  距离计算:
 *    dist_per_frame = speed_MOTOR × 0.016 × WHEEL_CIRCUMFERENCE_CM
 *    (与 get_realtime_coordinate 使用相同的物理模型)
 * ================================================================== */

InsStraightMotion g_straight = {0};
uint32_t time_layer = 0.0f; /* 预留: 暂未使用 */
float dist = 0.0f;        /* 预留: 暂未使用 */
uint8 flag_time_layer = 0;   /* 预留: 暂未使用 */

/*
 * 启动定向直走
 */
void ins_straight_start(int16 speed, float dist_cm)
{
    if (dist_cm <= 0.0f || speed == 0)
        return;
    
    //time_layer = (uint32_t)(dist_cm / (fabsf(speed) * 0.016f * WHEEL_CIRCUMFERENCE_CM));

    // 这个地方如果是视觉的话要改
    /* 锁定当前 yaw 角为前进方向 */
    Target_Yaw = imu660ra.eulerAngle.yaw;

    flag_time_layer = 1;

    turn_mode = 3;

    /* 设置目标速度 */
    Target_Speed = speed;
    if (flag_time_layer)
    {
        TCount_16ms++;
        dist += speed_MOTOR * 0.016f * WHEEL_CIRCUMFERENCE_CM;
    }

    if(dist >= dist_cm && dist_cm != 0)
    {
        flag_time_layer = 0;
        Target_Speed = 0;
        flag_road_test = 0;
        TCount_16ms = 0;
        Target_Yaw = 0;
        turn_mode = 7;
        yaw_ins = 0;
        
        


        //test
        flag_main = 2;  
        dist = 0.0f;  
    }

    
    
    /* 初始化状态 */
    
}

/*
 * 定向直走状态更新 (每 16ms 调用)
 * 返回: 0=进行中, 1=已完成
 */
//弃用
uint8 ins_straight_update(void)
{
    if (!g_straight.active)
        return 1;

    /* 累加本帧行驶距离
     *   speed_MOTOR (rev/s) × 0.016 (s) × WHEEL_CIRCUMFERENCE_CM (cm/rev) */
    float dist_frame = fabsf(speed_MOTOR * 0.016f * WHEEL_CIRCUMFERENCE_CM);
    g_straight.accum_dist_cm += dist_frame;

    /* 到达判定 */
    if (g_straight.accum_dist_cm >= g_straight.target_dist_cm)
    {
        /* 停车 */
        Target_Speed = 0;
        g_straight.active = 0;
        return 1;
    }

    return 0;
}

/*
 * 查询是否正在运行
 */
//弃用
uint8 ins_straight_is_active(void)
{
    return g_straight.active;
}

/*
 * 强制停止
 */
void ins_straight_stop(void)
{
    Target_Speed = 0;
    g_straight.active = 0;
}

/* ==================================================================
 *  特殊事件编号
 * ================================================================== */

/*
 * 获取当前特殊事件编号 m
 *
 * 约定:
 *   n = sp2_current (当前进入点索引)
 *   m = n / 2       (特殊事件编号)
 *
 * special_points2[] 布局:
 *   [0] = 事件0进入点, [1] = 事件0退出点
 *   [2] = 事件1进入点, [3] = 事件1退出点
 *   ...
 *   [2m]   = 事件m进入点
 *   [2m+1] = 事件m退出点
 *
 * 返回: 当前特殊事件编号 m
 */
uint16 ins_special_event_get_m(void)
{
    if (g_ins_auto.sp2_count == 0 || g_ins_auto.sp2_current >= g_ins_auto.sp2_count)
    {
        return 0;
    }

    return (uint16)((g_ins_auto.sp2_current / 2) + 1);
}