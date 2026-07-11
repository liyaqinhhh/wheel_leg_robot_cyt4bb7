/*
 * ins_special_event_utils.h
 *
 *  Created on: 2026年7月6日
 *      Author: LateRain
 *
 *  subject3 特殊事件工具函数
 *  ================================================================
 *
 *  功能: 提供特殊事件编号 m 的计算。
 *
 *  约定:
 *    n = sp2_current (当前进入点索引，偶数: 0, 2, 4, ...)
 *    m = n / 2       (特殊事件编号: 0, 1, 2, ...)
 *
 *  第 m 个特殊事件由 special_points2[2m] (进入点) 和
 *  special_points2[2m+1] (退出点) 组成。
 */

#ifndef _INS_SPECIAL_EVENT_UTILS_H_
#define _INS_SPECIAL_EVENT_UTILS_H_

#include "zf_common_typedef.h"

/*
 * 获取当前特殊事件编号 m
 * - m = sp2_current / 2
 * - sp2_current 始终指向偶数索引（进入点），故结果为整数
 * - 返回: m 值，若无特殊点则返回 0
 */
uint16 ins_special_event_get_m(void);

/* ==================================================================
 *  定向直走函数 (不依赖 get_realtime_coordinate / ins_auto_record_update)
 * ================================================================== */

/*
 * 启动定向直走
 * - 锁定当前 yaw 角为目标方向
 * - 设置 turn_mode=3 (偏航角度闭环走直线)
 * - 设置目标速度
 * - 在 16ms 中断中调用 ins_straight_update() 推动状态机
 *
 * 参数:
 *   speed:  目标速度 (编码器单位, 同 Target_Speed)
 *   dist_cm: 目标距离 (厘米)
 */
void ins_straight_start(int16 speed, float dist_cm);

/*
 * 定向直走状态更新 (每 16ms 调用一次)
 * - 累加行驶距离 (基于 speed_MOTOR × 0.016 × WHEEL_CIRCUMFERENCE_CM)
 * - 到达目标距离后自动停车 (Target_Speed=0)
 * - 返回: 0=进行中, 1=已完成
 */
uint8 ins_straight_update(void);

/*
 * 查询定向直走是否正在运行
 * - 返回: 0=空闲, 1=运行中
 */
uint8 ins_straight_is_active(void);

/*
 * 强制停止定向直走
 * - 清零 Target_Speed, 重置内部状态
 */
void ins_straight_stop(void);

#endif /* _INS_SPECIAL_EVENT_UTILS_H_ */