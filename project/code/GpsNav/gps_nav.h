/*********************************************************************************************************************
 * CYT4BB 智能车 GPS 导航 -  核心导航模块头文件
 *
 * 文件: gps_nav.h
 * 模块: M2 核心导航
 * 功能: 状态机与转向输出接口
 *
 * 状态机转换:
 *   IDLE -> CALIBRATING -> NAVIGATING -> ARRIVED -> COMPLETE
 *     |          |             |           |
 *     +----------+             +-----------+
 *   (停止/失败)           (推进航点/走完)
 *
 * 双缓冲架构:
 *   写入端 (主循环 10Hz) -> buf[write_idx]
 *   读取端 (4ms ISR 250Hz) -> buf[read_idx]
 *   交换时原子交换 write_idx <-> read_idx
 *   volatile uint8 写入在 ARM Cortex-M7 上是原子的
 ********************************************************************************************************************/

#ifndef GPS_NAV_H
#define GPS_NAV_H

#include "zf_common_typedef.h"

//====================================================状态枚举====================================================

typedef enum {
    GPS_NAV_IDLE        = 0,    // 空闲 / 停止
    GPS_NAV_CALIBRATING = 1,    // 校准中：计算 IMU-GPS 偏移
    GPS_NAV_NAVIGATING  = 2,    // 导航中：跟踪航点
    GPS_NAV_ARRIVED     = 3,    // 到达航点
    GPS_NAV_COMPLETE    = 4     // 导航完成
} gps_nav_state_enum;

//====================================================数据结构====================================================

/* 转向输出 */
typedef struct {
    float target_bearing_deg;      // 目标方位角 [0, 360)
    float distance_to_wp_m;        // 到航点距离(m)
    float imu_yaw_offset_deg;      // IMU-GPS 偏移修正(度)
} gps_steer_output_t;

/* 双缓冲 */
typedef struct {
    gps_steer_output_t buf[2];     // 双缓冲区
    volatile uint8 write_idx;      // 写入端索引 (主循环写入)
    volatile uint8 read_idx;       // 读取端索引 (4ms ISR 读取)
} gps_steer_pp_t;

//========================================================================================================

#define GPS_NAV_LPF_ALPHA              0.3f    // 低通滤波系数
#define GPS_NAV_ARRIVE_ENTER_M         2.0f    // 航点到达进入距离(m)
#define GPS_NAV_ARRIVE_LEAVE_M         3.5f    // 航点到达离开距离(m)
#define GPS_NAV_SIGNAL_LOSS_FRAMES     50      // GPS 信号丢失帧数阈值 (10Hz 下 5s = 50帧)
#define GPS_NAV_FIRST_FRAME_MAGIC      999.0f  // 低通滤波初始魔法值（首次直接赋值）

//========================================================================================================

void    gps_nav_init(void);          // 功能: 初始化航点 + 双缓冲 + 状态机
void    gps_nav_proc(void);          // 功能: 10Hz 主循环处理函数
uint8   gps_nav_get_state(void);     // 获取当前导航状态
uint8   gps_nav_set_wp_index(uint8 idx);  // 设置航点索引
void    gps_nav_start(void);         // 功能: 启动导航
void    gps_nav_stop(void);          // 功能: 停止导航，复位到 IDLE + turn_mode=0

//====================================================全局变量====================================================

extern gps_steer_pp_t  gps_steer_pp;    // 双缓冲转向输出
extern uint8           gps_nav_state;   // 导航状态 (gps_nav_state_enum)

/* ISR 读取宏: 从 read_idx 侧读取 */
#define GPS_STEER_READ()  (&gps_steer_pp.buf[gps_steer_pp.read_idx])

#endif /* GPS_NAV_H */
