/*********************************************************************************************************************
 * CYT4BB 智能车 GPS 导航 -  航点管理模块头文件
 *
 * 文件: gps_waypoint.h
 * 模块: M1 航点管理
 * 功能: 航点数据结构、常量定义
 *
 * 接口约定: 参见 GPS导航模块 3.1 节
 * Flash 页: 数据页 50(航点), 元数据页 51(校验)
 ********************************************************************************************************************/

#ifndef _gps_waypoint_h_
#define _gps_waypoint_h_

#include "zf_common_typedef.h"

//====================================================常量定义====================================================

#define GPS_WP_MAX_COUNT           40      // 最大航点数
#define GPS_WP_FLASH_DATA_PAGE     50      // Flash 航点数据页 (sector 0, page 50)
#define GPS_WP_FLASH_META_PAGE    51      // Flash 元数据页 (sector 0, page 51)
#define GPS_WP_MAGIC               0xA5    // 元数据页魔数校验

//====================================================类型定义====================================================

// 航点结构体 (纬度、经度)
typedef struct {
    double lat;    // 纬度 [-90, +90]
    double lng;    // 经度 [-180, +180]
} gps_waypoint_t;

// 航点集合
typedef struct {
    gps_waypoint_t waypoints[GPS_WP_MAX_COUNT];   // 航点数组
    uint8 count;           // 当前航点数量
    uint8 current_index;    // 当前目标航点索引
    uint8 valid;            // 有效标志: 1=有效, 0=无效
} gps_waypoint_set_t;

//========================================================================================================

void        gps_wp_init(void);
uint8       gps_wp_save_to_flash(void);
uint8       gps_wp_add(double lat, double lng);
void        gps_wp_clear(void);
gps_waypoint_t* gps_wp_current(void);
gps_waypoint_t* gps_wp_next(void);
uint8       gps_wp_advance(void);
uint8       gps_wp_get_count(void);
uint8       gps_wp_get_current_index(void);

//====================================================全局变量====================================================

extern gps_waypoint_set_t gps_wp_set;

#endif
