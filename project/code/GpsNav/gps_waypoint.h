/*********************************************************************************************************************
 * CYT4BB ˮֿ˻ GPS  -  ʵ㹹ģ
 *
 * ļ: gps_waypoint.h
 * ģ: M1 ʵ㹹
 * : ʵͶꡢͺ
 *
 * ӿԼ: ܼ GPSܼ 3.1 
 * Flash ҳ:  50(ʵ),  51(Ԫ)
 ********************************************************************************************************************/

#ifndef _gps_waypoint_h_
#define _gps_waypoint_h_

#include "zf_common_typedef.h"

//====================================================궨====================================================

#define GPS_WP_MAX_COUNT           40      // ʵ
#define GPS_WP_FLASH_DATA_PAGE     50      // Flash ʵҳ (sector 0, page 50)
#define GPS_WP_FLASH_META_PAGE    51      // Flash Ԫҳ (sector 0, page 51)
#define GPS_WP_MAGIC               0xA5    // ԪҳħУ

//====================================================Ͷ====================================================

// ʵ (γȡ)
typedef struct {
    double lat;    // γ [-90, +90]
    double lng;    // [-180, +180]
} gps_waypoint_t;

// ʵ弯
typedef struct {
    gps_waypoint_t waypoints[GPS_WP_MAX_COUNT];   // ʵ
    uint8 count;           // ǰʵ
    uint8 current_index;    // ǰĿʵ
    uint8 valid;            // Ч־: 1=Ч, 0=Ч
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

//====================================================ⲿ====================================================

extern gps_waypoint_set_t gps_wp_set;

#endif
