/*********************************************************************************************************************
 * CYT4BB ˮֿ˻ GPS  -  ״̬ģͷ
 *
 * ļ: gps_nav.h
 * ģ: M2 ״̬
 * : ״̬ö١ƹ塢ꡢ
 *
 * ״̬ת:
 *   IDLE → CALIBRATING → NAVIGATING → ARRIVED → COMPLETE
 *     ↑        |              |           |
 *     └────────┘              └───────────┘
 *   (ʧ/ֹͣ)           (лһʵ)
 *
 * ƹԭ:
 *   д (main while 10Hz)  buf[write_idx]
 *    (4ms ISR 250Hz)     buf[read_idx]
 *   ɺԭӽ write_idx ↔ read_idx
 *    volatile uint8 д ARM Cortex-M7 ԭӣ޹ж
 ********************************************************************************************************************/

#ifndef GPS_NAV_H
#define GPS_NAV_H

#include "zf_common_typedef.h"

//====================================================״̬ö====================================================

typedef enum {
    GPS_NAV_IDLE        = 0,    //  / ֹͣ
    GPS_NAV_CALIBRATING = 1,    // У׼У IMU-GPS ƫ
    GPS_NAV_NAVIGATING  = 2,    // Уʵ㵼
    GPS_NAV_ARRIVED     = 3,    // ѵǰʵ
    GPS_NAV_COMPLETE    = 4     // ʵȫ
} gps_nav_state_enum;

//====================================================ƹ====================================================

/* Ƶתݿ */
typedef struct {
    float target_bearing_deg;      // Ŀ귽λ [0, 360)˲
    float distance_to_wp_m;        // Ŀʵ
    float imu_yaw_offset_deg;      // IMU-GPS ƫ
} gps_steer_output_t;

/* ƹ */
typedef struct {
    gps_steer_output_t buf[2];     // ˫
    volatile uint8 write_idx;      // д ( main while ޸)
    volatile uint8 read_idx;       //  ( 4ms ISR ȡ)
} gps_steer_pp_t;

//========================================================================================================

#define GPS_NAV_LPF_ALPHA              0.3f    // ˲ϵ
#define GPS_NAV_ARRIVE_ENTER_M         2.0f    // ֵʵжϽֵ
#define GPS_NAV_ARRIVE_LEAVE_M         3.5f    // ֵʵ뿪ֵ
#define GPS_NAV_SIGNAL_LOSS_FRAMES     50      // GPS ǳ֡ (10Hz × 5s = 50)
#define GPS_NAV_FIRST_FRAME_MAGIC      999.0f  // ˲־ (ֵʾδʼ)

//========================================================================================================

void    gps_nav_init(void);          // ʼ: ʵ + ƹ + ״̬
void    gps_nav_proc(void);          // : 10Hz  while 
uint8   gps_nav_get_state(void);     // ȡǰ״̬
uint8   gps_nav_set_wp_index(uint8 idx);  // лĿʵ
void    gps_nav_start(void);         // ʼ: У׼
void    gps_nav_stop(void);          // ֹͣ: IDLE + turn_mode=0

//====================================================ȫ====================================================

extern gps_steer_pp_t  gps_steer_pp;    // ƹȫʵ
extern uint8           gps_nav_state;   // ǰ״̬ (gps_nav_state_enum)

/* ISR ٶȡ: ȡ read_idx ָĻ */
#define GPS_STEER_READ()  (&gps_steer_pp.buf[gps_steer_pp.read_idx])

#endif /* GPS_NAV_H */
