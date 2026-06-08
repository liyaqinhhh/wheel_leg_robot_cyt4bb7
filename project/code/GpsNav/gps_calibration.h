/*********************************************************************************************************************
 * CYT4BB ˮֿ˻ GPS  -  У׼ƫƲģ
 *
 * ļ: gps_calibration.h
 * ģ: M3 У׼ƫƲ
 * : У׼ͺ
 *
 * ӿԼ: ܼ GPSܼ 3.3 
 ********************************************************************************************************************/

#ifndef _gps_calibration_h_
#define _gps_calibration_h_

#include "zf_common_typedef.h"

//====================================================궨====================================================

#define GPS_DRIFT_CORRECTION_ALPHA    0.02f     // ƫƲٶ (ÿ 100ms  2%)
#define GPS_DRIFT_MIN_SPEED_KMH       5.0f      // Ͳٶ (km/h, GPS ɿ)
#define GPS_DRIFT_MAX_ERROR_DEG       30.0f     // ƫƲ (, ǶЧ)
#define GPS_CAL_MAX_OFFSET_DEG        90.0f     // У׼ƫ (ȳУ׼ʧ)

//========================================================================================================

uint8   gps_cal_startpoint(void);        // У׼:  IMU yaw  GPS λǼƫ
void    gps_cal_drift_correction(void);  // ƫƲ:  GPS ںƫ
float   gps_cal_get_offset(void);        // ȡǰƫ
void    gps_cal_reset(void);             // λƫ

//====================================================ⲿ====================================================

extern float gps_cal_offset_deg;         // ǰ IMU-GPS ƫ []

#endif
