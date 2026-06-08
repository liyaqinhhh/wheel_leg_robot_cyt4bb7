/*********************************************************************************************************************
 * CYT4BB ˮֿ˻ GPS  -  У׼ƫƲģʵ
 *
 * ļ: gps_calibration.c
 * ģ: M3 У׼ƫƲ
 * : У׼ + ƫƲ
 *
 * ʵ:
 *   T05: У׼ (gps_cal_startpoint)
 *   T06: ƫƲ (gps_cal_drift_correction)
 *
 * У׼ԭ:
 *   ǰ: ˻ͣڵһʵ, ڶʵ
 *   : offset = GPSλǼ - IMU_yaw
 *   : GPS_bearing = IMU_yaw + offset
 *
 * ƫƲԭ:
 *   : GPS Ч + ٶ > 5km/h +  < 30
 *   : offset += error * alpha (alpha=0.02, ÿ 100ms  2%)
 *   :  GPS ǶЧ, ƫƲ
 ********************************************************************************************************************/

#include "gps_calibration.h"
#include "gps_waypoint.h"
#include "zf_device_gnss.h"
#include "zf_device_imu660ra.h"

//====================================================ȫֱ====================================================

float gps_cal_offset_deg = 0.0f;

//====================================================˽к====================================================

/*--------------------------------------------------------------------------------------------------------------------
 * 㻷λǲ:  a  b ̻ڶ,  [-180, +180)
 * : angle_diff_circular(10, 350) = -20 (߶)
 *--------------------------------------------------------------------------------------------------------------------*/
static float angle_diff_circular(float a, float b)
{
    float diff = b - a;
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

/*--------------------------------------------------------------------------------------------------------------------
 * ƫƵ [-180, +180)
 *--------------------------------------------------------------------------------------------------------------------*/
static float normalize_angle_180(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

//====================================================T05: У׼====================================================

/*--------------------------------------------------------------------------------------------------------------------
 * У׼:  IMU yaw  GPS λǼƫ
 * ǰ: ʵ >= 2, IMU ѳʼ
 * : 1=У׼ɹ, 0=У׼ʧ (ʵ/ƫƳ)
 *
 * :
 *   1. ȡǰ IMU yaw
 *   2. ʵ 0  1  GPS λǼ
 *   3. offset = gps_bearing - imu_yaw
 *   4.  offset  [-180, +180)
 *   5. У: |offset| > 90 ʧ
 *--------------------------------------------------------------------------------------------------------------------*/
uint8 gps_cal_startpoint(void)
{
    float imu_yaw_at_start;
    double gps_bearing;
    float offset;

    /* ǰ: ʵ >= 2 */
    if (gps_wp_get_count() < 2)
        return 0;

    /* ȡǰ IMU yaw (eulerAngle.yaw Χ [-180, +180)) */
    imu_yaw_at_start = imu660ra.eulerAngle.yaw;

    /* ʵ 0  1  GPS λǼ [0, 360) */
    gps_bearing = (float)get_two_points_azimuth(
        gps_wp_set.waypoints[0].lat, gps_wp_set.waypoints[0].lng,
        gps_wp_set.waypoints[1].lat, gps_wp_set.waypoints[1].lng);

    /* ƫ = GPSλ - IMU_yaw */
    offset = gps_bearing - imu_yaw_at_start;

    /* ƫƵ [-180, +180) */
    offset = normalize_angle_180(offset);

    /* У: |offset| > 90 У׼ʧ */
    if (offset > GPS_CAL_MAX_OFFSET_DEG || offset < -GPS_CAL_MAX_OFFSET_DEG)
        return 0;

    gps_cal_offset_deg = offset;
    return 1;
}

//====================================================T06: ƫƲ====================================================

/*--------------------------------------------------------------------------------------------------------------------
 * ƫƲ:  GPS ںƫ
 * ǰ: , GPS Ч
 * : ƫ offset
 *
 * :
 *   1. : GPS Ч (gnss.state==1)
 *   2. : ٶ > 5 km/h (GPS ɿ)
 *   3.  GPS ں IMU ں
 *   4. :  < 30 (ǶЧ)
 *   5. offset += error * alpha
 *   6.  offset  [-180, +180)
 *--------------------------------------------------------------------------------------------------------------------*/
void gps_cal_drift_correction(void)
{
    float gps_heading;
    float imu_heading;
    float error;

    /*  1: GPS Ч */
    if (gnss.state != 1)
        return;

    /*  2: ٶ */
    if (gnss.speed < GPS_DRIFT_MIN_SPEED_KMH)
        return;

    /* GPS ں [0, 360) */
    gps_heading = gnss.direction;

    /* IMU ں = yaw + offset */
    imu_heading = imu660ra.eulerAngle.yaw + gps_cal_offset_deg;

    /*  3:  */
    error = angle_diff_circular(imu_heading, gps_heading);

    /*  4: ƫƲ */
    if (error > GPS_DRIFT_MAX_ERROR_DEG || error < -GPS_DRIFT_MAX_ERROR_DEG)
        return;

    /*  5: ƫƲ */
    gps_cal_offset_deg += error * GPS_DRIFT_CORRECTION_ALPHA;

    /*  6: ƫƵ [-180, +180) */
    gps_cal_offset_deg = normalize_angle_180(gps_cal_offset_deg);
}

/*--------------------------------------------------------------------------------------------------------------------
 * ȡǰƫ
 *--------------------------------------------------------------------------------------------------------------------*/
float gps_cal_get_offset(void)
{
    return gps_cal_offset_deg;
}

/*--------------------------------------------------------------------------------------------------------------------
 * λƫ
 *--------------------------------------------------------------------------------------------------------------------*/
void gps_cal_reset(void)
{
    gps_cal_offset_deg = 0.0f;
}
