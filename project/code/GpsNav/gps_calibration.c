/*********************************************************************************************************************
 * CYT4BB 智能车 GPS 导航 -  校准与漂移修正模块实现
 *
 * 文件: gps_calibration.c
 * 模块: M3 校准与漂移修正
 * 功能: 初始校准 + 运行中漂移修正
 *
 * 实现任务:
 *   T05: 初始校准 (gps_cal_startpoint)
 *   T06: 漂移修正 (gps_cal_drift_correction)
 *
 * 初始校准原理:
 *   前提: 车辆停在第一个航点, 面朝第二个航点
 *   计算: offset = GPS方位角 - IMU_yaw
 *   后续: GPS_bearing = IMU_yaw + offset
 *
 * 漂移修正原理:
 *   条件: GPS 有效 + 速度 > 5km/h + 误差 < 30度
 *   更新: offset += error * alpha (alpha=0.02, 每 100ms 修正 2%)
 *   保护: 当 GPS 角度无效时, 不修正漂移
 ********************************************************************************************************************/

#include "gps_calibration.h"
#include "gps_waypoint.h"
#include "zf_device_gnss.h"
#include "zf_device_imu660ra.h"
#include "imu660.h"
//====================================================全局变量====================================================

float gps_cal_offset_deg = 0.0f;

//====================================================私有函数====================================================

/*--------------------------------------------------------------------------------------------------------------------
 * 环形角度差: 计算 a 到 b 的最短路径差值，结果在 [-180, +180)
 * 示例: angle_diff_circular(10, 350) = -20 (两点最短路径)
 *--------------------------------------------------------------------------------------------------------------------*/
static float angle_diff_circular(float a, float b)
{
    float diff = b - a;
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

/*--------------------------------------------------------------------------------------------------------------------
 * 角度归一化到 [-180, +180)
 *--------------------------------------------------------------------------------------------------------------------*/
static float normalize_angle_180(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

//====================================================T05: 初始校准====================================================

/*--------------------------------------------------------------------------------------------------------------------
 * 初始校准: 用 IMU yaw 与 GPS 方位角计算初始偏移
 * 前提: 航点数 >= 2, IMU 已初始化
 * 返回: 1=校准成功, 0=校准失败 (航点不足/偏移过大)
 *
 * 步骤:
 *   1. 获取当前 IMU yaw
 *   2. 计算航点 0 到 航点 1 的 GPS 方位角
 *   3. offset = gps_bearing - imu_yaw
 *   4. 归一化 offset 到 [-180, +180)
 *   5. 校验: |offset| > 90度则失败
 *--------------------------------------------------------------------------------------------------------------------*/
uint8 gps_cal_startpoint(void)
{
    float imu_yaw_at_start;
    double gps_bearing;
    float offset;

    /* 前提: 航点数 >= 2 */
    if (gps_wp_get_count() < 2)
        return 0;

    /* 获取当前 IMU yaw (eulerAngle.yaw 范围 [-180, +180)) */
    imu_yaw_at_start = imu660ra.eulerAngle.yaw;

    /* 计算航点 0 到 航点 1 的 GPS 方位角 [0, 360) */
    gps_bearing = (float)get_two_points_azimuth(
        gps_wp_set.waypoints[0].lat, gps_wp_set.waypoints[0].lng,
        gps_wp_set.waypoints[1].lat, gps_wp_set.waypoints[1].lng);

    /* 偏移 = GPS方位角 - IMU_yaw */
    offset = gps_bearing - imu_yaw_at_start;

    /* 偏移归一化到 [-180, +180) */
    offset = normalize_angle_180(offset);

    /* 校验: |offset| > 90度则校准失败 */
    if (offset > GPS_CAL_MAX_OFFSET_DEG || offset < -GPS_CAL_MAX_OFFSET_DEG)
        return 0;

    gps_cal_offset_deg = offset;
    return 1;
}

//====================================================T06: 漂移修正====================================================

/*--------------------------------------------------------------------------------------------------------------------
 * 漂移修正: 利用 GPS 航向在线修正偏移
 * 前提: 导航中, GPS 有效
 * 结果: 更新偏移 offset
 *
 * 步骤:
 *   1. 检查: GPS 有效 (gnss.state==1)
 *   2. 检查: 速度 > 5 km/h (GPS 可靠)
 *   3. 计算 GPS 航向与 IMU 航向的误差
 *   4. 检查: 误差 < 30度 (过滤无效修正)
 *   5. offset += error * alpha
 *   6. 归一化 offset 到 [-180, +180)
 *--------------------------------------------------------------------------------------------------------------------*/
void gps_cal_drift_correction(void)
{
    float gps_heading;
    float imu_heading;
    float error;

    /* 检查 1: GPS 有效 */
    if (gnss.state != 1)
        return;

    /* 检查 2: 速度检查 */
    if (gnss.speed < GPS_DRIFT_MIN_SPEED_KMH)
        return;

    /* GPS 航向 [0, 360) */
    gps_heading = gnss.direction;

    /* IMU 航向 = yaw + offset */
    imu_heading = imu660ra.eulerAngle.yaw + gps_cal_offset_deg;

    /* 检查 3: 计算误差 */
    error = angle_diff_circular(imu_heading, gps_heading);

    /* 检查 4: 过滤过大误差 */
    if (error > GPS_DRIFT_MAX_ERROR_DEG || error < -GPS_DRIFT_MAX_ERROR_DEG)
        return;

    /* 步骤 5: 漂移修正 */
    gps_cal_offset_deg += error * GPS_DRIFT_CORRECTION_ALPHA;

    /* 步骤 6: 偏移归一化到 [-180, +180) */
    gps_cal_offset_deg = normalize_angle_180(gps_cal_offset_deg);
}

/*--------------------------------------------------------------------------------------------------------------------
 * 获取当前偏移量
 *--------------------------------------------------------------------------------------------------------------------*/
float gps_cal_get_offset(void)
{
    return gps_cal_offset_deg;
}

/*--------------------------------------------------------------------------------------------------------------------
 * 重置偏移量
 *--------------------------------------------------------------------------------------------------------------------*/
void gps_cal_reset(void)
{
    gps_cal_offset_deg = 0.0f;
}
