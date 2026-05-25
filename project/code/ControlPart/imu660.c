/*
 * imu660.c
 *
 *  Created on: 2024年1月29日
 *      Author: LateRain
 */
#include "zf_common_headfile.h"
#include "imu660.h"
#include "image.h"
#include "Math_Advanced.h"

imu660_struct imu660ra;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      单位转换，数据预处理
//  @param      六轴数据输入
//-------------------------------------------------------------------------------------------------------------------
#define alpha 0.1f
/*#define imu660ra_acc_x  imu660ra_acc_x
#define imu660ra_acc_y  imu660ra_acc_y
#define imu660ra_acc_z  imu660ra_acc_z

#define imu660ra_gyro_x  imu660ra_gyro_x
#define imu660ra_gyro_y  imu660ra_gyro_y
#define imu660ra_gyro_z  imu660ra_gyro_z*/

void date_handle(void)
{
    // 原始数据获取
    imu660ra_get_gyro();
    imu660ra_get_acc();

    // 零漂处理
    if (imu660ra_gyro_y < 5.1 && imu660ra_gyro_y > -1.1)
        imu660ra_gyro_y -= 2.5;
    if (imu660ra_gyro_x < 5.1 && imu660ra_gyro_x > -1.1)
        imu660ra_gyro_x -= 1.8;
    if (imu660ra_gyro_z <= 2 && imu660ra_gyro_z >= -5)
        imu660ra_gyro_z = 0;

    // 单位转换
    // RC低通滤波,单位m/s2
    imu660ra.data_Ripen.acc_x = (((float)imu660ra_acc_x) * alpha) * 9.79 / 4096 + imu660ra.data_Ripen.acc_x * (1 - alpha);
    imu660ra.data_Ripen.acc_y = (((float)imu660ra_acc_y) * alpha) * 9.79 / 4096 + imu660ra.data_Ripen.acc_y * (1 - alpha);
    imu660ra.data_Ripen.acc_z = (((float)imu660ra_acc_z) * alpha) * 9.79 / 4096 + imu660ra.data_Ripen.acc_z * (1 - alpha);
    // 单位rps
    imu660ra.data_Ripen.gyro_x = ((float)imu660ra_gyro_x - 19.0f /*- GyroOffset.Xdata*/) * M_PI / 180 / 16.4f;
    imu660ra.data_Ripen.gyro_y = (-(float)imu660ra_gyro_y + 76.0f /*- GyroOffset.Ydata*/) * M_PI / 180 / 16.4f;
    imu660ra.data_Ripen.gyro_z = ((float)imu660ra_gyro_z /*- GyroOffset.Zdata*/) * M_PI / 180 / 16.4f;

    imu660ra.data_Raw.acc_x = (float)imu660ra_acc_x;
    imu660ra.data_Raw.acc_y = (float)imu660ra_acc_y;
    imu660ra.data_Raw.acc_z = (float)imu660ra_acc_z;
    // 单位dps
    imu660ra.data_Raw.gyro_x = ((float)imu660ra_gyro_x - 19.0f) / 16.4f;
    imu660ra.data_Raw.gyro_y = ((float)imu660ra_gyro_y + 76.0f) / 16.4f;
    imu660ra.data_Raw.gyro_z = ((float)imu660ra_gyro_z) / 16.4f;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      四元数滤波    在中断中调用即可
//  @param      none
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
#define pi 3.14159265f

float Kp = 2;    // 2   0.8
float Ki = 0.01; // 0.01
float halfT = 0.001;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float exInt = 0, eyInt = 0, ezInt = 0;

// euler_param offset_angle;
// euler_param eulerAngle;
float per_pitch = 0;
void get_eulerAngle(void)
{
    // 处理数据得到标准单位
    date_handle();

    // 利用数据得到四元数
    float ax = imu660ra.data_Ripen.acc_x;
    float ay = imu660ra.data_Ripen.acc_y;
    float az = imu660ra.data_Ripen.acc_z;
    float gx = imu660ra.data_Ripen.gyro_x;
    float gy = imu660ra.data_Ripen.gyro_y;
    float gz = imu660ra.data_Ripen.gyro_z;

    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    float q00, q11, q22;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q1 = q1 * q1;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;

    q00 = q0;
    q11 = q1;
    q22 = q2;
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q00 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q00 * gy - q11 * gz + q3 * gx) * halfT;
    q3 = q3 + (q00 * gz + q11 * gy - q22 * gx) * halfT;

    norm = invSqrt(q0q0 + q1q1 + q2q2 + q3q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    // 四元数转换成角度
    //    complementaryFilter(imu660ra.data_Ripen.gyro_x,imu660ra.data_Ripen.acc_y,imu660ra.data_Ripen.acc_z,0.002,&per_pitch,&imu660ra.eulerAngle.pitch);
    //    imu660ra.eulerAngle.roll  = asin(2*(q0q1 + q2q3 )) * 57.2957795f - imu660ra.offset_angle.roll;
    imu660ra.eulerAngle.pitch = asin(2 * (q0q1 + q2q3)) * 57.2957795f - imu660ra.offset_angle.pitch;
    imu660ra.eulerAngle.roll = asin(2 * (q0q1 + q2q3)) * 57.2957795f - imu660ra.offset_angle.roll;

    //    imu660ra.eulerAngle.yaw   = Kalman_Filter( imu660ra.eulerAngle.yaw , imu660ra.data_Raw.gyro_z );

    /*  // 元素中使用陀螺仪积分
      if(IMU_JF_Flag)
      {
          Z_Yaw += imu660ra.data_Raw.gyro_z/500;
      }
      else
      {
          Z_Yaw = 0;
      }*/
}