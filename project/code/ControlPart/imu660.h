/*
 * imu660.h
 *
 *  Created on: 2024年1月29日
 *      Author: LateRain
 */

#ifndef CODE_BALANCE_IMU660_H_
#define CODE_BALANCE_IMU660_H_

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} imu_param;
typedef struct
{
    float pitch;
    float roll;
    float yaw;

    float last_yaw;
    int8 Dirchange;
} euler_param;
typedef struct
{
    euler_param offset_angle;
    imu_param data_Raw;            // 采集的数据
    imu_param data_Ripen;          // 处理过后的数据
    euler_param eulerAngle;        // 单位是角度
} imu660_struct;

extern imu660_struct imu660ra;     // 总结构体

void date_handle(void);
void date_raw(void);
void get_eulerAngle(void);
float Kalman_Filter(float angle_m, float gyro_m);
void complementaryFilter(float gyro_rate, float accel_x, float accel_y,
                         float dt, float *prev_angle, float *output_angle);

#endif /* CODE_BALANCE_IMU660_H_ */
