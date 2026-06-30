/*
 * imu660.h
 * IMU660RA六轴惯性传感器数据处理模块头文件
 *
 *  Created on: 2024年1月29日
 *      Author: LateRain
 */

#ifndef CODE_BALANCE_IMU660_H_
#define CODE_BALANCE_IMU660_H_

// @brief  IMU原始/处理后的六轴数据结构体
typedef struct
{
    float gyro_x;  // X轴角速度（陀螺仪）
    float gyro_y;  // Y轴角速度（陀螺仪）
    float gyro_z;  // Z轴角速度（陀螺仪）
    float acc_x;   // X轴加速度（加速度计）
    float acc_y;   // Y轴加速度（加速度计）
    float acc_z;   // Z轴加速度（加速度计）
} imu_param;

// @brief  欧拉角数据结构体
typedef struct
{
    float pitch;     // 俯仰角（度）
    float roll;      // 翻滚角（度）
    float yaw;       // 偏航角（度）

    float last_yaw;  // 上一次偏航角（用于偏航角变化检测）
    int8 Dirchange;  // 方向变化标志：标记偏航角是否发生了正负跳变
} euler_param;

// @brief  IMU660RA传感器总结构体（包含偏移校准、原始数据、处理后数据和欧拉角）
typedef struct
{
    euler_param offset_angle;     // 欧拉角零偏校准值（开机时设定）
    imu_param data_Raw;           // 采集的数据（原始ADC值转换后的dps/g单位）
    imu_param data_Ripen;         // 处理过后的数据（经低通滤波和单位转换后的rps/m/s2）
    euler_param eulerAngle;       // 单位是角度（由四元数解算得到）
} imu660_struct;

extern imu660_struct imu660ra;     // 总结构体（全局唯一实例）

// @brief  IMU数据处理 - 获取原始数据并进行零漂补偿、低通滤波和单位转换
void date_handle(void);

// @brief  读取IMU原始数据（仅获取ADC值，不做处理）
void date_raw(void);

// @brief  获取欧拉角 - 通过四元数互补滤波算法解算pitch/roll/yaw角度
void get_eulerAngle(void);

// @brief  卡尔曼滤波器 - 融合加速度计角度和陀螺仪角速度估计姿态角
// @param  angle_m  加速度计计算的角度测量值
// @param  gyro_m   陀螺仪角速度测量值
// @return 滤波后的角度估计值
float Kalman_Filter(float angle_m, float gyro_m);

// @brief  互补滤波器 - 融合陀螺仪和加速度计数据估计倾斜角
// @param  gyro_rate     陀螺仪角速度
// @param  accel_x       加速度计X轴分量
// @param  accel_y       加速度计Y轴分量
// @param  dt            采样周期（秒）
// @param  prev_angle    上一时刻角度（输入/输出）
// @param  output_angle  滤波输出角度（输出）
void complementaryFilter(float gyro_rate, float accel_x, float accel_y,
                         float dt, float *prev_angle, float *output_angle);

#endif /* CODE_BALANCE_IMU660_H_ */
