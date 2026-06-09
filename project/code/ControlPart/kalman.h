/*
 * kalman.h
 *
 *  Created on: 2025年6月26日
 *      Author: Administrator
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include "zf_common_headfile.h"

#define My_PI   3.141592653f
#define My_PI_2 1.570796327f
#define MAX_SPEED 20.0f             //最大合法速度
#define SLIP_THRESHOLD 0.2f         //滑移率打滑判断阈值
#define CURVE_THRESHOLD_YAW 0.3f    //转向偏航角速度阈值 rad/s
#define CURVE_THRESHOLD_ACC 1.0f    //转向横向加速度阈值 m/s^2
#define SPEED_REVERSE_THRESH 0.1f   //速度反转认定阈值
#define ACC_THRESH  5.0f            //最大合法加速度 m/s^2
#define MAX_READ_VALUE 10.0f        //最大读取合法值

/**
 * @brief 六轴 IMU 姿态卡尔曼滤波状态结构体
 */
typedef struct
{
    float roll;                     /**< 横滚角，单位为度 */
    float pitch;                    /**< 俯仰角，单位为度 */
    float yaw;                      /**< 偏航角，单位为度 */

    float gx;                       /**< X 轴角速度，单位为 rad/s */
    float gy;                       /**< Y 轴角速度，单位为 rad/s */
    float gz;                       /**< Z 轴角速度，单位为 rad/s */
    float ax;                       /**< X 轴加速度，单位为 g */
    float ay;                       /**< Y 轴加速度，单位为 g */
    float az;                       /**< Z 轴加速度，单位为 g */
    float mx;                       /**< X 轴磁力计量测值 */
    float my;                       /**< Y 轴磁力计量测值 */
    float mz;                       /**< Z 轴磁力计量测值 */

    float Xk_[3];                   /**< 先验状态估计，依次为 roll/pitch/yaw */
    float Xk[3];                    /**< 后验状态估计，依次为 roll/pitch/yaw */
    float Uk[3];                    /**< 系统输入，由角速度解算得到 */
    float Zk[3];                    /**< 测量状态，由传感器观测计算得到 */
    float Pk[3];                    /**< 后验估计误差协方差 */
    float Pk_[3];                   /**< 先验估计误差协方差 */
    float K[3];                     /**< 卡尔曼增益 */
    float Q[3];                     /**< 系统噪声协方差 */
    float R[3];                     /**< 测量噪声协方差 */

    float ax_linear;                /**< 去除重力后的 X 轴线性加速度，单位为 m/s^2 */
    float ay_linear;                /**< 去除重力后的 Y 轴线性加速度，单位为 m/s^2 */
    float az_linear;                /**< 去除重力后的 Z 轴线性加速度，单位为 m/s^2 */

    float T;                        /**< 离散采样周期，单位为 s */
    float resultant_acceleration;   /**< 合加速度模长，单位为 g */

    float imu_offset_fwd;           /**< IMU 相对旋转中心的前向偏移，后方为负，单位为 m */
    float imu_offset_left;          /**< IMU 相对旋转中心的左向偏移，右方为负，单位为 m */
    float yaw_accel;                /**< 偏航角加速度，单位为 rad/s^2 */
    float yaw_rate_prev;            /**< 上一次记录的偏航角速度，单位为 rad/s */
}imu963ra_struct;
extern imu963ra_struct imu;

/**
 * @brief 初始化六轴 IMU 姿态卡尔曼滤波器参数和状态
 * @param imu 待初始化的滤波器结构体指针
 * @param q 系统噪声协方差初值
 * @param r 测量噪声协方差初值
 * @param T 离散采样周期，单位为 s
 * @return void 无返回值
 */
void imu963ra_kalman_filter_init(imu963ra_struct * imu, float q, float r, float T);

/**
 * @brief 执行一次六轴 IMU 姿态卡尔曼滤波更新
 * @param imu 姿态滤波器结构体指针
 * @return void 无返回值
 */
void imu963ra_kalman_filter_update(imu963ra_struct * imu);

/**
 * @brief 补偿 IMU 偏心安装引入的向心加速度误差
 * @param imu IMU 滤波状态结构体指针
 * @return void 无返回值
 */
void imu_offset_compensate_acc(imu963ra_struct *imu);

/**
 * @brief 轮速与线加速度融合的速度卡尔曼滤波器结构体
 */
typedef struct {
    float Xk_[2];               /**< 先验状态估计：[位移, 速度] */
    float Xk[2];                /**< 后验状态估计：[位移, 速度] */
    float Pk_[2][2];            /**< 先验误差协方差矩阵 */
    float Pk[2][2];             /**< 后验误差协方差矩阵 */
    float K[2];                 /**< 卡尔曼增益 */
    float Q[2][2];              /**< 过程噪声协方差矩阵 */
    float R;                    /**< 测量噪声方差 */
    float T;                    /**< 采样周期，单位为 s */

    float original_R;           /**< 标准测量噪声方差，用于动态恢复 */
    float original_Q[2][2];     /**< 标准过程噪声方差矩阵，用于动态恢复 */
    float direction_factor;     /**< 速度方向一致性修正因子 */
    float slip_factor;          /**< 动态滑移系数，范围约为 [0, 1] */
    float slip_ratio;           /**< 当前估计的滑移率 */
    int is_slip;                /**< 打滑标志位 */

    float est_displacement;     /**< 累计估计位移 */
    float est_velocity;         /**< 当前估计速度 */
} KF_Velocity;
extern KF_Velocity vel_kf;
extern float standardized_curvature_ave;

/**
 * @brief 初始化速度卡尔曼滤波器参数与状态
 * @param kf 待初始化的速度滤波器指针
 * @param q_pos 位移状态过程噪声
 * @param q_vel 速度状态过程噪声
 * @param r 轮速测量噪声
 * @param T 离散采样周期，单位为 s
 * @return void 无返回值
 */
void imu963ra_menc15a_kalman_filter_init(KF_Velocity* kf, float q_pos, float q_vel, float r, float T);

/**
 * @brief 使用轮速和线加速度执行一次速度卡尔曼更新
 * @param kf 速度滤波器结构体指针
 * @param measured_speed 磁编码器测得的速度
 * @param linear_accel 去重力后的线性加速度
 * @return void 无返回值
 */
void imu963ra_menc15a_kalman_filter_Update(KF_Velocity* kf, float measured_speed, float linear_accel);

/**
 * @brief 根据转弯和速度偏差动态调整打滑工况下的滤波参数
 * @param kf 速度滤波器结构体指针
 * @param measured_speed 当前轮速测量值
 * @param yaw_rate 当前偏航角速度
 * @param lat_accel 当前横向线加速度
 * @return void 无返回值
 */
void handle_slip_condition(KF_Velocity* kf, float measured_speed, float yaw_rate, float lat_accel);

/**
 * @brief 校验 IMU 加速度与速度变化模型是否一致
 * @param imu_accel IMU 测得的加速度
 * @param speed_est 当前估计速度
 * @param measured_speed 当前轮速测量值
 * @param dt1 采样周期
 * @return int 1 表示加速度一致，0 表示不一致
 */
int validate_acceleration(float imu_accel, float speed_est, float measured_speed, float dt1);

#endif /* KALMAN_H_ */
