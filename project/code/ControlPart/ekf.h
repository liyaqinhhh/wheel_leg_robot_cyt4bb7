#include "zf_common_headfile.h"
#ifndef CODE_EKF_H_
#define CODE_EKF_H_


#define DEG_TO_RAD      (57.295779513082320876798154814105f)
#define dt              (0.001f)
#define Kk1               (1.0f)


/**
 * @brief IMU 原始测量数据结构体
 */
typedef struct
{
        float gyro_x;    /**< X 轴角速度，单位为 rad/s */
        float gyro_y;    /**< Y 轴角速度，单位为 rad/s */
        float gyro_z;    /**< Z 轴角速度，单位为 rad/s */
        float acc_x;     /**< X 轴加速度采样值 */
        float acc_y;     /**< Y 轴加速度采样值 */
        float acc_z;     /**< Z 轴加速度采样值 */
}imu_t;




/**
 * @brief 初始化扩展卡尔曼滤波器状态、过程噪声和协方差矩阵
 * @param void 无
 * @return void 无返回值
 */
void EKF_Init(void);

/**
 * @brief 执行一次姿态扩展卡尔曼滤波更新，输出最新欧拉角
 * @param void 无
 * @return void 无返回值
 */
void EKF_UpData(void);


#endif /* CODE_EKF_H_ */
