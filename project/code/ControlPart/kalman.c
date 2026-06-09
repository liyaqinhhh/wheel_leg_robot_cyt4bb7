/*
 * kalman.c
 *
 *  Created on: 2025年6月26日
 *      Author: Administrator
 */
/*
 * Kalman_fusion_of_imu660ra.c
 *
 *  Created on: 2024年11月22日
 *      Author: 17104
 */

#include "kalman.h"
#include "ekf.h"

#define GRAVITY 9.7997
float standardized_curvature_ave = 0;


imu963ra_struct imu;

/**
 * @brief 初始化六轴 IMU 姿态卡尔曼滤波器状态和参数
 * @param imu 待初始化的姿态滤波器结构体指针
 * @param q 系统噪声协方差初值
 * @param r 测量噪声协方差初值
 * @param T 离散采样周期，单位为 s
 * @return void 无返回值
 */
void imu963ra_kalman_filter_init(imu963ra_struct * imu, float q, float r, float T)
{
//    imu963ra_init();

    imu -> roll = 0;
    imu -> pitch = 0;
    imu -> yaw = 0;

    imu -> gx = 0;
    imu -> gy = 0;
    imu -> gz = 0;
    imu -> ax = 0;
    imu -> ay = 0;
    imu -> az = 0;
    imu -> mx = 0;
    imu -> my = 0;
    imu -> mz = 0;

    int i;
    for (i = 0; i < 3;i++)
    {
        // 初始化卡尔曼各状态量，准备 roll/pitch/yaw 三通道独立滤波
        imu -> Xk_[i] = 0;
        imu -> Xk[i] = 0;
        imu -> Uk[i] = 0;
        imu -> Zk[i] = 0;
        imu -> Pk[i] = 1;
        imu -> Pk_[i] = 0;
        imu -> K[i] = 0;
        imu -> Q[i] = q;
        imu -> R[i] = r;
    }

    imu -> T = T;
    imu -> resultant_acceleration = 0;

    imu -> imu_offset_fwd  = 0.025f;  // 前+后-，当前=后方2.5cm（原注释"后方6cm -0.06"，需实测确认）
    imu -> imu_offset_left =  -0.04f;   // 左+右-，当前=左方4cm（原注释"右方4cm -0.04"，符号已翻转，需实测确认）
    imu -> yaw_accel = 0;
    imu -> yaw_rate_prev = 0;
}

/**
 * @brief 补偿 IMU 偏心安装导致的向心加速度误差
 * @param imu IMU 滤波器结构体指针
 * @return void 无返回值
 */
void imu_offset_compensate_acc(imu963ra_struct *imu)
{
    float yaw_rate = imu->gz;  // 当前偏航角速度，单位 rad/s

    // 小角速度下偏心向心加速度远低于噪声，直接跳过避免引入补偿抖动
    const float DEAD_ZONE = 0.087266f;  // 5 deg/s
    if (fabsf(yaw_rate) < DEAD_ZONE) {
        return;
    }

    // 向心加速度模型：a_c = -ω^2 * r，分别求机体前向和左向投影
    float omega2 = yaw_rate * yaw_rate;
    float cent_x = -omega2 * imu->imu_offset_fwd;
    float cent_y = -omega2 * imu->imu_offset_left;

    const float G = 9.80665f;
    float comp_x = cent_x / G;
    float comp_y = cent_y / G;

    // 对补偿量做限幅，防止陀螺仪尖峰导致加速度观测异常
    const float MAX_COMP = 0.5f;
    if (comp_x >  MAX_COMP) comp_x =  MAX_COMP;
    if (comp_x < -MAX_COMP) comp_x = -MAX_COMP;
    if (comp_y >  MAX_COMP) comp_y =  MAX_COMP;
    if (comp_y < -MAX_COMP) comp_y = -MAX_COMP;

    imu->ax -= comp_x;
    imu->ay -= comp_y;
    // az 无需补偿，因为纯水平向心加速度在机体 z 轴上无投影
}

/**
 * @brief 执行一次六轴 IMU 姿态卡尔曼滤波更新
 * @param imu 姿态滤波器结构体指针
 * @return void 无返回值
 */
void imu963ra_kalman_filter_update(imu963ra_struct * imu)
{
    imu660ra_get_acc();
    imu660ra_get_gyro();

//    printf("%f,%f,%f\n",imu -> gx,imu -> gy,imu -> gz);
//    printf("%f,%f,%f\n",imu -> ax,imu -> ay,imu -> az);
//    printf("%f,%f,%f\n",imu -> roll,imu -> pitch,imu -> yaw);
    imu -> gx = imu660ra_gyro_transition(imu660ra_gyro_x) * My_PI / 180.f;
    imu -> gy = imu660ra_gyro_transition(-imu660ra_gyro_y) * My_PI / 180.f;
    imu -> gz = imu660ra_gyro_transition(imu660ra_gyro_z) * My_PI / 180.f;
    imu -> ax = imu660ra_acc_transition(imu660ra_acc_x);
    imu -> ay = imu660ra_acc_transition(imu660ra_acc_y);
    imu -> az = imu660ra_acc_transition(imu660ra_acc_z);


    if(fabsf(imu -> gx) > MAX_READ_VALUE || fabsf(imu -> gy) > MAX_READ_VALUE || fabsf(imu -> gz) > MAX_READ_VALUE || fabsf(imu -> ax) > MAX_READ_VALUE || fabsf(imu -> ay) > MAX_READ_VALUE || fabsf(imu -> az) > MAX_READ_VALUE)
    {return;}

    // 先修正偏心安装误差，避免转弯时横向向心加速度污染俯仰估计
    imu_offset_compensate_acc(imu);
    //printf("CMP_ACC: ax=%.4f, ay=%.4f, az=%.4f\r\n", imu->ax, imu->ay, imu->az);

    // 转弯越剧烈，加速度越不再近似纯重力，因此动态增大 pitch 观测噪声
    {
        float yaw_rate_dps = imu->gz * 180.0f / My_PI;
        float yaw_factor = 1.0f + 0.005f * (yaw_rate_dps * yaw_rate_dps);
        if (yaw_factor > 100.0f) yaw_factor = 100.0f;
        imu->R[1] = imu->R[0] * yaw_factor;
    }


    imu -> resultant_acceleration = imu -> ax * imu -> ax + imu -> ay * imu -> ay + imu -> az * imu -> az;
    if(imu -> resultant_acceleration > 0)
    {
        imu -> resultant_acceleration = sqrtf(imu -> resultant_acceleration);
    }
    else
    {
        imu -> resultant_acceleration = 0;
        return;
    }

    // 预测步骤 1：由机体系角速度解算欧拉角速度，得到系统输入 Uk
    imu -> Uk[0] = imu -> gx + sin(imu -> Xk[0]) * tan(imu -> Xk[1]) * imu -> gy + cos(imu -> Xk[0]) * tan(imu -> Xk[1]) * imu -> gz;
    imu -> Uk[1] = cos(imu -> Xk[0]) * imu -> gy - sin(imu -> Xk[0]) * imu -> gz;
    imu -> Uk[2] = sin(imu -> Xk[0]) * imu -> gy / cos(imu -> Xk[1]) + cos(imu -> Xk[0]) * imu -> gz / cos(imu -> Xk[1]);

    // 预测步骤 2：先验状态估计 X(k|k-1) = X(k-1|k-1) + T * Uk
    imu -> Xk_[0] = imu -> Xk[0] + imu -> T * imu -> Uk[0];
    imu -> Xk_[1] = imu -> Xk[1] + imu -> T * imu -> Uk[1];
    imu -> Xk_[2] = imu -> Xk[2] + imu -> T * imu -> Uk[2];

    // 预测步骤 3：先验误差协方差 P(k|k-1) = P(k-1|k-1) + Q
    imu -> Pk_[0] = imu -> Pk[0] + imu -> Q[0];
    imu -> Pk_[1] = imu -> Pk[1] + imu -> Q[1];
    imu -> Pk_[2] = imu -> Pk[2] + imu -> Q[2];

    // 更新步骤 1：计算卡尔曼增益，决定预测值和测量值的融合权重
    imu -> K[0] = imu -> Pk_[0] / (imu -> Pk_[0] + imu -> R[0]);
    imu -> K[1] = imu -> Pk_[1] / (imu -> Pk_[1] + imu -> R[1]);
    imu -> K[2] = 0;

    // 更新步骤 2：由加速度方向反解 roll / pitch 测量值，yaw 仍依赖陀螺积分
    imu -> Zk[0] = atan( imu -> ay / imu -> az );
    imu -> Zk[1] = -atan( imu -> ax / (sqrt( imu -> ay * imu -> ay + imu -> az * imu -> az )));
    imu -> Zk[2] = 0;

    // 更新步骤 3：后验状态估计 X(k|k) = (1-K)X(k|k-1) + KZ(k)
    imu -> Xk[0] = (1 - imu -> K[0]) * imu -> Xk_[0] + imu -> K[0] * imu -> Zk[0];
    imu -> Xk[1] = (1 - imu -> K[1]) * imu -> Xk_[1] + imu -> K[1] * imu -> Zk[1];
    imu -> Xk[2] = imu -> Xk_[2];

    // 更新步骤 4：后验协方差收缩，表示融合观测后不确定度下降
    imu -> Pk[0] = (1 - imu -> K[0]) * imu -> Pk_[0];
    imu -> Pk[1] = (1 - imu -> K[1]) * imu -> Pk_[1];
    imu -> Pk[2] = imu -> Pk_[2];

    imu -> roll = imu -> Xk[0] / My_PI * 180.f;
    imu -> pitch = imu -> Xk[1] / My_PI * 180.f;
    imu -> yaw = imu -> Xk[2] / My_PI * 180.f;
    //printf(" %f\n", imu -> roll);

    // 根据滤波后的姿态估计计算重力在机体系中的投影，用于线加速度分离
//    const float GRAVITY = 9.80665f; // 标准重力加速度

    float roll = imu->Xk[0];
    float pitch = imu->Xk[1];

    float sin_roll = sinf(roll);
    float cos_roll = cosf(roll);
    float sin_pitch = sinf(pitch);
    float cos_pitch = cosf(pitch);

    // 将重力向量从地理坐标系映射到机体坐标系
    float gx_body = -GRAVITY * sin_pitch;
    float gy_body = GRAVITY * sin_roll * cos_pitch;
    float gz_body = GRAVITY * cos_roll * cos_pitch;

    // 把加速度计量值从 g 转换为 m/s^2，便于后续动力学融合
    float ax_mps2 = imu->ax * GRAVITY;
    float ay_mps2 = imu->ay * GRAVITY;
    float az_mps2 = imu->az * GRAVITY;

    // 线性加速度 = 原始加速度 - 重力分量
    imu->ax_linear = ax_mps2 - gx_body;
    imu->ay_linear = ay_mps2 - gy_body;
    imu->az_linear = az_mps2 - gz_body;
//    printf("%f,%f,%f\n",imu->ax_linear,imu->ay_linear,imu->az_linear);

}

KF_Velocity vel_kf;

/**
 * @brief 初始化轮速与线加速度融合的速度卡尔曼滤波器
 * @param kf 待初始化的速度滤波器结构体指针
 * @param q_pos 位移状态过程噪声
 * @param q_vel 速度状态过程噪声
 * @param r 轮速测量噪声
 * @param T 采样周期，单位为 s
 * @return void 无返回值
 */
void imu963ra_menc15a_kalman_filter_init(KF_Velocity* kf, float q_pos, float q_vel, float r, float T) {

    kf->Xk_[0] = kf->Xk_[1] = 0;
    kf->Xk[0] = kf->Xk[1] = 0;

    // 初始化后验协方差矩阵，表示系统起始不确定度
    kf->Pk[0][0] = 1.0f; kf->Pk[0][1] = 0;
    kf->Pk[1][0] = 0;     kf->Pk[1][1] = 1.0f;

    // 过程噪声矩阵 Q 反映位移和速度预测模型的不确定性
    kf->Q[0][0] = q_pos; kf->Q[0][1] = 0;
    kf->Q[1][0] = 0;      kf->Q[1][1] = q_vel;

    kf->R = r;
    kf->T = T;

    kf->est_displacement = 0;
    kf->est_velocity = 0;

    kf->original_R = r;
    kf->original_Q[0][0] = q_pos;
    kf->original_Q[1][1] = q_vel;
    kf->direction_factor = 0;
    kf->slip_factor = 0;
    kf->slip_ratio = 0;
}

/**
 * @brief 使用轮速测量和线加速度执行一次速度卡尔曼更新
 * @param kf 速度滤波器结构体指针
 * @param measured_speed 轮速测量值
 * @param linear_accel 去重力后的线加速度
 * @return void 无返回值
 */
void imu963ra_menc15a_kalman_filter_Update(KF_Velocity* kf, float measured_speed, float linear_accel)
{
    // 在预测前根据打滑状态动态调整 Q / R，降低异常轮速对估计的影响
//    if(fabs(imu.resultant_acceleration) < 0.1f) {
//        // 加速度计数据异常，使用纯轮速估计
//        kf->Xk[1] = measured_speed;
//    }
//    if(/*fabs(linear_accel) > 1.0f && */fabs(measured_speed - kf->Xk_[1]) > 0.2f)
//    {
//        kf->R = kf->original_R * 10000.0f;
//        kf->Q[0][0] = kf->original_Q[0][0] * 10000.0f;
//        kf->Q[1][1] = kf->original_Q[1][1];
//    }

    handle_slip_condition(kf, measured_speed, imu.gz, imu.ax_linear);

    // 输入矩阵 B 把线加速度映射到位移和速度两个状态量
    const float B[2] = {0.5f * kf->T * kf->T, kf->T};

    // 状态转移矩阵 A，对应常加速度离散运动模型
    const float A[2][2] = {{1, kf->T},
                          {0, 1}};

    // 预测步骤 1：先验状态估计，融合上一时刻状态和本周期线加速度
    kf->Xk_[0] = A[0][0]*kf->Xk[0] + A[0][1]*kf->Xk[1] + B[0]*linear_accel;
    kf->Xk_[1] = A[1][0]*kf->Xk[0] + A[1][1]*kf->Xk[1] + B[1]*linear_accel;

    // 预测步骤 2：先算 A * P，便于后续得到 A * P * A^T
    float AP[2][2] = {
        {A[0][0]*kf->Pk[0][0] + A[0][1]*kf->Pk[1][0],
         A[0][0]*kf->Pk[0][1] + A[0][1]*kf->Pk[1][1]},
        {A[1][0]*kf->Pk[0][0] + A[1][1]*kf->Pk[1][0],
         A[1][0]*kf->Pk[0][1] + A[1][1]*kf->Pk[1][1]}
    };

    // 预测步骤 3：协方差预测 P(k|k-1) = A * P(k-1|k-1) * A^T + Q
    kf->Pk_[0][0] = AP[0][0]*A[0][0] + AP[0][1]*A[0][1] + kf->Q[0][0];
    kf->Pk_[0][1] = AP[0][0]*A[1][0] + AP[0][1]*A[1][1] + kf->Q[0][1];
    kf->Pk_[1][0] = AP[1][0]*A[0][0] + AP[1][1]*A[0][1] + kf->Q[1][0];
    kf->Pk_[1][1] = AP[1][0]*A[1][0] + AP[1][1]*A[1][1] + kf->Q[1][1];

    // 更新步骤 1：观测方程仅直接测量速度，因此 S 为速度协方差加测量噪声
    const float S = kf->Pk_[1][1] + kf->R;
    kf->K[0] = kf->Pk_[0][1] / S;
    kf->K[1] = kf->Pk_[1][1] / S;

    // 更新步骤 2：创新项 y = z - Hx，这里 z 为轮速测量值
    const float y = measured_speed - kf->Xk_[1];
    kf->Xk[0] += kf->K[0] * y;
    kf->Xk[1] += kf->K[1] * y;

    // 更新步骤 3：后验协方差矩阵收缩，反映量测修正后的不确定度下降
    const float P00 = kf->Pk_[0][0] - kf->K[0] * kf->Pk_[1][0];
    const float P01 = kf->Pk_[0][1] - kf->K[0] * kf->Pk_[1][1];
    const float P11 = kf->Pk_[1][1] - kf->K[1] * kf->Pk_[1][1];

    kf->Pk[0][0] = P00;
    kf->Pk[0][1] = P01;
    kf->Pk[1][0] = kf->Pk_[1][0] - kf->K[1] * kf->Pk_[1][0];
    kf->Pk[1][1] = P11;

    // 输出估计结果：累计位移与当前速度
    kf->est_displacement += kf->Xk[0];
    kf->est_velocity = kf->Xk[1];

    // 恢复基准噪声参数，下一周期再根据工况重新调整
    kf->R = kf->original_R;
    kf->Q[0][0] = kf->original_Q[0][0];
    kf->Q[1][1] = kf->original_Q[1][1];

    // 对速度状态做限幅，防止异常工况下估计发散
    kf->Xk[1] = fmaxf(fminf(kf->Xk[1], MAX_SPEED), -MAX_SPEED);

//        printf("%f,%f\n",kf->est_displacement,kf->est_velocity);
//    printf("%f,%f,%f,%d\n",Instantaneous_speed,kf->est_velocity,kf->slip_ratio,kf->is_slip);
}

/**
 * @brief 根据轮速偏差和弯道工况动态调整打滑补偿参数
 * @param kf 速度滤波器结构体指针
 * @param measured_speed 当前轮速测量值
 * @param yaw_rate 当前偏航角速度
 * @param lat_accel 当前横向加速度
 * @return void 无返回值
 */
void handle_slip_condition(KF_Velocity* kf, float measured_speed, float yaw_rate, float lat_accel)
{
    // 轮速测量与预测速度差越大，越可能存在轮胎打滑或轮速异常
    float speed_diff = fabsf(measured_speed - kf->Xk_[1]);
//    printf("%f\n",speed_diff);
    if(fabsf(standardized_curvature_ave) > 0.6) {
        // 大曲率转弯时估算向心加速度偏差，并反馈修正纵向线加速度
        float radius = fabs(kf->Xk[1] / (yaw_rate + 0.001f));
        float centripetal_accel = kf->Xk[1] * yaw_rate;
        float accel_bias = centripetal_accel - lat_accel;

        // 对偏差限幅，避免一次性注入过大的补偿量
        accel_bias = fmaxf(fminf(accel_bias, 2.0f), -2.0f);
        imu.ax_linear -= 0.4f * accel_bias;
    }

    // 方向一致性检测：倒车或速度反向时降低滑移率对滤波参数的影响
    if (measured_speed * kf->Xk_[1] >= 0) {
        kf->direction_factor = 1.0f;
    } else {
        float reverse_ratio = fminf(speed_diff / SPEED_REVERSE_THRESH, 1.0f);
        kf->direction_factor = cosf(My_PI_2 * reverse_ratio);
    }

    // 以较大速度作为归一化基准，避免低速时滑移率被噪声放大
    float base_speed = fmaxf(fabsf(measured_speed), fabsf(kf->Xk_[1]));
    if (base_speed < 0.2f) {
        kf->slip_ratio = 0;
    } else {
        float valid_speed_diff = speed_diff * kf->direction_factor;
        kf->slip_ratio = valid_speed_diff / base_speed;
    }

    // 依据轨迹曲率提高弯道工况下的敏感度
    float curve_factor = fabsf(standardized_curvature_ave) > 0.4 ? 1.5f : 1.0f;
    // 使用指数平滑得到连续变化的滑移因子，避免 Q / R 突变
    kf->slip_factor = 0.8f * kf->slip_factor + 0.2f * kf->slip_ratio;
    // 打滑越严重，越降低对轮速测量和位移预测的信任
    kf->R = kf->original_R * (1.0f + 10000.0f * kf->slip_factor) * curve_factor;
    kf->Q[0][0] = kf->original_Q[0][0] * (1.0f + 10000.0f * kf->slip_factor) * curve_factor;

    // 限制最大放大倍数，避免噪声协方差无限增大
    kf->R = fminf(kf->R, kf->original_R * 10000.0f);
    kf->Q[0][0] = fminf(kf->Q[0][0], kf->original_Q[0][0] * 10000.0f);

    // 弯道特殊处理保留接口，后续可继续放大速度过程噪声
//    if(is_curving) {
//        kf->Q[1][1] = kf->original_Q[1][1] * 2.0f; // 提高速度过程噪声
//    }
}

/**
 * @brief 校验 IMU 加速度与轮速变化推导的模型加速度是否一致
 * @param imu_accel IMU 测得的加速度
 * @param speed_est 当前估计速度
 * @param measured_speed 当前轮速测量值
 * @param dt1 采样周期
 * @return int 1 表示一致，0 表示差异过大
 */
int validate_acceleration(float imu_accel, float speed_est, float measured_speed, float dt1)
{
    float model_accel = (measured_speed - speed_est) / dt;
    return (fabsf(imu_accel - model_accel) < ACC_THRESH);
}
