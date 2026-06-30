#include "zf_common_headfile.h"
#include "ekf.h"
#include "matrix.h"

matrix_t exf_x;
matrix_t error;
EulerAngles euler_angle;
imu_t imu_data = {0, 0, 0, 0, 0, 0};
matrix_type r_yz = 0.001f;

const matrix_type q[4][4] = {{0.005, 0, 0, 0}, {0, 0.005, 0, 0}, {0, 0, 0.005, 0}, {0, 0, 0, 0.005}};
const matrix_type r[3][3] = {{10000, 0, 0}, {0, 10000, 0}, {0, 0, 10000}};
const matrix_type p[4][4] = {{1000000, 0, 0, 0}, {0, 1000000, 0, 0}, {0, 0, 1000000, 0}, {0, 0, 0, 1000000}};
const matrix_type ekf[4] = {1, 0, 0, 0};

static matrix_t Q;
static matrix_t R;
static matrix_t P;


/**
 * @brief 初始化 EKF 状态向量、过程噪声、测量噪声和协方差矩阵
 * @param void 无
 * @return void 无返回值
 */
void EKF_Init(void)
{
	Matrix_From_Array(&exf_x, (const matrix_type*)ekf, 4, 1);
	Matrix_From_Array(&Q, (const matrix_type*)q, 4, 4);
	Matrix_From_Array(&R, (const matrix_type*)r, 3, 3);
	Matrix_From_Array(&P, (const matrix_type*)p, 4, 4);
}




/**
 * @brief 将当前四元数姿态转换为欧拉角输出
 * @param void 无
 * @return void 无返回值
 */
static inline void quaternion_to_euler(void)
{
    float q0 = (exf_x.data[0][0]);
    float q1 = (exf_x.data[1][0]);
    float q2 = (exf_x.data[2][0]);
    float q3 = (exf_x.data[3][0]);

    euler_angle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * DEG_TO_RAD;                                  // 四元数转俯仰角
    euler_angle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * DEG_TO_RAD;   // 四元数转横滚角
    euler_angle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * DEG_TO_RAD;    // 四元数转偏航角
}




static int16 imu660ra_acc_x_l = 0;
static int16 imu660ra_acc_y_l = 0;
static int16 imu660ra_acc_z_l = 0;

/**
 * @brief 读取 IMU 传感器数据并完成低通滤波与单位换算
 * @param void 无
 * @return void 无返回值
 */
void imu_get_values(void)
{
    imu660ra_get_gyro();
    imu660ra_get_acc();

    // 对加速度做一阶低通滤波，减小高频噪声对观测更新的影响
    imu_data.acc_x = Kk1 * imu660ra_acc_x + (1 - Kk1) * imu660ra_acc_x_l;
    imu_data.acc_y = Kk1 * imu660ra_acc_y + (1 - Kk1) * imu660ra_acc_y_l;
    imu_data.acc_z = Kk1 * imu660ra_acc_z + (1 - Kk1) * imu660ra_acc_z_l;
    imu660ra_acc_x_l = (int16)imu_data.acc_x;
    imu660ra_acc_y_l = (int16)imu_data.acc_y;
    imu660ra_acc_z_l = (int16)imu_data.acc_z;

    // 将陀螺仪原始量换算为弧度制角速度，供四元数积分预测使用
    imu_data.gyro_x = imu660ra_gyro_x * PI / 180 / 16.384f;
    imu_data.gyro_y = imu660ra_gyro_y * PI / 180 / 16.384f;
    imu_data.gyro_z = imu660ra_gyro_z * PI / 180 / 16.384f;
}





/**
 * @brief 执行一次姿态扩展卡尔曼滤波更新
 * @param void 无
 * @return void 无返回值
 */
void EKF_UpData(void)
{

    float gx, gy, gz;
    imu_get_values();
    gx = imu_data.gyro_x;
    gy = imu_data.gyro_y;
    gz = imu_data.gyro_z;

    matrix_t Z;

    Matrix_Init(&Z, 3, 1);

    Z.data[0][0] = (matrix_type)imu_data.acc_x;
    Z.data[1][0] = (matrix_type)imu_data.acc_y;
    Z.data[2][0] = (matrix_type)imu_data.acc_z;

    // 对加速度观测向量归一化，使其仅表达重力方向信息
    normalize_vector(&Z);

    // 根据角速度构造离散状态转移矩阵，完成四元数先验预测
    matrix_type f[4][4]= {{1, -0.5f * gx * dt, -0.5f * gy * dt, -0.5f * gz * dt},
                          {0.5f * gx * dt, 1, 0.5f * gz * dt, -0.5f * gy * dt},
                          {0.5f * gy * dt, -0.5f * gz * dt, 1, 0.5f * gx * dt},
                          {0.5f * gz * dt, 0.5f * gy * dt, -0.5f * gx * dt, 1}};

    matrix_t F,FT;
    Matrix_From_Array(&F, (const matrix_type*)f, 4, 4);
    FT = Matrix_Transpose(&F);

    // 状态预测：X(k|k-1) = F * X(k-1|k-1)
    exf_x = multiply_matrices(&F, &exf_x);
    // 四元数必须保持单位范数，否则姿态解会逐渐漂移
    normalize_vector(&exf_x);

    float q0 = (exf_x.data[0][0]);
    float q1 = (exf_x.data[1][0]);
    float q2 = (exf_x.data[2][0]);
    float q3 = (exf_x.data[3][0]);

    // 计算观测模型的雅可比矩阵 H，将非线性重力观测在线性化点处展开
    matrix_type h[3][4]={{-2 * q2, 2 * q3, -2 * q0, 2 * q1},
					     {2 * q1, 2 * q0, 2 * q3, 2 * q2},
					     {2 * q0, -2 * q1, -2 * q2, 2 * q3}};

    matrix_t H, HT;
    Matrix_From_Array(&H, (const matrix_type*)h, 3, 4);
    HT = Matrix_Transpose(&H);
    matrix_t PK_;

    // 协方差预测：P(k|k-1) = F * P(k-1|k-1) * F^T + Q
    PK_ = multiply_matrices(&F, &P);
    PK_ = multiply_matrices(&PK_, &FT);
    P = add_matrices(&PK_, &Q);


    // 创新协方差：S = H * P(k|k-1) * H^T + R
    matrix_t DK, invDK;
    DK = multiply_matrices(&H, &P);
    DK = multiply_matrices(&DK, &HT);
    DK = add_matrices(&DK, &R);

    if(inverse_matrix(&DK, &invDK))
    {
		// 若创新协方差不可逆，则仅保留预测结果，跳过本次观测更新
    	quaternion_to_euler();
    	return;
    }

    // 创新项：e = z - h(x)，用于衡量加速度观测与预测姿态的偏差
    matrix_t EK, EKT;
    EK = multiply_matrices(&H, &exf_x);
    EK = subtract_matrices(&Z, &EK);
    EKT = Matrix_Transpose(&EK);

    // 马氏距离检验：异常观测会使残差能量过大，此时拒绝更新
    error = multiply_matrices(&EKT, &invDK);
    error = multiply_matrices(&error, &EK);

    if(error.data[0][0] > r_yz)
    {
		// 剧烈线加速度或碰撞会污染重力观测，超阈值时直接丢弃本次量测
    	quaternion_to_euler();
    	return;
    }

    // 卡尔曼增益：K = P(k|k-1) * H^T * S^-1
    matrix_t Kk;
    Kk = multiply_matrices(&P, &HT);
    Kk = multiply_matrices(&Kk, &invDK);

    // 状态更新：X(k|k) = X(k|k-1) + K * e
    matrix_t temp;
    temp = multiply_matrices(&Kk, &EK);
    exf_x = add_matrices(&exf_x, &temp);
    normalize_vector(&exf_x);

    // 协方差更新：P(k|k) = (I - K * H) * P(k|k-1)
    matrix_t I;
    Matrix_Identity(&I, 4);
    temp = multiply_matrices(&Kk, &H);
    temp = subtract_matrices(&I, &temp);
    P = multiply_matrices(&temp, &P);
	quaternion_to_euler();

}
