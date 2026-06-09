#include "zf_common_headfile.h"
#ifndef CODE_MATRIX_H_
#define CODE_MATRIX_H_


#define __weak                __attribute__((weak))
#define clip(x, min, max)    (((x) > (max)) ? (max) : (((x) < (min)) ? (min) : (x)))
#define ABS(x)               (((x) > 0) ? (x) : (-(x)))
#define clip2(x, num)        (clip((x), (-ABS(num)), (ABS(num))))
#define MAX(a, b)            (((a) > (b)) ? (a) : (b))
#define MIN(a, b)            (((a) < (b)) ? (a) : (b))



#define MAX_SIZE (4)
#define ASSERT(x) zf_assert(x)


typedef float matrix_type;

/**
 * @brief 通用矩阵结构体
 */
typedef struct
{
    int rows;                                /**< 矩阵行数 */
    int cols;                                /**< 矩阵列数 */
    matrix_type data[MAX_SIZE][MAX_SIZE];    /**< 矩阵元素存储区 */
}matrix_t;
extern matrix_t error;
extern matrix_t exf_x;

/**
 * @brief 欧拉角输出结构体
 */
typedef struct
{
    matrix_type roll;     /**< 横滚角，单位为度 */
    matrix_type pitch;    /**< 俯仰角，单位为度 */
    matrix_type yaw;      /**< 偏航角，单位为度 */
}EulerAngles;
extern EulerAngles euler_angle;


/**
 * @brief 初始化指定尺寸的零矩阵
 * @param martix 待初始化的矩阵指针
 * @param rows 矩阵行数
 * @param col 矩阵列数
 * @return void 无返回值
 */
void Matrix_Init(matrix_t*martix,int rows,int col);

/**
 * @brief 使用一维数组按行填充矩阵
 * @param mat 目标矩阵指针
 * @param array 源数组首地址
 * @param rows 矩阵行数
 * @param cols 矩阵列数
 * @return void 无返回值
 */
void Matrix_From_Array(matrix_t* mat, const matrix_type* array,const int rows,const int cols);

/**
 * @brief 创建指定阶数的单位矩阵
 * @param matrix 目标矩阵指针
 * @param size 单位矩阵阶数
 * @return void 无返回值
 */
void Matrix_Identity(matrix_t* matrix, int size);

/**
 * @brief 计算矩阵转置
 * @param src 源矩阵指针
 * @return matrix_t 转置后的矩阵
 */
matrix_t Matrix_Transpose(const matrix_t* src);

/**
 * @brief 计算两个矩阵的乘积
 * @param A 左操作数矩阵
 * @param B 右操作数矩阵
 * @return matrix_t 矩阵乘法结果
 */
matrix_t multiply_matrices(const matrix_t* A, const matrix_t* B);

/**
 * @brief 计算两个矩阵的逐元素和
 * @param A 第一个加数矩阵
 * @param B 第二个加数矩阵
 * @return matrix_t 矩阵加法结果
 */
matrix_t add_matrices(const matrix_t* A, const matrix_t* B);

/**
 * @brief 计算两个矩阵的逐元素差
 * @param A 被减矩阵
 * @param B 减数矩阵
 * @return matrix_t 矩阵减法结果
 */
matrix_t subtract_matrices(const matrix_t* A, const matrix_t* B);

/**
 * @brief 使用高斯消元法求矩阵逆
 * @param A 待求逆的方阵
 * @param invA 输出的逆矩阵
 * @return int 0 表示求逆成功，1 表示矩阵不可逆
 */
int inverse_matrix(matrix_t *A, matrix_t *invA);

/**
 * @brief 对行向量或列向量执行归一化
 * @param v 待归一化的向量
 * @return void 无返回值
 */
void normalize_vector(matrix_t *v);

/**
 * @brief 打印矩阵内容，便于调试观察
 * @param matrix 待打印的矩阵指针
 * @return void 无返回值
 */
void print_matrix(const matrix_t* matrix);


#endif /* CODE_MATRIX_H_ */
