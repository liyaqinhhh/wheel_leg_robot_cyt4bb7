#include "zf_common_headfile.h"
#include "matrix.h"

/**
 * @brief 初始化指定行列尺寸的零矩阵
 * @param martix 待初始化的矩阵指针
 * @param rows 矩阵行数
 * @param cols 矩阵列数
 * @return void 无返回值
 */
void Matrix_Init(matrix_t* martix, int rows, int cols)
{
    ASSERT(rows > 0 && cols > 0);
    martix->rows = rows;
    martix->cols = cols;
    memset(martix->data, 0, MAX_SIZE * MAX_SIZE * sizeof(matrix_type));
}



/**
 * @brief 构造指定阶数的单位矩阵
 * @param matrix 目标矩阵指针
 * @param size 单位矩阵阶数
 * @return void 无返回值
 */
void Matrix_Identity(matrix_t* matrix, int size)
{
    ASSERT(size > 0);
    matrix->rows = size;
    matrix->cols = size;
    memset(matrix->data, 0, sizeof(matrix->data));
    for(int i = 0; i < size; i++)
    {
        matrix->data[i][i] = 1.0f;
    }
}



/**
 * @brief 使用一维数组按行优先顺序初始化矩阵
 * @param mat 目标矩阵指针
 * @param array 源数组首地址
 * @param rows 矩阵行数
 * @param cols 矩阵列数
 * @return void 无返回值
 */
void Matrix_From_Array(matrix_t* mat, const matrix_type* array,const int rows,const int cols)
{
    ASSERT(NULL != array);
    Matrix_Init(mat, rows, cols);
    for(int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            mat->data[i][j] = array[i * cols + j];
        }
    }
}




/**
 * @brief 计算矩阵转置
 * @param src 源矩阵指针
 * @return matrix_t 转置后的矩阵
 */
matrix_t Matrix_Transpose(const matrix_t* src)
{
    matrix_t dest;
    Matrix_Init(&dest, src->cols, src->rows);

    // 将源矩阵第 i 行第 j 列元素映射到结果矩阵第 j 行第 i 列
    for(int i = 0; i < src->rows; i++)
    {
        for(int j = 0; j < src->cols; j++)
        {
            dest.data[j][i] = src->data[i][j];
        }
    }
    return dest;
}




/**
 * @brief 计算两个矩阵的乘积
 * @param A 左操作数矩阵
 * @param B 右操作数矩阵
 * @return matrix_t 乘法结果矩阵
 */
matrix_t multiply_matrices(const matrix_t* A, const matrix_t* B)
{
    ASSERT(A->cols == B->rows);

    matrix_t dest;
    Matrix_Init(&dest, A->rows, B->cols);

    // 三重循环按行列内积计算每个元素：C(i,j) = Σ A(i,k) * B(k,j)
    for(int i = 0; i < A->rows; i++)
    {
        for(int j = 0; j < B->cols; j++)
        {
            for(int k = 0; k < A->cols; k++)
            {
                dest.data[i][j] += A->data[i][k] * B->data[k][j];
            }
        }
    }

    return dest;
}



/**
 * @brief 计算两个矩阵的逐元素和
 * @param A 第一个加数矩阵
 * @param B 第二个加数矩阵
 * @return matrix_t 加法结果矩阵
 */
matrix_t add_matrices(const matrix_t* A, const matrix_t* B)
{
    ASSERT(A->rows == B->rows && A->cols == B->cols);

    matrix_t result;
    Matrix_Init(&result, A->rows, A->cols);

    // 对应位置元素逐个相加，保持原矩阵维度不变
    for(int i = 0; i < A->rows; i++)
    {
        for(int j = 0; j < A->cols; j++)
        {
            result.data[i][j] = A->data[i][j] + B->data[i][j];
        }
    }

    return result;
}




/**
 * @brief 计算两个矩阵的逐元素差
 * @param A 被减矩阵
 * @param B 减数矩阵
 * @return matrix_t 减法结果矩阵
 */
matrix_t subtract_matrices(const matrix_t* A, const matrix_t* B)
{
    ASSERT(A->rows == B->rows && A->cols == B->cols);

    matrix_t result;
    Matrix_Init(&result, A->rows, A->cols);

    // 对应位置元素逐个相减，适用于状态残差和协方差修正
    for(int i = 0; i < A->rows; i++)
    {
        for(int j = 0; j < A->cols; j++)
        {
            result.data[i][j] = A->data[i][j] - B->data[i][j];
        }
    }

    return result;
}




/**
 * @brief 使用高斯消元法求方阵逆矩阵
 * @param A 待求逆的方阵
 * @param invA 输出的逆矩阵
 * @return int 0 表示求逆成功，1 表示矩阵不可逆
 */
int inverse_matrix(matrix_t* A, matrix_t* invA)
{
    ASSERT(A->rows == A->cols);

    const matrix_type THRESHOLD = 1e-6;

    Matrix_Init(invA, A->rows, A->cols);

    int n = A->rows;

    matrix_type augmented[MAX_SIZE][2 * MAX_SIZE];

    // 构造增广矩阵 [A | I]，左侧为原矩阵，右侧为单位矩阵
    for(int i = 0; i < n; i++)
    {
        for(int j = 0; j < n; j++)
        {
            augmented[i][j] = A->data[i][j];
            augmented[i][j + n] = (float)((i == j) ? 1 : 0);
        }
    }

    // 逐列执行高斯-若尔当消元
    for(int i = 0; i < n; i++)
    {
        // 选取当前列绝对值最大的主元，提高数值稳定性
        int max_row = i;
        for(int j = i + 1; j < n; j++)
        {
            if(fabs(augmented[j][i]) > fabs(augmented[max_row][i]))
            {
                max_row = j;
            }
        }

        // 主元过小意味着矩阵奇异或病态，无法可靠求逆
        if(fabs(augmented[max_row][i]) < THRESHOLD)
        {
            return 1;
        }

        // 通过交换行把最大主元移动到当前消元行
        if(max_row != i)
        {
            for(int j = 0; j < 2 * n; j++)
            {
                matrix_type temp = augmented[i][j];
                augmented[i][j] = augmented[max_row][j];
                augmented[max_row][j] = temp;
            }
        }

        // 将主元行归一化，使主对角元素变为 1
        matrix_type pivot = augmented[i][i];
        for(int j = 0; j < 2 * n; j++)
        {
            augmented[i][j] /= pivot;
        }

        // 用当前主元行消去本列其他行元素，最终把左半部分化为单位矩阵
        for(int j = 0; j < n; j++)
        {
            if(j != i)
            {
                matrix_type factor = augmented[j][i];
                for(int k = 0; k < 2 * n; k++)
                {
                    augmented[j][k] -= factor * augmented[i][k];
                }
            }
        }
    }

    // 增广矩阵右半部分即为 A^-1
    for(int i = 0; i < n; i++)
    {
        for(int j = 0; j < n; j++)
        {
            invA->data[i][j] = augmented[i][j + n];
        }
    }

    return 0;
}





/**
 * @brief 快速计算平方根倒数，用于向量归一化
 * @param x 输入标量
 * @return float 1/sqrt(x) 的近似值
 */
static inline float invSqrt(float x)
{
    float xhalf = 0.5f * x;

    int i = *(int*)&x;

    i = 0x5f375a86 - (i >> 1);

    x = *(float*)&i;

    x = x * (1.5f - xhalf * x * x);

    return x;
}




/**
 * @brief 对行向量或列向量做单位化处理
 * @param v 待归一化的向量
 * @return void 无返回值
 */
void normalize_vector(matrix_t *v)
{
    ASSERT(1 == v->cols || 1 == v->rows);

    matrix_type norm = 0;

    if(1 == v->rows)
    {
        for(int i = 0; i < v->cols; ++i)
        {
            norm += (v->data[0][i] * v->data[0][i]);
        }
    }
    if(1 == v->cols)
    {
        for(int i = 0; i < v->rows; ++i)
        {
            norm += (v->data[i][0] * v->data[i][0]);
        }
    }

    // 先求模长倒数，再统一缩放每个元素，减少除法运算量
    norm = invSqrt((float)norm);
    if(1 == v->rows)
    {
        for(int i = 0; i < v->cols; ++i)
        {
            v->data[0][i] *= norm;
        }
    }
    if(1 == v->cols)
    {
        for(int i = 0; i < v->rows; ++i)
        {
            v->data[i][0] *= norm;
        }
    }
}





/**
 * @brief 打印矩阵内容，便于调试观察
 * @param matrix 待打印的矩阵指针
 * @return void 无返回值
 */
void print_matrix(const matrix_t* matrix)
{
    for(int i = 0; i < matrix->rows; i++)
    {
        for(int j = 0; j < matrix->cols; j++)
        {
            printf("%2f ", matrix->data[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}
