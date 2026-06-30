
#ifndef CODE_COMMUNAL_MATH_ADVANCED_H_
#define CODE_COMMUNAL_MATH_ADVANCED_H_


#include "zf_common_headfile.h"


#define M_2PI 6.283185307179586             // 2*Pi 常量
#define M_PI 3.141592653589793f              // Pi 常量
#define M_PI_2 1.570796326794897f            // Pi/2 常量
#define DEG_TO_RAD 0.017453292519943295769236907684886f  // 角度转弧度系数 (Pi/180)
#define RAD_TO_DEG 57.295779513082320876798154814105f    // 弧度转角度系数 (180/Pi)
#define max(a,b) ((a)>(b)?(a):(b))          // 取两数最大值宏
#define min(a,b) ((a)<(b)?(a):(b))          // 取两数最小值宏


// 函数声明

// @brief  最小二乘法线性拟合 - 对数组指定区间进行直线拟合，求斜率k和截距b
// @param  arr        数据数组
// @param  startline  拟合起始索引
// @param  endline    拟合结束索引
// @param  k          输出斜率
// @param  b          输出截距
void least_squares_fit(float arr[], int startline, int endline, float *k, float *b);
// @brief  快速计算 1/Sqrt(x)，使用牛顿迭代法近似
float invSqrt(float x);                                     // 快速计算 1/Sqrt(x)
// @brief  快速反正切计算，返回弧度值
float fast_atan(float v);                                   // 求反正切
// @brief  将浮点数约束在 [low, high] 范围内
float constrain_float(float amt, float low, float high);    // 约束值
// @brief  将16位整数约束在 [low, high] 范围内
int16 constrain_int16(int16 amt, int16 low, int16 high);    // 约束值
// @brief  将32位整数约束在 [low, high] 范围内
int32 constrain_int32(int32 amt, int32 low, int32 high);    // 约束值
// @brief  角度转弧度
float radians(float deg);                                   // 度转化为弧度
// @brief  弧度转角度
float degrees(float rad);                                   // 弧度转化为度
// @brief  计算平方值
float sq(float v);                                          // 求平方
// @brief  计算二维矢量长度 sqrt(a^2+b^2)
float pythagorous2(float a, float b);                       // 二维矢量长度
// @brief  计算三维矢量长度 sqrt(a^2+b^2+c^2)
float pythagorous3(float a, float b, float c);              // 三维矢量长度
// @brief  将角度误差包裹到 [-180, +180] 范围内（单位：百分之一度）
float wrap_180_cd(float error);                             // 以摄氏度为单位包裹角度
// @brief  将角度误差包裹到 [-90, +90] 范围内（单位：百分之一度）
float wrap_90_cd(float error);                              // 以摄氏度为单位包裹角度
// @brief  浮点数取整（向零截断）
int Round_Number( float f );                                // 取整函数
// @brief  浮点数四舍五入取整
int HalfAdjust_Number( float f );                           // 四舍五入函数
// @brief  限幅：将 value 限制在 [-bound, +bound] 范围内
float limit( float value, float bound );                    // 限幅：将 value 限制在 [-bound, +bound]


#endif
