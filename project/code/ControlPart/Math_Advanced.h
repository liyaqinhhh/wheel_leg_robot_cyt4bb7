
#ifndef CODE_COMMUNAL_MATH_ADVANCED_H_
#define CODE_COMMUNAL_MATH_ADVANCED_H_


#include "zf_common_headfile.h"


#define M_2PI 6.283185307179586
#define M_PI 3.141592653589793f
#define M_PI_2 1.570796326794897f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f
#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))


// 函数声明
void least_squares_fit(float arr[], int startline, int endline, float *k, float *b);
float invSqrt(float x);                                     // 快速计算 1/Sqrt(x)
float fast_atan(float v);                                   // 求反正切
float constrain_float(float amt, float low, float high);    // 约束值
int16 constrain_int16(int16 amt, int16 low, int16 high);    // 约束值
int32 constrain_int32(int32 amt, int32 low, int32 high);    // 约束值
float radians(float deg);                                   // 度转化为弧度
float degrees(float rad);                                   // 弧度转化为度
float sq(float v);                                          // 求平方
float pythagorous2(float a, float b);                       // 二维矢量长度
float pythagorous3(float a, float b, float c);              // 三维矢量长度
float wrap_180_cd(float error);                             // 以摄氏度为单位包裹角度
float wrap_90_cd(float error);                              // 以摄氏度为单位包裹角度
int Round_Number( float f );                                // 取整函数
int HalfAdjust_Number( float f );                           // 四舍五入函数
float limit( float value, float bound );                    // 限幅：将 value 限制在 [-bound, +bound]


#endif
