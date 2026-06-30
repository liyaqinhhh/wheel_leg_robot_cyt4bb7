#include "Math_Advanced.h"
#include "zf_common_headfile.h"
#include "math.h"

// 最小二乘法拟合直线
// y = kx + b
void least_squares_fit(float arr[], int startline, int endline, float *k, float *b) {
    int n = endline - startline + 1;  // 数据点数量
    float sum_x = 0.0f, sum_y = 0.0f, sum_xy = 0.0f, sum_x2 = 0.0f;

    // 计算各项累加和
    for (int y = startline; y <= endline; y++) {
        float x = arr[y];  // x 是 arr[y]
        float y_val = (float)(y - startline);  // y 从 0 开始
        sum_x += x;
        sum_y += y_val;
        sum_xy += x * y_val;
        sum_x2 += x * x;
    }

    // 计算斜率和截距
    float denominator = n * sum_x2 - sum_x * sum_x;
    if (denominator == 0.0f) {
        // 防止除零错误（数据点共线或不足）
        *k = 0.0f;
        *b = 0.0f;
        return;
    }

    *k = (n * sum_xy - sum_x * sum_y) / denominator;
    *b = (sum_y - (*k) * sum_x) / n;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      快速计算 1/Sqrt(x)
////  @param      x       所要进行运算的数----开方
////  @return     y       结果
////  @note       快速计算 1/Sqrt(x)  罕绕胀^qrt()函数要快四倍See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
////-------------------------------------------------------------------------------------------------------------------
//float invSqrt(float x)
//{
//    float halfx = 0.5f * x;
//    float y = x;
//    long i = *(long*)&y;
//    i = 0x5f3759df - (i>>1);
//    y = *(float*)&i;
//    y = y * (1.5f - (halfx * y * y));
//    return y;
//}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      a faster varient of atan.  accurate to 6 decimal places for values between -1 ~ 1 but then diverges quickly
//  @param      v       所要进行运算的数
//  @return     结果
//  @note       a faster varient of atan.  accurate to 6 decimal places for values between -1 ~ 1 but then diverges quickly
//-------------------------------------------------------------------------------------------------------------------
float fast_atan(float v)
{
    float v2 = v*v;
    return (v*(1.6867629106f + v2*0.4378497304f)/(1.6867633134f + v2));
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      约束值     constrain a value
//  @param      amt
//  @param      low         最低值
//  @param      high        最高值
//  @return     结果
//  @note       约束值     constrain a value
//-------------------------------------------------------------------------------------------------------------------
float constrain_float(float amt, float low, float high)
{
    // the check for NaN as a float prevents propogation of
    // floating point errors through any function that uses
    // constrain_float(). The normal float semantics already handle -Inf
    // and +Inf
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      约束值     constrain a int16_t value
//  @param      amt
//  @param      low         最低值
//  @param      high        最高值
//  @return     结果
//  @note       约束值     constrain a int16_t value
//-------------------------------------------------------------------------------------------------------------------
int16 constrain_int16(int16 amt, int16 low, int16 high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      约束值     constrain a int32_t value
//  @param      amt
//  @param      low         最低值
//  @param      high        最高值
//  @return     结果
//  @note       约束值     constrain a int32_t value
//-------------------------------------------------------------------------------------------------------------------
int32 constrain_int32(int32 amt, int32 low, int32 high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      度到弧度的转化     degrees -> radians
//  @param      deg     角度
//  @return     结果
//  @note       度到弧度的转化     degrees -> radians
//-------------------------------------------------------------------------------------------------------------------
float radians(float deg)
{
    return deg * DEG_TO_RAD;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      弧度到度的转化     radians -> degrees
//  @param      deg     弧度
//  @return     结果
//  @note       弧度到度的转化     radians -> degrees
//-------------------------------------------------------------------------------------------------------------------
float degrees(float rad)
{
    return rad * RAD_TO_DEG;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      求平方     square
//  @param      v       所要进行运算的数字
//  @return     结果
//  @note       求平方     square
//-------------------------------------------------------------------------------------------------------------------
float sq(float v)
{
    return v*v;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      二维矢量长度      vector length
//  @param      a       所要进行运算的数字
//  @param      b       所要进行运算的数字
//  @return     结果
//  @note       二维矢量长度      vector length
//-------------------------------------------------------------------------------------------------------------------
float pythagorous2(float a, float b)
{
    return sqrtf(sq(a)+sq(b));
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      三维矢量长度      vector length
//  @param      a       所要进行运算的数字
//  @param      b       所要进行运算的数字
//  @param      c       所要进行运算的数字
//  @return     结果
//  @note       三维矢量长度      vector length
//-------------------------------------------------------------------------------------------------------------------
float pythagorous3(float a, float b, float c)
{
    return sqrtf(sq(a)+sq(b)+sq(c));
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      以摄氏度为单位包裹角度 wrap an angle in centi-degrees to -18000..18000
//  @param      error
//  @return     结果
//  @note       以摄氏度为单位包裹角度 wrap an angle in centi-degrees to -18000..18000
//-------------------------------------------------------------------------------------------------------------------
float wrap_180_cd(float error)
{
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    return error;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      以摄氏度为单位包裹角度 wrap an angle in centi-degrees to -90..90
//  @param      error
//  @return     结果
//  @note       以摄氏度为单位包裹角度 wrap an angle in centi-degrees to -90..90
//-------------------------------------------------------------------------------------------------------------------
float wrap_90_cd(float error)
{
    while (error > 90) error -= 180;
    while (error < -90) error += 180;
    return error;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      取整函数
//  @param      f       输入浮点数
//  @return     取整[f]
//  @note       [1.5] = 1  [-1.5] = -2
//-------------------------------------------------------------------------------------------------------------------
int Round_Number( float f )
{
    if( f >= 0.0 )
    {
        return (int)(f + 0.5);
    }
    else
    {
        return (int)(f - 0.5);
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      四舍五入函数
//  @param      f       输入浮点数
//  @return     取整[f]
//  @note       [1.5] = 1  [-1.5] = -2
//-------------------------------------------------------------------------------------------------------------------
int HalfAdjust_Number( float f )
{
    int intPart = (int)f;
    float fractionPart = f - intPart;

    if( fractionPart >= 0.5 )
    {
        if( f >= 0.0 )
        {
            intPart++;
        }
        else
        {
            intPart--;
        }
    }

    return intPart;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      限幅函数
//  @param      value   输入值
//  @param      bound   限幅范围（正值），输出被限制在 [-bound, +bound]
//  @return     限幅后的值
//  @note       limit(value, bound) -> clamp to [-bound, +bound]
//-------------------------------------------------------------------------------------------------------------------
float limit(float value, float bound)
{
    if(value > bound)  return bound;
    if(value < -bound) return -bound;
    return value;
}

//
//
//
//
////自定义pow函数
//
//double func_pow(double x,int k)
//
//{
//
//    if (k > 0)
//
//        return x * func_pow(x, k - 1);
//
//    else if (k == 0)
//
//        return 1;
//
//    else
//
//        return 1.0 / func_pow(x, -k);
//
//}
//
////自定义sqrt函数
//
//double func_sqrt(double x)
//
//{
//
//    double j,k;
//
//    j=0.0;
//
//    k=x/2;
//
//    while(j!=k)
//
//    {
//
//      j=k;
//
//      k=(j+x/j)/2;
//
//    }
//
//    return j;
//
// }
//
////自定义cos函数
//
//double func_cos(double x)
//
//{
//
//    x=func_abs(x);
//
//
//
//    int t,q=1;
//
//    double term,factorial=1.0,sum2=1,sxm,sum1=0;
//
//    for(t=2;;t++)
//
//    {
//
//        factorial=factorial*t;
//
//        if(t%2==0)
//
//        {
//
//             sum1=sum2;
//
//             q=q*(-1);
//
//             sxm=func_abs(func_pow(x,t));
//
//             term=sxm/factorial;
//
//             sum2=q*term+sum2;
//
//        }
//
//        if(func_abs(sum2-sum1)<=1e-13)
//
//        break;
//
//    }
//
//    return sum2;
//
// }
//
////自定义sin函数
//
//double func_sin(double x)
//
//{
//
//    int counter=0;
//
//    if(x<0)
//
//        counter = 1;
//
//    else
//
//        counter = 0;
//
//    x=func_abs(x);
//
//
//
//    int t,q=1;
//
//    double term,factorial=1.0,sum2,sxm,sum1=0;
//
//
//
//    sum2=x;
//
//    for(t=2;;t++)
//
//    {
//
//        factorial=factorial*t;
//
//        if(t%2!=0)
//
//        {
//
//            sum1=sum2;
//
//            q=q*(-1);
//
//            sxm=func_abs(func_pow(x,t));
//
//            term=sxm/factorial;
//
//            sum2=q*term+sum2;
//
//        }
//
//        if(func_abs(sum2-sum1)<=1e-13)
//
//            break;
//
//    }
//
//
//
//    if(counter==1)
//
//    {
//
//        sum2=-sum2;
//
//        return sum2;
//
//    }
//
//    else
//
//        return sum2;
//
// }
//
//
//
//
