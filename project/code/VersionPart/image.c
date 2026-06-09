/*
 * image.c
 * 图像处理与赛道元素识别模块。
 *
 * 该文件负责摄像头图像预处理、边线提取、元素识别、控制量计算以及
 * 相关调试参数维护；同时包含寻迹与元素处理所需的全局配置和中间状态。
 * 本文件中还包含大体量图像查找表/点阵数据，维护时仅应修改逻辑代码或
 * 文件头说明，避免对静态数据数组进行无关改动。
 */
#include "zf_common_headfile.h"
#include "math.h"
#include "image.h"
#include "Interrupt.h"
#include "servo.h"
#include "imu660.h"
#include "kalman.h"
#include "small_driver_uart_control.h"
#include "Math_Advanced.h"
#include "ips.h"
#include "PID.h"

uint16 set_speed,forward_target;

uint8 Code_type = 2;
uint8 buzzer_flag = 0;
float Z_Yaw = 0;
//extern uint8 buzzer_flag;
/*摄像头定位数据：（米）
 *          高：          0.305
 *          近瞻：        0.13
 *          远瞻：        2.60
 */
//******************元素处理标志位************************
//用于决定是否识别并处理对应元素，并选择处理方式
//当组别赛道中无此元素时，将对应标志挂0
uint8 Jump_Control_Flag     =    0;
uint8 Single_Control_Flag   =    0;

uint8 Straightaway_Flag     =    1;       //直道             0：关闭        1：开启
uint8 L_Turn_Flag           =    1;       //左转弯道         0：关闭        1：开启
uint8 R_Turn_Flag           =    1;       //右转弯道         0：关闭        1：开启
uint8 L_Circle_Flag         =    1;       //左环岛           0：关闭        1：单边巡线处理     2.三点法补圆       3.二次贝塞尔补线     4.三次贝塞尔补线     5.拉格朗日补线   //最小二乘法拟合圆  牛顿插值补线   样条插值
uint8 R_Circle_Flag         =    1;       //右环岛           0：关闭        1：单边巡线处理     2.三点法补圆       3.二次贝塞尔补线     4.三次贝塞尔补线     5.拉格朗日补线   //最小二乘法拟合圆  牛顿插值补线   样条插值
uint8 Cross_Flag            =    1;       //十字路口         0：关闭        1：不做特殊处理     2.最小二乘法补线    3.二次贝塞尔补线处理
uint8 L_Oblique_Cross_Flag  =    0;       //斜十字路口       0：关闭        1：开启（补线处理）
uint8 R_Oblique_Cross_Flag  =    0;       //斜十字路口       0：关闭        1：开启（补线处理）
uint8 Zebra_Flag            =    1;       //斑马线           0：关闭        1：开启
uint8 Ramp_Flag             =    0;       //坡道             0：关闭        1：TOF判断  2：图像处理
uint8 Three_Bif_Flag        =    0;       //三岔             0：关闭        1：开启
uint8 Barrier_Flag          =    1;       //障碍             0：关闭        1：开启
uint8 Disconnection_Flag    =    0;       //断路             0：关闭        1：开启
uint8 T_Way_Flag            =    0;       //T路口            0：关闭        1：开启
uint8 L_Garage_Flag         =    0;       //左车库           0：关闭        1：开启
uint8 R_Garage_Flag         =    0;       //右车库           0：关闭        1：开启
uint8 Small_S_Flag          =    0;       //小S弯            0:关闭         1:识别

//******************速度控制************************
uint16 ConvenTion_Speed                  =       1000;
uint16 Derailment_Speed                  =       0;        //出赛道速度
uint16 Straightaway_Speed                =       1000;        //直道速度
uint16 L_Turn_Speed                      =       1000;        //左转弯道速度
uint16 R_Turn_SPeed                      =       1000;        //右转弯道速度
uint16 Jump_speed                        =       1200;
uint16 Single_speed_1                    =       300;     // 切换姿态时的速度
uint16 Single_speed_2                    =       500;    // 通过单边桥的速度

uint16 L_Circle_State_1_Speed            =       1200;        //左环岛预入环  1  速度
uint16 L_Circle_State_2_Speed            =       1200;        //左环岛入环  2  速度
uint16 L_Circle_State_3_Speed            =       1400;        //左环岛环中  3  速度
uint16 L_Circle_State_4_Speed            =       1400;        //左环岛预出环  4  速度
uint16 L_Circle_State_5_Speed            =       1400;        //左环岛出环  5  速度

uint16 R_Circle_State_1_Speed            =       1200;        //右环岛状态  1  速度
uint16 R_Circle_State_2_Speed            =       1200;        //右环岛状态  2  速度
uint16 R_Circle_State_3_Speed            =       1400;        //右环岛状态  3  速度
uint16 R_Circle_State_4_Speed            =       1400;        //右环岛状态  4  速度
uint16 R_Circle_State_5_Speed            =       1400;        //右环岛状态  5  速度

uint16 Cross_Speed                       =       1500;        // 通过十字路口速度
//uint16 Cross_State_3_Speed               =       100;        // 十字中速度(积分中)
uint16 Cross_State_4_Speed               =       1500;        // 十字中速度(积分中)

uint16 Zebra_Speed                       =       0;        //斑马线速度

uint16 Ramp_State_1_Speed                =       800;        //坡道速度
uint16 Ramp_State_2_Speed                =       800;        //坡道速度
uint16 Ramp_State_3_Speed                =       800;        //坡道速度

uint16 Three_Bif_Speed           =       200;       //三岔速度
uint16 Barrier_Speed                     =       200;       //障碍速度
uint16 Disconnection_Speed       =       200;       //断路速度
uint16 T_Way_Speed               =       200;       //T路口速度
uint16 L_Garage_Speed            =       200;       //左车库速度
uint16 R_Garage_Speed            =       200;       //右车库速度
uint16 L_Oblique_Cross_Speed            =       150;       //斜入十字——左
uint16 R_Oblique_Cross_Speed            =       150;       //斜入十字——右
uint16 Small_S_Speed                    =       150;       //小S弯
//*******************************************************

//******************可调参数************************
//自适应阈值相关
float Thres_Interfere = 0.0f;           //1783.0f;    //手动干预计算阈值的总灰度值
float Thres_Num_Interfere = 25.0f;      //34.0f;  //手动干预计算阈值时所除个数
//第一张图补黑框的像素值
uint8 Black_Box_Value = 50;
//控制相关
float Straightaway_Error_Coefficient = 0.5;     //削弱直线时的偏差值

//是否开启平滑反向滤波标志位
    /*
     * 有自动处理，但可手动干预开启滤波
     * 当室内光线较均匀时，反向平滑滤波反而在一定程度会影响图像处理，正常情况下不建议开启
     */
uint8 Thres_Filiter_Flag_1 = 0;   //当赛道存在光线剧烈变化的情况时，可手动开启反向平滑滤波

//模糊处理标志位
uint8 Vague_Flag = 0;
//*************************************************

//******************内外部调用参数************************
//元素标志位
uint8 Element_State = 0;
//偏差值
float Deviation_Value = 0;
//停车标志位
uint8 Stop_Flag = 0;
//外部调用，用于按键关闭当前标志位
uint8 Element_Break_Flag = 0;
//显示模式标志位
extern uint8 Show_Flag;
//其余显示标志位（开启代码内部测试的某些显示）
extern uint8 Other_Show_Flag;
//获取一帧逆透视标志位
uint8 Inverse_Flag = 0;
//速度比例，用于速度决策
float Speed_Proportion = 0;
//*****************************************************

//****************** 图像参数 ************************
//记录处理图像的个数
uint32 Image_Num = 0;

//用来处理的主图像
uint8 Find_Line_Image[Image_Y][Image_X];
//存放边线的一维数组
uint8 R_Border[Image_Y] = {0};
uint8 L_Border[Image_Y] = {0};

//存放边线点的x，y坐标（二维数组）
uint8 L_Line[(uint16)Use_Num][2] = {{0}};//左线
uint8 R_Line[(uint16)Use_Num][2] = {{0}};//右线
uint8 C_Line[(uint16)Use_Num] = {0};

//统计找到的边线点的个数
uint16 L_Statics = 0;//统计左边找到点的个数
uint16 R_Statics = 0;//统计右边找到点的个数

//起点
uint8 L_Start_Point[2] = { 0 };
uint8 R_Start_Point[2] = { 0 };

//图像边界
uint8 X_Border_Min = 0;
uint8 X_Border_Max = Image_X - 1;
uint8 Y_Border_Min = 0;
uint8 Y_Border_Max = Image_Y - 1;

//记录边线每个点的生长方向
int8 L_Grow_Dir[(uint16)Use_Num] = {0};     //记录左边爬线生长方向
int8 R_Grow_Dir[(uint16)Use_Num] = {0};     //记录右边爬线生长方向

//记录左右爬线的相遇点
uint8 Y_Meet = 0;   //相遇点的Y坐标
uint8 X_Meet = 0;   //相遇点的X坐标

//记录爬线时每个点的阈值
float L_Thres_Record[(uint16)Use_Num] = {0};
float R_Thres_Record[(uint16)Use_Num] = {0};

//斑马线检定阈值，用于判断斑马线
uint8 Zbra_Thres = 100;

//自动开启反向平滑滤波标志位
uint8 Thres_Filiter_Flag_2 = 0;
//*****************************************************

//***************************  双边线拟合相关参数  *******************************（这部分变量记住就好，代码原理为实验失败品，跳过）
//当场地有大量 暗-明-暗 的光线变化时， 当关闭双边线拟合， 使用自适应边线并打开反向平滑滤波 Thres_Filiter_Flag_1 = 1
uint8 Bilatreal_Line_Fitting_Flag = 1;          //双边线拟合标志位
                                                //0: 关闭，正常人一般没这种需求（线都没了跑啥啊）
                                                //1：自适应边线权值为1,即完全信任自适应边线
                                                //2: 二值化边线权值为1，即完全信任二值化边线
                                                //3： 双边线加权拟合新边线

//光线不稳定时，自适应权值更大，光线稳定时，二值化权值更大
uint8 Weight_Thres_Min = 0;         //双边线拟合加权的最小阈值，根据实际测试，当阈值小于1.1 * Weight_Thres_Min时，自适应边线权值为1
uint8 Weight_Thres_Mid = 0;         //双边线拟合加权的中间阈值，根据实际测试，当阈值等于 Weight_Thres_Mid 时，二值化边线权值为1
uint8 weight_Thres_Max = 0;         //双边线拟合加权的最大阈值，根据实际测试，当阈值大于0.9 * weight_Thres_Max时，自适应边线权值为1

uint8 Adaptive_L_Line[(uint16)Use_Num][2] = {{0}};      //自适应求得左线
uint8 Adaptive_R_Line[(uint16)Use_Num][2] = {{0}};      //自适应求得右线

uint16 Adaptive_L_Statics = 0;                   //统计记录自适应左边找到点的个数
uint16 Adaptive_R_Statics = 0;                   //统计记录自适应右边找到点的个数

uint8 Adaptive_X_Meet = 0;
uint8 Adaptive_Y_Meet = 0;

//记录边线每个点的生长方向
int8 Adaptive_L_Grow_Dir[(uint16)Use_Num] = {0};           //记录左边爬线生长方向
int8 Adaptive_R_Grow_Dir[(uint16)Use_Num] = {0};           //记录右边爬线生长方向

uint8 Adaptive_L_Start_Point[2] = { 0 };
uint8 Adaptive_R_Start_Point[2] = { 0 };

uint8 Binarization_L_Line[(uint16)Use_Num][2] = {{0}};      //二值化求得左线
uint8 Binarization_R_Line[(uint16)Use_Num][2] = {{0}};      //二值化求得右线

uint16 Binarization_L_Statics = 0;                   //统计记录自适应左边找到点的个数
uint16 Binarization_R_Statics = 0;                   //统计记录自适应右边找到点的个数

uint8 Binarization_X_Meet = 0;
uint8 Binarization_Y_Meet = 0;

//记录边线每个点的生长方向
int8 Binarization_L_Grow_Dir[(uint16)Use_Num] = {0};          //记录左边爬线生长方向
int8 Binarization_R_Grow_Dir[(uint16)Use_Num] = {0};          //记录右边爬线生长方向

uint8 Binarization_L_Start_Point[2] = { 0 };
uint8 Binarization_R_Start_Point[2] = { 0 };
//******************************************************************************

//***********************  入环刹车系数  ***************************
//图像计数定时，当帧率为500时，一张图像为2ms，那么就可以将此用于简单的定时
//但实测188*120图像帧率摄像头最高只能到250左右，被硬件限制
int32 Image_Count = 0;
//开启图像计时标志位
uint8 Image_Count_Flag = 0;

float Circle_Brake_Value = 0;           //入环出环时的刹车系数，根据实际速度调整
int16 Brake_Time = 0;                   //入环时的刹车时间，根据实际速度调整
//*****************************************************************

//****************************  速度控制相关参数  *************************************
//灵敏度，反映中线弯曲程度，可理解为为曲率
float Sensitivity = 0;
//弯道最小速度与直道最大速度的比值
float Speed_Min_Proportion = 0;
//减速拉伸系数，此参数越大，减速距离越短，最大值为1
float Stretch_Coefficient = 0.3f;
//设定的最大速度，即在长直道的速度
uint16 Max_Speed = 1600;
uint16 Min_Speed = 1350;
//开启速度决策的标志位，过斑马线后打开（开始速度较高会导致摩托发车极其困难）
uint8 Speed_Contral_Flag = 1;

uint16 Speed_Control = 0;
//***********************************************************************************
//障碍距离
int16 Barrier_Distance = 0;
//开启陀螺仪积分标志位
uint8 IMU_JF_Flag = 0;
//处理标志位，使每张图像只处理一次
uint8 Process_Flag = 0;         // 0：不处理            1： 处理
//************************************************************************灰度爬线部分*******************************************************************************
/**
* 函数功能：      初始化 TFT180 和 总钻风摄像头
* 特殊说明：      与 Cammer_Init_IPS200 函数只可调用其中一个，当都不调用时，默认开启摄像头初始化
* 形  参：        无
* 示例：          Cammer_Init_TFT180();
* 返回值：        无
*/
void Cammer_Init_TFT180(void)                  //初始化摄像头和显示屏     *
{
    tft180_show_string(0,0,"mt9v034 init.");
    while(1)
    {
        if(mt9v03x_init())      //摄像头初始化
        {
            tft180_show_string(0,16,"mt9v034 reinit.");
        }
        else
        {
            break;
        }
    }
    tft180_show_string(0,16,"init success.");
    tft180_clear();

}

/**
* 函数功能：      初始化 IPS200 和 总钻风摄像头
* 特殊说明：      与 Cammer_Init_TFT180 函数只可调用其中一个，当都不调用时，默认开启摄像头初始化
* 形  参：        无
* 示例：          Cammer_Init_IPS200();
* 返回值：        无
*/
void Cammer_Init_IPS200(void)                  //初始化摄像头和显示屏         *
{
    ips200_show_string(0,0,"mt9v034 init.");
    while(1)
    {
        if(mt9v03x_init())      //摄像头初始化
        {
            ips200_show_string(0,16,"mt9v034 reinit.");
        }
        else
        {
            break;
        }
    }
    ips200_show_string(0,16,"init success.");
    ips200_clear();
}

/**
* 函数功能：      初始化总钻风摄像头
* 特殊说明：      当 Cammer_Init_TFT180 和 Cammer_Init_IPS200 都不调用时，默认开启摄像头初始化
* 形  参：        无
* 示例：          Cammer_Init();
* 返回值：        无
*/
void Cammer_Init(void)                  //初始化摄像头和显示屏            *
{
    while(1)
    {
        if(mt9v03x_init())      //摄像头初始化
        {
        }
        else
        {
            break;
        }
    }
}

/**
* 函数功能：      复制并压缩图像，此处压缩为四分之一
* 特殊说明：      总钻风使用手册中说明：图像分辨率为  752 * 480， 376 * 240， 188 * 120 这三种分辨率视野是一样的，三者呈整数倍关系
*                其他分辨率是通过裁减得到的(这个裁减包含比188 * 120小的任何分辨率，如 94 * 60)，如376 * 240 的视野反而比752 * 400 的视野广
*                此处将总钻风传回图像 188 * 120 压缩为 80 * 60， 所以将 j 乘系数 2.35（188 / 80）
*                经实际测试，当设置图像大小为 94 * 60 时，传回的图像视野是 188 * 120 的四分之一，虽然也和 752 * 480 呈整数倍关系，但和上述情况不同
* 形  参：        无
* 示例：          Copy_Zip_Image();
* 返回值：        无
*/

void Copy_Zip_Image(void)               //*****
{
    uint8 i,j;
    if(mt9v03x_finish_flag == 1 && Inverse_Flag == 0)
    {
        for(i = 0; i < Image_Y; i++)
        {
            for(j = 0; j < Image_X; j++)
            {
                Find_Line_Image[i][j] = mt9v03x_image[i * 2][(uint8)(j * 2.35)];
            }
        }
        if(Image_Count_Flag == 1)
        {
            Image_Count ++;
        }
        else if(Image_Count_Flag == 0)
        {
            Image_Count = 0;
        }
        Image_Num ++;
        mt9v03x_finish_flag = 0;
    }
    else if(mt9v03x_finish_flag == 1 && Inverse_Flag == 1)
    {
        for(i = 0; i < Image_Y; i++)
        {
            for(j = 0; j < Image_X; j++)
            {
                Find_Line_Image[i][j] = mt9v03x_image[i * 2][(uint8)(j * 2.35f)];
            }
        }
//        Get_Inverse_Perspective_Image(Find_Line_Image, I_Perspective_Image);
        if(Image_Count_Flag == 1)
        {
            Image_Count ++;
        }
        else if(Image_Count_Flag == 0)
        {
            Image_Count = 0;
        }
        Image_Num ++;
        mt9v03x_finish_flag = 0;
    }
}

void Copy_Image(uint8(*source_image)[Image_X], uint8(*target_image)[Image_X])
{
    uint8 i = 0, j = 0;
    for(i = 0; i < Image_Y; i ++)
    {
        for(j = 0; j < Image_X; j++)
        {
            target_image[i][j] = source_image[i][j];
        }
    }
}

/**
* 函数功能：      32位整形变量取绝对值
* 特殊说明：      注意调用时参数类型，摄像头代码参数较多，不同类型错误传参可能导致计算结果出现较大问题，严重时卡死单片机运行
* 形  参：        value：       32位整型变量
* 示例：          My_ABS();
* 返回值：        传入参数的绝对值
*/
int My_ABS(int value)               //*
{
    if(value >= 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

/**
* 函数功能：      浮点型变量取绝对值
* 特殊说明：      注意调用时参数类型，摄像头代码参数较多，不同类型错误传参可能导致计算结果出现较大问题，严重时卡死单片机运行
* 形  参：        value：       浮点型变量
* 示例：          My_ABS_F();
* 返回值：        传入参数的绝对值
*/
float My_ABS_F(float value)               //*
{
    if(value >= 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

uint8 My_ABS_uint8(uint8 value)               //*
{
    return value; // uint8 无符号，不可能为负，直接返回（原 if(value>=0) 恒真，消除 Pe186 警告）
}
/**
* 函数功能：      32位整形变量限幅
* 特殊说明：      注意调用时参数类型，摄像头代码参数较多，不同类型错误传参可能导致计算结果出现较大问题，严重时卡死单片机运行
* 形  参：        x：       要限幅的参数
*                a：       最小值
*                b：       最大值
* 示例：          Limit(x， a， b);
* 返回值：        传入参数限幅后的值
*/
int Limit(int x, int a, int b)               //*
{
    if(x > b)
    {
        return b;
    }
    else if(x < a)
    {
        return a;
    }
    else
    {
        return x;
    }
}

//8位无符号整形变量限幅
//不多讲了，肯定能看懂
uint8 Limit_u8(uint8 x, uint8 a, uint8 b)               //*
{
    if(x > b)
    {
        return b;
    }
    else if(x < a)
    {
        return a;
    }
    else
    {
        return x;
    }
}
//16位有符号整形变量限幅
int16 Limit_16(int16 x, int16 a, int16 b)               //*
{
    if(x > b)
    {
        return b;
    }
    else if(x < a)
    {
        return a;
    }
    else
    {
        return x;
    }
}

//16位无符号整型变量限幅
uint16 Limit_u16(uint16 x, uint16 a, uint16 b)               //*
{
    if(x > b)
    {
        return b;
    }
    else if(x < a)
    {
        return a;
    }
    else
    {
        return x;
    }
}

//浮点型变量限幅
float Limit_Float(float x, float a, float b)               //*
{
    if(x > b)
    {
        return b;
    }
    else if(x < a)
    {
        return a;
    }
    else
    {
        return x;
    }
}

uint8 Compare_Value = 20;
//差比和   ****
int16 Compare_Num(int16 a, int16 b, uint8 compare_value)               //****
{
    if((((a - b) << 7) / (a + b)) > compare_value)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int8 KernelSize_3 = 3; // 高斯核大小，必须是奇数
int8 KernelSize_5 = 5;
float sigma = 1.0f; // 高斯函数的标准差
float Kernel_Core_3[9] = {0};
float Kernel_Core_5[25] = {0};
uint8 Vague_Image[Image_Y][Image_X];
/**
* 函数功能：      生成高斯核
* 特殊说明：      高斯核大小必须为奇数，常为 3 或 5
* 形  参：        float *kernel            存储高斯核的数组
*                uint8 kernelSize          高斯核大小
*                float sigma               高斯核的标准差，一般为1.0
*
* 示例：          Generate_GaussianKernel(Kernel_Core, KernelSize, 1.0f);
* 返回值：        无
*/

void Generate_GaussianKernel(float *kernel, uint8 kernelSize, float sigma)     //sigma为标准差 1.0           **
{
    int8 i, j;
    float sum = 0.0f;

    int8 Temp_Size = (int8)(kernelSize / 2);
    float Temp_Sigma = 2.0f * sigma * sigma;
    for (i = 0; i < kernelSize; i++)
    {
        for (j = 0; j < kernelSize; j++)
        {
            float g = exp(-(float)((i - Temp_Size) * (i - Temp_Size)) / Temp_Sigma - (float)((j - Temp_Size) * (j - Temp_Size)) / Temp_Sigma);
            kernel[i * kernelSize + j] = g;
            sum += g;
        }
    }

    // 归一化高斯核
    for (i = 0; i < kernelSize * kernelSize; i++)
    {
        kernel[i] /= sum;
    }
}

/**
* 函数功能：      对指定区域进行高斯模糊，配合爬线使用，即边爬线边高斯模糊
* 特殊说明：      1.需先调用“Generate_GaussianKernel”函数计算出高斯核后才能进行高斯模糊
*                2.通常用于光线变化不均匀区域的模糊处理和逆透视图像爬边线的模糊处理，可一定程度上抑制边缘锯齿和光线剧烈变化
*                3.需要高斯模糊处理时，先对图像进行复制，得到一样的新图像，在新图像进行爬线，同时根据爬线时每次的中心点由源图像计算出九个点的模糊值，再在新图像判断下一个中心点
* 形  参：        uint8(*source_image)[Image_X]            源图像
*                uint8(*target_image)[Image_X]             模糊结果输出图像，同时用于爬线
*                float kernel_core                         高斯核
*                int8 start_x                              要模糊区域的左上角第一个点x坐标
*                int8 start_y                              要模糊区域的左上角第一个点y坐标
*                int8 width                                要模糊区域的宽
*                int8 height                               要模糊区域的高
* 示例：          Gaussian_BlurRegion(Find_Line_Image, target_image, Kernel_Core, start_x, start_y, 3, 3)；
* 返回值：        无
*/
void Gaussian_BlurRegion(uint8(*source_image)[Image_X], uint8(*target_image)[Image_X], float *kernel_core, int8 start_x, int8 start_y, int8 width, int8 height, int8 kernel_size)          //**
{
    int8 i = 0, j = 0;
    int8 x = 0, y = 0;
    int8 Half_Kernel_Size = kernel_size / 2;

    // 对指定区域进行卷积操作
    for (i = start_y; i < start_y + height; i++)
    {
        for (j = start_x; j < start_x + width; j++)
        {
            float sum = 0;
            for (y = -Half_Kernel_Size; y <= Half_Kernel_Size; y++)
            {
                for (x = -Half_Kernel_Size; x <= Half_Kernel_Size; x++)
                {
                    int8 Temp_Count = (y + Half_Kernel_Size) * kernel_size + (x + Half_Kernel_Size);
                    sum += ((float)(source_image[i + y][j + x]) * kernel_core[Temp_Count]);
                }
            }
            target_image[i][j] = (uint8)Limit_Float(sum, 0.0f, 255.0f);
        }
    }
}

void Gaussian_Point_Vague_3(uint8(*source_image)[Image_X], uint8(*target_image)[Image_X], int8 Y, int8 X)
{
    int8 x = 0, y = 0;
    float sum = 0;
    for (y = -1; y <= 1; y++)
    {
        for (x = -1; x <= 1; x++)
        {
            int8 Temp_Count = (y + 1) * 3 + (x + 1);
            sum += ((float)(source_image[Y + y][X + x]) * Kernel_Core_3[Temp_Count]);
        }
    }
    target_image[Y][X] = (uint8)Limit_Float(sum, 0.0f, 255.0f);
}

void Gaussian_Point_Vague_5(uint8(*source_image)[Image_X], uint8(*target_image)[Image_X], int8 Y, int8 X)
{
    int8 x = 0, y = 0;
    float sum = 0;
    for (y = -2; y <= 2; y++)
    {
        for (x = -2; x <= 2; x++)
        {
            int8 Temp_Count = (y + 2) * 5 + (x + 2);
            sum += ((float)(source_image[Y + y][X + x]) * Kernel_Core_5[Temp_Count]);
        }
    }
    target_image[Y][X] = (uint8)Limit_Float(sum, 0.0f, 255.0f);;
}

void Gaussian_Vague_Optimize_3(uint8(*source_image)[Image_X], uint8(*target_image)[Image_X], int8 Direction, int8 X, int8 Y)
{
    switch(Direction)
    {
        case 0:
        {
            target_image[Y - 2][X - 2] = source_image[Y - 2][X - 2];    target_image[Y - 2][X - 1] = source_image[Y - 2][X - 1];    target_image[Y - 2][X    ] = source_image[Y - 2][X    ];
            target_image[Y - 2][X + 1] = source_image[Y - 2][X + 1];    target_image[Y - 2][X + 2] = source_image[Y - 2][X + 2];    target_image[Y - 1][X - 2] = source_image[Y - 1][X - 2];
            target_image[Y - 1][X + 2] = source_image[Y - 1][X + 2];    target_image[Y    ][X - 2] = source_image[Y    ][X - 2];    target_image[Y    ][X + 2] = source_image[Y    ][X + 2];
            target_image[Y + 1][X - 2] = source_image[Y + 1][X - 2];    target_image[Y + 1][X + 2] = source_image[Y + 1][X + 2];    target_image[Y + 2][X - 2] = source_image[Y + 2][X - 2];
            target_image[Y + 2][X - 1] = source_image[Y + 2][X - 1];    target_image[Y + 2][X    ] = source_image[Y + 2][X    ];    target_image[Y + 2][X + 1] = source_image[Y + 2][X + 1];
            target_image[Y + 2][X + 2] = source_image[Y + 2][X + 2];

            Gaussian_Point_Vague_3(source_image, target_image, Y - 1, X - 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y - 1, X    );
            Gaussian_Point_Vague_3(source_image, target_image, Y - 1, X + 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y    , X - 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y    , X    );
            Gaussian_Point_Vague_3(source_image, target_image, Y    , X + 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y + 1, X - 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y + 1, X    );
            Gaussian_Point_Vague_3(source_image, target_image, Y + 1, X + 1);
            break;
        }
        case -2:
        {
            target_image[Y - 2][X - 2] = source_image[Y - 2][X - 2];    target_image[Y - 2][X - 1] = source_image[Y - 2][X - 1];    target_image[Y - 2][X    ] = source_image[Y - 2][X    ];
            target_image[Y - 2][X + 1] = source_image[Y - 2][X + 1];    target_image[Y - 2][X + 2] = source_image[Y - 2][X + 2];    target_image[Y - 1][X - 2] = source_image[Y - 1][X - 2];
            target_image[Y    ][X - 2] = source_image[Y    ][X - 2];    target_image[Y + 1][X - 2] = source_image[Y + 1][X - 2];    target_image[Y + 2][X - 2] = source_image[Y + 2][X - 2];

            Gaussian_Point_Vague_3(source_image, target_image, Y - 1, X - 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y - 1, X    );
            Gaussian_Point_Vague_3(source_image, target_image, Y - 1, X + 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y    , X - 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y + 1, X - 1);
            break;
        }
        case -3:
        {
            target_image[Y - 2][X - 2] = source_image[Y - 2][X - 2];    target_image[Y - 1][X - 2] = source_image[Y - 1][X - 2];    target_image[Y    ][X - 2] = source_image[Y    ][X - 2];
            target_image[Y + 1][X - 2] = source_image[Y + 1][X - 2];    target_image[Y + 2][X - 2] = source_image[Y + 2][X - 2];

            Gaussian_Point_Vague_3(source_image, target_image, Y - 1, X - 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y    , X - 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y + 1, X - 1);
            break;
        }
        case -4:
        {
            target_image[Y - 2][X - 2] = source_image[Y - 2][X - 2];    target_image[Y - 1][X - 2] = source_image[Y - 1][X - 2];    target_image[Y    ][X - 2] = source_image[Y    ][X - 2];
            target_image[Y + 1][X - 2] = source_image[Y + 1][X - 2];    target_image[Y + 2][X - 2] = source_image[Y + 2][X - 2];    target_image[Y + 2][X - 1] = source_image[Y + 2][X - 1];
            target_image[Y + 2][X    ] = source_image[Y + 2][X    ];    target_image[Y + 2][X + 1] = source_image[Y + 2][X + 1];    target_image[Y + 2][X + 2] = source_image[Y + 2][X + 2];

            Gaussian_Point_Vague_3(source_image, target_image, Y - 1, X - 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y    , X - 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y + 1, X - 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y + 1, X    );
            Gaussian_Point_Vague_3(source_image, target_image, Y + 1, X + 1);
            break;
        }
        case -1:
        {
            target_image[Y + 2][X - 2] = source_image[Y + 2][X - 2];    target_image[Y + 2][X - 1] = source_image[Y + 2][X - 1];    target_image[Y + 2][X    ] = source_image[Y + 2][X    ];
            target_image[Y + 2][X + 1] = source_image[Y + 2][X + 1];    target_image[Y + 2][X + 2] = source_image[Y + 2][X + 2];

            Gaussian_Point_Vague_3(source_image, target_image, Y + 1, X - 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y + 1, X    );
            Gaussian_Point_Vague_3(source_image, target_image, Y + 1, X + 1);
            break;
        }
        case 2:
        {
            target_image[Y - 2][X + 2] = source_image[Y - 2][X + 2];    target_image[Y - 1][X + 2] = source_image[Y - 1][X + 2];    target_image[Y    ][X + 2] = source_image[Y    ][X + 2];
            target_image[Y + 1][X + 2] = source_image[Y + 1][X + 2];    target_image[Y + 2][X - 2] = source_image[Y + 2][X - 2];    target_image[Y + 2][X - 1] = source_image[Y + 2][X - 1];
            target_image[Y + 2][X    ] = source_image[Y + 2][X    ];    target_image[Y + 2][X + 1] = source_image[Y + 2][X + 1];    target_image[Y + 2][X + 2] = source_image[Y + 2][X + 2];

            Gaussian_Point_Vague_3(source_image, target_image, Y - 1, X + 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y    , X + 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y + 1, X - 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y + 1, X    );
            Gaussian_Point_Vague_3(source_image, target_image, Y + 1, X + 1);
            break;
        }
        case 3:
        {
            target_image[Y - 2][X + 2] = source_image[Y - 2][X + 2];    target_image[Y - 1][X + 2] = source_image[Y - 1][X + 2];    target_image[Y    ][X + 2] = source_image[Y    ][X + 2];
            target_image[Y + 1][X + 2] = source_image[Y + 1][X + 2];    target_image[Y + 2][X + 2] = source_image[Y + 2][X + 2];

            Gaussian_Point_Vague_3(source_image, target_image, Y - 1, X + 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y    , X + 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y + 1, X + 1);
            break;
        }
        case 4:
        {
            target_image[Y - 2][X - 2] = source_image[Y - 2][X - 2];    target_image[Y - 2][X - 1] = source_image[Y - 2][X - 1];    target_image[Y - 2][X    ] = source_image[Y - 2][X    ];
            target_image[Y - 2][X + 1] = source_image[Y - 2][X + 1];    target_image[Y - 2][X + 2] = source_image[Y - 2][X + 2];    target_image[Y - 1][X + 2] = source_image[Y - 1][X + 2];
            target_image[Y    ][X + 2] = source_image[Y    ][X + 2];    target_image[Y + 1][X + 2] = source_image[Y + 1][X + 2];    target_image[Y + 2][X + 2] = source_image[Y + 2][X + 2];

            Gaussian_Point_Vague_3(source_image, target_image, Y - 1, X - 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y - 1, X    );
            Gaussian_Point_Vague_3(source_image, target_image, Y - 1, X + 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y    , X + 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y + 1, X + 1);
            break;
        }
        case 1:
        {
            target_image[Y - 2][X - 2] = source_image[Y - 2][X - 2];    target_image[Y - 2][X - 1] = source_image[Y - 2][X - 1];    target_image[Y - 2][X    ] = source_image[Y - 2][X    ];
            target_image[Y - 2][X + 1] = source_image[Y - 2][X + 1];    target_image[Y - 2][X + 2] = source_image[Y - 2][X + 2];

            Gaussian_Point_Vague_3(source_image, target_image, Y - 1, X - 1);
            Gaussian_Point_Vague_3(source_image, target_image, Y - 1, X    );
            Gaussian_Point_Vague_3(source_image, target_image, Y - 1, X + 1);
            break;
        }
    }
}
void Gaussian_Vague_Optimize_5(uint8(*source_image)[Image_X], uint8(*target_image)[Image_X], int8 Direction, int8 X, int8 Y)
{
    switch(Direction)
    {
        case 0:
        {
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X - 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X    );
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X + 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X + 2);

            Gaussian_Point_Vague_5(source_image, target_image, Y - 1, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 1, X - 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 1, X    );
            Gaussian_Point_Vague_5(source_image, target_image, Y - 1, X + 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 1, X + 2);

            Gaussian_Point_Vague_5(source_image, target_image, Y    , X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y    , X - 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y    , X    );
            Gaussian_Point_Vague_5(source_image, target_image, Y    , X + 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y    , X + 2);

            Gaussian_Point_Vague_5(source_image, target_image, Y + 1, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 1, X - 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 1, X    );
            Gaussian_Point_Vague_5(source_image, target_image, Y + 1, X + 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 1, X + 2);

            target_image[Y + 2][X - 2] = source_image[Y + 2][X - 2];    target_image[Y + 2][X - 1] = source_image[Y + 2][X - 1];    target_image[Y + 2][X    ] = source_image[Y + 2][X    ];
            target_image[Y + 2][X + 1] = source_image[Y + 2][X + 1];    target_image[Y + 2][X + 2] = source_image[Y + 2][X + 2];

            break;
        }
        case -2:
        {
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X - 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X    );
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X + 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X + 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 1, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y    , X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 1, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X - 2);
            break;
        }
        case -3:
        {
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 1, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y    , X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 1, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X - 2);
            break;
        }
        case -4:
        {
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 1, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y    , X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 1, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X - 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X    );
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X + 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X + 2);
            break;
        }
        case -1:
        {
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X - 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X    );
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X + 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X + 2);
            break;
        }
        case 2:
        {
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X + 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 1, X + 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y    , X + 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 1, X + 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X - 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X    );
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X + 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X + 2);
            break;
        }
        case 3:
        {
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X + 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 1, X + 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y    , X + 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 1, X + 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X + 2);
            break;
        }
        case 4:
        {
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X - 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X    );
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X + 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X + 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 1, X + 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y    , X + 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 1, X + 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y + 2, X + 2);
            break;
        }
        case 1:
        {
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X - 2);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X - 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X    );
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X + 1);
            Gaussian_Point_Vague_5(source_image, target_image, Y - 2, X + 2);
            break;
        }
    }
}

void Gaussian_Vague_Process(uint8 X, uint8 Y, uint16 Temp_Num, int8 Direction, uint8 L_or_R)
{
    int a = 0, b = 0;

    switch(Vague_Flag)
    {
        case 3:
        {
            if(X >= 3 && X <= 76 && Y >= 3 && Y <= 57)
            {
                if(L_or_R == 0)
                {
                    Gaussian_Vague_Optimize_3(Find_Line_Image, Vague_Image, Direction, X - 1, Y);
                }
                else if(L_or_R == 1)
                {
                    Gaussian_Vague_Optimize_3(Find_Line_Image, Vague_Image, Direction, X + 1, Y);
                }
            }
            else
            {
                for(a = -2; a <= 2; a++)
                {
                    for(b = -2; b <= 2; b++)
                    {
                        Vague_Image[(uint8)((int8)Y + a)][(uint8)((int8)X + b)] = Find_Line_Image[(uint8)((int8)Y + a)][(uint8)((int8)X + b)];
                    }
                }
            }
            break;
        }
        case 5:
        {
            if(X >= 3 && X <= 76 && Y >= 3 && Y <= 56)
            {
                Gaussian_Vague_Optimize_5(Find_Line_Image, Vague_Image, Direction, X, Y);
            }
            else
            {
                for(a = -2; a <= 2; a++)
                {
                    for(b = -2; b <= 2; b++)
                    {
                        Vague_Image[(uint8)((int8)Y + a)][(uint8)((int8)X + b)] = Find_Line_Image[(uint8)((int8)Y + a)][(uint8)((int8)X + b)];
                    }
                }
            }
            break;
        }
        case 33:
        {
//            if(X >= (L_No_Image_Border[Y] + 2) && X <= (R_No_Image_Border[Y] - 2))
//            {
//                if(L_or_R == 0)
//                {
//                    Gaussian_Vague_Optimize_3(I_Perspective_Image, I_Vague_Image, Direction, X - 1, Y);
//                }
//                else if(L_or_R == 1)
//                {
//                    Gaussian_Vague_Optimize_3(I_Perspective_Image, I_Vague_Image, Direction, X + 1, Y);
//                }
//            }
//            else
//            {
//                for(a = -2; a <= 2; a++)
//                {
//                    for(b = -2; b <= 2; b++)
//                    {
//                        I_Vague_Image[(uint8)((int8)Y + a)][(uint8)((int8)X + b)] = I_Perspective_Image[(uint8)((int8)Y + a)][(uint8)((int8)X + b)];
//                    }
//                }
//            }
            break;
        }
        case 55:
        {
//            if(X >= (L_No_Image_Border[Y] + 2) && X <= (R_No_Image_Border[Y] - 2))
//            {
//                Gaussian_Vague_Optimize_5(I_Perspective_Image, I_Vague_Image, Direction, X, Y);
//            }
//            else
//            {
//                for(a = -2; a <= 2; a++)
//                {
//                    for(b = -2; b <= 2; b++)
//                    {
//                        I_Vague_Image[(uint8)((int8)Y + a)][(uint8)((int8)X + b)] = I_Perspective_Image[(uint8)((int8)Y + a)][(uint8)((int8)X + b)];
//                    }
//                }
//            }
            break;
        }
        default:
        {
            break;
        }
    }
}
uint8 Black_Box_Value_FFF = 50;
uint8 Black_Box_Value_FF = 50;
uint8 Black_Box_Value_F = 50;
//画黑框（必须为一个像素宽度，边界务必空出一格）
void Draw_Black_Box(uint8 black_box_value, uint8(*image)[Image_X])          //*****
{
    uint8 i,j;

    Black_Box_Value_FFF = Black_Box_Value_FF;
    Black_Box_Value_FF = Black_Box_Value_F;
    Black_Box_Value_F = black_box_value;
    black_box_value = (uint8)(0.5f * Black_Box_Value_F + 0.3f * Black_Box_Value_FF + 0.2f * Black_Box_Value_FFF);
    Black_Box_Value = black_box_value;
    for(i = 1; i < 60; i++)
    {
        image[i][Image_X - 2] = black_box_value;
        image[i][1] = black_box_value;
    }
    for(j = 1; j < Image_X - 2; j++)
    {
        image[1][j] = black_box_value;
    }
}

//找起点
uint8 Start_Flag = 0;
uint8 Black_Box_Value_1 = 0;
uint8 Get_Start_Point(uint8 start_row, uint8(*image)[Image_X], uint8 *l_start_point, uint8 *r_start_point, uint8 l_border_x, uint8 r_border_x)          //*****
{
    uint8 i = 0, j = 0;
    uint8 L_Is_Found = 0, R_Is_Found = 0;
    uint8 Start_X  = 0;
    uint8 Start_Row_0 = 0;

    Start_Row_0 = start_row;
    Start_X = Image_X / 2;
    //从中间往左边，先找起点
    for(j = 0; j < 10; j ++)
    {
        l_start_point[1] = start_row;//y
        r_start_point[1] = start_row;//y

        if(Start_Flag == 0 || Element_State == Zebra)
        {
            Start_X = Image_X / 2;
        }
        else
        {
            Start_X = (l_start_point[0] + r_start_point[0]) / 2;
        }

        {
            for (i = Start_X; i > l_border_x - 1; i--)
            {
                if (Compare_Num(image[start_row][i + 5], image[start_row][i], Compare_Value))//差比和为真
                {
                    {
                        l_start_point[0] = i;
                        L_Is_Found = 1;
                        break;
                    }
                }
            }

            for (i = Start_X; i < r_border_x + 1; i++)
            {
                if (Compare_Num(image[start_row][i - 5], image[start_row][i], Compare_Value))//差比和为真
                {
                    {
                        r_start_point[0] = i;
                        R_Is_Found = 1;
                        break;
                    }
                }
            }
            if(L_Is_Found && R_Is_Found)
            {
                Start_Flag = 1;
                if(l_start_point[0] != l_border_x && r_start_point[0] != r_border_x)
                {
                    Black_Box_Value_1 = Zbra_Thres - 40;
                    Black_Box_Value_1 = Limit_u8(Black_Box_Value_1, 0, 255);
                }
                else if(l_start_point[0] != l_border_x && r_start_point[0] == r_border_x)
                {
                    Black_Box_Value_1 = image[start_row][l_start_point[0]] - 20;
                    Black_Box_Value_1 = Limit_u8(Black_Box_Value_1, 0, 255);
                }
                else if(l_start_point[0] == l_border_x && r_start_point[0] != r_border_x)
                {
                    Black_Box_Value_1 = image[start_row][r_start_point[0]] - 20;
                    Black_Box_Value_1 = Limit_u8(Black_Box_Value_1, 0, 255);
                }
                else
                {
                    Black_Box_Value_1 = 20;
                }

                return 1;
            }
            else
            {
                start_row = start_row - 1;
            }
        }
    }

    Black_Box_Value = 0;
    Draw_Black_Box(Black_Box_Value, image);
    start_row = Start_Row_0;
    for (i = Start_X; i > l_border_x - 1; i--)
    {
        if (Compare_Num(image[start_row][i + 5], image[start_row][i], Compare_Value))//差比和为真
        {
            l_start_point[0] = i;
            L_Is_Found = 1;
            break;
        }
    }

    for (i = Start_X; i < r_border_x + 1; i++)
    {
        if (Compare_Num(image[start_row][i - 5], image[start_row][i], Compare_Value))//差比和为真
        {
            r_start_point[0] = i;
            R_Is_Found = 1;
            break;
        }
    }
    if(L_Is_Found && R_Is_Found)
    {
        Start_Flag = 1;
        return 1;
    }
    else
    {
        return 0;
    }
}
/*
//均值阈值二值化
void Average_Value_Binarization(uint8(*image)[Image_X], uint8 Thres)
{
    uint8 i = 0, j = 0;
    for(i = 0; i < Image_Y; i++)
    {
        for(j = 0; j < Image_X; j++)
        {
            if(image[i][j] <= Thres)
            {
                image[i][j] = Black;
            }
            else
            {
                image[i][j] = Write;
            }
        }
    }
}
*/
//二值化图像补黑框
void Binarization_Draw_Black_Box(uint8(*image)[Image_X])          //*****
{
    uint8 i,j;

    for(i = 1; i < 60; i++)
    {
        image[i][Image_X - 2] = 0;
        image[i][1] = 0;
    }
    for(j = 1; j < Image_X - 2; j++)
    {
        Find_Line_Image[1][j] = 0;
    }
}
//二值化找起点
uint8 Binarization_Get_Start_Point(uint8 start_row, uint8(*image)[Image_X], uint8 start_x, uint8 *l_start_point, uint8 *r_start_point, uint8 l_border_x, uint8 r_border_x, uint8 Thres)
{
    uint8 i = 0;
    uint8 L_Is_Found = 0, R_Is_Found = 0;

    for(i = start_x; i > l_border_x; i --)
    {
        if(image[start_row][i] < Thres)
        {
            l_start_point[0] = i;
            l_start_point[1] = start_row;
            L_Is_Found = 1;
            break;
        }
    }

    for(i = start_x; i < r_border_x; i ++)
    {
        if(image[start_row][i] < Thres)
        {
            r_start_point[0] = i;
            r_start_point[1] = start_row;
            R_Is_Found = 1;
            break;
        }
    }

    if(L_Is_Found && R_Is_Found)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
//**************方向计算****************
//记录方向时，要将每个坐标变为单一数字，便于简化后续算法。例如八邻域的方向（逆时针    顺时针）
//3  2  1               1  2  3
//4     0               0     4
//5  6  7               7  6  5
/////////////////////////////////
//{-1,-1},{0,-1},{+1,-1},
//{-1, 0},       {+1, 0},
//{-1,+1},{0,+1},{+1,+1},
//迷宫缺点是无法像八邻域一样逐步记录方向，但八邻域无非就是以上八个点，且与中心点差值固定，不会随迷宫算法的朝向和移动方向而改变，因此先将每个横坐标乘 3
//{-3,-1},{0,-1},{+3,-1},
//{-3, 0},       {+3, 0},
//{-3,+1},{0,+1},{+3,+1},
//再将横坐标减去纵坐标
// -2, 1, 4
// -3,    3
// -4,-1, 2
//由此可以得到一个八邻方向坐标。只需在每次移动后，在方向数组里记录对应数字，就可确定生长方向
//此算法无任何原理，只是为了得到八个不一样的值可以用来判定方向，横坐标可以乘大于 2 的任意值，2以内会出现重复
//************************************
//******************自适应方向迷宫参数************************

const int8 L_Face_Dir[4][2] = {{0,-1},{1,0},{0,1},{-1,0}};  //左侧迷宫面向
//  0
//3   1
//  2

const int8 L_Face_Dir_L[4][2] = {{-1,-1},{1,-1},{1,1},{-1,1}};  //左侧面向的左前方
//0   1
//
//3   2

const int8 R_Face_Dir[4][2] = {{0,-1},{1,0},{0,1},{-1,0}};  //右侧迷宫面向
//  0
//3   1
//  2

const int8 R_Face_Dir_R[4][2] = {{1,-1},{1,1},{-1,1},{-1,-1}};  //右侧面向的右前方
//3   0
//
//2   1

const int8 Square_0[25][2] = {              //一个5 * 5的矩阵，用来求中心点周围的局部阈值（局部阈值）
{-2,-2},{-1,-2},{0,-2},{+1,-2},{+2,-2},
{-2,-1},{-1,-1},{0,-1},{+1,-1},{+2,-1},
{-2,-0},{-1, 0},{0, 0},{+1, 0},{+2,-0},
{-2,+1},{-1,+1},{0,+1},{+1,+1},{+2,+1},
{-2,+2},{-1,+2},{0,+2},{+1,+2},{+2,+2}
};

//迷宫单侧停止爬线标志位
uint8 L_Stop_Flag = 0;
uint8 R_Stop_Flag = 0;
//**********************************************************
//自适应方向迷宫巡线（5 * 5阈值）    Vague_Image
void Dir_Labyrinth_5(uint16 Break_Flag, uint8(*image)[Image_X], uint8(*l_line)[2], uint8(*r_line)[2], int8 *l_dir, int8 *r_dir, uint16 *l_stastic, uint16 *r_stastic, uint8 *x_meet, uint8 *y_meet,
                     uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8 clip_value)
{
    uint8 j = 0;

    L_Stop_Flag = 0;
    R_Stop_Flag = 0;
//左边变量
    uint8  L_Center_Point[2] = {0};     //存放每次找到的XY坐标
    uint16 L_Data_Statics = 0;          //统计左边找到的边线点的个数

    uint8  L_Front_Value = 0;           //左侧 面向的前方点的灰度值
    uint8  L_Front_L_Value = 0;         //左侧 面向的左前方点的灰度值

    uint8  L_Dir = 0;                   //此参数用于转向
    uint8  L_Turn_Num = 0;              //记录转向次数，若中心点前后左右都是黑色像素，就会在一个点转向四次，记录到四次时退出循环防止卡死程序
    uint16 L_Pixel_Value_Sum = 0;       //中心点与周围24个点的像素值和
    float L_Thres = 0;                 //局部阈值,即L_Pixel_Value_Sum / 25

//右边变量
    uint8  R_Center_Point[2] = {0};     //存放每次找到的XY坐标
    uint16 R_Data_Statics = 0;          //统计右边找到的边线点的个数

    uint8  R_Front_Value = 0;           //右侧 面向的前方点的灰度值
    uint8  R_Front_R_Value = 0;         //右侧 面向的左前方点的灰度值

    uint8  R_Dir = 0;                   //此参数用于转向
    uint8  R_Turn_Num = 0;              //记录转向次数，若中心点前后左右都是黑色像素，就会在一个点转向四次，记录到四次时退出循环防止卡死程序
    uint16 R_Pixel_Value_Sum = 0;       //中心点与周围24个点的像素值和
    float R_Thres = 0;                 //局部阈值

//第一次更新坐标点  将找到的起点值传进来
    L_Center_Point[0] = l_start_x + 1;//x
    L_Center_Point[1] = l_start_y;//y
    R_Center_Point[0] = r_start_x - 1;//x
    R_Center_Point[1] = r_start_y;//y

    //开启方向迷宫循环
    while (Break_Flag--)
    {
         //左边
//        L_Search_Again:
        if(L_Stop_Flag == 0)
        {
            l_line[L_Data_Statics][0] = L_Center_Point[0];  //找到的中心点X坐标计入左边线数组
            l_line[L_Data_Statics][1] = L_Center_Point[1];  //找到的中心点Y坐标计入左边线数组

            if(L_Data_Statics != 0)
            {
                switch(Vague_Flag)
                {
                    case 3:
                    {
                        Gaussian_Vague_Process(L_Center_Point[0], L_Center_Point[1], L_Data_Statics, l_dir[L_Data_Statics - 1], 0);
                        break;
                    }
                    case 5:
                    {
                        Gaussian_Vague_Process(L_Center_Point[0], L_Center_Point[1], L_Data_Statics, l_dir[L_Data_Statics - 1], 0);
                        break;
                    }
                    case 33:
                    {
                        Gaussian_Vague_Process(L_Center_Point[0], L_Center_Point[1], L_Data_Statics, l_dir[L_Data_Statics - 1], 0);
                        break;
                    }
                    case 55:
                    {
                        Gaussian_Vague_Process(L_Center_Point[0], L_Center_Point[1], L_Data_Statics, l_dir[L_Data_Statics - 1], 0);
                        break;
                    }
                }
                switch(l_dir[L_Data_Statics - 1])  //下面这一坨可以根据上一个点的生长方向大幅优化爬线时间
                {
                    case 1:
                    {
                        L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] + 3][L_Center_Point[0] + 2] - image[L_Center_Point[1] + 3][L_Center_Point[0] + 1]
                                                              - image[L_Center_Point[1] + 3][L_Center_Point[0] + 0] - image[L_Center_Point[1] + 3][L_Center_Point[0] - 1]
                                                              - image[L_Center_Point[1] + 3][L_Center_Point[0] - 2]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] - 2][L_Center_Point[0] + 1]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] + 0] + image[L_Center_Point[1] - 2][L_Center_Point[0] - 1]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] - 2];
                        break;
                    }
                    case -2:
                    {
                        L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] - 1][L_Center_Point[0] + 3] - image[L_Center_Point[1] - 0][L_Center_Point[0] + 3]
                                                              - image[L_Center_Point[1] + 1][L_Center_Point[0] + 3] - image[L_Center_Point[1] + 2][L_Center_Point[0] + 3]
                                                              - image[L_Center_Point[1] + 3][L_Center_Point[0] + 3] - image[L_Center_Point[1] + 3][L_Center_Point[0] + 2]
                                                              - image[L_Center_Point[1] + 3][L_Center_Point[0] + 1] - image[L_Center_Point[1] + 3][L_Center_Point[0] - 0]
                                                              - image[L_Center_Point[1] + 3][L_Center_Point[0] - 1]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] - 2]
                                                              + image[L_Center_Point[1] + 0][L_Center_Point[0] - 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] - 2]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] - 2][L_Center_Point[0] - 1]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] - 0] + image[L_Center_Point[1] - 2][L_Center_Point[0] + 1]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] + 2];
                        break;
                    }
                    case -3:
                    {
                        L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] - 2][L_Center_Point[0] + 3] - image[L_Center_Point[1] - 1][L_Center_Point[0] + 3]
                                                              - image[L_Center_Point[1] + 0][L_Center_Point[0] + 3] - image[L_Center_Point[1] + 1][L_Center_Point[0] + 3]
                                                              - image[L_Center_Point[1] + 2][L_Center_Point[0] + 3]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] - 2]
                                                              + image[L_Center_Point[1] - 0][L_Center_Point[0] - 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] - 2]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] - 2];
                        break;
                    }
                    case -4:
                    {
                        L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] - 3][L_Center_Point[0] - 1] - image[L_Center_Point[1] - 3][L_Center_Point[0] + 0]
                                                              - image[L_Center_Point[1] - 3][L_Center_Point[0] + 1] - image[L_Center_Point[1] - 3][L_Center_Point[0] + 2]
                                                              - image[L_Center_Point[1] - 3][L_Center_Point[0] + 3] - image[L_Center_Point[1] - 2][L_Center_Point[0] + 3]
                                                              - image[L_Center_Point[1] - 1][L_Center_Point[0] + 3] - image[L_Center_Point[1] + 0][L_Center_Point[0] + 3]
                                                              - image[L_Center_Point[1] + 1][L_Center_Point[0] + 3]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] - 2]
                                                              + image[L_Center_Point[1] + 0][L_Center_Point[0] - 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] - 2]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] + 2][L_Center_Point[0] - 1]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] - 0] + image[L_Center_Point[1] + 2][L_Center_Point[0] + 1]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] + 2];
                        break;
                    }
                    case -1:
                    {
                        L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] - 3][L_Center_Point[0] - 2] - image[L_Center_Point[1] - 3][L_Center_Point[0] - 1]
                                                              - image[L_Center_Point[1] - 3][L_Center_Point[0] + 0] - image[L_Center_Point[1] - 3][L_Center_Point[0] + 1]
                                                              - image[L_Center_Point[1] - 3][L_Center_Point[0] + 2]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] - 2] + image[L_Center_Point[1] + 2][L_Center_Point[0] - 1]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] + 0] + image[L_Center_Point[1] + 2][L_Center_Point[0] + 1]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] + 2];
                        break;
                    }
                    case 2:
                    {
                        L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] + 1][L_Center_Point[0] - 3] - image[L_Center_Point[1] + 0][L_Center_Point[0] - 3]
                                                              - image[L_Center_Point[1] - 1][L_Center_Point[0] - 3] - image[L_Center_Point[1] - 2][L_Center_Point[0] - 3]
                                                              - image[L_Center_Point[1] - 3][L_Center_Point[0] - 3] - image[L_Center_Point[1] - 3][L_Center_Point[0] - 2]
                                                              - image[L_Center_Point[1] - 3][L_Center_Point[0] - 1] - image[L_Center_Point[1] - 3][L_Center_Point[0] + 0]
                                                              - image[L_Center_Point[1] - 3][L_Center_Point[0] + 1]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] + 2]
                                                              + image[L_Center_Point[1] - 0][L_Center_Point[0] + 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] + 2]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] + 2][L_Center_Point[0] + 1]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] + 0] + image[L_Center_Point[1] + 2][L_Center_Point[0] - 1]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] - 2];
                        break;
                    }
                    case 3:
                    {
                        L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] + 2][L_Center_Point[0] - 3] - image[L_Center_Point[1] + 1][L_Center_Point[0] - 3]
                                                              - image[L_Center_Point[1] - 0][L_Center_Point[0] - 3] - image[L_Center_Point[1] - 1][L_Center_Point[0] - 3]
                                                              - image[L_Center_Point[1] - 2][L_Center_Point[0] - 3]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] + 2]
                                                              + image[L_Center_Point[1] + 0][L_Center_Point[0] + 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] + 2]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] + 2];
                        break;
                    }
                    case 4:
                    {
                        L_Pixel_Value_Sum = L_Pixel_Value_Sum - image[L_Center_Point[1] + 3][L_Center_Point[0] + 1] - image[L_Center_Point[1] + 3][L_Center_Point[0] - 0]
                                                              - image[L_Center_Point[1] + 3][L_Center_Point[0] - 1] - image[L_Center_Point[1] + 3][L_Center_Point[0] - 2]
                                                              - image[L_Center_Point[1] + 3][L_Center_Point[0] - 3] - image[L_Center_Point[1] + 2][L_Center_Point[0] - 3]
                                                              - image[L_Center_Point[1] + 1][L_Center_Point[0] - 3] - image[L_Center_Point[1] - 0][L_Center_Point[0] - 3]
                                                              - image[L_Center_Point[1] - 1][L_Center_Point[0] - 3]
                                                              + image[L_Center_Point[1] + 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] + 1][L_Center_Point[0] + 2]
                                                              + image[L_Center_Point[1] - 0][L_Center_Point[0] + 2] + image[L_Center_Point[1] - 1][L_Center_Point[0] + 2]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] + 2] + image[L_Center_Point[1] - 2][L_Center_Point[0] + 1]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] + 0] + image[L_Center_Point[1] - 2][L_Center_Point[0] - 1]
                                                              + image[L_Center_Point[1] - 2][L_Center_Point[0] - 2];
                        break;
                    }
                }
            }
            else
            {
                switch(Vague_Flag)
                {
                    case 3:
                    {
                        Gaussian_Vague_Process(L_Center_Point[0], L_Center_Point[1], L_Data_Statics, 0, 0);
                        break;
                    }
                    case 5:
                    {
                        Gaussian_Vague_Process(L_Center_Point[0], L_Center_Point[1], L_Data_Statics, 0, 0);
                        break;
                    }
                    case 33:
                    {
                        Gaussian_Vague_Process(L_Center_Point[0], L_Center_Point[1], L_Data_Statics, 0, 0);
                        break;
                    }
                    case 55:
                    {
                        Gaussian_Vague_Process(L_Center_Point[0], L_Center_Point[1], L_Data_Statics, 0, 0);
                        break;
                    }
                }
                for (j = 0; j < 25; j++)    //第一个阈值将25个点全部加一遍，后续点的阈值根据生长方向计算
                {
                    L_Pixel_Value_Sum += image[L_Center_Point[1] + Square_0[j][1]][L_Center_Point[0] + Square_0[j][0]];
                }
            }

            L_Thres = (L_Pixel_Value_Sum + Thres_Interfere) / Thres_Num_Interfere;   //阈值为25个点灰度值的平均值
            L_Thres -= clip_value;              //将得到的灰度阈值减去一个经验值，用来优化判定

            if(Thres_Filiter_Flag_1 == 1 || Thres_Filiter_Flag_2 == 1)
            {
                if(L_Data_Statics > 3)
                {
                    L_Thres = L_Thres * 1.3f - L_Thres_Record[L_Data_Statics - 1] * 0.2f - L_Thres_Record[L_Data_Statics - 2] * 0.1f;
                }
            }
            L_Thres_Record[L_Data_Statics] = L_Thres;
            L_Data_Statics++;                   //每找到一个点统计个数+1

            L_Judge_Again:    //L_Judge_Again 与 goto 配合使用
            if(L_Stop_Flag == 0)
            {
                L_Front_Value = image[L_Center_Point[1] + L_Face_Dir[L_Dir][1]][L_Center_Point[0] + L_Face_Dir[L_Dir][0]];          //记录面向的前方点的灰度值
                L_Front_L_Value = image[L_Center_Point[1] + L_Face_Dir_L[L_Dir][1]][L_Center_Point[0] + L_Face_Dir_L[L_Dir][0]];    //记录面向的左前方点的灰度值
                if((float)L_Front_Value < L_Thres)     //面向的前方点是黑色
                {
                    L_Dir = (L_Dir + 1) % 4;    //需右转一次
                    L_Turn_Num ++;
                    if(L_Turn_Num == 4)        //死区处理
                    {
                        L_Stop_Flag = 1;       //当前后左右都是黑色时，进入死区，停止左侧爬线
                    }
                    goto L_Judge_Again;
                }
                else if((float)L_Front_L_Value < L_Thres)   //左前方点是黑色，前方点是白色
                {
                    L_Center_Point[0] += L_Face_Dir[L_Dir][0];
                    L_Center_Point[1] += L_Face_Dir[L_Dir][1];      //向前走一步
                    l_dir[L_Data_Statics - 1] = (L_Face_Dir[L_Dir][0] * 3) - L_Face_Dir[L_Dir][1];
                    L_Turn_Num = 0;
                }
                else        //左前方和前方都是白色点
                {
                    L_Center_Point[0] += L_Face_Dir_L[L_Dir][0];
                    L_Center_Point[1] += L_Face_Dir_L[L_Dir][1];        //向左前方走一步
                    l_dir[L_Data_Statics - 1] = (L_Face_Dir_L[L_Dir][0] * 3) - L_Face_Dir_L[L_Dir][1];
                    L_Dir = (L_Dir + 3) % 4;        //左转一次
                    L_Turn_Num = 0;
                }
                if(L_Data_Statics >= 4)     //O环处理
                {
                    if(l_line[L_Data_Statics][0] == l_line[L_Data_Statics - 4][0]&&
                       l_line[L_Data_Statics][1] == l_line[L_Data_Statics - 4][1])
                    {
                        L_Stop_Flag = 1;
                    }
                }
            }
        }

        if(R_Stop_Flag == 0)
        {
            r_line[R_Data_Statics][0] = R_Center_Point[0];
            r_line[R_Data_Statics][1] = R_Center_Point[1];

            if(R_Data_Statics != 0)
            {
                switch(Vague_Flag)
                {
                    case 3:
                    {
                        Gaussian_Vague_Process(R_Center_Point[0], R_Center_Point[1], R_Data_Statics, r_dir[R_Data_Statics - 1], 1);
                        break;
                    }
                    case 5:
                    {
                        Gaussian_Vague_Process(R_Center_Point[0], R_Center_Point[1], R_Data_Statics, r_dir[R_Data_Statics - 1], 1);
                        break;
                    }
                    case 33:
                    {
                        Gaussian_Vague_Process(R_Center_Point[0], R_Center_Point[1], R_Data_Statics, r_dir[R_Data_Statics - 1], 1);
                        break;
                    }
                    case 55:
                    {
                        Gaussian_Vague_Process(R_Center_Point[0], R_Center_Point[1], R_Data_Statics, r_dir[R_Data_Statics - 1], 1);
                        break;
                    }
                }
                switch(r_dir[R_Data_Statics - 1])
                {
                    case 1:
                    {
                        R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] + 3][R_Center_Point[0] + 2] - image[R_Center_Point[1] + 3][R_Center_Point[0] + 1]
                                                              - image[R_Center_Point[1] + 3][R_Center_Point[0] + 0] - image[R_Center_Point[1] + 3][R_Center_Point[0] - 1]
                                                              - image[R_Center_Point[1] + 3][R_Center_Point[0] - 2]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] - 2][R_Center_Point[0] + 1]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] + 0] + image[R_Center_Point[1] - 2][R_Center_Point[0] - 1]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] - 2];
                        break;
                    }
                    case -2:
                    {
                        R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] - 1][R_Center_Point[0] + 3] - image[R_Center_Point[1] - 0][R_Center_Point[0] + 3]
                                                              - image[R_Center_Point[1] + 1][R_Center_Point[0] + 3] - image[R_Center_Point[1] + 2][R_Center_Point[0] + 3]
                                                              - image[R_Center_Point[1] + 3][R_Center_Point[0] + 3] - image[R_Center_Point[1] + 3][R_Center_Point[0] + 2]
                                                              - image[R_Center_Point[1] + 3][R_Center_Point[0] + 1] - image[R_Center_Point[1] + 3][R_Center_Point[0] - 0]
                                                              - image[R_Center_Point[1] + 3][R_Center_Point[0] - 1]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] - 2]
                                                              + image[R_Center_Point[1] + 0][R_Center_Point[0] - 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] - 2]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] - 2][R_Center_Point[0] - 1]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] - 0] + image[R_Center_Point[1] - 2][R_Center_Point[0] + 1]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] + 2];
                        break;
                    }
                    case -3:
                    {
                        R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] - 2][R_Center_Point[0] + 3] - image[R_Center_Point[1] - 1][R_Center_Point[0] + 3]
                                                              - image[R_Center_Point[1] + 0][R_Center_Point[0] + 3] - image[R_Center_Point[1] + 1][R_Center_Point[0] + 3]
                                                              - image[R_Center_Point[1] + 2][R_Center_Point[0] + 3]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] - 2]
                                                              + image[R_Center_Point[1] - 0][R_Center_Point[0] - 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] - 2]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] - 2];
                        break;
                    }
                    case -4:
                    {
                        R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] - 3][R_Center_Point[0] - 1] - image[R_Center_Point[1] - 3][R_Center_Point[0] + 0]
                                                              - image[R_Center_Point[1] - 3][R_Center_Point[0] + 1] - image[R_Center_Point[1] - 3][R_Center_Point[0] + 2]
                                                              - image[R_Center_Point[1] - 3][R_Center_Point[0] + 3] - image[R_Center_Point[1] - 2][R_Center_Point[0] + 3]
                                                              - image[R_Center_Point[1] - 1][R_Center_Point[0] + 3] - image[R_Center_Point[1] + 0][R_Center_Point[0] + 3]
                                                              - image[R_Center_Point[1] + 1][R_Center_Point[0] + 3]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] - 2]
                                                              + image[R_Center_Point[1] + 0][R_Center_Point[0] - 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] - 2]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] + 2][R_Center_Point[0] - 1]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] - 0] + image[R_Center_Point[1] + 2][R_Center_Point[0] + 1]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] + 2];
                        break;
                    }
                    case -1:
                    {
                        R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] - 3][R_Center_Point[0] - 2] - image[R_Center_Point[1] - 3][R_Center_Point[0] - 1]
                                                              - image[R_Center_Point[1] - 3][R_Center_Point[0] + 0] - image[R_Center_Point[1] - 3][R_Center_Point[0] + 1]
                                                              - image[R_Center_Point[1] - 3][R_Center_Point[0] + 2]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] - 2] + image[R_Center_Point[1] + 2][R_Center_Point[0] - 1]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] + 0] + image[R_Center_Point[1] + 2][R_Center_Point[0] + 1]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] + 2];
                        break;
                    }
                    case 2:
                    {
                        R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] + 1][R_Center_Point[0] - 3] - image[R_Center_Point[1] + 0][R_Center_Point[0] - 3]
                                                              - image[R_Center_Point[1] - 1][R_Center_Point[0] - 3] - image[R_Center_Point[1] - 2][R_Center_Point[0] - 3]
                                                              - image[R_Center_Point[1] - 3][R_Center_Point[0] - 3] - image[R_Center_Point[1] - 3][R_Center_Point[0] - 2]
                                                              - image[R_Center_Point[1] - 3][R_Center_Point[0] - 1] - image[R_Center_Point[1] - 3][R_Center_Point[0] + 0]
                                                              - image[R_Center_Point[1] - 3][R_Center_Point[0] + 1]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] + 2]
                                                              + image[R_Center_Point[1] - 0][R_Center_Point[0] + 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] + 2]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] + 2][R_Center_Point[0] + 1]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] + 0] + image[R_Center_Point[1] + 2][R_Center_Point[0] - 1]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] - 2];
                        break;
                    }
                    case 3:
                    {
                        R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] + 2][R_Center_Point[0] - 3] - image[R_Center_Point[1] + 1][R_Center_Point[0] - 3]
                                                              - image[R_Center_Point[1] - 0][R_Center_Point[0] - 3] - image[R_Center_Point[1] - 1][R_Center_Point[0] - 3]
                                                              - image[R_Center_Point[1] - 2][R_Center_Point[0] - 3]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] + 2]
                                                              + image[R_Center_Point[1] + 0][R_Center_Point[0] + 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] + 2]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] + 2];
                        break;
                    }
                    case 4:
                    {
                        R_Pixel_Value_Sum = R_Pixel_Value_Sum - image[R_Center_Point[1] + 3][R_Center_Point[0] + 1] - image[R_Center_Point[1] + 3][R_Center_Point[0] - 0]
                                                              - image[R_Center_Point[1] + 3][R_Center_Point[0] - 1] - image[R_Center_Point[1] + 3][R_Center_Point[0] - 2]
                                                              - image[R_Center_Point[1] + 3][R_Center_Point[0] - 3] - image[R_Center_Point[1] + 2][R_Center_Point[0] - 3]
                                                              - image[R_Center_Point[1] + 1][R_Center_Point[0] - 3] - image[R_Center_Point[1] - 0][R_Center_Point[0] - 3]
                                                              - image[R_Center_Point[1] - 1][R_Center_Point[0] - 3]
                                                              + image[R_Center_Point[1] + 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] + 1][R_Center_Point[0] + 2]
                                                              + image[R_Center_Point[1] - 0][R_Center_Point[0] + 2] + image[R_Center_Point[1] - 1][R_Center_Point[0] + 2]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] + 2] + image[R_Center_Point[1] - 2][R_Center_Point[0] + 1]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] + 0] + image[R_Center_Point[1] - 2][R_Center_Point[0] - 1]
                                                              + image[R_Center_Point[1] - 2][R_Center_Point[0] - 2];
                        break;
                    }
                }
            }
            else
            {
                switch(Vague_Flag)
                {
                    case 3:
                    {
                        Gaussian_Vague_Process(R_Center_Point[0], R_Center_Point[1], R_Data_Statics, 0, 1);
                        break;
                    }
                    case 5:
                    {
                        Gaussian_Vague_Process(R_Center_Point[0], R_Center_Point[1], R_Data_Statics, 0, 1);
                        break;
                    }
                    case 33:
                    {
                        Gaussian_Vague_Process(R_Center_Point[0], R_Center_Point[1], R_Data_Statics, 0, 1);
                        break;
                    }
                    case 55:
                    {
                        Gaussian_Vague_Process(R_Center_Point[0], R_Center_Point[1], R_Data_Statics, 0, 1);
                        break;
                    }
                }

                for (j = 0; j < 25; j++)
                {
                    R_Pixel_Value_Sum += image[R_Center_Point[1] + Square_0[j][1]][R_Center_Point[0] + Square_0[j][0]];
                }
            }

            R_Thres = (R_Pixel_Value_Sum + Thres_Interfere) / Thres_Num_Interfere;
            R_Thres -= clip_value;

            if(Thres_Filiter_Flag_1 == 1 || Thres_Filiter_Flag_2 == 1)
            {
                if(R_Data_Statics > 3)
                {
                    R_Thres = R_Thres * 1.3f - R_Thres_Record[R_Data_Statics - 1] * 0.2f - R_Thres_Record[R_Data_Statics - 2] * 0.1f;
                }
            }

            R_Thres_Record[R_Data_Statics] = R_Thres;

            R_Data_Statics++;

            R_Judgme_Again:
            if(R_Stop_Flag == 0)
            {
                R_Front_Value = image[R_Center_Point[1] + R_Face_Dir[R_Dir][1]][R_Center_Point[0] + R_Face_Dir[R_Dir][0]];
                R_Front_R_Value = image[R_Center_Point[1] + R_Face_Dir_R[R_Dir][1]][R_Center_Point[0] + R_Face_Dir_R[R_Dir][0]];
                if((float)R_Front_Value < R_Thres)
                {
                    R_Dir = (R_Dir + 3) % 4;
                    R_Turn_Num ++;
                    if(R_Turn_Num == 4)
                    {
                        R_Stop_Flag = 1;
                    }
                    goto R_Judgme_Again;
                }
                else if((float)R_Front_R_Value < R_Thres)
                {
                    R_Center_Point[0] += R_Face_Dir[R_Dir][0];
                    R_Center_Point[1] += R_Face_Dir[R_Dir][1];
                    r_dir[R_Data_Statics - 1] = R_Face_Dir[R_Dir][0] * 3 - R_Face_Dir[R_Dir][1];
                    R_Turn_Num = 0;
                }
                else
                {
                    R_Center_Point[0] += R_Face_Dir_R[R_Dir][0];
                    R_Center_Point[1] += R_Face_Dir_R[R_Dir][1];
                    r_dir[R_Data_Statics - 1] = R_Face_Dir_R[R_Dir][0] * 3 - R_Face_Dir_R[R_Dir][1];
                    R_Dir = (R_Dir + 1) % 4;
                    R_Turn_Num = 0;
                }
                if(R_Data_Statics >= 4)
                {
                    if(r_line[R_Data_Statics][0] == r_line[R_Data_Statics - 4][0]&&
                       r_line[R_Data_Statics][1] == r_line[R_Data_Statics - 4][1])
                    {
                        R_Stop_Flag = 1;
                    }
                }
            }
        }

        if(L_Stop_Flag == 0 && R_Stop_Flag == 0)
        {
            if ((My_ABS(r_line[R_Data_Statics - 1][0] - l_line[L_Data_Statics - 1][0]) <= 1)
                && (My_ABS(r_line[R_Data_Statics - 1][1] - l_line[L_Data_Statics - 1][1]) <= 1))        //两侧爬线相遇，退出循环，一张图像爬线结束
            {
                *y_meet = (r_line[R_Data_Statics - 1][1] + l_line[L_Data_Statics - 1][1]) >> 1;  //记录相遇点Y
                *x_meet = (r_line[R_Data_Statics - 1][0] + l_line[L_Data_Statics - 1][0]) >> 1;  //记录相遇点X
                break;
            }
        }
        else
        {
            if ((My_ABS(r_line[R_Data_Statics - 1][0] - l_line[L_Data_Statics - 1][0]) <= 3)
                && (My_ABS(r_line[R_Data_Statics - 1][1] - l_line[L_Data_Statics - 1][1]) <= 3))        //两侧爬线相遇，退出循环，一张图像爬线结束
            {
                *y_meet = (r_line[R_Data_Statics - 1][1] + l_line[L_Data_Statics - 1][1]) >> 1;  //记录相遇点Y
                *x_meet = (r_line[R_Data_Statics - 1][0] + l_line[L_Data_Statics - 1][0]) >> 1;  //记录相遇点X
                break;
            }
        }
    }
    L_Stop_Flag = 0;
    R_Stop_Flag = 0;
    *l_stastic = L_Data_Statics;    //记录左侧边线点个数
    *r_stastic = R_Data_Statics;    //记录右侧边线点个数
}

void Binarization_Labyrinth(uint16 Break_Flag, uint8(*image)[Image_X], uint8 Thres, uint8(*l_line)[2], uint8(*r_line)[2], int8 *l_dir, int8 *r_dir, uint16 *l_stastic, uint16 *r_stastic, uint8 *x_meet, uint8 *y_meet,
                            uint8 *l_start, uint8 *r_start)
{
    L_Stop_Flag = 0;
    R_Stop_Flag = 0;

//左边变量
    uint8  L_Center_Point[2] = {0};     //存放每次找到的XY坐标
    uint16 L_Data_Statics = 0;          //统计左边找到的边线点的个数

    uint8  L_Front_Value = 0;           //左侧 面向的前方点的灰度值
    uint8  L_Front_L_Value = 0;         //左侧 面向的左前方点的灰度值

    uint8  L_Dir = 0;                   //此参数用于转向
    uint8  L_Turn_Num = 0;                  //若中心点前后左右都是黑色像素，退出循环防止卡死程序

//右边变量
    uint8  R_Center_Point[2] = {0};     //存放每次找到的XY坐标
    uint16 R_Data_Statics = 0;          //统计右边找到的边线点的个数

    uint8  R_Front_Value = 0;           //右侧 面向的前方点的灰度值
    uint8  R_Front_R_Value = 0;         //右侧 面向的左前方点的灰度值

    uint8  R_Dir = 0;                   //此参数用于转向
    uint8  R_Turn_Num = 0;                  //若中心点前后左右都是黑色像素，退出循环防止卡死程序

//第一次更新坐标点  将找到的起点值传进来
    L_Center_Point[0] = l_start[0];//x
    L_Center_Point[1] = l_start[1];//y
    R_Center_Point[0] = r_start[0];//x
    R_Center_Point[1] = r_start[1];//y
    //开启方向迷宫循环
    while (Break_Flag--)
    {
         //左边
//        L_Search_Again:
        if(L_Stop_Flag == 0)
        {
            l_line[L_Data_Statics][0] = L_Center_Point[0];  //找到的中心点X坐标计入左边线数组
            l_line[L_Data_Statics][1] = L_Center_Point[1];  //找到的中心点Y坐标计入左边线数组

            L_Data_Statics++;                   //每找到一个点统计个数+1

            L_Judge_Again:    //L_Judge_Again 与 goto 配合使用
            if(L_Stop_Flag == 0)
            {
                L_Front_Value = image[L_Center_Point[1] + L_Face_Dir[L_Dir][1]][L_Center_Point[0] + L_Face_Dir[L_Dir][0]];          //记录面向的前方点的灰度值
                L_Front_L_Value = image[L_Center_Point[1] + L_Face_Dir_L[L_Dir][1]][L_Center_Point[0] + L_Face_Dir_L[L_Dir][0]];    //记录面向的左前方点的灰度值
                if((float)L_Front_Value <= Thres)     //面向的前方点是黑色
                {
                    L_Dir = (L_Dir + 1) % 4;    //需右转一次
                    L_Turn_Num ++;
                    if(L_Turn_Num == 4)        //死区处理
                    {
                        L_Stop_Flag = 1;       //当前后左右都是黑色时，进入死区，停止左侧爬线
                    }
                    goto L_Judge_Again;
                }
                else if((float)L_Front_L_Value <= Thres)   //左前方点是黑色，前方点是白色
                {
                    L_Center_Point[0] += L_Face_Dir[L_Dir][0];
                    L_Center_Point[1] += L_Face_Dir[L_Dir][1];      //向前走一步
                    l_dir[L_Data_Statics - 1] = (L_Face_Dir[L_Dir][0] * 3) - L_Face_Dir[L_Dir][1];
                    L_Turn_Num = 0;
                }
                else        //左前方和前方都是白色点
                {
                    L_Center_Point[0] += L_Face_Dir_L[L_Dir][0];
                    L_Center_Point[1] += L_Face_Dir_L[L_Dir][1];        //向左前方走一步
                    l_dir[L_Data_Statics - 1] = (L_Face_Dir_L[L_Dir][0] * 3) - L_Face_Dir_L[L_Dir][1];
                    L_Dir = (L_Dir + 3) % 4;        //左转一次
                    L_Turn_Num = 0;
                }
                if(L_Data_Statics >= 4)     //O环处理
                {
                    if(l_line[L_Data_Statics][0] == l_line[L_Data_Statics - 4][0]&&
                       l_line[L_Data_Statics][1] == l_line[L_Data_Statics - 4][1])
                    {
                        L_Stop_Flag = 1;
                    }
                }
            }
        }

        if(R_Stop_Flag == 0)
        {
            r_line[R_Data_Statics][0] = R_Center_Point[0];
            r_line[R_Data_Statics][1] = R_Center_Point[1];

            R_Data_Statics++;

            R_Judgme_Again:
            if(R_Stop_Flag == 0)
            {
                R_Front_Value = image[R_Center_Point[1] + R_Face_Dir[R_Dir][1]][R_Center_Point[0] + R_Face_Dir[R_Dir][0]];
                R_Front_R_Value = image[R_Center_Point[1] + R_Face_Dir_R[R_Dir][1]][R_Center_Point[0] + R_Face_Dir_R[R_Dir][0]];
                if((float)R_Front_Value <= Thres)
                {
                    R_Dir = (R_Dir + 3) % 4;
                    R_Turn_Num ++;
                    if(R_Turn_Num == 4)
                    {
                        R_Stop_Flag = 1;
                    }
                    goto R_Judgme_Again;
                }
                else if((float)R_Front_R_Value <= Thres)
                {
                    R_Center_Point[0] += R_Face_Dir[R_Dir][0];
                    R_Center_Point[1] += R_Face_Dir[R_Dir][1];
                    r_dir[R_Data_Statics - 1] = R_Face_Dir[R_Dir][0] * 3 - R_Face_Dir[R_Dir][1];
                    R_Turn_Num = 0;
                }
                else
                {
                    R_Center_Point[0] += R_Face_Dir_R[R_Dir][0];
                    R_Center_Point[1] += R_Face_Dir_R[R_Dir][1];
                    r_dir[R_Data_Statics - 1] = R_Face_Dir_R[R_Dir][0] * 3 - R_Face_Dir_R[R_Dir][1];
                    R_Dir = (R_Dir + 1) % 4;
                    R_Turn_Num = 0;
                }
                if(R_Data_Statics >= 4)
                {
                    if(r_line[R_Data_Statics][0] == r_line[R_Data_Statics - 4][0]&&
                       r_line[R_Data_Statics][1] == r_line[R_Data_Statics - 4][1])
                    {
                        R_Stop_Flag = 1;
                    }
                }
            }
        }

        if(L_Stop_Flag == 0 && R_Stop_Flag == 0)
        {
            if ((My_ABS(r_line[R_Data_Statics - 1][0] - l_line[L_Data_Statics - 1][0]) < 2)
                && (My_ABS(r_line[R_Data_Statics - 1][1] - l_line[L_Data_Statics - 1][1]) < 2))        //两侧爬线相遇，退出循环，一张图像爬线结束
            {
                *y_meet = (r_line[R_Data_Statics - 1][1] + l_line[L_Data_Statics - 1][1]) >> 1;  //记录相遇点Y
                *x_meet = (r_line[R_Data_Statics - 1][0] + l_line[L_Data_Statics - 1][0]) >> 1;  //记录相遇点X
                break;
            }
        }
        else
        {
            if ((My_ABS(r_line[R_Data_Statics - 1][0] - l_line[L_Data_Statics - 1][0]) <= 3)
                && (My_ABS(r_line[R_Data_Statics - 1][1] - l_line[L_Data_Statics - 1][1]) <= 3))        //两侧爬线相遇，退出循环，一张图像爬线结束
            {
                *y_meet = (r_line[R_Data_Statics - 1][1] + l_line[L_Data_Statics - 1][1]) >> 1;  //记录相遇点Y
                *x_meet = (r_line[R_Data_Statics - 1][0] + l_line[L_Data_Statics - 1][0]) >> 1;  //记录相遇点X
                break;
            }
        }
    }
    L_Stop_Flag = 0;
    R_Stop_Flag = 0;
    *l_stastic = L_Data_Statics;    //记录左侧边线点个数
    *r_stastic = R_Data_Statics;    //记录右侧边线点个数
}

//三角滤波，对一维边线
float Triangular_Filter_Weights[3] = {0.26, 0.48, 0.26};
void Triangular_Filter(uint8 *line, uint8 start, uint8 end)
{
    uint8 i = 0;
    for(i = start + 1; i < end - 1; i ++)
    {
        line[i] = (uint8)(Triangular_Filter_Weights[0] * (float)line[i - 1] + Triangular_Filter_Weights[1] * (float)line[i    ] + Triangular_Filter_Weights[2] * (float)line[i + 1]);
    }
}

uint8 Adaptive_L_Thres_Max = 0;
uint8 Adaptive_R_Thres_Max = 0;
uint8 Adaptive_L_Thres_Min = 0;
uint8 Adaptive_R_Thres_Min = 0;
uint8 Adaptive_Thres_Average = 0;
uint8 Last_Adaptive_Thres_Average = 0;
//提取阈值中的最大最小值,并求出均值作为二值化阈值，均值计算时间小于4us
void Thres_Record_Process(void)
{
    uint8 i = 0;
    uint8 Left_Temp_Value_1 = 0;
    uint32 Left_Temp_Value_2 = 0;
    uint8 Right_Temp_Value_1 = 0;
    uint32 Right_Temp_Value_2 = 0;
    uint8 L_Average_Thres = 0;
    uint8 R_Average_Thres = 0;

    Adaptive_L_Thres_Max = 0;
    Adaptive_R_Thres_Max = 0;
    Adaptive_L_Thres_Min = 0;
    Adaptive_R_Thres_Min = 0;

    // 移植注释：Pa093 - float→uint8 IAR仍报警，改为双重转换 (uint8)(int)
    // Adaptive_L_Thres_Max = (uint8)L_Thres_Record[0];
    // Adaptive_L_Thres_Min = (uint8)L_Thres_Record[0];
    // Adaptive_R_Thres_Max = (uint8)R_Thres_Record[0];
    // Adaptive_R_Thres_Min = (uint8)R_Thres_Record[0];
    Adaptive_L_Thres_Max = (uint8)(int)L_Thres_Record[0];
    Adaptive_L_Thres_Min = (uint8)(int)L_Thres_Record[0];
    Adaptive_R_Thres_Max = (uint8)(int)R_Thres_Record[0];
    Adaptive_R_Thres_Min = (uint8)(int)R_Thres_Record[0];

    for(i = 0; i < Adaptive_L_Statics; i += 2)
    {
        if(L_Line[i][0] != 2 && L_Line[i][1] != 2)
        {
            if(L_Thres_Record[i] < Adaptive_L_Thres_Min)
            {
                // 移植注释：Pa093 - 同上
                // Adaptive_L_Thres_Min = (uint8)L_Thres_Record[i];
                Adaptive_L_Thres_Min = (uint8)(int)L_Thres_Record[i];
            }
            if(L_Thres_Record[i] > Adaptive_L_Thres_Max)
            {
                // 移植注释：Pa093 - 同上
                // Adaptive_L_Thres_Max = (uint8)L_Thres_Record[i];
                Adaptive_L_Thres_Max = (uint8)(int)L_Thres_Record[i];
            }
            Left_Temp_Value_1 ++;
            Left_Temp_Value_2 += L_Thres_Record[i];
        }
    }
    for(i = 0; i < Adaptive_R_Statics; i += 2)
    {
        if(Adaptive_R_Line[i][0] != 77 && Adaptive_R_Line[i][1] != 2)
        {
            if(R_Thres_Record[i] < Adaptive_R_Thres_Min)
            {
                // 移植注释：Pa093 - 同上
                // Adaptive_R_Thres_Min = (uint8)R_Thres_Record[i];
                Adaptive_R_Thres_Min = (uint8)(int)R_Thres_Record[i];
            }
            if(R_Thres_Record[i] > Adaptive_R_Thres_Max)
            {
                // 移植注释：Pa093 - 同上
                // Adaptive_R_Thres_Max = (uint8)R_Thres_Record[i];
                Adaptive_R_Thres_Max = (uint8)(int)R_Thres_Record[i];
            }
            Right_Temp_Value_1 ++;
            Right_Temp_Value_2 += R_Thres_Record[i];
        }
    }

    if(Left_Temp_Value_1 == 0)
    {
        L_Average_Thres = 0;
    }
    else
    {
        L_Average_Thres = (uint8)(Left_Temp_Value_2 / Left_Temp_Value_1);
    }

    if(Right_Temp_Value_1 == 0)
    {
        R_Average_Thres = 0;
    }
    else
    {
        R_Average_Thres = (uint8)(Right_Temp_Value_2 / Right_Temp_Value_1);
    }

    if(Image_Num <= 1)
    {
        Last_Adaptive_Thres_Average = (uint8)((L_Average_Thres + R_Average_Thres) / 2);
    }
    else
    {
        if(My_ABS_uint8(L_Average_Thres - R_Average_Thres) >= 40)
        {
            if(My_ABS_uint8(L_Average_Thres - Last_Adaptive_Thres_Average) <= My_ABS_uint8(R_Average_Thres - Last_Adaptive_Thres_Average))
            {
                Adaptive_Thres_Average = (uint8)((Last_Adaptive_Thres_Average + L_Average_Thres) / 2);
                Last_Adaptive_Thres_Average = Adaptive_Thres_Average;
            }
            else
            {
                Adaptive_Thres_Average = (uint8)((Last_Adaptive_Thres_Average + R_Average_Thres) / 2);
                Last_Adaptive_Thres_Average = Adaptive_Thres_Average;
            }
        }
        else
        {
            Adaptive_Thres_Average = (uint8)((Last_Adaptive_Thres_Average + L_Average_Thres + R_Average_Thres) / 3);
            Last_Adaptive_Thres_Average = Adaptive_Thres_Average;
        }
    }

    //获得判断斑马线的阈值
    Zbra_Thres = Adaptive_Thres_Average - 10;

    //获得起始点差比和的阈值
    if(Adaptive_Thres_Average >= 100 && Adaptive_Thres_Average <= 140)
    {
        Compare_Value = 20;
    }
    else if(Adaptive_Thres_Average < 100)
    {
        Compare_Value = 20 - (uint8)(((float)(100 - Adaptive_Thres_Average) / 60.0f) * 10.0f);
    }
    else if(Adaptive_Thres_Average > 140)
    {
        Compare_Value = 20 - (uint8)(((float)(Adaptive_Thres_Average - 140) / 60.0f) * 10.0f);
    }

    if(Other_Show_Flag == 1)
    {
        tft180_show_float(65,220,Adaptive_L_Thres_Max,3,2);
        tft180_show_float(65,230,Adaptive_L_Thres_Min,3,2);
        tft180_show_float(65,240,Adaptive_R_Thres_Max,3,2);
        tft180_show_float(65,250,Adaptive_R_Thres_Min,3,2);
        tft180_show_float(65,260,Adaptive_Thres_Average,3,2);
    }
    if(Other_Show_Flag == 2)
    {
//        ips200_show_float(65,220,Adaptive_L_Thres_Max,3,2);
//        ips200_show_float(65,230,Adaptive_L_Thres_Min,3,2);
//        ips200_show_float(65,240,Adaptive_R_Thres_Max,3,2);
//        ips200_show_float(65,250,Adaptive_R_Thres_Min,3,2);
//        ips200_show_float(65,260,Adaptive_Thres_Average,3,2);
    }
}

void Get_Border(uint16 l_total, uint16 r_total, uint8 start, uint8 end, uint8 *l_border, uint8 *r_border, uint8(*l_line)[2], uint8(*r_line)[2])
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    for (i = 0; i < Image_Y; i++)
    {
        l_border[i] = X_Border_Min;
        r_border[i] = X_Border_Max;     //右边线初始化放到最右边，左边线放到最左边，这样闭合区域外的中线就会在中间，不会干扰得到的数据
    }
    h = start;
    //右边
    for (j = 0; j < r_total; j++)
    {
        if (r_line[j][1] == h)
        {
            r_border[h] = r_line[j][0];
        }
        else
        {
            continue;//每行只取一个点，没到下一行就不记录
        }
        h--;
        if (h == end)
        {
            break;//到最后一行退出
        }
    }
    h = start;
    for (j = 0; j < l_total; j++)
    {
        if (l_line[j][1] == h)
        {
            l_border[h] = l_line[j][0];
        }
        else
        {
            continue;//每行只取一个点，没到下一行就不记录
        }
        h--;
        if (h == end)
        {
            break;//到最后一行退出
        }
    }
}

//当遇到斑马线时，边线数组特殊处理
void Zebra_Get_Border(uint16 l_total, uint16 r_total, uint8 start, uint8 end)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    for (i = 2; i < Image_Y - 3; i++)
    {
        L_Border[i] = X_Border_Min;
        R_Border[i] = X_Border_Max;//右边线初始化放到最右边，左边线放到最左边，这样闭合区域外的中线就会在中间，不会干扰得到的数据
    }
    h = start;
    //右边
    for (j = 0; j < r_total; j++)
    {
        if (R_Line[j][1] == h)
        {
            R_Border[h] = R_Line[j][0];
        }
        else
        {
            continue;//每行只取一个点，没到下一行就不记录
        }
        h--;
        if (h == end)
        {
            break;//到最后一行退出
        }
    }
    h = start;
    for (j = 0; j < l_total; j++)
    {
        if (L_Line[j][1] == h)
        {
            L_Border[h] = L_Line[j][0];
        }
        else
        {
            continue;//每行只取一个点，没到下一行就不记录
        }
        h--;
        if (h == end)
        {
            break;//到最后一行退出
        }
    }
}

void Bilatreal_Line_Fitting()
{
    uint32 i = 0;
    if(Bilatreal_Line_Fitting_Flag == 1)            //当双边线拟合标志位为1时，边线完全来自于自适应边线
    {
        for(i = 0; i < Adaptive_L_Statics; i++)
        {
            L_Line[i][0] = Adaptive_L_Line[i][0];
            L_Line[i][1] = Adaptive_L_Line[i][1];
        }
        for(i = 0; i < Adaptive_R_Statics; i++)
        {
            R_Line[i][0] = Adaptive_R_Line[i][0];
            R_Line[i][1] = Adaptive_R_Line[i][1];
        }
        for(i = 0; i < Adaptive_L_Statics; i++)
        {
            L_Grow_Dir[i] = Adaptive_L_Grow_Dir[i];
        }
        for(i = 0; i < Adaptive_R_Statics; i++)
        {
            R_Grow_Dir[i] = Adaptive_R_Grow_Dir[i];
        }
        L_Start_Point[0] = Adaptive_L_Start_Point[0];
        L_Start_Point[1] = Adaptive_L_Start_Point[1];
        R_Start_Point[0] = Adaptive_R_Start_Point[0];
        R_Start_Point[1] = Adaptive_R_Start_Point[1];
        L_Statics = Adaptive_L_Statics;
        R_Statics = Adaptive_R_Statics;
        X_Meet = Adaptive_X_Meet;
        Y_Meet = Adaptive_Y_Meet;
    }
    else if(Bilatreal_Line_Fitting_Flag == 2)       ////当双边线拟合标志位为2时，边线完全来自于二值化边线
    {
        for(i = 0; i < Binarization_L_Statics; i++)
        {
            L_Line[i][0] = Binarization_L_Line[i][0];
            L_Line[i][1] = Binarization_L_Line[i][1];
        }
        for(i = 0; i < Binarization_R_Statics; i++)
        {
            R_Line[i][0] = Binarization_R_Line[i][0];
            R_Line[i][1] = Binarization_R_Line[i][1];
        }
        for(i = 0; i < Binarization_L_Statics; i++)
        {
            L_Grow_Dir[i] = Binarization_L_Grow_Dir[i];
        }
        for(i = 0; i < Binarization_R_Statics; i++)
        {
            R_Grow_Dir[i] = Binarization_R_Grow_Dir[i];
        }
        L_Start_Point[0] = Binarization_L_Start_Point[0];
        L_Start_Point[1] = Binarization_L_Start_Point[1];
        R_Start_Point[0] = Binarization_R_Start_Point[0];
        R_Start_Point[1] = Binarization_R_Start_Point[1];
        L_Statics = Binarization_L_Statics;
        R_Statics = Binarization_R_Statics;
        X_Meet = Binarization_X_Meet;
        Y_Meet = Binarization_Y_Meet;
    }

}
//综合梳理
void Process_Image(void)
{
    L_Statics = 0;
    R_Statics = 0;

    Adaptive_L_Statics = 0;
    Adaptive_R_Statics = 0;
    Binarization_L_Statics = 0;
    Binarization_R_Statics = 0;
//    Inverse_Flag = 1;

    Copy_Zip_Image();
    Draw_Black_Box(Black_Box_Value, Find_Line_Image);
    if(Get_Start_Point(Image_Y - 3, Find_Line_Image, Adaptive_L_Start_Point, Adaptive_R_Start_Point, 1, 78) == 1)
    {
        if(Vague_Flag == 0)
        {
            Dir_Labyrinth_5((uint16)Use_Num, Find_Line_Image, Adaptive_L_Line, Adaptive_R_Line, Adaptive_L_Grow_Dir, Adaptive_R_Grow_Dir, &Adaptive_L_Statics, &Adaptive_R_Statics, &Adaptive_X_Meet, &Adaptive_Y_Meet,
                            Adaptive_L_Start_Point[0], Adaptive_L_Start_Point[1], Adaptive_R_Start_Point[0], Adaptive_R_Start_Point[1], 0);
        }
        else
        {
            Dir_Labyrinth_5((uint16)Use_Num, Vague_Image, Adaptive_L_Line, Adaptive_R_Line, Adaptive_L_Grow_Dir, Adaptive_R_Grow_Dir, &Adaptive_L_Statics, &Adaptive_R_Statics, &Adaptive_X_Meet, &Adaptive_Y_Meet,
                            Adaptive_L_Start_Point[0], Adaptive_L_Start_Point[1], Adaptive_R_Start_Point[0], Adaptive_R_Start_Point[1], 0);
        }
        if((Adaptive_L_Statics <= 10 && (Adaptive_R_Statics - Adaptive_L_Statics >= 50)) || (Adaptive_R_Statics <= 10 && (Adaptive_L_Statics - Adaptive_R_Statics >= 50)))
        {
            L_Statics = 0;
            R_Statics = 0;
            if(Get_Start_Point(50, Find_Line_Image, Adaptive_L_Start_Point, Adaptive_R_Start_Point, 1, 78) == 1)
            {
                if(Vague_Flag == 0)
                {
                    Dir_Labyrinth_5((uint16)Use_Num, Find_Line_Image, Adaptive_L_Line, Adaptive_R_Line, Adaptive_L_Grow_Dir, Adaptive_R_Grow_Dir, &Adaptive_L_Statics, &Adaptive_R_Statics, &Adaptive_X_Meet, &Adaptive_Y_Meet,
                                    Adaptive_L_Start_Point[0], Adaptive_L_Start_Point[1], Adaptive_R_Start_Point[0], Adaptive_R_Start_Point[1], 0);
                }
                else
                {
                    Dir_Labyrinth_5((uint16)Use_Num, Vague_Image, Adaptive_L_Line, Adaptive_R_Line, Adaptive_L_Grow_Dir, Adaptive_R_Grow_Dir, &Adaptive_L_Statics, &Adaptive_R_Statics, &Adaptive_X_Meet, &Adaptive_Y_Meet,
                                    Adaptive_L_Start_Point[0], Adaptive_L_Start_Point[1], Adaptive_R_Start_Point[0], Adaptive_R_Start_Point[1], 0);
                }
            }
        }
    }

    Thres_Record_Process();
    if(Bilatreal_Line_Fitting_Flag == 2)
    {
        if(Binarization_Get_Start_Point(Adaptive_L_Start_Point[1], Find_Line_Image, (Adaptive_L_Start_Point[0] + Adaptive_R_Start_Point[0]) / 2, Binarization_L_Start_Point, Binarization_R_Start_Point, 0, 79, Adaptive_Thres_Average - 20))
        {
            Binarization_Labyrinth((uint16)Use_Num, Find_Line_Image, Adaptive_Thres_Average - 20, Binarization_L_Line, Binarization_R_Line, Binarization_L_Grow_Dir, Binarization_R_Grow_Dir,
                                    &Binarization_L_Statics, &Binarization_R_Statics, &Binarization_X_Meet, &Binarization_Y_Meet, Binarization_L_Start_Point, Binarization_R_Start_Point);
        }
    }

    Black_Box_Value = (uint8)(0.45f * (float)sqrt(Adaptive_L_Thres_Min * Adaptive_L_Thres_Min + Adaptive_R_Thres_Min * Adaptive_R_Thres_Min)) + (uint8)(0.1f * (float)Black_Box_Value_1);      //0.45f即0.9 * 0.5，0.9为权重，0.5可自己调节
    Bilatreal_Line_Fitting();
    Get_Border(L_Statics, R_Statics, Image_Y - 3, 2, L_Border, R_Border, L_Line, R_Line);
}

//***************************************************************************************************************************************************************************




//*************************************************************************提取信息，判断元素**********************************************************************************
/**
* 函数功能：      最小二乘法计算斜率
* 特殊说明：      无
* 形  参：        uint8 begin                输入起点
*                 uint8 end                  输入终点
*                 uint8 *border              输入需要计算斜率的一维边线数组
* 示例：          Slope_Calculate(start, end, border);
* 返回值：        Result    计算出的斜率
*/
float Slope_Calculate(uint8 begin, uint8 end, uint8 *border)    //注：begin 必须小于 end,一般 begin 位于图像上方， end 位于图像下方
{
    float X_Sum = 0, Y_Sum = 0, XY_Sum = 0, X2_Sum = 0;
    int16 i = 0;
    float Result = 0;
    static float Result_Last;

    for(i = begin; i < end ; i++)
    {
        X_Sum += (float)i;
        Y_Sum += (float)border[i];
        XY_Sum += (float)i * (border[i]);
        X2_Sum += (float)i * i;
    }

    if((end - begin) * X2_Sum - X_Sum * X_Sum)      //防止为0的情况出现
    {
        Result = ((float)(end - begin) * XY_Sum - X_Sum * Y_Sum) / ((float)(end - begin) * X2_Sum - X_Sum * X_Sum);
        Result_Last = Result;
    }
    else
    {
        Result = Result_Last;
    }
    return Result;
}
float Slope_Calculate_break(uint8 begin, uint8 end, uint8 break_begin, uint8 break_end, uint8 *border)
{
    float X_Sum = 0, Y_Sum = 0, XY_Sum = 0, X2_Sum = 0;
    int16 i = 0;
    float Result = 0;
    static float Result_Last;

    /* 处理前半段区间：begin到break_begin-1 */
    for(i = begin; i < break_begin && i < end; i++) {  // [3,4](@ref)
        X_Sum += (float)i;
        Y_Sum += (float)border[i];
        XY_Sum += (float)i * border[i];
        X2_Sum += (float)i * i;
    }

    /* 处理后半段区间：break_end+1到end */
    uint8 start = (break_end >= end) ? end : break_end + 1;  // 防越界处理[3](@ref)
    for(i = start; i < end; i++) {
        X_Sum += (float)i;
        Y_Sum += (float)border[i];
        XY_Sum += (float)i * border[i];
        X2_Sum += (float)i * i;
    }

    /* 斜率计算 */
    float denominator = (float)(end - begin) * X2_Sum - X_Sum * X_Sum;
    if(denominator != 0) {
        Result = ((float)(end - begin) * XY_Sum - X_Sum * Y_Sum) / denominator;
        Result_Last = Result;
    } else {
        Result = Result_Last;  // 继承上次有效值[1](@ref)
    }
    return Result;
}

/**
* @brief 计算斜率截距
* @param uint8 start                输入起点
* @param uint8 end                  输入终点
* @param uint8 *border              输入需要计算斜率的边界
* @param float *slope_rate          输入斜率地址
* @param float *intercept           输入截距地址
*  @see CTest       calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/

/**
* 函数功能：      计算斜率截距
* 特殊说明：      调用最小二乘法计算斜率
* 形  参：        uint8 start                输入起点
*                 uint8 end                  输入终点
*                 uint8 *border              输入需要计算斜率的一维边线数组
*                 float *slope_rate          存储斜率的变量地址
*                 float *intercept           存储截距的变量地址
* 示例：          Calculate_Slope_Intercept(start, end, L_Border, &L_Straightaway_Lope_Rate_C, &L_Intercept);
* 返回值：        无
*/
void Calculate_Slope_Intercept(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept)
{
    uint16 i, Num = 0;
    uint16 X_Sum = 0, Y_Sum = 0;
    float Y_Average = 0, X_Average = 0;

    for(i = start; i < end; i++)
    {
        X_Sum += i;
        Y_Sum += border[i];
        Num ++;
    }

    if(Num)
    {
        X_Average = (float)(X_Sum / Num);
        Y_Average = (float)(Y_Sum / Num);
    }

    *slope_rate = Slope_Calculate(start, end, border);
    *intercept = (float)(Y_Average - (*slope_rate) * X_Average);
}
void Calculate_Slope_Intercept_break(uint8 start, uint8 end, uint8 break_begin, uint8 break_end, uint8 *border, float *slope_rate, float *intercept)
{
    uint16 i, Num = 0;
    uint32 X_Sum = 0, Y_Sum = 0;  // 改用32位防溢出
    float Y_Average = 0, X_Average = 0;

    /* 处理前半段：start到break_begin-1 */
    uint8 first_end = (break_begin > end) ? end : break_begin;
    for(i = start; i < first_end; i++) {
        X_Sum += i;
        Y_Sum += border[i];
        Num++;
    }

    /* 处理后半段：break_end+1到end */
    uint8 second_start = (break_end >= end) ? end : (break_end + 1);
    for(i = second_start; i < end; i++) {
        X_Sum += i;
        Y_Sum += border[i];
        Num++;
    }

    if(Num) {
        X_Average = (float)X_Sum / Num;  // 改用浮点除法提升精度[1](@ref)
        Y_Average = (float)Y_Sum / Num;
    } else {
        *slope_rate = 0;  // 防止除零错误
        *intercept = 0;
        return;
    }

    // 调用已支持断点区间的Slope_Calculate函数
    *slope_rate = Slope_Calculate_break(start, end, break_begin, break_end, border);
    *intercept = Y_Average - (*slope_rate) * X_Average;
}

void Calculate_Slope_Intercept_Twodot(uint8 x1, uint8 y1, uint8 x2, uint8 y2,
                              float *slope_rate, float *intercept)
{
    int dec = x1 - x2;

    if(dec == 0)
        return;

    *slope_rate = (float)(y1-y2) / dec;
    *intercept  = (float)(y2*x1 - y1*x2) / dec;
}

uint8 Straight_Thres = 15;
uint8 Sml_Arc_Thres = 10;
uint8 Big_Arc_thres = 130;

//求方差
uint8 Num = 26;
void Get_Variance(uint8 start, uint8 end, int16 *fitting_line, uint8 *actual_line, float *variance, uint8 multiple)
{
    uint8 i = 0;
    float S = 0;
    uint8 total_num = 0;
    float Temp = 0;

    if(start % 3 != 0)
    {
        for(i = 1; i < 3; i++)
        {
            start = start + i;
            if(start % 3 == 1)
            {
                break;
            }
        }
    }

    total_num = (uint8)((end - start) / multiple);

    for(i = 0; i < total_num; i++)
    {
        uint8 Temp_Value = 0;
        Temp_Value = i * multiple + start;
        Temp = (float)(actual_line[Temp_Value] - (uint8)fitting_line[Temp_Value / 3]);
        S += Temp * Temp;
    }
    S /= (float)total_num;
    *variance = S;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     获取指定上下点数形成的夹角
  @param     当前点索引 mid_point，step:前后步长， (*line)[2]：二维边线数组
  @return    angle_rad
  Sample
  @error        -1.0:越界
                -2.0:错误参数
-------------------------------------------------------------------------------------------------------------------*/
float Get_Angle_Points(uint16 mid_point, uint16 step , uint8 (*line)[2] )
{
    if((mid_point + step) > Use_Num || (mid_point - step) < 0)
    {
        return -1.0;
    }
    int16 v1_x = line[mid_point][0] - line[mid_point - step][0];
    int16 v1_y = line[mid_point][1] - line[mid_point - step][1];
    int16 v2_x = line[mid_point + step][0] - line[mid_point][0];
    int16 v2_y = line[mid_point + step][1] - line[mid_point][1];

    int32 dot = v1_x * v2_x + v1_y * v2_y;
    int32 mag1_sq = v1_x * v1_x + v1_y * v1_y;
    int32 mag2_sq = v2_x * v2_x + v2_y * v2_y;

    if (mag1_sq == 0 || mag2_sq == 0) return -2.0f;

    float cos_theta = (float)dot / sqrtf((float)mag1_sq * (float)mag2_sq);

    // 修正 cos_theta 数值范围，避免 acos 出现 NAN
    if (cos_theta > 1.0f) cos_theta = 1.0f;
    if (cos_theta < -1.0f) cos_theta = -1.0f;

    float angle_rad = acosf(cos_theta);           // 弧度
    return angle_rad * (180.0f / 3.14159f);        // 转成角度返回
}


/**
* 函数功能：      寻找上转左拐点
* 特殊说明：      分为三个严格等级，根据每个点的生长方向判断
*                 寻找方式为遍历二维数组边线的每个点
* 形  参：        uint8 Grade      选择判定严格等级，常用2，即中等等级
*
* 示例：          Get_L_Up_Turn_Left_Point_1(2);
* 返回值：        无
*/
uint8 L_Up_Turn_Left_Point_1[2] = {0};      //存储上转左拐点的坐标
uint8 L_Up_Turn_Left_Point_Flag_1 = 0;      //上转左拐点存在标志位，找到时置1
uint8 L_Up_Turn_Left_Point_Position_1 = 0;  //记录位置，即是二维数组中的第几个点
float L_Up_Turn_Left_Point_Angle_1 = 0;     //记录找到的拐点的角度（逆透视求取）
void Get_L_Up_Turn_Left_Point_1(uint8 Grade)
{
    uint16 i = 0;
    L_Up_Turn_Left_Point_Flag_1 = 0;
    L_Up_Turn_Left_Point_1[0] = 0;
    L_Up_Turn_Left_Point_1[1] = 0;
    L_Up_Turn_Left_Point_Position_1 = 0;
    L_Up_Turn_Left_Point_Angle_1 = 0;       //将各个参数清零
    switch(Grade)
    {
        case 1:     //严格判断
        {
            for (i = 4; i < (L_Statics - 4); i++)
            {
                if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((L_Grow_Dir[i - 2] == 4 || L_Grow_Dir[i - 2] == 1) && (L_Grow_Dir[i - 4] == 4 || L_Grow_Dir[i - 4] == 1) &&
                        (L_Grow_Dir[i + 2] == -3) && (L_Grow_Dir[i + 4] == -3) && (L_Grow_Dir[i] == -2 || L_Grow_Dir[i] == -3))
                    {
                        L_Up_Turn_Left_Point_1[0] = L_Line[i][0];
                        L_Up_Turn_Left_Point_1[1] = L_Line[i][1];
                        L_Up_Turn_Left_Point_Flag_1  = 1;
                        break;
                    }
                }
            }
            break;
        }
        case 2:     //中等判断
        {
            for (i = 4; i < (L_Statics - 4); i++)
            {
                if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((L_Grow_Dir[i - 2] == 4 || L_Grow_Dir[i - 2] == 1) && (L_Grow_Dir[i - 4] == 4 || L_Grow_Dir[i - 4] == 1) &&
                        (L_Grow_Dir[i + 2] == -3 || L_Grow_Dir[i + 2] == -4) && (L_Grow_Dir[i + 4] == -3 || L_Grow_Dir[i + 4] == -4) &&
                        (L_Grow_Dir[i] == -2 || L_Grow_Dir[i] == -3 || L_Grow_Dir[i] == -4))
                    {
                        L_Up_Turn_Left_Point_1[0] = L_Line[i][0];
                        L_Up_Turn_Left_Point_1[1] = L_Line[i][1];
                        L_Up_Turn_Left_Point_Flag_1  = 1;
                        break;
                    }
                }
            }
            break;
        }
        case 3:     //宽松判断
        {
            for (i = 4; i < (L_Statics - 4); i++)
            {
                if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((L_Grow_Dir[i - 2] == 4 || L_Grow_Dir[i - 2] == 1 || L_Grow_Dir[i - 2] == -2) && (L_Grow_Dir[i - 4] == 4 || L_Grow_Dir[i - 4] == 1 || L_Grow_Dir[i - 2] == -2) &&
                        (L_Grow_Dir[i + 2] == -3 || L_Grow_Dir[i + 2] == -4 || L_Grow_Dir[i + 2] == -2) && (L_Grow_Dir[i + 4] == -3 || L_Grow_Dir[i + 4] == -4 || L_Grow_Dir[i + 4] == -2) &&
                        (L_Grow_Dir[i] == -2 || L_Grow_Dir[i] == -3 || L_Grow_Dir[i] == -4))
                    {
                        L_Up_Turn_Left_Point_1[0] = L_Line[i][0];
                        L_Up_Turn_Left_Point_1[1] = L_Line[i][1];
                        L_Up_Turn_Left_Point_Flag_1  = 1;
                        break;
                    }
                }
            }
            break;
        }
    }
    if(L_Up_Turn_Left_Point_Flag_1 == 1)
    {
        L_Up_Turn_Left_Point_Position_1 = i;    //记录拐点的位置
//        L_Up_Turn_Left_Point_Angle_1 = Get_Turn_Point_Angle(L_Line[i - 5][0], L_Line[i - 5][1], L_Line[i][0], L_Line[i][1], L_Line[i + 5][0], L_Line[i + 5][1]);    //逆透视求取拐点角度，后续补充
    }
}

/**
* 函数功能：      寻找上转左拐点
* 特殊说明：      通过检测X值的跳变判定拐点的存在
*                 寻找方式为遍历一维数组边线的每个点
* 形  参：        uint8 Region_Start      起始行
*                 uint8 Region_End        截止行
*
* 示例：          Get_L_Up_Turn_Left_Point_2(2, 57);
* 返回值：        无
*/
uint8 L_Up_Turn_Left_Point_2[2] = {0};
uint8 L_Up_Turn_Left_Point_Flag_2 = 0;
uint8 L_Up_Turn_Left_Point_Position_2 = 0;
void Get_L_Up_Turn_Left_Point_2(uint8 Region_Start, uint8 Region_End)
{
    uint8 i = 0;
    L_Up_Turn_Left_Point_2[0] = 0;
    L_Up_Turn_Left_Point_2[1] = 0;
    L_Up_Turn_Left_Point_Position_2 = 0;
    if(Region_Start <= Region_End)
    {
        for(i = Region_Start; i <= Region_End; i++)
        {
            if(L_Border[i] - L_Border[i - 1] >= 6)
            {
                if(L_Border[i + 1] - L_Border[i - 2] >= 6)
                {
                    if(L_Border[i - 1] == 2 && L_Border[i - 2] == 2 && L_Border[i] != 2)
                    {
                        L_Up_Turn_Left_Point_2[0] = L_Border[i];
                        L_Up_Turn_Left_Point_2[1] = i;
                        L_Up_Turn_Left_Point_Flag_2 = 1;
                        L_Up_Turn_Left_Point_Position_2 = i;
                    }
                }
            }
        }
    }
}
uint8 L_Up_Turn_Left_Point[2] = {0};
uint8 L_Up_Turn_Left_Point_Flag = 0;
void Get_L_Up_Turn_Left_Point(void)
{
    L_Up_Turn_Left_Point_Flag = 0;
    Get_L_Up_Turn_Left_Point_1(2);
    if(L_Up_Turn_Left_Point_Flag_1 == 1)
    {
        L_Up_Turn_Left_Point_Flag_1 = 0;
        Get_L_Up_Turn_Left_Point_2(L_Up_Turn_Left_Point_1[1] - 7, L_Up_Turn_Left_Point_1[1] + 7);
        {
            if(L_Up_Turn_Left_Point_Flag_2 == 1)
            {
                L_Up_Turn_Left_Point_Flag_2 = 0;
                L_Up_Turn_Left_Point_Flag = 1;
                L_Up_Turn_Left_Point[0] = L_Up_Turn_Left_Point_1[0];
                L_Up_Turn_Left_Point[1] = L_Up_Turn_Left_Point_1[1];
            }
        }
    }
}
/**ramp
*/
uint8 L_Right_Turn_Up_Point_1_ramp[2] = {0};
uint8 L_Right_Turn_Up_Point_Flag_1_ramp = 0;
uint8 L_Right_Turn_Up_Point_Position_1_ramp = 0;
//float L_Right_Turn_Up_Point_Angle_1_ramp = 0;
void Get_L_Right_Turn_Up_Point_1_ramp(uint8 Grade)
{
    uint16 i = 0;
    L_Right_Turn_Up_Point_Flag_1_ramp = 0;
    L_Right_Turn_Up_Point_1_ramp[0] = 0;
    L_Right_Turn_Up_Point_1_ramp[1] = 0;
    L_Right_Turn_Up_Point_Position_1_ramp = 0;
//    L_Right_Turn_Up_Point_Angle_1 = 0;
    switch(Grade)
    {
        case 1:
        {
            for (i = 5; i < (L_Statics - 4); i++)
            {
                if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((L_Grow_Dir[i + 2] == 4 || L_Grow_Dir[i + 2] == 1) && (L_Grow_Dir[i + 5] == 4 || L_Grow_Dir[i + 5] == 1) && (L_Grow_Dir[i + 7] == 4 || L_Grow_Dir[i + 7] == 1) &&
                        (L_Grow_Dir[i - 2] == 3) && (L_Grow_Dir[i - 5] == 3) && (L_Grow_Dir[i] == 4 || L_Grow_Dir[i] == 1))
                    {
                        L_Right_Turn_Up_Point_1_ramp[0] = L_Line[i][0];
                        L_Right_Turn_Up_Point_1_ramp[1] = L_Line[i][1];
                        L_Right_Turn_Up_Point_Flag_1_ramp  = 1;
                        break;
                    }
                }
            }
            break;
        }
        case 2:
        {
            for (i = 5; i < (L_Statics - 4); i++)
            {
                if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((L_Grow_Dir[i + 2] == 4 || L_Grow_Dir[i + 2] == 1) && (L_Grow_Dir[i + 5] == 4 || L_Grow_Dir[i + 5] == 1) && (L_Grow_Dir[i + 7] == 4 || L_Grow_Dir[i + 7] == 1) &&
                        (L_Grow_Dir[i - 2] == 2 || L_Grow_Dir[i - 2] == 3) && (L_Grow_Dir[i - 3] == 2 || L_Grow_Dir[i - 3] == 3) &&
                        (L_Grow_Dir[i] == 4 || L_Grow_Dir[i] == 1))
                    {
                        L_Right_Turn_Up_Point_1_ramp[0] = L_Line[i][0];
                        L_Right_Turn_Up_Point_1_ramp[1] = L_Line[i][1];
                        L_Right_Turn_Up_Point_Flag_1_ramp  = 1;
                        break;
                    }
                }
            }
            break;
        }
        case 3:
        {
            for (i = 5; i < (L_Statics - 4); i++)
            {
                if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((L_Grow_Dir[i + 2] == 4 || L_Grow_Dir[i + 2] == 1 || L_Grow_Dir[i + 2] == -2) && (L_Grow_Dir[i + 5] == 4 || L_Grow_Dir[i + 5] == 1 || L_Grow_Dir[i + 5] == -2) &&
                        (L_Grow_Dir[i + 7] == 4 || L_Grow_Dir[i + 7] == 1 || L_Grow_Dir[i + 7] == -2) &&
                        (L_Grow_Dir[i - 2] == 2 || L_Grow_Dir[i - 2] == 3 || L_Grow_Dir[i - 2] == 4) && (L_Grow_Dir[i - 5] == 2 || L_Grow_Dir[i - 5] == 3 || L_Grow_Dir[i - 5] == 4) &&
                        (L_Grow_Dir[i] == 4 || L_Grow_Dir[i] == 1))
                    {
                        L_Right_Turn_Up_Point_1_ramp[0] = L_Line[i][0];
                        L_Right_Turn_Up_Point_1_ramp[1] = L_Line[i][1];
                        L_Right_Turn_Up_Point_Flag_1_ramp  = 1;
                        break;
                    }
                }
            }
            break;
        }
    }
    if(L_Right_Turn_Up_Point_Flag_1_ramp == 1)
    {
        L_Right_Turn_Up_Point_Position_1_ramp = i;
//        L_Right_Turn_Up_Point_Angle_1 = Get_Turn_Point_Angle(L_Line[i - 5][0], L_Line[i - 5][1], L_Line[i][0], L_Line[i][1], L_Line[i + 5][0], L_Line[i + 5][1]);
    }
}

/**
* 函数功能：      寻找右转上拐点
* 特殊说明：      分为三个严格等级，根据每个点的生长方向判断
*                 寻找方式为遍历二维数组边线的每个点
* 形  参：        uint8 Grade      选择判定严格等级，常用2，即中等等级
*
* 示例：          Get_L_Right_Turn_Up_Point_1(2);
* 返回值：        无
*/
uint8 L_Right_Turn_Up_Point_1[2] = {0};
uint8 L_Right_Turn_Up_Point_Flag_1 = 0;
uint8 L_Right_Turn_Up_Point_Position_1 = 0;
float L_Right_Turn_Up_Point_Angle_1 = 0;
void Get_L_Right_Turn_Up_Point_1(uint8 Grade)
{
    uint16 i = 0;
    L_Right_Turn_Up_Point_Flag_1 = 0;
    L_Right_Turn_Up_Point_1[0] = 0;
    L_Right_Turn_Up_Point_1[1] = 0;
    L_Right_Turn_Up_Point_Position_1 = 0;
    L_Right_Turn_Up_Point_Angle_1 = 0;
    switch(Grade)
    {
        case 1:
        {
            for (i = 5; i < (L_Statics - 4); i++)
            {
                if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((L_Grow_Dir[i + 2] == 4 || L_Grow_Dir[i + 2] == 1) && (L_Grow_Dir[i + 5] == 4 || L_Grow_Dir[i + 5] == 1) && (L_Grow_Dir[i + 7] == 4 || L_Grow_Dir[i + 7] == 1) &&
                        (L_Grow_Dir[i - 2] == 3) && (L_Grow_Dir[i - 5] == 3) && (L_Grow_Dir[i] == 4 || L_Grow_Dir[i] == 1))
                    {
                        L_Right_Turn_Up_Point_1[0] = L_Line[i][0];
                        L_Right_Turn_Up_Point_1[1] = L_Line[i][1];
                        L_Right_Turn_Up_Point_Flag_1  = 1;
                        break;
                    }
                }
            }
            break;
        }
        case 2:
        {
            for (i = 5; i < (L_Statics - 4); i++)
            {
                if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((L_Grow_Dir[i + 2] == 4 || L_Grow_Dir[i + 2] == 1) && (L_Grow_Dir[i + 5] == 4 || L_Grow_Dir[i + 5] == 1) && (L_Grow_Dir[i + 7] == 4 || L_Grow_Dir[i + 7] == 1) &&
                        (L_Grow_Dir[i - 2] == 2 || L_Grow_Dir[i - 2] == 3) && (L_Grow_Dir[i - 5] == 2 || L_Grow_Dir[i - 5] == 3) &&
                        (L_Grow_Dir[i] == 4 || L_Grow_Dir[i] == 1))
                    {
                        L_Right_Turn_Up_Point_1[0] = L_Line[i][0];
                        L_Right_Turn_Up_Point_1[1] = L_Line[i][1];
                        L_Right_Turn_Up_Point_Flag_1  = 1;
                        break;
                    }
                }
            }
            break;
        }
        case 3:
        {
            for (i = 5; i < (L_Statics - 4); i++)
            {
                if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((L_Grow_Dir[i + 2] == 4 || L_Grow_Dir[i + 2] == 1 || L_Grow_Dir[i + 2] == -2) && (L_Grow_Dir[i + 5] == 4 || L_Grow_Dir[i + 5] == 1 || L_Grow_Dir[i + 5] == -2) &&
                        (L_Grow_Dir[i + 7] == 4 || L_Grow_Dir[i + 7] == 1 || L_Grow_Dir[i + 7] == -2) &&
                        (L_Grow_Dir[i - 2] == 2 || L_Grow_Dir[i - 2] == 3 || L_Grow_Dir[i - 2] == 4) && (L_Grow_Dir[i - 5] == 2 || L_Grow_Dir[i - 5] == 3 || L_Grow_Dir[i - 5] == 4) &&
                        (L_Grow_Dir[i] == 4 || L_Grow_Dir[i] == 1))
                    {
                        L_Right_Turn_Up_Point_1[0] = L_Line[i][0];
                        L_Right_Turn_Up_Point_1[1] = L_Line[i][1];
                        L_Right_Turn_Up_Point_Flag_1  = 1;
                        break;
                    }
                }
            }
            break;
        }
    }
    if(L_Right_Turn_Up_Point_Flag_1 == 1)
    {
//        L_Right_Turn_Up_Point_Position_1 = i;
//        L_Right_Turn_Up_Point_Angle_1 = Get_Turn_Point_Angle(L_Line[i - 5][0], L_Line[i - 5][1], L_Line[i][0], L_Line[i][1], L_Line[i + 5][0], L_Line[i + 5][1]);
    }
}

/**
* 函数功能：      寻找右转上拐点
* 特殊说明：      通过检测X值的跳变判定拐点的存在
*                 寻找方式为遍历一维数组边线的每个点
* 形  参：        uint8 Region_Start      起始行
*                 uint8 Region_End        截止行
*
* 示例：          Get_L_Right_Turn_Up_Point_2(2, 57);
* 返回值：        无
*/
uint8 L_Right_Turn_Up_Point_2[2] = {0};
uint8 L_Right_Turn_Up_Point_Flag_2 = 0;
uint8 L_Right_Turn_Up_Point_Position_2 = 0;
void Get_L_Right_Turn_Up_Point_2(uint8 Region_Start, uint8 Region_End)
{
    uint8 i = 0;
    L_Right_Turn_Up_Point_2[0] = 0;
    L_Right_Turn_Up_Point_2[1] = 0;
    L_Right_Turn_Up_Point_Position_2 = 0;
    if(Region_Start <= Region_End)
    {
        for(i = Region_Start; i <= Region_End; i++)
        {
            if(L_Border[i] - L_Border[i + 1] >= 6)
            {
                if(L_Border[i - 1] - L_Border[i + 2] >= 6)
                {
                    if(L_Border[i + 1] == 2 && L_Border[i + 2] == 2 && L_Border[i] != 2)
                    {
                        L_Right_Turn_Up_Point_2[0] = L_Border[i];
                        L_Right_Turn_Up_Point_2[1] = i;
                        L_Right_Turn_Up_Point_Flag_2 = 1;
                        L_Right_Turn_Up_Point_Position_2 = i;
                    }
                }
            }
        }
    }
}
uint8 L_Right_Turn_Up_Point[2] = {0};
uint8 L_Right_Turn_Up_Point_Flag = 0;
void Get_L_Right_Turn_Up_Point(void)
{
    L_Right_Turn_Up_Point_Flag = 0;
    Get_L_Right_Turn_Up_Point_1(2);
    if(L_Right_Turn_Up_Point_Flag_1 == 1)
    {
        L_Right_Turn_Up_Point_Flag_1 = 0;
        Get_L_Right_Turn_Up_Point_2(L_Right_Turn_Up_Point_1[1] - 7, L_Right_Turn_Up_Point_1[1] + 7);
        {
            if(L_Right_Turn_Up_Point_Flag_2 == 1)
            {
                L_Right_Turn_Up_Point_Flag_2 = 0;
                L_Right_Turn_Up_Point_Flag = 1;
                L_Right_Turn_Up_Point[0] = L_Right_Turn_Up_Point_1[0];
                L_Right_Turn_Up_Point[1] = L_Right_Turn_Up_Point_1[1];
            }
        }
    }
}

/**
* 函数功能：      寻找上转右拐点
* 特殊说明：      分为三个严格等级，根据每个点的生长方向判断
*                 寻找方式为遍历二维数组边线的每个点
* 形  参：        uint8 Grade      选择判定严格等级，常用2，即中等等级
*
* 示例：          Get_R_Up_Turn_Right_Point_1(2);
* 返回值：        无
*/
uint8 R_Up_Turn_Right_Point_1[2] = {0};
uint8 R_Up_Turn_Right_Point_Flag_1 = 0;
uint8 R_Up_Turn_Right_Point_Position_1 = 0;
float R_Up_Turn_Right_Point_Angle_1 = 0;
void Get_R_Up_Turn_Right_Point_1(uint8 Grade)
{
    uint16 i = 0;
    R_Up_Turn_Right_Point_Flag_1 = 0;
    R_Up_Turn_Right_Point_1[0] = 0;
    R_Up_Turn_Right_Point_1[1] = 0;
    R_Up_Turn_Right_Point_Position_1 = 0;
    R_Up_Turn_Right_Point_Angle_1 = 0;

    switch(Grade)
    {
        case 1:
        {
            for (i = 4; i < (R_Statics - 4); i++)
            {
                if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((R_Grow_Dir[i - 2] == -2 || R_Grow_Dir[i - 2] == 1) && (R_Grow_Dir[i - 4] == -2 || R_Grow_Dir[i - 4] == 1) &&
                        (R_Grow_Dir[i + 2] == 3) && (R_Grow_Dir[i + 4] == 3) && (R_Grow_Dir[i] == 2 || R_Grow_Dir[i] == 3 || R_Grow_Dir[i] == 4))
                    {
                        R_Up_Turn_Right_Point_1[0] = R_Line[i][0];
                        R_Up_Turn_Right_Point_1[1] = R_Line[i][1];
                        R_Up_Turn_Right_Point_Flag_1 = 1;
                        break;
                    }
                }
            }
            break;
        }
        case 2:
        {
            for (i = 4; i < (R_Statics - 4); i++)
            {
                if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((R_Grow_Dir[i - 2] == -2 || R_Grow_Dir[i - 2] == 1) && (R_Grow_Dir[i - 4] == -2 || R_Grow_Dir[i - 4] == 1) &&
                        (R_Grow_Dir[i + 2] == 2 || R_Grow_Dir[i + 2] == 3) && (R_Grow_Dir[i + 4] == 2 || R_Grow_Dir[i + 4] == 3) &&
                        (R_Grow_Dir[i] == 2 || R_Grow_Dir[i] == 3 || R_Grow_Dir[i] == 4))
                    {
                        R_Up_Turn_Right_Point_1[0] = R_Line[i][0];
                        R_Up_Turn_Right_Point_1[1] = R_Line[i][1];
                        R_Up_Turn_Right_Point_Flag_1 = 1;
                        break;
                    }
                }
            }
            break;
        }
        case 3:
        {
            for (i = 4; i < (R_Statics - 4); i++)
            {
                if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((R_Grow_Dir[i - 2] == -2 || R_Grow_Dir[i - 2] == 1 || R_Grow_Dir[i - 2] == 4) && (R_Grow_Dir[i - 4] == -2 || R_Grow_Dir[i - 4] == 1 || R_Grow_Dir[i - 4] == 4) &&
                        (R_Grow_Dir[i + 2] == 2 || R_Grow_Dir[i + 2] == 3 || R_Grow_Dir[i + 2] == 4) && (R_Grow_Dir[i + 4] == 2 || R_Grow_Dir[i + 4] == 3 || R_Grow_Dir[i + 4] == 4) &&
                        (R_Grow_Dir[i] == 2 || R_Grow_Dir[i] == 3 || R_Grow_Dir[i] == 4))
                    {
                        R_Up_Turn_Right_Point_1[0] = R_Line[i][0];
                        R_Up_Turn_Right_Point_1[1] = R_Line[i][1];
                        R_Up_Turn_Right_Point_Flag_1 = 1;
                        break;
                    }
                }
            }
            break;
        }
    }
    if(R_Up_Turn_Right_Point_Flag_1 == 1)
    {
        R_Up_Turn_Right_Point_Position_1 = i;
//        R_Up_Turn_Right_Point_Angle_1 = Get_Turn_Point_Angle(R_Line[i - 5][0], R_Line[i - 5][1], R_Line[i][0], R_Line[i][1], R_Line[i + 5][0], R_Line[i + 5][1]);
    }
}

/**
* 函数功能：      寻找上转右拐点
* 特殊说明：      通过检测X值的跳变判定拐点的存在
*                 寻找方式为遍历一维数组边线的每个点
* 形  参：        uint8 Region_Start      起始行
*                 uint8 Region_End        截止行
*
* 示例：          Get_R_Up_Turn_Right_Point_2(2, 57);
* 返回值：        无
*/
uint8 R_Up_Turn_Right_Point_2[2] = {0};
uint8 R_Up_Turn_Right_Point_Flag_2 = 0;
uint8 R_Up_Turn_Right_Point_Position_2 = 0;
void Get_R_Up_Turn_Right_Point_2(uint8 Region_Start, uint8 Region_End)
{
    uint8 i = 0;
    R_Up_Turn_Right_Point_2[0] = 0;
    R_Up_Turn_Right_Point_2[1] = 0;
    R_Up_Turn_Right_Point_Position_2 = 0;
    if(Region_Start <= Region_End)
    {
        for(i = Region_Start; i <= Region_End; i++)
        {
            if((int8)R_Border[i] - (int8)R_Border[i - 1] <= -6)
            {
                if((int8)R_Border[i + 1] - (int8)R_Border[i - 2] <= -6)
                {
                    if(R_Border[i - 1] == 77 && R_Border[i - 2] == 77 && R_Border[i] != 77)
                    {
                        R_Up_Turn_Right_Point_2[0] = R_Border[i];
                        R_Up_Turn_Right_Point_2[1] = i;
                        R_Up_Turn_Right_Point_Flag_2 = 1;
                        R_Up_Turn_Right_Point_Position_2  = i;
                    }
                }
            }
        }
    }
}
uint8 R_Up_Turn_Right_Point[2] = {0};
uint8 R_Up_Turn_Right_Point_Flag = 0;
void Get_R_Up_Turn_Right_Point(void)
{
    R_Up_Turn_Right_Point_Flag = 0;
    Get_R_Up_Turn_Right_Point_1(2);
    if(R_Up_Turn_Right_Point_Flag_1 == 1)
    {
        R_Up_Turn_Right_Point_Flag_1 = 0;
        Get_R_Up_Turn_Right_Point_2(R_Up_Turn_Right_Point_1[1] - 7, R_Up_Turn_Right_Point_1[1] + 7);
        {
            if(R_Up_Turn_Right_Point_Flag_2 == 1)
            {
                R_Up_Turn_Right_Point_Flag_2 = 0;
                R_Up_Turn_Right_Point_Flag = 1;
                R_Up_Turn_Right_Point[0] = R_Up_Turn_Right_Point_1[0];
                R_Up_Turn_Right_Point[1] = R_Up_Turn_Right_Point_1[1];
            }
        }
    }
}
/**ramp
*/
uint8 R_Left_Turn_Up_Point_1_ramp[2] = {0};
uint8 R_Left_Turn_Up_Point_Flag_1_ramp = 0;
uint8 R_Left_Turn_Up_Point_Position_1_ramp = 0;
//float R_Left_Turn_Up_Point_Angle_1 = 0;
void Get_R_Left_Turn_Up_Point_1_ramp(uint8 Grade)
{
    uint16 i =0;
    R_Left_Turn_Up_Point_Flag_1_ramp = 0;
    R_Left_Turn_Up_Point_1_ramp[0] = 0;
    R_Left_Turn_Up_Point_1_ramp[1] = 0;
    R_Left_Turn_Up_Point_Position_1_ramp = 0;
//    R_Left_Turn_Up_Point_Angle_1 = 0;
    switch(Grade)
    {
        case 1:
        {
            for (i = 5; i < (R_Statics - 4); i++)
            {
                if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((R_Grow_Dir[i + 2] == -2 || R_Grow_Dir[i + 2] == 1) && (R_Grow_Dir[i + 5] == -2 || R_Grow_Dir[i + 5] == 1) && (R_Grow_Dir[i + 7] == -2 || R_Grow_Dir[i + 7] == 1) &&
                        (R_Grow_Dir[i - 2] == -3) && (R_Grow_Dir[i - 5] == -3) && (R_Grow_Dir[i] == -2 || R_Grow_Dir[i] == 1))
                    {
                        R_Left_Turn_Up_Point_1_ramp[0] = R_Line[i][0];
                        R_Left_Turn_Up_Point_1_ramp[1] = R_Line[i][1];
                        R_Left_Turn_Up_Point_Flag_1_ramp = 1;
                        break;
                    }
                }
            }
            break;
        }
        case 2:
        {
            for (i = 5; i < (R_Statics - 4); i++)
            {
                if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((R_Grow_Dir[i + 2] == -2 || R_Grow_Dir[i + 2] == 1) && (R_Grow_Dir[i + 5] == -2 || R_Grow_Dir[i + 5] == 1) && (R_Grow_Dir[i + 7] == -2 || R_Grow_Dir[i + 7] == 1) &&
                        (R_Grow_Dir[i - 2] == -3 || R_Grow_Dir[i - 2] == -4) && (R_Grow_Dir[i - 3] == -3 || R_Grow_Dir[i - 3] == -4) && (R_Grow_Dir[i] == -2 || R_Grow_Dir[i] == 1))
                    {
                        R_Left_Turn_Up_Point_1_ramp[0] = R_Line[i][0];
                        R_Left_Turn_Up_Point_1_ramp[1] = R_Line[i][1];
                        R_Left_Turn_Up_Point_Flag_1_ramp = 1;;
                        break;
                    }
                }
            }
            break;
        }
        case 3:
        {
            for (i = 5; i < (R_Statics - 4); i++)
            {
                if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((R_Grow_Dir[i + 2] == -2 || R_Grow_Dir[i + 2] == 1 || R_Grow_Dir[i + 2] == 4) && (R_Grow_Dir[i + 5] == -2 || R_Grow_Dir[i + 5] == 1 || R_Grow_Dir[i + 5] == 4) &&
                        (R_Grow_Dir[i + 7] == -2 || R_Grow_Dir[i + 7] == 1 || R_Grow_Dir[i + 7] == 4) &&
                        (R_Grow_Dir[i - 2] == -3 || R_Grow_Dir[i - 2] == -4) && (R_Grow_Dir[i - 5] == -3 || R_Grow_Dir[i - 5] == -4) && (R_Grow_Dir[i] == -2 || R_Grow_Dir[i] == 1))
                    {
                        R_Left_Turn_Up_Point_1_ramp[0] = R_Line[i][0];
                        R_Left_Turn_Up_Point_1_ramp[1] = R_Line[i][1];
                        R_Left_Turn_Up_Point_Flag_1_ramp = 1;;
                        break;
                    }
                }
            }
            break;
        }
    }
    if(R_Left_Turn_Up_Point_Flag_1_ramp == 1)
    {
//        R_Left_Turn_Up_Point_Position_1_ramp = i;
//        R_Left_Turn_Up_Point_Angle_1 = Get_Turn_Point_Angle(R_Line[i - 5][0], R_Line[i - 5][1], R_Line[i][0], R_Line[i][1], R_Line[i + 5][0], R_Line[i + 5][1]);
    }
}
/**
* 函数功能：      寻找左转上拐点
* 特殊说明：      分为三个严格等级，根据每个点的生长方向判断
*                 寻找方式为遍历二维数组边线的每个点
* 形  参：        uint8 Grade      选择判定严格等级，常用2，即中等等级
*
* 示例：          Get_R_Left_Turn_Up_Point_1(2);
* 返回值：        无
*/
uint8 R_Left_Turn_Up_Point_1[2] = {0};
uint8 R_Left_Turn_Up_Point_Flag_1 = 0;
uint8 R_Left_Turn_Up_Point_Position_1 = 0;
float R_Left_Turn_Up_Point_Angle_1 = 0;
void Get_R_Left_Turn_Up_Point_1(uint8 Grade)
{
    uint16 i =0;
    R_Left_Turn_Up_Point_Flag_1 = 0;
    R_Left_Turn_Up_Point_1[0] = 0;
    R_Left_Turn_Up_Point_1[1] = 0;
    R_Left_Turn_Up_Point_Position_1 = 0;
    R_Left_Turn_Up_Point_Angle_1 = 0;
    switch(Grade)
    {
        case 1:
        {
            for (i = 5; i < (R_Statics - 4); i++)
            {
                if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((R_Grow_Dir[i + 2] == -2 || R_Grow_Dir[i + 2] == 1) && (R_Grow_Dir[i + 5] == -2 || R_Grow_Dir[i + 5] == 1) && (R_Grow_Dir[i + 7] == -2 || R_Grow_Dir[i + 7] == 1) &&
                        (R_Grow_Dir[i - 2] == -3) && (R_Grow_Dir[i - 5] == -3) && (R_Grow_Dir[i] == -2 || R_Grow_Dir[i] == 1))
                    {
                        R_Left_Turn_Up_Point_1[0] = R_Line[i][0];
                        R_Left_Turn_Up_Point_1[1] = R_Line[i][1];
                        R_Left_Turn_Up_Point_Flag_1 = 1;
                        break;
                    }
                }
            }
            break;
        }
        case 2:
        {
            for (i = 5; i < (R_Statics - 4); i++)
            {
                if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((R_Grow_Dir[i + 2] == -2 || R_Grow_Dir[i + 2] == 1) && (R_Grow_Dir[i + 5] == -2 || R_Grow_Dir[i + 5] == 1) && (R_Grow_Dir[i + 7] == -2 || R_Grow_Dir[i + 7] == 1) &&
                        (R_Grow_Dir[i - 2] == -3 || R_Grow_Dir[i - 2] == -4) && (R_Grow_Dir[i - 5] == -3 || R_Grow_Dir[i - 5] == -4) && (R_Grow_Dir[i] == -2 || R_Grow_Dir[i] == 1))
                    {
                        R_Left_Turn_Up_Point_1[0] = R_Line[i][0];
                        R_Left_Turn_Up_Point_1[1] = R_Line[i][1];
                        R_Left_Turn_Up_Point_Flag_1 = 1;;
                        break;
                    }
                }
            }
            break;
        }
        case 3:
        {
            for (i = 5; i < (R_Statics - 4); i++)
            {
                if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((R_Grow_Dir[i + 2] == -2 || R_Grow_Dir[i + 2] == 1 || R_Grow_Dir[i + 2] == 4) && (R_Grow_Dir[i + 5] == -2 || R_Grow_Dir[i + 5] == 1 || R_Grow_Dir[i + 5] == 4) &&
                        (R_Grow_Dir[i + 7] == -2 || R_Grow_Dir[i + 7] == 1 || R_Grow_Dir[i + 7] == 4) &&
                        (R_Grow_Dir[i - 2] == -3 || R_Grow_Dir[i - 2] == -4) && (R_Grow_Dir[i - 5] == -3 || R_Grow_Dir[i - 5] == -4) && (R_Grow_Dir[i] == -2 || R_Grow_Dir[i] == 1))
                    {
                        R_Left_Turn_Up_Point_1[0] = R_Line[i][0];
                        R_Left_Turn_Up_Point_1[1] = R_Line[i][1];
                        R_Left_Turn_Up_Point_Flag_1 = 1;;
                        break;
                    }
                }
            }
            break;
        }
    }
    if(R_Left_Turn_Up_Point_Flag_1 == 1)
    {
        R_Left_Turn_Up_Point_Position_1 = i;
//        R_Left_Turn_Up_Point_Angle_1 = Get_Turn_Point_Angle(R_Line[i - 5][0], R_Line[i - 5][1], R_Line[i][0], R_Line[i][1], R_Line[i + 5][0], R_Line[i + 5][1]);
    }
}

/**
* 函数功能：      寻找左转上拐点
* 特殊说明：      通过检测X值的跳变判定拐点的存在
*                 寻找方式为遍历一维数组边线的每个点
* 形  参：        uint8 Region_Start      起始行
*                 uint8 Region_End        截止行
*
* 示例：          Get_R_Left_Turn_Up_Point_2(2, 57);
* 返回值：        无
*/
uint8 R_Left_Turn_Up_Point_2[2] = {0};
uint8 R_Left_Turn_Up_Point_Flag_2 = 0;
uint8 R_Left_Turn_Up_Point_Position_2 = 0;
void Get_R_Left_Turn_Up_Point_2(uint8 Region_Start, uint8 Region_End)
{
    uint8 i = 0;
    if(Region_Start <= Region_End)
    {
        for(i = Region_Start; i <= Region_End; i++)
        {
            if((int8)R_Border[i] - (int8)R_Border[i + 1] <= -6)
            {
                if((int8)R_Border[i - 1] - (int8)R_Border[i + 2] <= -6)
                {
                    if(R_Border[i + 1] == 77 && R_Border[i + 2] == 77 && R_Border[i] != 77)
                    {
                        R_Left_Turn_Up_Point_2[0] = R_Border[i];
                        R_Left_Turn_Up_Point_2[1] = i;
                        R_Left_Turn_Up_Point_Flag_2 = 1;
                        R_Left_Turn_Up_Point_Position_2 = i;
                    }
                }
            }
        }
    }
}
uint8 R_Left_Turn_Up_Point[2] = {0};
uint8 R_Left_Turn_Up_Point_Flag = 0;
void Get_R_Left_Turn_Up_Point(void)
{
    R_Left_Turn_Up_Point_Flag = 0;
    Get_R_Left_Turn_Up_Point_1(2);
    if(R_Left_Turn_Up_Point_Flag_1 == 1)
    {
        R_Left_Turn_Up_Point_Flag_1 = 0;
        Get_R_Left_Turn_Up_Point_2(R_Left_Turn_Up_Point_1[1] - 7, R_Left_Turn_Up_Point_1[1] + 7);
        {
            if(R_Left_Turn_Up_Point_Flag_2 == 1)
            {
                R_Left_Turn_Up_Point_Flag_2 = 0;
                R_Left_Turn_Up_Point_Flag = 1;
                R_Left_Turn_Up_Point[0] = R_Left_Turn_Up_Point_1[0];
                R_Left_Turn_Up_Point[1] = R_Left_Turn_Up_Point_1[1];
            }
        }
    }
}
/****************************************************************************************/
/**
* 函数功能：      寻找上转左拐点(右侧)
* 特殊说明：      分为三个严格等级，根据每个点的生长方向判断
*                 寻找方式为遍历二维数组边线的每个点
* 形  参：        uint8 Grade      选择判定严格等级，常用2，即中等等级
*
* 示例：          Get_L_Up_Turn_Left_Point_1(2);
* 返回值：        无
*/
uint8 R_Up_Turn_Left_Point_1[2] = {0};      //存储上转左拐点的坐标
uint8 R_Up_Turn_Left_Point_1_2[2] = {0};      //存储上转左拐点的坐标
uint8 R_Up_Turn_Left_Point_Flag_1 = 0;      //上转左拐点存在标志位，找到时置1
uint8 R_Up_Turn_Left_Point_Position_1 = 0;  //记录位置，即是二维数组中的第几个点
float R_Up_Turn_Left_Point_Angle_1 = 0;     //记录找到的拐点的角度（逆透视求取）
void Get_R_Up_Turn_Left_Point_1(uint8 Grade, uint16 Start_Point)
{
    uint16 i = 0;
    R_Up_Turn_Left_Point_Flag_1 = 0;
    R_Up_Turn_Left_Point_1[0] = 0;
    R_Up_Turn_Left_Point_1[1] = 0;
    R_Up_Turn_Left_Point_1_2[0] = 0;
    R_Up_Turn_Left_Point_1_2[1] = 0;
    R_Up_Turn_Left_Point_Position_1 = 0;
    R_Up_Turn_Left_Point_Angle_1 = 0;       //将各个参数清零
//    uint8 temp = 0;
    switch(Grade)
    {
        case 1:     //严格判断
        {
            for (i = Start_Point; i < (R_Statics - 4); i++)
            {
                if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((R_Grow_Dir[i - 2] == -2 || R_Grow_Dir[i - 2] == 1) && (R_Grow_Dir[i - 4] == -2 || R_Grow_Dir[i - 4] == 1) &&
                        (R_Grow_Dir[i + 2] == -3) && (R_Grow_Dir[i + 4] == -3) && (R_Grow_Dir[i + 3] == -3) && (R_Grow_Dir[i] == -2 || R_Grow_Dir[i] == -3))
                    {
                        R_Up_Turn_Left_Point_1[0] = R_Line[i][0];
                        R_Up_Turn_Left_Point_1[1] = R_Line[i][1];
                        R_Up_Turn_Left_Point_Flag_1  = 1;
                        break;
                    }
                }
            }
            break;
        }
        case 2:     //中等判断
        {
            for (i = Start_Point; i < (R_Statics - 4); i++)
            {
                if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((R_Grow_Dir[i - 2] == -2 || R_Grow_Dir[i - 2] == 1) && (R_Grow_Dir[i - 4] == -2 || R_Grow_Dir[i - 4] == 1) &&
                        (R_Grow_Dir[i + 2] == -3 || R_Grow_Dir[i + 2] == -4) && (R_Grow_Dir[i + 4] == -3 || R_Grow_Dir[i + 4] == -4) &&
                        (R_Grow_Dir[i] == -2 || R_Grow_Dir[i] == -3 || R_Grow_Dir[i] == -4))
                    {
                        if(R_Up_Turn_Left_Point_Flag_1 == 0)
                        {
                            R_Up_Turn_Left_Point_1[0] = R_Line[i][0];
                            R_Up_Turn_Left_Point_1[1] = R_Line[i][1];
                            R_Up_Turn_Left_Point_Flag_1  = 1;
                            break;
                        }
//                        else if(R_Up_Turn_Left_Point_Flag_1 == 1)
//                        {
//                            R_Up_Turn_Left_Point_1_2[0] = R_Line[i][0];
//                            R_Up_Turn_Left_Point_1_2[1] = R_Line[i][1];
//                            R_Up_Turn_Left_Point_Flag_1  = 2;
//                            break;
//                        }
                    }
                }
            }
            break;
        }
        case 3:     //宽松判断
        {
            for (i = Start_Point; i < (R_Statics - 4); i++)
            {
                if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((R_Grow_Dir[i - 2] == 4 || R_Grow_Dir[i - 2] == 1 || R_Grow_Dir[i - 2] == -2) && (R_Grow_Dir[i - 4] == 4 || R_Grow_Dir[i - 4] == 1 || R_Grow_Dir[i - 2] == -2) &&
                        (R_Grow_Dir[i + 2] == -3 || R_Grow_Dir[i + 2] == -4 || R_Grow_Dir[i + 2] == -2) && (R_Grow_Dir[i + 4] == -3 || R_Grow_Dir[i + 4] == -4 || R_Grow_Dir[i + 4] == -2) &&
                        (R_Grow_Dir[i] == -2 || R_Grow_Dir[i] == -3 || R_Grow_Dir[i] == -4))
                    {
                        R_Up_Turn_Left_Point_1[0] = R_Line[i][0];
                        R_Up_Turn_Left_Point_1[1] = R_Line[i][1];
                        R_Up_Turn_Left_Point_Flag_1  = 1;
                        break;
                    }
                }
            }
            break;
        }
    }
//    if( R_Up_Turn_Left_Point_Flag_1 == 2 && R_Up_Turn_Left_Point_1[1] > R_Up_Turn_Left_Point_1_2[1])
//    {
//        temp = R_Up_Turn_Left_Point_1[0];
//        R_Up_Turn_Left_Point_1[0] = R_Up_Turn_Left_Point_1_2[0];
//        R_Up_Turn_Left_Point_1_2[0] = temp;
//        temp = R_Up_Turn_Left_Point_1[1];
//        R_Up_Turn_Left_Point_1[1] = R_Up_Turn_Left_Point_1_2[1];
//        R_Up_Turn_Left_Point_1_2[1] = temp;
//    }
    if(R_Up_Turn_Left_Point_Flag_1 == 1)
    {
        if(    (40.0f < R_Up_Turn_Left_Point_1[0]) &&
                (75.0f > R_Up_Turn_Left_Point_1[0]) &&
                (55.0f > R_Up_Turn_Left_Point_1[1]) &&
                (5.0f < R_Up_Turn_Left_Point_1[1]) )
        {
            R_Up_Turn_Left_Point_Position_1 = i;    //记录拐点的位置
            R_Up_Turn_Left_Point_Flag_1 = 1;
        }
        else
        {
            R_Up_Turn_Left_Point_Flag_1 = 0;
            R_Up_Turn_Left_Point_1[0] = 0;
            R_Up_Turn_Left_Point_1[1] = 0;
        }
//        R_Up_Turn_Left_Point_Position_1 = i;    //记录拐点的位置
        R_Up_Turn_Left_Point_Angle_1 = Get_Angle_Points(R_Up_Turn_Left_Point_Position_1, 5, R_Line);    //逆透视求取拐点角度，后续补充
    }
}
/**
* 函数功能：      寻找右转上拐点(右侧)
* 特殊说明：      分为三个严格等级，根据每个点的生长方向判断
*                 寻找方式为遍历二维数组边线的每个点
* 形  参：        uint8 Grade      选择判定严格等级，常用2，即中等等级
*
* 示例：          Get_L_Right_Turn_Up_Point_1(2);
* 返回值：        无
*/
uint8 R_Right_Turn_Up_Point_1[2] = {0};
uint8 R_Right_Turn_Up_Point_1_2[2] = {0};
uint8 R_Right_Turn_Up_Point_Flag_1 = 0;
uint8 R_Right_Turn_Up_Point_Position_1 = 0;
float R_Right_Turn_Up_Point_Angle_1 = 0;
void Get_R_Right_Turn_Up_Point_1(uint8 Grade, uint16 Start_Point)
{
    uint16 i = 0;
    R_Right_Turn_Up_Point_Flag_1 = 0;
    R_Right_Turn_Up_Point_1[0] = 0;
    R_Right_Turn_Up_Point_1[1] = 0;
    R_Right_Turn_Up_Point_1_2[0] = 0;
    R_Right_Turn_Up_Point_1_2[1] = 0;
    R_Right_Turn_Up_Point_Position_1 = 0;
    R_Right_Turn_Up_Point_Angle_1 = 0;
//    uint8 temp = 0;
    switch(Grade)
    {
        case 1:
        {
            for (i = Start_Point; i < (R_Statics - 4); i++)
            {
                if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((R_Grow_Dir[i + 2] == -2 || R_Grow_Dir[i + 2] == 1) && (R_Grow_Dir[i + 5] == -2 || R_Grow_Dir[i + 5] == 1) && (R_Grow_Dir[i + 7] == -2 || R_Grow_Dir[i + 7] == 1) &&
                        (R_Grow_Dir[i - 2] == 3) && (R_Grow_Dir[i - 3] == 3) && (R_Grow_Dir[i - 4] == 3) && (R_Grow_Dir[i] == -2 || R_Grow_Dir[i] == 1))
                    {
                        R_Right_Turn_Up_Point_1[0] = R_Line[i][0];
                        R_Right_Turn_Up_Point_1[1] = R_Line[i][1];
                        R_Right_Turn_Up_Point_Flag_1  = 1;
                        break;
                    }
                }
            }
            break;
        }
        case 2:
        {
            for (i = Start_Point; i < (R_Statics - 4); i++)
            {
                if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((R_Grow_Dir[i + 2] == -2 || R_Grow_Dir[i + 2] == 1) && (R_Grow_Dir[i + 5] == -2 || R_Grow_Dir[i + 5] == 1) && (R_Grow_Dir[i + 7] == -2 || R_Grow_Dir[i + 7] == 1) &&
                        (R_Grow_Dir[i - 2] == 4 || R_Grow_Dir[i - 2] == 3) && (R_Grow_Dir[i - 5] == 4 || R_Grow_Dir[i - 5] == 3) &&
                        (R_Grow_Dir[i] == -2 || R_Grow_Dir[i] == 1))
                    {
                        if(R_Right_Turn_Up_Point_Flag_1 == 0)
                        {
                            R_Right_Turn_Up_Point_Flag_1 = 1;
                            R_Right_Turn_Up_Point_1[0] = R_Line[i][0];
                            R_Right_Turn_Up_Point_1[1] = R_Line[i][1];
                        }
//                        else if(R_Right_Turn_Up_Point_Flag_1 == 1)
//                        {
//                            R_Right_Turn_Up_Point_Flag_1 = 2;
//                            R_Right_Turn_Up_Point_1_2[0] = R_Line[i][0];
//                            R_Right_Turn_Up_Point_1_2[1] = R_Line[i][1];
//                            break;
//                        }
                    }
                }
            }
            break;
        }
        case 3:
        {
            for (i = Start_Point; i < (R_Statics - 4); i++)
            {
                if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((R_Grow_Dir[i + 2] == 4 || R_Grow_Dir[i + 2] == 1 || R_Grow_Dir[i + 2] == -2) && (R_Grow_Dir[i + 5] == 4 || R_Grow_Dir[i + 5] == 1 || R_Grow_Dir[i + 5] == -2) &&
                        (R_Grow_Dir[i + 7] == 4 || R_Grow_Dir[i + 7] == 1 || R_Grow_Dir[i + 7] == -2) &&
                        (R_Grow_Dir[i - 2] == 2 || R_Grow_Dir[i - 2] == 3 || R_Grow_Dir[i - 2] == 4) && (R_Grow_Dir[i - 5] == 2 || R_Grow_Dir[i - 5] == 3 || R_Grow_Dir[i - 5] == 4) &&
                        (R_Grow_Dir[i] == 4 || R_Grow_Dir[i] == 1))
                    {
                        R_Right_Turn_Up_Point_1[0] = R_Line[i][0];
                        R_Right_Turn_Up_Point_1[1] = R_Line[i][1];
                        R_Right_Turn_Up_Point_Flag_1  = 1;
                        break;
                    }
                }
            }
            break;
        }
    }
//    if( R_Right_Turn_Up_Point_Flag_1 == 2 && R_Right_Turn_Up_Point_1[1] > R_Right_Turn_Up_Point_1_2[1])
//    {
//        temp = R_Right_Turn_Up_Point_1[0];
//        R_Right_Turn_Up_Point_1[0] = R_Right_Turn_Up_Point_1_2[0];
//        R_Right_Turn_Up_Point_1_2[0] = temp;
//        temp = R_Right_Turn_Up_Point_1[1];
//        R_Right_Turn_Up_Point_1[1] = R_Right_Turn_Up_Point_1_2[1];
//        R_Right_Turn_Up_Point_1_2[1] = temp;
//    }
    if(R_Right_Turn_Up_Point_Flag_1 == 1)
    {
        if(    (40.0f < R_Right_Turn_Up_Point_1[0]) &&
                (75.0f > R_Right_Turn_Up_Point_1[0]) &&
                (55.0f > R_Right_Turn_Up_Point_1[1]) &&
                (5.0f < R_Right_Turn_Up_Point_1[1]) )
        {
            R_Right_Turn_Up_Point_Position_1 = i;    //记录拐点的位置
            R_Right_Turn_Up_Point_Flag_1 = 1;
        }
        else
        {
            R_Right_Turn_Up_Point_Flag_1 = 0;
            R_Right_Turn_Up_Point_1[0] = 0;
            R_Right_Turn_Up_Point_1[1] = 0;
        }
//        R_Right_Turn_Up_Point_Position_1 = i;
//        L_Right_Turn_Up_Point_Angle_1 = Get_Turn_Point_Angle(L_Line[i - 5][0], L_Line[i - 5][1], L_Line[i][0], L_Line[i][1], L_Line[i + 5][0], L_Line[i + 5][1]);
    }
}
/**
* 函数功能：      寻找上转右拐点(左侧)
* 特殊说明：      分为三个严格等级，根据每个点的生长方向判断
*                 寻找方式为遍历二维数组边线的每个点
* 形  参：        uint8 Grade      选择判定严格等级，常用2，即中等等级
*
* 示例：          Get_R_Up_Turn_Right_Point_1(2);
* 返回值：        无
*/
uint8 L_Up_Turn_Right_Point_1[2] = {0};
uint8 L_Up_Turn_Right_Point_1_2[2] = {0};
uint8 L_Up_Turn_Right_Point_Flag_1 = 0;
uint8 L_Up_Turn_Right_Point_Position_1 = 0;
float L_Up_Turn_Right_Point_Angle_1 = 0;
void Get_L_Up_Turn_Right_Point_1(uint8 Grade, uint16 Start_Point)
{
    uint16 i = 0;
    L_Up_Turn_Right_Point_Flag_1 = 0;
    L_Up_Turn_Right_Point_1[0] = 0;
    L_Up_Turn_Right_Point_1[1] = 0;
    L_Up_Turn_Right_Point_1_2[0] = 0;
    L_Up_Turn_Right_Point_1_2[1] = 0;
    L_Up_Turn_Right_Point_Position_1 = 0;
    L_Up_Turn_Right_Point_Angle_1 = 0;

    switch(Grade)
    {
        case 1:
        {
            for (i = Start_Point; i < (L_Statics - 4); i++)
            {
                if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((L_Grow_Dir[i - 2] == 4 || L_Grow_Dir[i - 2] == 1) && (L_Grow_Dir[i - 4] == 4 || L_Grow_Dir[i - 4] == 1) &&
                        (L_Grow_Dir[i + 2] == 3) && (L_Grow_Dir[i + 4] == 3) &&
                        (L_Grow_Dir[i] == 3 || L_Grow_Dir[i] == 4))
                    {
                        L_Up_Turn_Right_Point_1[0] = L_Line[i][0];
                        L_Up_Turn_Right_Point_1[1] = L_Line[i][1];
                        L_Up_Turn_Right_Point_Flag_1 = 1;
                        break;
                    }
                }
            }
            break;
        }
        case 2:
        {
            for (i = Start_Point; i < (L_Statics - 4); i++)
            {
                if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((L_Grow_Dir[i - 2] == 4 || L_Grow_Dir[i - 2] == 1) && (L_Grow_Dir[i - 4] == 4 || L_Grow_Dir[i - 4] == 1) &&
                        (L_Grow_Dir[i + 2] == 3 || L_Grow_Dir[i + 2] == 2) && (L_Grow_Dir[i + 4] == 3 || L_Grow_Dir[i + 2] == 2) &&
                        (L_Grow_Dir[i] == 2 || L_Grow_Dir[i] == 3))
                    {
                        if(L_Up_Turn_Right_Point_Flag_1 == 0)
                        {
                            L_Up_Turn_Right_Point_Flag_1 = 1;
                            L_Up_Turn_Right_Point_1[0] = L_Line[i][0];
                            L_Up_Turn_Right_Point_1[1] = L_Line[i][1];
                            break;
                        }
//                        else if(L_Up_Turn_Right_Point_Flag_1 == 1)
//                        {
//                            L_Up_Turn_Right_Point_Flag_1 = 2;
//                            L_Up_Turn_Right_Point_1_2[0] = L_Line[i][0];
//                            L_Up_Turn_Right_Point_1_2[1] = L_Line[i][1];
//                            break;
//                        }
                    }
                }
            }
            break;
        }
        case 3:
        {
            for (i = Start_Point; i < (L_Statics - 4); i++)
            {
                if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((L_Grow_Dir[i - 2] == -2 || L_Grow_Dir[i - 2] == 1 || L_Grow_Dir[i - 2] == 4) && (L_Grow_Dir[i - 4] == -2 || L_Grow_Dir[i - 4] == 1 || L_Grow_Dir[i - 4] == 4) &&
                        (L_Grow_Dir[i + 2] == 2 || L_Grow_Dir[i + 2] == 3 || L_Grow_Dir[i + 2] == 4) && (L_Grow_Dir[i + 4] == 2 || L_Grow_Dir[i + 4] == 3 || L_Grow_Dir[i + 4] == 4) &&
                        (L_Grow_Dir[i] == 2 || L_Grow_Dir[i] == 3 || L_Grow_Dir[i] == 4))
                    {
                        L_Up_Turn_Right_Point_1[0] = L_Line[i][0];
                        L_Up_Turn_Right_Point_1[1] = L_Line[i][1];
                        L_Up_Turn_Right_Point_Flag_1 = 1;
                        break;
                    }
                }
            }
            break;
        }
    }

    if(L_Up_Turn_Right_Point_Flag_1 == 1)
    {
        if(    (40.0f > L_Up_Turn_Right_Point_1[0]) &&
                (5.0f < L_Up_Turn_Right_Point_1[0]) &&
                (55.0f > L_Up_Turn_Right_Point_1[1]) &&
                (5.0f < L_Up_Turn_Right_Point_1[1]) )
        {
            L_Up_Turn_Right_Point_Position_1 = i;
            L_Up_Turn_Right_Point_Flag_1 = 1;
        }
        else
        {
            L_Up_Turn_Right_Point_Flag_1 = 0;
            L_Up_Turn_Right_Point_1[0] = 0;
            L_Up_Turn_Right_Point_1[1] = 0;
        }
//        L_Up_Turn_Right_Point_Position_1 = i;
//        L_Up_Turn_Right_Point_Angle_1 = Get_Angle_Points(R_Up_Turn_Left_Point_Position_1, 5, L_Line);;
    }
}
/**
* 函数功能：      寻找左转上拐点(左侧)
* 特殊说明：      分为三个严格等级，根据每个点的生长方向判断
*                 寻找方式为遍历二维数组边线的每个点
* 形  参：        uint8 Grade      选择判定严格等级，常用2，即中等等级
*
* 示例：          Get_R_Left_Turn_Up_Point_1(2);
* 返回值：        无
*/
uint8 L_Left_Turn_Up_Point_1[2] = {0};
uint8 L_Left_Turn_Up_Point_1_2[2] = {0};
uint8 L_Left_Turn_Up_Point_Flag_1 = 0;
uint8 L_Left_Turn_Up_Point_Position_1 = 0;
float L_Left_Turn_Up_Point_Angle_1 = 0;
void Get_L_Left_Turn_Up_Point_1(uint8 Grade, uint16 Start_Point)
{
    uint16 i =0;
    L_Left_Turn_Up_Point_Flag_1 = 0;
    L_Left_Turn_Up_Point_1[0] = 0;
    L_Left_Turn_Up_Point_1[1] = 0;
    L_Left_Turn_Up_Point_1_2[0] = 0;
    L_Left_Turn_Up_Point_1_2[1] = 0;
    L_Left_Turn_Up_Point_Position_1 = 0;
    L_Left_Turn_Up_Point_Angle_1 = 0;
    switch(Grade)
    {
        case 1:
        {
            for (i = Start_Point; i < (L_Statics - 4); i++)
            {
                if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((L_Grow_Dir[i + 2] == 4 || L_Grow_Dir[i + 2] == 1) && (L_Grow_Dir[i + 5] == 4 || L_Grow_Dir[i + 5] == 1) && (L_Grow_Dir[i + 7] == 4 || L_Grow_Dir[i + 7] == 1) &&
                        (L_Grow_Dir[i - 2] == -3) && (L_Grow_Dir[i - 3] == -3) && (L_Grow_Dir[i - 4] == -3) && (L_Grow_Dir[i] == 4 || L_Grow_Dir[i] == 1))
                    {
                        L_Left_Turn_Up_Point_1[0] = L_Line[i][0];
                        L_Left_Turn_Up_Point_1[1] = L_Line[i][1];
                        L_Left_Turn_Up_Point_Flag_1 = 1;
                        break;
                    }
                }
            }
            break;
        }
        case 2:
        {
            for (i = Start_Point; i < (L_Statics - 4); i++)
            {
                if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((L_Grow_Dir[i + 2] == 4 || L_Grow_Dir[i + 2] == 1) && (L_Grow_Dir[i + 5] == 4 || L_Grow_Dir[i + 5] == 1) && (L_Grow_Dir[i + 7] == 4 || L_Grow_Dir[i + 7] == 1) &&
                        (L_Grow_Dir[i - 2] == -3 || L_Grow_Dir[i - 2] == -2) && (L_Grow_Dir[i - 5] == -3 || L_Grow_Dir[i - 5] == -2) &&
                        (L_Grow_Dir[i] == 4 || L_Grow_Dir[i] == 1))
                    {
                        L_Left_Turn_Up_Point_1[0] = L_Line[i][0];
                        L_Left_Turn_Up_Point_1[1] = L_Line[i][1];
                        L_Left_Turn_Up_Point_Flag_1 = 1;;
                        if(L_Left_Turn_Up_Point_Flag_1 == 0)
                        {
                            L_Left_Turn_Up_Point_1[0] = R_Line[i][0];
                            L_Left_Turn_Up_Point_1[1] = R_Line[i][1];
                            L_Left_Turn_Up_Point_Flag_1  = 1;
                            break;
                        }
//                        else if(L_Left_Turn_Up_Point_Flag_1 == 1)
//                        {
//                            L_Left_Turn_Up_Point_1_2[0] = R_Line[i][0];
//                            L_Left_Turn_Up_Point_1_2[1] = R_Line[i][1];
//                            L_Left_Turn_Up_Point_Flag_1  = 2;
//                            break;
//                        }
                    }
                }
            }
            break;
        }
        case 3:
        {
            for (i = Start_Point; i < (L_Statics - 4); i++)
            {
                if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 2)))
                {
                    if ((L_Grow_Dir[i + 2] == -2 || L_Grow_Dir[i + 2] == 1 || L_Grow_Dir[i + 2] == 4) && (L_Grow_Dir[i + 5] == -2 || L_Grow_Dir[i + 5] == 1 || L_Grow_Dir[i + 5] == 4) &&
                        (L_Grow_Dir[i + 7] == -2 || L_Grow_Dir[i + 7] == 1 || L_Grow_Dir[i + 7] == 4) &&
                        (L_Grow_Dir[i - 2] == -3 || L_Grow_Dir[i - 2] == -4) && (L_Grow_Dir[i - 5] == -3 || L_Grow_Dir[i - 5] == -4) && (L_Grow_Dir[i] == -2 || L_Grow_Dir[i] == 1))
                    {
                        L_Left_Turn_Up_Point_1[0] = L_Line[i][0];
                        L_Left_Turn_Up_Point_1[1] = L_Line[i][1];
                        L_Left_Turn_Up_Point_Flag_1 = 1;;
                        break;
                    }
                }
            }
            break;
        }
    }
    if(L_Left_Turn_Up_Point_Flag_1 == 1)
    {
        if(    (40.0f > L_Left_Turn_Up_Point_1[0]) &&
                (5.0f < L_Left_Turn_Up_Point_1[0]) &&
                (55.0f > L_Left_Turn_Up_Point_1[1]) &&
                (5.0f < L_Left_Turn_Up_Point_1[1]) )
        {
            L_Left_Turn_Up_Point_Position_1 = i;
            L_Left_Turn_Up_Point_Flag_1 = 1;
        }
        else
        {
            L_Left_Turn_Up_Point_Flag_1 = 0;
            L_Left_Turn_Up_Point_1[0] = 0;
            L_Left_Turn_Up_Point_1[1] = 0;
        }
//        L_Left_Turn_Up_Point_Position_1 = i;
//        R_Left_Turn_Up_Point_Angle_1 = Get_Turn_Point_Angle(R_Line[i - 5][0], R_Line[i - 5][1], R_Line[i][0], R_Line[i][1], R_Line[i + 5][0], R_Line[i + 5][1]);
    }
}
/****************************************************************************************/

/**
* 函数功能：      寻找左侧圆弧拐点
* 特殊说明：      无
* 形  参：        无
*
* 示例：          Get_L_Arc_Turn_Point();
* 返回值：        无
*/
uint8 L_Arc_Turn_Point[3][2] = {{0}};   //最多找三个就足够使用
uint8 L_Arc_Turn_Point_Flag = 0;    //找到一个就挂出标志位
uint8 L_Arc_Turn_Point_Num = 0;     //记录找到的个数
void Get_L_Arc_Turn_Point(void)
{
    uint8 i = 0;
    L_Arc_Turn_Point_Flag = 0;

    for(i = 0; i < 3; i++)      //每次调用先清零
    {
        L_Arc_Turn_Point_Num = 0;
        L_Arc_Turn_Point[i][0] = 0;
        L_Arc_Turn_Point[i][1] = 0;
    }

    for(i =7; i < (L_Statics - 5); i++)
    {
        if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 7)))
        {
            if((L_Line[i][0] > L_Line[i - 7][0]) && (L_Line[i][0] > L_Line[i - 4][0]) && (L_Line[i][0] >= L_Line[i - 1][0]) &&
               (L_Line[i][0] >= L_Line[i + 1][0]) && (L_Line[i][0] > L_Line[i + 3][0]) && (L_Line[i][0] > L_Line[i + 5][0]) &&
               (L_Grow_Dir[i + 1] == -2 || L_Grow_Dir[i + 1] == 1) && (L_Grow_Dir[i + 3] == -2 || L_Grow_Dir[i + 3] == 1) &&
               (L_Grow_Dir[i - 1] == 1 || L_Grow_Dir[i + 1] == 4) && (L_Grow_Dir[i - 3] == 1 || L_Grow_Dir[i - 3] == 4))                    //左侧内凹
            {
                L_Arc_Turn_Point[L_Arc_Turn_Point_Num][0] = L_Line[i][0];
                L_Arc_Turn_Point[L_Arc_Turn_Point_Num][1] = L_Line[i][1];
                L_Arc_Turn_Point_Flag = 1;
                L_Arc_Turn_Point_Num ++;
                i += 15;    //每个圆弧拐点前后几个点可能都满足圆弧拐点的条件，所以找到后加15个点
                if(L_Arc_Turn_Point_Num == 3)
                {
                    break;
                }
            }
        }
    }

    for(i =7; i < (L_Statics - 5); i++)
    {
        if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 7)))
        {

            if((L_Line[i][0] < L_Line[i - 7][0]) && (L_Line[i][0] < L_Line[i - 4][0]) && (L_Line[i][0] <= L_Line[i - 1][0]) &&
               (L_Line[i][0] <= L_Line[i + 1][0]) && (L_Line[i][0] < L_Line[i + 3][0]) && (L_Line[i][0] < L_Line[i + 5][0]) &&
               (L_Grow_Dir[i + 1] == 4 || L_Grow_Dir[i + 1] == 1) && (L_Grow_Dir[i + 3] == 4 || L_Grow_Dir[i + 3] == 1) &&
               (L_Grow_Dir[i - 1] == 1 || L_Grow_Dir[i + 1] == -2) && (L_Grow_Dir[i - 3] == 1 || L_Grow_Dir[i - 3] == -2))                    //左侧外凸
            {
                L_Arc_Turn_Point[L_Arc_Turn_Point_Num][0] = L_Line[i][0];
                L_Arc_Turn_Point[L_Arc_Turn_Point_Num][1] = L_Line[i][1];
                L_Arc_Turn_Point_Flag = 1;
                L_Arc_Turn_Point_Num ++;
                i += 15;
                if(L_Arc_Turn_Point_Num == 3)
                {
                    break;
                }
            }
        }
    }
}

/**
* 函数功能：      寻找右侧圆弧拐点
* 特殊说明：      无
* 形  参：        无
*
* 示例：          Get_R_Arc_Turn_Point();
* 返回值：        无
*/
uint8 R_Arc_Turn_Point[3][2] = {{0}};
uint8 R_Arc_Turn_Point_Flag = 0;
uint8 R_Arc_Turn_Point_Num = 0;
void Get_R_Arc_Turn_Point(void)
{
    uint8 i = 0;
    R_Arc_Turn_Point_Flag = 0;
    for(i = 0; i < 3; i++)
    {
        R_Arc_Turn_Point_Num = 0;
        R_Arc_Turn_Point[i][0] = 0;
        R_Arc_Turn_Point[i][1] = 0;
    }

    for(i = 7; i < (R_Statics - 5); i++)
    {
        if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
        {
            if((R_Line[i][0] < R_Line[i - 7][0]) && (R_Line[i][0] < R_Line[i - 4][0]) && (R_Line[i][0] <= R_Line[i - 1][0]) &&
               (R_Line[i][0] <= R_Line[i + 1][0]) && (R_Line[i][0] < R_Line[i + 3][0]) && (R_Line[i][0] < R_Line[i + 5][0]) &&
               (R_Grow_Dir[i + 1] == 4 || R_Grow_Dir[i + 1] == 1) && (R_Grow_Dir[i + 3] == 4 || R_Grow_Dir[i + 3] == 1) &&
               (R_Grow_Dir[i - 1] == 1 || R_Grow_Dir[i + 1] == -2) && (R_Grow_Dir[i - 3] == 1 || R_Grow_Dir[i - 3] == -2))                    //右侧内凹
            {
                R_Arc_Turn_Point[R_Arc_Turn_Point_Num][0] = R_Line[i][0];
                R_Arc_Turn_Point[R_Arc_Turn_Point_Num][1] = R_Line[i][1];
                R_Arc_Turn_Point_Flag = 1;
                R_Arc_Turn_Point_Num ++;
                i += 15;
                if(R_Arc_Turn_Point_Num == 3)
                {
                    break;
                }
            }
        }
    }
    for(i = 7; i < (R_Statics - 5); i++)
    {
        if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
        {
            if((R_Line[i][0] > R_Line[i - 7][0]) && (R_Line[i][0] > R_Line[i - 4][0]) && (R_Line[i][0] >= R_Line[i - 1][0]) &&
               (R_Line[i][0] >= R_Line[i + 1][0]) && (R_Line[i][0] > R_Line[i + 3][0]) && (R_Line[i][0] > R_Line[i + 5][0]) &&
               (R_Grow_Dir[i + 1] == -2 || R_Grow_Dir[i + 1] == 1) && (R_Grow_Dir[i + 3] == -2 || R_Grow_Dir[i + 3] == 1) &&
               (R_Grow_Dir[i - 1] == 1 || R_Grow_Dir[i + 1] == 4) && (R_Grow_Dir[i - 3] == 1 || R_Grow_Dir[i - 3] == 4))                    //右侧外凸
            {
                R_Arc_Turn_Point[R_Arc_Turn_Point_Num][0] = R_Line[i][0];
                R_Arc_Turn_Point[R_Arc_Turn_Point_Num][1] = R_Line[i][1];
                R_Arc_Turn_Point_Flag = 1;
                R_Arc_Turn_Point_Num ++;
                i += 15;
                if(R_Arc_Turn_Point_Num == 3)
                {
                    break;
                }
            }
        }
    }
}

float Max_Slope_Dif_Thre = 0.6;     //实测得出，并未使用
float Min_Slope_Dif_Thre = 0.15;    //实测得出，当两段线段的斜率低于这个阈值时，可认为斜率相等
//斜率和截距
float R_Straightaway_Lope_Rate_A = 0;   //第一段的斜率
float R_Straightaway_Lope_Rate_B = 0;   //第二段的斜率
float R_Straightaway_Lope_Rate_C = 0;   //整段边线的斜率
float R_Intercept = 0;      //整段边线的截距
uint8 R_Straight_Flag = 0;      //右侧边线是否为直线标志位

/**
* 函数功能：      求右侧边线斜率截距，并判断是否为直线
* 特殊说明：      无
* 形  参：        uint8 start          //右侧边线起始点Y坐标
*                 uint8 end            //右侧边线终止点Y坐标
*
* 示例：          Get_R_Intercept_And_Slope(R_Line[R_Statics][1] + 5, R_Line[0][1]);
* 返回值：        无
*/
void Get_R_Intercept_And_Slope(uint8 start, uint8 end)
{
    R_Straightaway_Lope_Rate_A = 0;
    R_Straightaway_Lope_Rate_B = 0;
    R_Straightaway_Lope_Rate_C = 0;
    R_Intercept = 0;
    R_Straight_Flag = 0;            //所有值先清零

    R_Straightaway_Lope_Rate_A = Slope_Calculate(start, ((end - start) / 2) + start, R_Border);
    R_Straightaway_Lope_Rate_B = Slope_Calculate(((end - start) / 2) + start, end, R_Border);       //计算两段斜率
    Calculate_Slope_Intercept(start, end, R_Border, &R_Straightaway_Lope_Rate_C, &R_Intercept);     //计算整段边线斜率和截距

    if(My_ABS_F(R_Straightaway_Lope_Rate_A - R_Straightaway_Lope_Rate_B) <= Min_Slope_Dif_Thre &&
       My_ABS_F(R_Straightaway_Lope_Rate_B - R_Straightaway_Lope_Rate_C) <= Min_Slope_Dif_Thre &&
       My_ABS_F(R_Straightaway_Lope_Rate_A - R_Straightaway_Lope_Rate_C) <= Min_Slope_Dif_Thre)      //判断是否为直线
    {
        R_Straight_Flag = 1;
    }
}


float L_Straightaway_Lope_Rate_A = 0;
float L_Straightaway_Lope_Rate_B = 0;
float L_Straightaway_Lope_Rate_C = 0;
float L_Intercept = 0;
uint8 L_Straight_Flag = 0;

/**
* 函数功能：      求左侧边线斜率截距，并判断是否为直线
* 特殊说明：      无
* 形  参：        uint8 start          //左侧边线起始点Y坐标
*                 uint8 end            //左侧边线终止点Y坐标
*
* 示例：          Get_L_Intercept_And_Slope(L_Line[L_Statics][1] + 5, L_Line[0][1]);
* 返回值：        无
*/
void Get_L_Intercept_And_Slope(uint8 start, uint8 end)
{
    L_Straightaway_Lope_Rate_A = 0;
    L_Straightaway_Lope_Rate_B = 0;
    L_Straightaway_Lope_Rate_C = 0;
    L_Intercept = 0;
    L_Straight_Flag = 0;

    L_Straightaway_Lope_Rate_A = Slope_Calculate(start, ((end - start) / 2) + start, L_Border);
    L_Straightaway_Lope_Rate_B = Slope_Calculate(((end - start) / 2) + start, end, L_Border);
    Calculate_Slope_Intercept(start, end, L_Border, &L_Straightaway_Lope_Rate_C, &L_Intercept);

    if(My_ABS_F(L_Straightaway_Lope_Rate_A - L_Straightaway_Lope_Rate_B) <= Min_Slope_Dif_Thre &&
       My_ABS_F(L_Straightaway_Lope_Rate_B - L_Straightaway_Lope_Rate_C) <= Min_Slope_Dif_Thre &&
       My_ABS_F(L_Straightaway_Lope_Rate_A - L_Straightaway_Lope_Rate_C) <= Min_Slope_Dif_Thre)
    {
        L_Straight_Flag = 1;
    }
}

//拟合曲线
int16 L_Fitting_Line[] = {0};
int16 R_Fitting_Line[] = {0};
//方差
float L_Variance = 0;
float R_Variance = 0;

/**
* 函数功能：      计算左右两侧边线的方差
* 特殊说明：      每三行计算一次，减少计算量，但获取的信息不那么精确，可以选择计算每一行
* 形  参：        无
*
* 示例：          Get_Fitting_Line_And_Variance();
* 返回值：        无
*/
void Get_Fitting_Line_And_Variance(void)
{
    uint8 i = 0;

    for(i = 1; i <= 19; i ++)
    {
        L_Fitting_Line[i] = (int16)(L_Straightaway_Lope_Rate_C * (float)(i * 3) + L_Intercept); //y = kx+b(把XY轴交换下就能想通了)
        L_Fitting_Line[i] = Limit_16(L_Fitting_Line[i], X_Border_Min + 2, X_Border_Max - 2);

        R_Fitting_Line[i] = (int16)(R_Straightaway_Lope_Rate_C * (float)(i * 3) + R_Intercept);
        R_Fitting_Line[i] = Limit_16(R_Fitting_Line[i], X_Border_Min + 2, X_Border_Max - 2);
    }

    Get_Variance(L_Line[L_Statics][1] + 1, L_Line[0][1], L_Fitting_Line, L_Border , &L_Variance, 3);
    Get_Variance(R_Line[R_Statics][1] + 1, R_Line[0][1], R_Fitting_Line, R_Border, &R_Variance, 3);
}


/**
* 函数功能：      寻找左侧边线的边界点个数
* 特殊说明：      无
* 形  参：        无
*
* 示例：          Get_L_Border_Point_Num();
* 返回值：        无
*/
uint8 L_Border_Point_Num = 0;
uint8 L_UP_Border_Point_Num = 0;    //记录位于顶部黑框上的边界点个数
void Get_L_Border_Point_Num(void)
{
    uint8 i = 0;
    L_Border_Point_Num = 0;
    L_UP_Border_Point_Num = 0;

    for(i = Y_Meet; i < L_Line[0][1]; i++)
    {
        if(L_Border[i] == 2)    //左右两侧用一维边线即可
        {
            L_Border_Point_Num ++;
        }
    }
    for(i = 40; i < L_Statics - 2; i++)
    {
        if(L_Line[i][1] == 2)   //顶部的边界点使用二维边线
        {
            L_UP_Border_Point_Num ++;
        }
    }
}

/**
* 函数功能：      寻找右侧边线的边界点个数
* 特殊说明：      无
* 形  参：        无
*
* 示例：          Get_R_Border_Point_Num();
* 返回值：        无
*/
uint8 R_Border_Point_Num = 0;
uint8 R_UP_Border_Point_Num = 0;
void Get_R_Border_Point_Num(void)
{
    uint8 i = 0;
    R_Border_Point_Num = 0;
    R_UP_Border_Point_Num = 0;

    for(i = Y_Meet; i < R_Line[0][1]; i++)
    {
        if(R_Border[i] == 77)
        {
            R_Border_Point_Num ++;
        }
    }
    for(i = 40; i < R_Statics - 2; i++)
    {
        if(R_Line[i][1] == 2)
        {
            R_UP_Border_Point_Num ++;
        }
    }
}

/**
* 函数功能：      寻找对位边界点的个数
* 特殊说明：      无
* 形  参：       无
*
* 示例：          Get_Relative_Border_Point_Num();
* 返回值：        无
*/
uint8 Relative_Border_Point_Num = 0;        //存储对位边界行的个数
void Get_Relative_Border_Point_Num(void)
{
    uint8 i = 0;
    Relative_Border_Point_Num = 0;  //函数每张图像边线都调用一次，每调用一次都要清零
    for(i = Y_Meet; i < L_Start_Point[1]; i++)
    {
        if(L_Border[i] == 2 && R_Border[i] == 77)   //2为左侧边界的X坐标，77为右侧边界
        {
            Relative_Border_Point_Num ++;
        }
    }
}

int8 Row_Difference[20] = {0};
uint8 Max_Row_Dif_Line_Num = 0;     //当某一行左侧边线点位于左侧边界上，右侧边线点位于右侧边线上，与对位边界行同理，可以记录下来
/**
* 函数功能：      寻找对位边界行的个数
* 特殊说明：      无
* 形  参：        无
*
* 示例：          Get_Relative_Border_Point_Num();
* 返回值：        无
*/
void Get_Row_Difference(uint8 *l_border, uint8 *r_border)
{
    uint8 i = 0;
    Max_Row_Dif_Line_Num = 0;
    for(i = 0; i < 20; i ++)
    {
        Row_Difference[i] = (int8)(r_border[i * 3] - l_border[i * 3]);
        if(Row_Difference[i] == 75)
        {
            Max_Row_Dif_Line_Num ++;
        }
    }
}
uint8 width[60];
uint8 width_change_flag = 0;
void Get_width(uint8 *l_border, uint8 *r_border)
{
    uint8 i = 0;
    memset(width, 0, sizeof(width));
//    width_change_flag = 0;

//    if(l_border[L_Up_Turn_Right_Point_1[1]] - l_border[L_Up_Turn_Right_Point_1[1] - 2] > 5 &&
//       l_border[L_Up_Turn_Right_Point_1[1] + 2] - l_border[L_Up_Turn_Right_Point_1[1]] < 5)
//    {
//        ips200_draw_line(l_border[L_Up_Turn_Right_Point_1[0]], l_border[L_Up_Turn_Right_Point_1[1]], r_border[L_Up_Turn_Right_Point_1[0]], r_border[L_Up_Turn_Right_Point_1[1]], RGB565_RED);
//        width_change_flag = 1;
//    }
    for(i = Y_Meet; i < L_Line[0][1]; i++)
    {
        if(R_Border[i] >= L_Border[i] && L_Border[i])    //左右两侧用一维边线即可
        {
            width[i] = R_Border[i] - L_Border[i];
        }
        else
            width[i] = 0;
    }
}
uint8 Ramp_Judge_Image_Flag = 0;
uint8 L_Ramp_Flag = 0; // 左侧斜率变化标志
uint8 R_Ramp_Flag = 0; // 右侧斜率变化标志
float Ramp_Up_Slope_L = 0;
float Ramp_Down_Slope_L = 0;
float Ramp_Up_Slope_R = 0;
float Ramp_Down_Slope_R = 0;
void Get_Ramp_Slope()
{
    Ramp_Judge_Image_Flag = 0;
    L_Ramp_Flag = 0;
    R_Ramp_Flag = 0;
    Ramp_Up_Slope_L = 0;
    Ramp_Up_Slope_R = 0;
    Ramp_Down_Slope_L = 0;
    Ramp_Down_Slope_R = 0;
    uint8 temp_flag = 0;

    static const uint8 L_Staright[60] = { 0, 0, 0, 6, 7, 7, 7, 8, 8, 9,
                             9, 10, 10, 10, 11, 11,12,12,13,13,
                            13,14,14,15,16,16,16,17,17,18,
                            18,18,19,19,20,20,21,22,22,23,
                            23,24,25,25,25,26,26,27,27,28,
                            28,28,29,29,30,30,31,32,0,0};
    static const uint8 R_Staright[60] = { 0, 0, 0, 7, 8, 8, 9, 9, 10, 10,
                             11, 11, 12, 12, 13, 13,14,14,15,15,
                            16,16,17,17,18,18,19,19,20,20,
                            21,21,22,22,23,23,23,23,24,24,
                            25,25,25,26,27,27,28,28,28,29,
                            29,30,31,31,31,32,32,32,0,0};

    if((L_Up_Turn_Right_Point_Flag_1 == 1) && (R_Up_Turn_Left_Point_Flag_1 == 1))
    {
        // 计算拐点附近斜率
        Ramp_Up_Slope_L = Slope_Calculate(L_Up_Turn_Right_Point_1[1], L_Up_Turn_Right_Point_1[1] + 5, L_Border);
        Ramp_Up_Slope_R = Slope_Calculate(R_Up_Turn_Left_Point_1[1], R_Up_Turn_Left_Point_1[1] + 5, R_Border);
        Ramp_Down_Slope_L = Slope_Calculate(L_Start_Point[1]-8, L_Start_Point[1]-3, L_Border);
        Ramp_Down_Slope_R = Slope_Calculate(R_Start_Point[1]-8, R_Start_Point[1]-3, R_Border);
        if(func_abs(Ramp_Up_Slope_L - Ramp_Down_Slope_L) >= 0.25f)
        {
            L_Ramp_Flag = 1;
        }
        if(func_abs(Ramp_Up_Slope_R - Ramp_Down_Slope_R) >= 0.25f)
        {

            R_Ramp_Flag = 1;
        }
    }
    if(L_Ramp_Flag && R_Ramp_Flag)
    {
        for(uint8 i = L_Up_Turn_Right_Point_1[1]+2;i <= L_Up_Turn_Right_Point_1[1]+7; i++ )
        {
            if((width[i]/2) > (L_Staright[i]))
            {
                temp_flag = 1;
            }
            if((width[i]/2) > (R_Staright[i]))
            {
                temp_flag = 1;
            }
            if(temp_flag)
                break;
        }
        if(temp_flag == 0)
            Ramp_Judge_Image_Flag = 1;
    }
}

/**
* 函数功能：      获取所有信息
* 特殊说明：      无
* 形  参：        无
*
* 示例：          Get_Information();
* 返回值：        无
*/
void Get_Information(void)
{
    Get_L_Border_Point_Num();
    Get_R_Border_Point_Num();
    Get_Relative_Border_Point_Num();
    Get_L_Up_Turn_Left_Point_1(2);
    Get_L_Right_Turn_Up_Point_1(2);
    Get_R_Up_Turn_Right_Point_1(2);
    Get_R_Left_Turn_Up_Point_1(2);

    Get_L_Up_Turn_Right_Point_1(1,5);
    Get_L_Left_Turn_Up_Point_1(1,5);
    Get_R_Up_Turn_Left_Point_1(1,5);
    Get_R_Right_Turn_Up_Point_1(1,5);

    Get_L_Arc_Turn_Point();
    Get_R_Arc_Turn_Point();
    Get_L_Intercept_And_Slope(L_Line[L_Statics][1] + 5, L_Line[0][1]);
    Get_R_Intercept_And_Slope(R_Line[R_Statics][1] + 5, R_Line[0][1]);
    Get_Fitting_Line_And_Variance();
    Get_Row_Difference(L_Border, R_Border);
    Get_width(L_Border, R_Border);
//    Get_Ramp_Slope();
    if(Image_Num % 10 == 0)
    {
//        Barrier_Distance = TOF_Get_Distance_mm();
    }
}


uint8 Zebra_Count_Flag = 0;
uint16 ramp_judge_dis = 200;
uint16 ramp_end_dis = 1200;
uint16 ramp_up_delay = 30;
uint8 Element_Judgement(uint8(*image)[Image_X], uint16 l_total_num, uint16 r_total_num, int8 *l_dir, int8 *r_dir, uint8(*l_line)[2], uint8(*r_line)[2], uint8 x_meet, uint8 y_meet,
                        uint8 *l_border, uint8 *r_border)
{
    uint8 Element = 0;
    uint8 i = 0;
    uint8 Black_Num = 0;
    uint8 Black_Num1 = 0;

    //扫行判断斑马线
    if(Zebra_Flag != 0)
    {
//        uint8 Black_Num = 0;

        if(L_Border[50] != 2 && R_Border[50] != 77/* && (R_Border[45] - L_Border[45]) <= 60*/)
        {
            for(i = L_Border[50]; i < R_Border[50]; i ++)
            {
                if((Find_Line_Image[50][i] <= (Adaptive_Thres_Average)) && (Find_Line_Image[50][i + 2] >= (Adaptive_Thres_Average + 10)))
                {
                    Black_Num ++;
                }
            }
        }
        if(L_Border[45] != 2 && R_Border[45] != 77/* && (R_Border[45] - L_Border[45]) <= 60*/)
        {
            for(i = L_Border[45]; i < R_Border[45]; i ++)
            {
                if((Find_Line_Image[45][i] <= (Adaptive_Thres_Average)) && (Find_Line_Image[45][i + 2] >= (Adaptive_Thres_Average + 10)))
                {
                    Black_Num ++;
                }
            }
        }

        if(Black_Num >= 10)
        {
            Element = Zebra;
        }
    }

    if(Zebra_Flag != 0)
    {
//        uint8 Black_Num = 0;
        for(i = L_Border[40]; i < R_Border[40]; i ++)
        {
            if((Find_Line_Image[40][i] <= (Adaptive_Thres_Average - 10)) && (Find_Line_Image[40][i + 1] >= (Adaptive_Thres_Average + 10)))
            {
                Black_Num1 ++;
            }
        }
        for(i = L_Border[35]; i < R_Border[35]; i ++)
        {
            if((Find_Line_Image[35][i] <= (Adaptive_Thres_Average - 10)) && (Find_Line_Image[40][i + 1] >= (Adaptive_Thres_Average + 10)))
            {
                Black_Num1 ++;
            }
        }
        if(Black_Num1 >= 10)
        {
            Element = Zebra;
        }
    }
//    ips200_show_uint(0,30*8,Black_Num,3);
//    ips200_show_uint(0,31*8,Black_Num1,3);

    //直线判断
    if(Element == 0 && Straightaway_Flag != 0)
    {
        if(L_Straight_Flag == 1 && R_Straight_Flag == 1)
        {
            if(L_Variance <= Straight_Thres && R_Variance <= Straight_Thres
               && My_ABS_F(R_Straightaway_Lope_Rate_A) <= 0.6 && My_ABS_F(R_Straightaway_Lope_Rate_B) <= 0.6 && My_ABS_F(R_Straightaway_Lope_Rate_C) <= 0.6
               && My_ABS_F(L_Straightaway_Lope_Rate_A) <= 0.6 && My_ABS_F(L_Straightaway_Lope_Rate_B) <= 0.6 && My_ABS_F(L_Straightaway_Lope_Rate_C) <= 0.6)
            {
                if((L_Border_Point_Num + R_Border_Point_Num <= 5) || (L_Border_Point_Num <= 10 && R_Border_Point_Num == 0) || (L_Border_Point_Num == 0 && R_Border_Point_Num <= 10))
                {
                    {
                        if(Y_Meet <= 6 && X_Meet <= 55 && X_Meet >= 25)
                        {
                            Element = Straightaway;
                        }
                    }
                }
            }
        }

        //坡道判断嵌入直线判断内
        if(Element == Straightaway && Ramp_Flag != 0/*&& Image_Num >= 10*/)
        {
            if(Ramp_Flag == 1)
            {
                if(dis_tof_mm <= ramp_judge_dis && L_Border_Point_Num <= 4 && R_Border_Point_Num <= 4 && Y_Meet <= 4)
                {
                    if(/*func_abs((float)(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data) - Yao.Target_Speed) <= 100 &&
                            func_abs(imu660ra.eulerAngle.pitch) <= 2 */
                            -2.0f <= imu660ra.eulerAngle.pitch && imu660ra.eulerAngle.pitch <= 3.0f)
                        Element = Ramp;
                }
            }
//            else if(Ramp_Flag == 2)
//            {
//                if(Ramp_Judge_Image_Flag == 1)
//                    Element = Ramp;
//            }
        }
    }
    if((Element == Straightaway || Element == 0) && Ramp_Flag == 2)
    {
        Get_R_Left_Turn_Up_Point_1_ramp(2);
        Get_L_Right_Turn_Up_Point_1_ramp(2);
//        if(Ramp_Judge_Image_Flag == 1)
//            Element = Ramp;
        if((L_Up_Turn_Right_Point_Flag_1 && R_Up_Turn_Left_Point_Flag_1) && L_Right_Turn_Up_Point_Flag_1_ramp + R_Left_Turn_Up_Point_Flag_1_ramp >= 1)
        {
            if(L_Right_Turn_Up_Point_Flag_1_ramp)
            {
                if(func_abs(L_Up_Turn_Right_Point_1[1] - R_Up_Turn_Left_Point_1[1]) <= 3 &&
                   func_abs(L_Up_Turn_Right_Point_1[1] - L_Right_Turn_Up_Point_1_ramp[1]) <= 3 &&
                   func_abs(L_Up_Turn_Right_Point_1[0] - L_Right_Turn_Up_Point_1_ramp[0]) <= 6)
                    Element = Ramp;
            }
            if(R_Left_Turn_Up_Point_Flag_1_ramp)
            {
                if(func_abs(L_Up_Turn_Right_Point_1[1] - R_Up_Turn_Left_Point_1[1]) <= 3 &&
                   func_abs(R_Up_Turn_Right_Point_1[1] - R_Left_Turn_Up_Point_1_ramp[1]) <= 3 &&
                   func_abs(R_Up_Turn_Right_Point_1[0] - R_Left_Turn_Up_Point_1_ramp[0]) <= 6)
                    Element = Ramp;
            }
        }
    }

    //左弯道判断
    if(Element == 0 && L_Turn_Flag != 0)
    {
        if(x_meet <= 30/* && y_meet >= 5*/ && L_Border_Point_Num >= 7 &&/* L_UP_Border_Point_Num == 0 && R_UP_Border_Point_Num == 0 &&*/ R_Border_Point_Num <= 5)
        {
            Element = L_Turn;
        }
    }
    //右弯道判断
    if(Element == 0 && R_Turn_Flag != 0)
    {
        if(x_meet >= 50/* && y_meet >= 5*/ && R_Border_Point_Num >= 7 &&/* L_UP_Border_Point_Num == 0 && R_UP_Border_Point_Num == 0 &&*/ L_Border_Point_Num <= 5)
        {
            Element = R_Turn;
        }
    }
    uint8 single_judge_line = 10;
    // 单边桥判断 Number_1
    if((Element == 0 || Element == 2) &&
             Single_Control_Flag != 0 &&
             L_Border_Point_Num == 0 &&
             R_Border_Point_Num == 0 &&
             (L_Up_Turn_Right_Point_Flag_1 == 1) && (R_Up_Turn_Left_Point_Flag_1 == 1)
      )
    {
        single_judge_line = (L_Up_Turn_Right_Point_1[1] > R_Up_Turn_Left_Point_1[1] ? L_Up_Turn_Right_Point_1[1] : R_Up_Turn_Left_Point_1[1]);
        if((width[single_judge_line-5] != 0) && (width[single_judge_line] != 0) && func_abs(width[single_judge_line] - width[single_judge_line+2]) <= 4 && func_abs(width[single_judge_line] - width[single_judge_line-2]) >= 4 )
        {
            if((L_Border[single_judge_line] >= L_Border[single_judge_line+2] ) && (L_Border[single_judge_line+2] > L_Line[0][0]) && (L_Border[single_judge_line-2] >= L_Border[single_judge_line]) &&
               (R_Border[single_judge_line] <= R_Border[single_judge_line+2] ) && (R_Border[single_judge_line+2] < R_Line[0][0]) && (R_Border[single_judge_line-2] <= R_Border[single_judge_line])
               )
            Element = Single_State;
        }
    }
    // 移植注释：Pe550 - 变量赋值后未使用，(void)无法消除IAR Pe550，注释此行
    // uint8 jump_judge_line = 10;
    // 跳跃垂直路障判断
    if(      (Element == Straightaway || Element == 0) && Jump_Control_Flag != 0 &&
            R_Straight_Flag && L_Straight_Flag &&
//             (Element_State == Straightaway || Element_State == Jump_State) &&
             /*y_meet >= 20 &&
             L_Statics <= 50 &&
             R_Statics <= 50 &&*/
             (L_Up_Turn_Right_Point_Flag_1 == 1) && (R_Up_Turn_Left_Point_Flag_1 == 1)
//             L_UP_Border_Point_Num == 0 &&
//             R_UP_Border_Point_Num == 0 &&
//             func_abs(x_meet - R_Border[y_meet+1]) <= 8 &&
//             func_abs(x_meet - L_Border[y_meet+1]) <= 8
             )
    {
        // 移植注释：Pe550 - jump_judge_line 赋值后从未读取，对应声明已注释
        // jump_judge_line = L_Up_Turn_Right_Point_1[1] > R_Up_Turn_Left_Point_1[1] ? L_Up_Turn_Right_Point_1[1] : R_Up_Turn_Left_Point_1[1];
        if(func_abs(L_Up_Turn_Right_Point_1[1] - R_Up_Turn_Left_Point_1[1]) <= 2 &&
           func_abs(L_Up_Turn_Right_Point_1[1] - Y_Meet) <= 2 &&
           func_abs(R_Up_Turn_Left_Point_1[1] - Y_Meet) <= 2 &&
           width[L_Up_Turn_Right_Point_1[1]-4] == 0)
            Element = Jump_State;
    }


//十字路口判断
    //正入判断
    if(Element == 0 && Cross_Flag != 0)
    {
        if((L_Up_Turn_Left_Point_Flag_1 + R_Up_Turn_Right_Point_Flag_1 + L_Right_Turn_Up_Point_Flag_1 + R_Left_Turn_Up_Point_Flag_1) >= 3)
        {
            uint8 L_Count = 0;
            uint8 R_Count = 0;
            //判断左右各有大量横向生长的点
            for(i = 0; i < l_total_num; i++)
            {
                if(l_dir[i] == (-3) || l_dir[i] == 3)
                {
                    L_Count ++;
                }
            }
            for(i = 0; i < r_total_num; i++)
            {
                if(r_dir[i] == (-3) || r_dir[i] == 3)
                {
                    R_Count++;
                }
            }
            if(L_Count >= 18 && R_Count >= 18 && L_Border_Point_Num >= 10 && R_Border_Point_Num >= 10)
            {
                if(Relative_Border_Point_Num >= 9)
                {
                    Element = Cross;
                }
            }
        }
    }
//    左斜入判断
    if(Element == 0 && Cross_Flag != 0)
    {
        if(R_Start_Point[0] >= 65 && L_Start_Point[0] >= 20 && L_Up_Turn_Left_Point_Flag_1 == 1 && R_Left_Turn_Up_Point_Flag_1 == 1)
        {
            if(R_Left_Turn_Up_Point_1[1] <= 35)
            {
                uint8 Temp_Num = 0;
                for(i = 57; i > R_Left_Turn_Up_Point_1[1]; i--)
                {
                    if(R_Border[i] == 77)
                    {
                        Temp_Num ++;
                    }
                }
                if((float)Temp_Num / (float)(57 - R_Left_Turn_Up_Point_1[1]) >= 0.9f)
                {
                    if(Y_Meet >= 4 && L_Border_Point_Num >= 12 && R_Border_Point_Num >= 12)
                    {
                        if(Relative_Border_Point_Num >= 12)
                        {
                            Element = L_Oblique_Cross;
                        }
                    }
                }
            }
        }
    }
    //右斜入判断
    if(Element == 0 && Cross_Flag != 0)
    {
        if(L_Start_Point[0] <= 15 && R_Start_Point[0] <= 60 && L_Right_Turn_Up_Point_Flag_1 == 1 && R_Up_Turn_Right_Point_Flag_1 == 1)
        {
            if(L_Right_Turn_Up_Point_1[1] <= 35)
            {
                uint8 Temp_Num = 0;
                for(i = 57; i > L_Right_Turn_Up_Point_1[1]; i--)
                {
                    if(L_Border[i] == 2)
                    {
                        Temp_Num ++;
                    }
                }
                if((float)Temp_Num / (float)(57 - L_Right_Turn_Up_Point_1[1]) >= 0.9f)
                {
                    if(Y_Meet >= 4 && L_Border_Point_Num >= 12 && R_Border_Point_Num >= 12)
                    {
                        if(Relative_Border_Point_Num >= 12)
                        {
                            Element = R_Oblique_Cross;
                        }
                    }
                }
            }
        }
    }
//    //转完45度右转弯后斜入判断
//    if(Element == 0 && Cross_Flag != 0)
//    {
//        if(L_Arc_Turn_Point_Flag == 0 && R_Arc_Turn_Point_Flag == 1)
//        {
//            if(R_Border_Point_NUm >= 5)
//        }
//    }
//判断左环岛
    //一次远距判断
    if((Element == 0 || Element == 3 || Element == 4) && L_Circle_Flag != 0)
    {
        if(L_Up_Turn_Left_Point_Flag_1 == 1 && R_Up_Turn_Right_Point_Flag_1 == 0 && R_Straight_Flag == 1)
        {
            if(R_Border_Point_Num <= 40 && L_Border_Point_Num >= 10 && Y_Meet >= 3 && L_Up_Turn_Left_Point_1[1] >= 20)
            {
                uint8 Temp_Num = 0;
                for(i = 0; i < R_Statics / 2; i ++)
                {
                    if(R_Grow_Dir[i] == -3 || R_Grow_Dir[i] == 3)
                    {
                        Temp_Num ++;
                    }
                }
                if(Temp_Num <= 4)
                {
                    if(Relative_Border_Point_Num <= 8)
                    {
                        buzzer_flag = 1;
                        Element = L_Circle;
                    }
                }
            }
        }
    }
    //二次近距判断
    if((Element == 0 || Element == 3 || Element == 4) && L_Circle_Flag != 0)
    {
        //判断左存在拐点, 判断右侧为直线
        if(L_Up_Turn_Left_Point_Flag_1 == 1 && R_Up_Turn_Right_Point_Flag_1 == 0 && R_Straight_Flag == 1 && L_Arc_Turn_Point_Flag == 1)
        {
            if(L_Up_Turn_Left_Point_1[1] > 30 && (L_Up_Turn_Left_Point_1[1] - L_Arc_Turn_Point[0][1]) >= 15)//(L_Up_Turn_Left_Point_1[1] - L_Arc_Turn_Point[0][1]) > 15)
            {
                //判断右侧直线不在边界上
                if(R_Border_Point_Num <= 10 && L_Border_Point_Num >= 18)
                {
                    uint8 Temp_Num = 0;
                    for(i = 0; i < R_Statics / 2; i ++)
                    {
                        if(R_Grow_Dir[i] == -3 || R_Grow_Dir[i] == 3)
                        {
                            Temp_Num ++;
                        }
                    }
                    if(Temp_Num <= 4)
                    {
                        if(Relative_Border_Point_Num <= 8)
                        {
                            buzzer_flag = 1;
                            Element = L_Circle;
                        }
                    }
                }
            }
        }
    }
//    //三次斜入判断(偏向圆环一侧)
//    if((Element == 0 || Element == 3 || Element == 4) && L_Circle_Flag != 0)
//    {
//        if(R_Start_Point[0] >= 72)
//        {
//            uint8 Temp_Num = 0;
//            for(i = 57; i > 2; i--)
//            {
//                if(L_Border[i] == 2 && R_Border[i] != 77)
//                {
//                    Temp_Num ++;
//                }
//                if(L_Border[i] != 2 && R_Border[i] == 77)
//                {
//                    Temp_Num ++;
//                }
//            }
//            if(Temp_Num >= 30 && R_UP_Border_Point_Num >= 3 && L_Border_Point_Num >= 16 && R_Border_Point_Num >= 13)
//            {
//                uint8 Temp_Num_2 = 0;
//                for(i = 0; i < R_Statics / 2; i ++)
//                {
//                    if(R_Grow_Dir[i] == -3 || R_Grow_Dir[i] == 3)
//                    {
//                        Temp_Num_2 ++;
//                    }
//                }
//                if(Temp_Num_2 <= 4)
//                {
//                    if(Relative_Border_Point_Num <= 8)
//                    {
//                        buzzer_flag = 1;
//                        Element = L_Circle;
//                    }
//                }
//            }
//        }
//    }
//    //四次斜入判断（偏向直线一侧）
//    if(Element == 0 && L_Circle_Flag != 0)
//    {
//        if(L_Start_Point[0] <= 10 && R_Start_Point[0] <= 50)
//        {
//            if(L_Border_Point_Num >= 25 && L_Arc_Turn_Point_Flag == 1 && L_Up_Turn_Left_Point_Flag_1 == 0)
//            {
//                if(R_Straight_Flag == 1 && R_Border_Point_Num == 0)
//                {
//                    if(Y_Meet >= 3 && X_Meet <= 30)
//                    {
//                        buzzer_flag = 1;
//                        Element = L_Circle;
//                    }
//                }
//            }
//        }
//    }
//判断右环岛
    //一次远距判断
    if((Element == 0 || Element == 3 || Element == 4) && R_Circle_Flag != 0)
    {
        if(L_Up_Turn_Left_Point_Flag_1 == 0 && R_Up_Turn_Right_Point_Flag_1 == 1 && L_Straight_Flag == 1)
        {
            if(L_Border_Point_Num <= 40 && R_Border_Point_Num >= 10 && Y_Meet >= 3 && R_Up_Turn_Right_Point_1[1] >= 20)
            {
                uint8 Temp_Num = 0;
                for(i = 0; i < L_Statics / 2; i ++)
                {
                    if(L_Grow_Dir[i] == -3 || L_Grow_Dir[i] == 3)
                    {
                        Temp_Num ++;
                    }
                }
                if(Temp_Num <= 4)
                {
                    if(Relative_Border_Point_Num <= 8)
                    {
                        buzzer_flag = 1;
                        Element = R_Circle;
                    }
                }
            }
        }
    }
    //二次近距判断
    if((Element == 0 || Element == 3 || Element == 4) && R_Circle_Flag != 0)
    {
        //判断右存在拐点, 判断左侧为直线
        if(R_Up_Turn_Right_Point_Flag_1 == 1 && L_Up_Turn_Left_Point_Flag_1 == 0 && L_Straight_Flag == 1 && R_Arc_Turn_Point_Flag == 1)
        {
            if(R_Up_Turn_Right_Point_1[1] > 30 && (R_Up_Turn_Right_Point_1[1] - R_Arc_Turn_Point[0][1]) >= 15)//(R_Up_Turn_Right_Point_1[1] - R_Arc_Turn_Point[0][1]) > 15)
            {
                //判断左侧直线不在边界上
                if(L_Border_Point_Num <= 10 && R_Border_Point_Num >= 18)
                {
                    uint8 Temp_Num = 0;
                    for(i = 0; i < L_Statics / 2; i ++)
                    {
                        if(L_Grow_Dir[i] == -3 || L_Grow_Dir[i] == 3)
                        {
                            Temp_Num ++;
                        }
                    }
                    if(Temp_Num <= 4)
                    {
                        if(Relative_Border_Point_Num <= 8)
                        {
                            buzzer_flag = 1;
                            Element = R_Circle;
                        }
                    }
                }
            }
        }
    }
//    //三次斜入判断
//    if((Element == 0 || Element == 3 || Element == 4) && R_Circle_Flag != 0)
//    {
//        if(Y_Meet >= 3 && X_Meet >= 50 && L_Start_Point[0] == 1)
//        {
//            uint8 Temp_Num = 0;
//            for(i = 57; i > 2; i--)
//            {
//                if(L_Border[i] == 2 && R_Border[i] != 77)
//                {
//                    Temp_Num ++;
//                }
//                if(L_Border[i] != 2 && R_Border[i] == 77)
//                {
//                    Temp_Num ++;
//                }
//            }
//            if(Temp_Num >= 30 && L_UP_Border_Point_Num >= 3 && L_Border_Point_Num >= 13 && R_Border_Point_Num >= 16)
//            {
//                uint8 Temp_Num_2 = 0;
//                for(i = 0; i < L_Statics / 2; i ++)
//                {
//                    if(L_Grow_Dir[i] == -3 || L_Grow_Dir[i] == 3)
//                    {
//                        Temp_Num_2 ++;
//                    }
//                }
//                if(Temp_Num_2 <= 4)
//                {
//                    if(Relative_Border_Point_Num <= 8)
//                    {
//                        buzzer_flag = 1;
//                        Element = R_Circle;
//                    }
//                }
//            }
//        }
//    }
//    //四次斜入判断（偏向直线一侧）
//    if(Element == 0 && R_Circle_Flag != 0)
//    {
//        if(L_Start_Point[0] >= 30 && R_Start_Point[0] >= 70)
//        {
//            if(R_Border_Point_Num >= 25 && R_Arc_Turn_Point_Flag == 1 && R_Up_Turn_Right_Point_Flag_1 == 0)
//            {
//                if(L_Straight_Flag == 1 && L_Border_Point_Num == 0)
//                {
//                    if(Y_Meet >= 3 && X_Meet >= 50)
//                    {
//                        buzzer_flag = 1;
//                        Element = R_Circle;
//                    }
//                }
//            }
//        }
//    }
    //小S弯判断
    if(Element == 0 && Small_S_Flag != 0)
    {
        if(L_Arc_Turn_Point_Flag == 1 && R_Arc_Turn_Point_Flag == 1 && ((L_Arc_Turn_Point_Num + R_Arc_Turn_Point_Num) >= 3))
        {
            if(Y_Meet <= 5)
            {
                Element = Small_S;
            }
        }
    }
//    //左侧障碍物识别
//    if((Element == 0 || Element == Straightaway) && Barrier_Flag != 0)
//    {
//        if(L_Border_Point_Num == 0)
//        {
//            if(L_Straight_Flag == 1 && R_Straight_Flag == 1)
//            {
//                uint8 Temp_Barrier_Flag = 0;
//                for(i = 57; i > 2; i--)
//                {
//                    if(L_Border[i] - L_Border[i - 1] >= 5)
//                    {
//                        Temp_Barrier_Flag = 1;
//                        break;
//                    }
//                }
//                uint8 Temp_X = L_Border[i];
//                uint8 Temp_Y = i;
//                if(Temp_Barrier_Flag == 1)
//                {
//                    uint8 j = 0;
//                    uint8 Temp_Num = 0;
//                    for(i = Temp_Y - 5; i < Temp_Y; i++)
//                    {
//                        for(j = Temp_X + 1; j < Temp_X + 6; j ++)
//                        {
//                            if(Find_Line_Image[i][j] < (Adaptive_Thres_Average - 20))
//                            {
//                                Temp_Num ++;
//                            }
//                        }
//                    }
//                    if(Temp_Num >= 23)
//                    {
//                        Element = L_Barrier;
//                    }
//                }
//            }
//        }
//    }
    //三岔路口判断
    if(Element == 0 && Three_Bif_Flag != 0)
    {
    }
    //障碍物判断
    if(Element == 0 && Barrier_Flag != 0)
    {
    }
    //断路判断
    if(Element == 0 && Disconnection_Flag != 0)
    {
    }
    //T路口判断
    if(Element == 0 &&T_Way_Flag != 0)
    {
    }
    //左车库判断
    if(Element == 0 && L_Garage_Flag != 0)
    {
    }
    //右车库判断
    if(Element == 0 && R_Garage_Flag != 0)
    {
    }
    //出赛道判断
    if(y_meet >= 40 && L_Straightaway_Lope_Rate_A == L_Straightaway_Lope_Rate_B && L_Straightaway_Lope_Rate_A == L_Straightaway_Lope_Rate_C
                    && R_Straightaway_Lope_Rate_A == R_Straightaway_Lope_Rate_B && R_Straightaway_Lope_Rate_A == R_Straightaway_Lope_Rate_C)
    {
        Element = Derailment;
    }
    return Element;
}

//*****************************特殊判断***********************************
uint8 L_Circle_R_T_U_Point[2] = {0};
uint8 L_Circle_R_T_U_Point_Flag = 0;
void Get_Special_R_T_U_Point_2(void)
{
    uint8 i = 0, j = 0;
    uint8 break_flag = 0;
    uint8 Temp_Num = 0;
    L_Circle_R_T_U_Point_Flag = 0;
    L_Circle_R_T_U_Point[0] = 0;
    L_Circle_R_T_U_Point[1] = 0;

    for(i = 30; i < L_Statics; i++)
    {
        if((L_Line[i][1] >= (Y_Border_Min + 2)) && (L_Line[i][1] <= (Y_Border_Max - 2)))
        {
            if ((L_Grow_Dir[i + 2] == 4 || L_Grow_Dir[i + 2] == 1) && (L_Grow_Dir[i + 5] == 4 || L_Grow_Dir[i + 5] == 1) && (L_Grow_Dir[i + 7] == 4 || L_Grow_Dir[i + 7] == 1) &&
                (L_Grow_Dir[i - 2] == 2 || L_Grow_Dir[i - 2] == 3) && (L_Grow_Dir[i - 5] == 2 || L_Grow_Dir[i - 5] == 3) && (L_Grow_Dir[i] == 4 || L_Grow_Dir[i] == 1))
            {
                for(j = 1; j < L_Statics - i; j++)
                {
                    if(L_Grow_Dir[i + j] == -3 || L_Grow_Dir[i + j] == -2)
                    {
                        Temp_Num ++;
                    }
                }
                if(Temp_Num >= 5)
                {
                    break_flag = 1;
                }
                if(break_flag == 0)
                {
                    L_Circle_R_T_U_Point[0] = L_Line[i][0];
                    L_Circle_R_T_U_Point[1] = L_Line[i][1];
                    L_Circle_R_T_U_Point_Flag = 1;
                    break;
                }
            }
        }
    }
}


uint8 R_Circle_L_T_U_Point[2] = {0};
uint8 R_Circle_L_T_U_Point_Flag = 0;
void Get_Special_L_T_U_Point_2(void)
{
    uint8 i = 0, j = 0;
    uint8 break_flag = 0;
    uint8 Temp_Num = 0;
    R_Circle_L_T_U_Point_Flag = 0;
    R_Circle_L_T_U_Point[0] = 0;
    R_Circle_L_T_U_Point[1] = 0;

    for(i = 30; i < R_Statics; i++)
    {
        if((R_Line[i][1] >= (Y_Border_Min + 2)) && (R_Line[i][1] <= (Y_Border_Max - 2)))
        {
            if ((R_Grow_Dir[i + 2] == -2 || R_Grow_Dir[i + 2] == 1) && (R_Grow_Dir[i + 5] == -2 || R_Grow_Dir[i + 5] == 1) && (R_Grow_Dir[i + 7] == -2 || R_Grow_Dir[i + 7] == 1) &&
                (R_Grow_Dir[i - 2] == -3 || R_Grow_Dir[i - 2] == -4) && (R_Grow_Dir[i - 5] == -3 || R_Grow_Dir[i - 5] == -4) && (R_Grow_Dir[i] == -2 || R_Grow_Dir[i] == 1))
            {
                for(j = 1; j < R_Statics - i; j++)
                {
                    if(R_Grow_Dir[i + j] == 3 || R_Grow_Dir[i + j] == 4)
                    {
                        Temp_Num++;
                    }
                }
                if(Temp_Num >= 5)
                {
                    break_flag = 1;
                }
                if(break_flag == 0)
                {
                    R_Circle_L_T_U_Point[0] = R_Line[i][0];
                    R_Circle_L_T_U_Point[1] = R_Line[i][1];
                    R_Circle_L_T_U_Point_Flag = 1;
                    break;
                }
            }
        }
    }
}
//***********************************************************************
//***************************************************************************************************************************************************************************




//******************************************************************************元素处理部分**********************************************************************************
//***********************************圆环处补曲线代码***************************************************************************
//相关参数
uint8 Arc_Point_1[2] = {0};
uint8 Arc_Point_2[2] = {0};
uint8 Arc_Point_3[2] = {0};

uint8 Arc_Point_1_Num = 0;
uint8 Arc_Point_2_Num = 0;

uint8 Last_Arc_Point_1[2] = {0};
uint8 Last_Arc_Point_2[2] = {0};
uint8 Last_Arc_Point_3[2] = {0};

uint8 Arc_Point_1_Flag = 0;
uint8 Arc_Point_2_Flag = 0;
uint8 Arc_Point_3_Flag = 0;

uint8 Circlr_State_2_Flag = 0;
uint8 Stop_Find_1_2_Flag = 0;

//求三个点，用于补线
void Get_Three_Point(uint8(*l_line)[2], uint8(*r_line)[2], uint8 *l_border, uint8 *r_border, uint16 statics, uint8 *l_start_point, uint8 *r_start_point, uint8 Circle_Flag, uint8 Circle_State, uint8 Change_Flag)
{
    uint8 i = 0;

    Arc_Point_1_Flag = 0;
    Arc_Point_2_Flag = 0;
    Arc_Point_3_Flag = 0;

    switch(Circle_Flag)
    {
        case 1:
        {
            switch(Circle_State)
            {
                case 2:
                {
                    if(Circlr_State_2_Flag == 0)
                    {
                        for(i = 30; i < statics - 10; i++)
                        {
                            if(l_line[i][0] == 2 && l_line[i - 1][0] == 2 && l_line[i - 2][0] == 2 && l_line[i - 3][0] == 2 && l_line[i - 5][0] == 2 &&
                               l_line[i + 1][0] != 2 && l_line[i + 2][0] != 2 && l_line[i + 4][0] != 2)
                            {
                                Arc_Point_1[0] = l_line[i][0];
                                Arc_Point_1[1] = l_line[i][1];
                                Arc_Point_1_Flag = 1;
                                Arc_Point_1_Num = i;
                                break;
                            }
                        }

                        if(Arc_Point_1_Flag == 1)
                        {
                            Last_Arc_Point_1[0] = Arc_Point_1[0];
                            Last_Arc_Point_1[1] = Arc_Point_1[1];
                            Arc_Point_1_Flag = 0;
                        }
                        else
                        {
                            Arc_Point_1[0] = Last_Arc_Point_1[0];
                            Arc_Point_1[1] = Last_Arc_Point_1[1];
                        }

                        for(i = Arc_Point_1_Num; i < statics; i ++)
                        {
                            if(l_line[i][1] >= l_line[i - 1][1])
                            {
                                if((l_line[i][0] <= l_line[i + 1][0]) && (l_line[i][1] <= l_line[i + 2][1]) && (l_line[i][1] < l_line[i + 4][1]))
                                {
                                    Arc_Point_2[0] = l_line[i][0];
                                    Arc_Point_2[1] = l_line[i][1];
                                    Arc_Point_2_Flag = 1;
                                    Arc_Point_2_Num = i;
                                    break;
                                }
                            }
                            if(l_line[i][1] <= 5)
                            {
                                break;
                            }
                        }
                        if(Arc_Point_2_Flag == 1)
                        {
                            Last_Arc_Point_2[0] = Arc_Point_2[0];
                            Last_Arc_Point_2[1] = Arc_Point_2[1];
                            Arc_Point_2_Flag = 0;
                        }
                        else
                        {
                            Arc_Point_2[0] = Last_Arc_Point_2[0];
                            Arc_Point_2[1] = Last_Arc_Point_2[1];
                        }

                        Arc_Point_3[0] = r_start_point[0];
                        Arc_Point_3[1] = r_start_point[1];
                        Arc_Point_3_Flag = 1;

                        if(((Arc_Point_2[1] >= 35 && Arc_Point_2[0] >= 55) || (X_Meet <= 49 && Y_Meet >= 20)) && Change_Flag == 1)
                        {
                            Circlr_State_2_Flag = 1;
                        }
                    }

                    else if(Circlr_State_2_Flag == 1 && Change_Flag == 1)
                    {
                        for(i = 30; i < statics - 10; i++)
                        {
                            if(l_line[i][0] == 2 && l_line[i - 1][0] == 2 && l_line[i - 2][0] == 2 && l_line[i - 3][0] == 2 && l_line[i - 5][0] == 2 &&
                               l_line[i + 1][0] != 2 && l_line[i + 2][0] != 2 && l_line[i + 4][0] != 2)
                            {
                                Arc_Point_1[0] = l_line[i][0];
                                Arc_Point_1[1] = l_line[i][1];
                                Arc_Point_1_Flag = 1;
                                Arc_Point_1_Num = i;
                                break;
                            }
                        }

                        if(Arc_Point_1_Flag == 1)
                        {
                            Last_Arc_Point_1[0] = Arc_Point_1[0];
                            Last_Arc_Point_1[1] = Arc_Point_1[1];
                            Arc_Point_1_Flag = 0;
                        }
                        else
                        {
                            Arc_Point_1[0] = Last_Arc_Point_1[0];
                            Arc_Point_1[1] = Last_Arc_Point_1[1];
                        }

                        for(i = Arc_Point_1_Num; i < statics - 4; i ++)
                        {
                            if(l_line[i][1] >= l_line[i - 1][1])
                            {
                                if((l_line[i][0] <= l_line[i + 1][0]) && (l_line[i][1] <= l_line[i + 2][1]) && (l_line[i][1] < l_line[i + 4][1]))
                                {
                                    Arc_Point_3[0] = l_line[i][0];
                                    Arc_Point_3[1] = l_line[i][1];
                                    Arc_Point_3_Flag = 1;
                                    break;
                                }
                            }
                            if(l_line[i][1] <= 5)
                            {
                                break;
                            }
                        }
                        if(Arc_Point_3_Flag == 1)
                        {
                            Last_Arc_Point_3[0] = Arc_Point_3[0];
                            Last_Arc_Point_3[1] = Arc_Point_3[1];
                            Arc_Point_3_Flag = 0;
                        }
                        else
                        {
                            Arc_Point_3[0] = Last_Arc_Point_3[0];
                            Arc_Point_3[1] = Last_Arc_Point_3[1];
                        }

                        Arc_Point_2[0] = l_line[Arc_Point_1_Num + (statics - Arc_Point_1_Num) / 2][0];
                        Arc_Point_2[1] = l_line[Arc_Point_1_Num + (statics - Arc_Point_1_Num) / 2][1];
                    }
                    break;
                }

                case 4:
                {
                    uint8 Temp_Border_Num = 0;
                    if(Stop_Find_1_2_Flag == 0)
                    {
                        if(r_start_point[0] == 77 && r_line[2][0] == 77 && r_line[4][0] == 77)
                        {
                            for(i = 0; i < statics / 2; i++)
                            {
                                if(r_line[i][0] == 77 && r_line[i - 1][0] == 77 && r_line[i - 2][0] == 77 && r_line[i - 3][0] == 77 &&
                                   r_line[i + 1][0] != 77 && r_line[i + 2][0] != 77 && r_line[i + 4][0] != 77)
                                {
                                    Arc_Point_1[0] = r_line[i][0];
                                    Arc_Point_1[1] = r_line[i][1];
                                    Arc_Point_1_Num = i;
                                    break;
                                }
                            }
                        }
                        else
                        {
                            Arc_Point_1[0] = r_start_point[0];
                            Arc_Point_1[1] = r_start_point[1];
                            Arc_Point_1_Num = 1;
                        }

                        for(i = Arc_Point_1_Num ; i < statics - 10; i++)
                        {
                            if(r_line[i][0] <= r_line[i - 1][0] && r_line[i][0] < r_line[i - 2][0] && r_line[i][0] < r_line[i - 4][0] &&
                               r_line[i][0] <= r_line[i + 1][0] && r_line[i][0] < r_line[i + 2][0] && r_line[i][0] < r_line[i + 4][0])
                            {
                                Arc_Point_2[0] = r_line[i][0];
                                Arc_Point_2[1] = r_line[i][1];
                                break;
                            }
                        }

                        for(i = Y_Meet + 3; i < 57; i++)
                        {
                            if(r_border[i] == 77)
                            {
                                Temp_Border_Num ++;
                            }
                        }
                        if(Temp_Border_Num >= (57 - (Y_Meet + 3) - 10))
                        {
                            Stop_Find_1_2_Flag = 1;
                        }
                    }

                    for(i = 30; i < statics - 10; i++)
                    {
                        if(l_line[i][0] == 2 && l_line[i - 1][0] == 2 && l_line[i - 2][0] == 2 && l_line[i - 3][0] == 2 && l_line[i - 5][0] == 2 &&
                           l_line[i + 1][0] != 2 && l_line[i + 2][0] != 2 && l_line[i + 4][0] != 2)
                        {
                            Arc_Point_3[0] = l_line[i][0];
                            Arc_Point_3[1] = l_line[i][1];
                            break;
                        }
                    }
                    break;
                }
            }
            break;
        }

        case 2:
        {
            switch(Circle_State)
            {
                case 2:
                {
                    if(Circlr_State_2_Flag == 0)
                    {
                        for(i = 30; i < statics - 10; i++)
                        {
                            if(r_line[i][0] == 77 && r_line[i - 1][0] == 77 && r_line[i - 2][0] == 77 && r_line[i - 3][0] == 77 && r_line[i - 5][0] == 77 &&
                               r_line[i + 1][0] != 77 && r_line[i + 2][0] != 77 && r_line[i + 4][0] != 77)
                            {
                                Arc_Point_1[0] = r_line[i][0];
                                Arc_Point_1[1] = r_line[i][1];
                                Arc_Point_1_Flag = 1;
                                Arc_Point_1_Num = i;
                            }
                        }
                        if(Arc_Point_1_Flag == 1)
                        {
                            Last_Arc_Point_1[0] = Arc_Point_1[0];
                            Last_Arc_Point_1[1] = Arc_Point_1[1];
                            Arc_Point_1_Flag = 0;
                        }
                        else
                        {
                            Arc_Point_1[0] = Last_Arc_Point_1[0];
                            Arc_Point_1[1] = Last_Arc_Point_1[1];
                        }

                        for(i = Arc_Point_1_Num; i < statics - 4; i ++)
                        {
                            if(r_line[i][1] >= r_line[i - 1][1])
                            {
                                if((r_line[i][0] >= r_line[i + 1][0]) && (r_line[i][1] > r_line[i + 2][1]) && (r_line[i][1] > r_line[i + 4][1]))
                                {
                                    Arc_Point_2[0] = r_line[i][0];
                                    Arc_Point_2[1] = r_line[i][1];
                                    Arc_Point_2_Flag = 1;
                                    Arc_Point_2_Num = i;
                                }
                            }
                            if(r_line[i][1] <= 5)
                            {
                                break;
                            }
                        }
                        if(Arc_Point_2_Flag == 1)
                        {
                            Last_Arc_Point_2[0] = Arc_Point_2[0];
                            Last_Arc_Point_2[1] = Arc_Point_2[1];
                            Arc_Point_2_Flag = 0;
                        }
                        else
                        {
                            Arc_Point_2[0] = Last_Arc_Point_2[0];
                            Arc_Point_2[1] = Last_Arc_Point_2[1];
                        }

                        Arc_Point_3[0] = l_start_point[0];
                        Arc_Point_3[1] = l_start_point[1];

                        if(((Arc_Point_2[0] <= 35 && Arc_Point_2[1] >= 35) || (X_Meet >= 30 && Y_Meet >= 20)) && Change_Flag == 1)
                        {
                            Circlr_State_2_Flag = 1;
                        }
                    }

                    else if(Circlr_State_2_Flag == 1 && Change_Flag == 1)
                    {
                        for(i = 30; i < statics - 10; i++)
                        {
                            if(r_line[i][0] == 77 && r_line[i - 1][0] == 77 && r_line[i - 2][0] == 77 && r_line[i - 3][0] == 77 && r_line[i - 5][0] == 77 &&
                               r_line[i + 1][0] != 77 && r_line[i + 2][0] != 77 && r_line[i + 4][0] != 77)
                            {
                                Arc_Point_1[0] = r_line[i][0];
                                Arc_Point_1[1] = r_line[i][1];
                                Arc_Point_1_Flag = 1;
                                Arc_Point_1_Num = i;
                            }
                        }
                        if(Arc_Point_1_Flag == 1)
                        {
                            Last_Arc_Point_1[0] = Arc_Point_1[0];
                            Last_Arc_Point_1[1] = Arc_Point_1[1];
                            Arc_Point_1_Flag = 0;
                        }
                        else
                        {
                            Arc_Point_1[0] = Last_Arc_Point_1[0];
                            Arc_Point_1[1] = Last_Arc_Point_1[1];
                        }

                        for(i = Arc_Point_1_Num; i < statics; i ++)
                        {
                            if(r_line[i][1] >= r_line[i - 1][1])
                            {
                                if((r_line[i][0] >= r_line[i + 1][0]) && (r_line[i][1] > r_line[i + 2][1]) && (r_line[i][1] > r_line[i + 4][1]))
                                {
                                    Arc_Point_3[0] = r_line[i][0];
                                    Arc_Point_3[1] = r_line[i][1];
                                    Arc_Point_3_Flag = 1;
                                }
                            }
                            if(r_line[i][1] <= 5)
                            {
                                break;
                            }
                        }
                        if(Arc_Point_3_Flag == 1)
                        {
                            Last_Arc_Point_3[0] = Arc_Point_3[0];
                            Last_Arc_Point_3[1] = Arc_Point_3[1];
                            Arc_Point_3_Flag = 0;
                        }
                        else
                        {
                            Arc_Point_3[0] = Last_Arc_Point_3[0];
                            Arc_Point_3[1] = Last_Arc_Point_3[1];
                        }

                        Arc_Point_2[0] = r_line[Arc_Point_1_Num + (statics - Arc_Point_1_Num) / 2][0];
                        Arc_Point_2[1] = r_line[Arc_Point_1_Num + (statics - Arc_Point_1_Num) / 2][1];
                    }
                    break;
                }

                case 4:
                {
                    uint8 Temp_Border_Num = 0;
                    if(Stop_Find_1_2_Flag == 0)
                    {
                        if(l_start_point[0] == 2 && l_line[2][0] == 2 && l_line[4][0] == 2)
                        {
                            for(i = 0; i < statics / 2; i++)
                            {
                                if(l_line[i][0] == 2 && l_line[i - 1][0] == 2 && l_line[i - 2][0] == 2 && l_line[i - 3][0] == 2 &&
                                   l_line[i + 1][0] != 2 && l_line[i + 2][0] != 2 && l_line[i + 4][0] != 2)
                                {
                                    Arc_Point_1[0] = l_line[i][0];
                                    Arc_Point_1[1] = l_line[i][1];
                                    Arc_Point_1_Num = i;
                                }
                            }
                        }
                        else
                        {
                            Arc_Point_1[0] = l_start_point[0];
                            Arc_Point_1[1] = l_start_point[1];
                            Arc_Point_1_Num = 1;
                        }

                        for(i = Arc_Point_1_Num ; i < statics - 10; i++)
                        {
                            if(l_line[i][0] >= l_line[i - 1][0] && l_line[i][0] > l_line[i - 2][0] && l_line[i][0] > l_line[i - 4][0] &&
                               l_line[i][0] >= l_line[i + 1][0] && l_line[i][0] > l_line[i + 2][0] && l_line[i][0] > l_line[i + 4][0])
                            {
                                Arc_Point_2[0] = l_line[i][0];
                                Arc_Point_2[1] = l_line[i][1];
                            }
                        }

                        for(i = Y_Meet + 3; i < 57; i++)
                        {
                            if(l_border[i] == 2)
                            {
                                Temp_Border_Num ++;
                            }
                        }
                        if(Temp_Border_Num >= (57 - (Y_Meet + 3) - 10))
                        {
                            Stop_Find_1_2_Flag = 1;
                        }
                    }

                    for(i = 10; i < statics - 10; i++)
                    {
                        if(r_line[i][0] == 77 && r_line[i - 1][0] == 77 && r_line[i - 2][0] == 77 && r_line[i - 3][0] == 77 && r_line[i - 5][0] == 77 &&
                           r_line[i + 1][0] != 77 && r_line[i + 2][0] != 77 && r_line[i + 4][0] != 77)
                        {
                            Arc_Point_3[0] = r_line[i][0];
                            Arc_Point_3[1] = r_line[i][1];
                        }
                    }
                    break;
                }
            }
            break;
        }
    }
}

float Last_Cir_Cent[2] = {0};
float Last_Cir_R = 0;

//***************************************三点式求圆补线******************************************
void Three_Point_Arc_Patching(uint8 Point1_X, uint8 Point1_Y, uint8 Point2_X, uint8 Point2_Y, uint8 Point3_X, uint8 Point3_Y, uint8 Start, uint8 End, uint8 *border, uint8 Circle_Flag)
{
    uint8 i = 0;
    float Slope_1_2 = 0, Slope_2_3 = 0;
    float Mid_Point_1_2[2] = {0}, Mid_Point_2_3[2] = {0};
    float PerPendicluar_Slope_1_2 = 0, PerPendicluar_Slope_2_3 = 0;
    float Intercept_1_2 = 0, Intercept_2_3 = 0;
    float Cir_Cent[2] = {0};
    float Cir_R = 0;
//计算两点之间的斜率
    if(Point1_Y == Point2_Y)
    {
        Slope_1_2 = ((float)(Point2_Y + 1 - Point1_Y) / (float)(Point2_X - Point1_X));
    }
    else
    {
        Slope_1_2 = ((float)(Point2_Y - Point1_Y) / (float)(Point2_X - Point1_X));
    }

    if(Point2_Y == Point3_Y)
    {
        Slope_2_3 = ((float)(Point3_Y + 1 - Point2_Y) / (float)(Point3_X - Point2_X));
    }
    else
    {
        Slope_2_3 = ((float)(Point3_Y - Point2_Y) / (float)(Point3_X - Point2_X));
    }

//计算两点之间的中点
    Mid_Point_1_2[0] = ((float)(Point1_X + Point2_X) / 2.0f);
    Mid_Point_1_2[1] = ((float)(Point1_Y + Point2_Y) / 2.0f);

    Mid_Point_2_3[0] = ((float)(Point2_X + Point3_X) / 2.0f);
    Mid_Point_2_3[1] = ((float)(Point2_Y + Point3_Y) / 2.0f);

//计算通过两点中点的垂直线的斜率
    PerPendicluar_Slope_1_2 = -(1.0 / Slope_1_2);
    PerPendicluar_Slope_2_3 = -(1.0 / Slope_2_3);

//计算直线与y轴交点
    Intercept_1_2 = Mid_Point_1_2[1] - PerPendicluar_Slope_1_2 * Mid_Point_1_2[0];
    Intercept_2_3 = Mid_Point_2_3[1] - PerPendicluar_Slope_2_3 * Mid_Point_2_3[0];

//计算两条直线交点,即圆心
    Cir_Cent[0] = (Intercept_2_3 - Intercept_1_2) / (PerPendicluar_Slope_1_2 - PerPendicluar_Slope_2_3);
    Cir_Cent[1] = PerPendicluar_Slope_1_2 * Cir_Cent[0] + Intercept_1_2;

    if(Cir_Cent[0] == 0 && Cir_Cent[1] == 0)
    {
        Cir_Cent[0] = Last_Cir_Cent[0];
        Cir_Cent[1] = Last_Cir_Cent[1];
    }
    else
    {
        Last_Cir_Cent[0] = Cir_Cent[0];
        Last_Cir_Cent[1] = Cir_Cent[1];
    }
//计算圆的半径
    float dx = Cir_Cent[0] - Point2_X;
    float dy = Cir_Cent[1] - Point2_Y;
    Cir_R = sqrt(dx * dx + dy * dy);

    if(Cir_R == 0)
    {
        Cir_R = Last_Cir_R;
    }
    else
    {
        Last_Cir_R = Cir_R;
    }

//补曲线
    if(Circle_Flag == 1)
    {
        for(i = Start; i <= End; i++)
        {
            // 移植注释：Pa093 - float→uint8，改为经int中转
            // border[i] = (uint8)sqrt(Cir_R * Cir_R - (i - Cir_Cent[1]) * (i - Cir_Cent[1])) + Cir_Cent[0];
            border[i] = (uint8)((int)sqrt(Cir_R * Cir_R - (i - Cir_Cent[1]) * (i - Cir_Cent[1])) + Cir_Cent[0]);
            Limit_u8(border[i], 2, 77);
        }
    }
    else if(Circle_Flag == 2)
    {
        for(i = Start; i <= End; i++)
        {
            // 移植注释：Pa093 - float→uint8，改为经int中转
            // border[i] = (uint8)sqrt(Cir_R * Cir_R - (i - Cir_Cent[1]) * (i - Cir_Cent[1])) - Cir_Cent[0];
            border[i] = (uint8)((int)sqrt(Cir_R * Cir_R - (i - Cir_Cent[1]) * (i - Cir_Cent[1])) - Cir_Cent[0]);
            Limit_u8(border[i], 2, 77);
        }
    }
}
//*************************************************************************************************

//****************************************贝塞尔补线************************************************
//二次贝塞尔曲线
void Quadratic_BezierCurve(uint8 Point1_X, uint8 Point1_Y, uint8 Point2_X, uint8 Point2_Y, uint8 Point3_X, uint8 Point3_Y, float Step_Dis, uint8 Start, uint8 End, uint8 *border, uint8 Direction_Flag)
{
    float t = 1.0f;
    uint8 i = 0;

    if(Direction_Flag == 1)
    {
        i = Start;
    }
    else if(Direction_Flag == 2)
    {
        i = End;
    }

    // 移植注释：Pe550 - Last_Temp_X 只被赋值从未被读取，注释声明行
    // uint8 Last_Temp_X = 0;
    uint8 Last_Temp_Y = 0;

    uint8 Temp_X = 0;
    uint8 Temp_Y = 0;
    for(t = 0.0f; t <= 1.0f; t += Step_Dis)
    {
        float X_K1 = pow((1.0f - t), 2);
        float X_K2 = 2.0f * (1.0f - t) * Step_Dis;
        float X_K3 = pow(t, 2);

        float Y_K1 = pow((1.0f - t), 2);
        float Y_K2 = 2.0f * (1.0f - t) * Step_Dis;
        float Y_K3 = pow(t, 2);

        // Last_Temp_X = Temp_X;  // 移植注释：Pe550 - 对应声明已注释
        Last_Temp_Y = Temp_Y;

        Temp_X = (uint8)(X_K1 * (float)Point1_X + X_K2 * (float)Point2_X + X_K3 * (float)Point3_X);
        Temp_Y = (uint8)(Y_K1 * (float)Point1_Y + Y_K2 * (float)Point2_Y + Y_K3 * (float)Point3_Y);

        switch(Direction_Flag)
        {
            case 1:         //由上到下
            {
                if(Temp_Y > Last_Temp_Y)
                {
                    border[i] = Temp_X;
                    if(i < End)
                    {
                        i ++;
                    }
                }
                break;
            }
            case 2:         //由下到上
            {
                if(Temp_Y < Last_Temp_Y)
                {
                    border[i] = Temp_X;
                    if(i > Start)
                    {
                        i --;
                    }
                }
            }
        }
    }
}

uint8 Mid_Point_Num = 0;
//三次贝塞尔曲线
void Cubic_BezierCurve(uint8 Point1_X, uint8 Point1_Y, uint8 Point2_X, uint8 Point2_Y, uint8 Point3_X, uint8 Point3_Y, float Step_Dis, uint8 Start, uint8 End, uint8 *border, uint8(*line)[2])
{
    float t = 1.0f;
    uint8 i = Start;

    // 移植注释：Pe550 - Last_Temp_X 只被赋值从未被读取，注释声明行
    // uint8 Last_Temp_X = 0;
    uint8 Last_Temp_Y = 0;

    uint8 Temp_X = 0;
    uint8 Temp_Y = 0;

    float P1_X = Point1_X, P2_X = 0, P3_X = Point2_X, P4_X = Point3_X;
    float P1_Y = Point1_Y, P2_Y = 0, P3_Y = Point2_Y, P4_Y = Point3_Y;

    P2_X = line[Mid_Point_Num][0];
    P2_Y = line[Mid_Point_Num][1];

    for(t = 0.0f; t <= 1.0f; t += Step_Dis)
    {
        // Last_Temp_X = Temp_X;  // 移植注释：Pe550 - 对应声明已注释
        Last_Temp_Y = Temp_Y;

        Temp_X = (uint8)(pow((1 - t), 3) * P1_X + 3 * pow((1 - t), 2) * t * P2_X + 3 * (1 - t) * pow(t, 2) * P3_X + pow(t, 3) * P4_X);
        Temp_Y = (uint8)(pow((1 - t), 3) * P1_Y + 3 * pow((1 - t), 2) * t * P2_Y + 3 * (1 - t) * pow(t, 2) * P3_Y + pow(t, 3) * P4_Y);

        if(Temp_Y > Last_Temp_Y)
        {
            border[i] = Temp_X;
            if(i < End)
            {
                i ++;
            }
        }
    }
}
//*************************************************************************************************
//*********************************************************************************************************************************

//**************************************补直线代码**************************************************
void Unilateral_Patrol(uint8 Y_1, uint8 Y_2, uint8 Start, uint8 End, uint8 *border)
{
    uint8 i = 0;
    float Temp_Slope_Rate = 0;
    float Temp_Intercept = 0;

    Calculate_Slope_Intercept(Y_1, Y_2, border, &Temp_Slope_Rate, &Temp_Intercept);

    for(i = Start; i < End; i++)
    {
        // 移植注释：Pa093 - float→uint8，改为经int中转
        // border[i] = (uint8)(i * Temp_Slope_Rate + Temp_Intercept);
        border[i] = (uint8)(int)(i * Temp_Slope_Rate + Temp_Intercept);
        Limit_u8(border[i], 2, 77);
    }
}

//*************************************************************************************************
#define L_Patching      1
#define R_Patching      2
#define L_R_Patching    3

#define L_Arc_Patching_State_2     4
#define R_Arc_Patching_State_2     5
#define L_Arc_Patching_State_4     6
#define R_Arc_Patching_State_4     7

#define L_Unilateral_Patrol         1
#define R_Unilateral_Patrol         2

#define L_Arc_Unilateral_Patrol_2     3
#define R_Arc_Unilateral_Patrol_2     4
#define L_Arc_Unilateral_Patrol_4     5
#define R_Arc_Unilateral_Patrol_4     6

void Get_Middle_Line(uint8 *l_border, uint8 *r_border, uint8 *c_line, uint8 point_num, uint8 patching_flag, uint8 unilateral_patrol_flag)
{
    uint8 i = 0;

    switch(patching_flag)   //补线
    {
        case 0:
        {
            break;
        }
        case L_Patching:
        {
            //Unilateral_Patrol();
            break;
        }
        case R_Patching:
        {
            break;
        }
        case L_R_Patching:
        {
            for(i = 2; i < point_num + 2; i++)
            {
                c_line[i] = (l_border[i] + r_border[i]) / 2;
            }
            break;
        }
        case L_Arc_Patching_State_2:
        {
            switch(L_Circle_Flag)
            {
                case 2:         //三点求圆
                {
                    Get_Three_Point(L_Line, R_Line, L_Border, R_Border, L_Statics, L_Start_Point, R_Start_Point, 1, 2, 1);
                    Three_Point_Arc_Patching(Arc_Point_1[0], Arc_Point_1[1], Arc_Point_2[0], Arc_Point_2[1], Arc_Point_3[0], Arc_Point_3[1], Arc_Point_1[1], 57, R_Border, 1);
                    break;
                }
                case 3:         //贝塞尔二次补线
                {
//                    Get_BezierCurve_Three_Point(L_Line, L_Statics, 1, 0);
                    Get_Three_Point(L_Line, R_Line, L_Border, R_Border, L_Statics, L_Start_Point, R_Start_Point, 1, 2, 0);
                    Quadratic_BezierCurve(Arc_Point_3[0], Arc_Point_3[1], Arc_Point_2[0], Arc_Point_2[1], Arc_Point_1[0], Arc_Point_1[1], 0.01, Arc_Point_1[1], 57, R_Border, 2);
                    break;
                }
                case 4:         //贝塞尔三次补线
                {
//                    Get_BezierCurve_Three_Point(L_Line, L_Statics, 1, 0);
                    Get_Three_Point(L_Line, R_Line, L_Border, R_Border, L_Statics, L_Start_Point, R_Start_Point, 1, 2, 0);
                    Cubic_BezierCurve(Arc_Point_1[0], Arc_Point_1[1], Arc_Point_2[0], Arc_Point_2[1], Arc_Point_3[0], Arc_Point_3[1], 0.01, Arc_Point_1[1], 57, R_Border, L_Line);
                    break;
                }
                case 5:         //拉格朗日差值法
                {
                    break;
                }
            }
            X_Meet = Arc_Point_1[0];
            Y_Meet = Arc_Point_1[1];
            break;
        }
        case L_Arc_Patching_State_4:
        {
            switch(L_Circle_Flag)
            {
                case 2:         //三点求圆
                {
                    Get_Three_Point(L_Line, R_Line, L_Border, R_Border, R_Statics, L_Start_Point, R_Start_Point, 1, 4, 1);
                    Three_Point_Arc_Patching(Arc_Point_1[0], Arc_Point_1[1], Arc_Point_2[0], Arc_Point_2[1], Arc_Point_3[0], Arc_Point_3[1], Arc_Point_3[1], Arc_Point_1[1], R_Border, 1);
                    break;
                }
                case 3:         //贝塞尔二次补线
                {
                    Get_Three_Point(L_Line, R_Line, L_Border, R_Border, R_Statics, L_Start_Point, R_Start_Point, 1, 4, 0);
                    Quadratic_BezierCurve(Arc_Point_1[0], Arc_Point_1[1], Arc_Point_2[0], Arc_Point_2[1], Arc_Point_3[0], Arc_Point_3[1], 0.01, Arc_Point_1[1], 57, R_Border, 2);
                    break;
                }
                case 4:         //贝塞尔三次补线
                {
                    Get_Three_Point(L_Line, R_Line, L_Border, R_Border, R_Statics, L_Start_Point, R_Start_Point, 1, 4, 0);
                    Cubic_BezierCurve(Arc_Point_1[0], Arc_Point_1[1], Arc_Point_2[0], Arc_Point_2[1], Arc_Point_3[0], Arc_Point_3[1], 0.01, Arc_Point_1[1], 57, R_Border, R_Line);
                    break;
                }
                case 5:         //拉格朗日差值法
                {
                    break;
                }
            }
            X_Meet = Arc_Point_3[0];
            Y_Meet = Arc_Point_3[1];
            break;
        }
        case R_Arc_Patching_State_2:
        {
            switch(R_Circle_Flag)
            {
                case 2:
                {
                    Get_Three_Point(L_Line, R_Line, L_Border, R_Border, R_Statics, L_Start_Point, R_Start_Point, 2, 2, 1);
                    Three_Point_Arc_Patching(Arc_Point_1[0], Arc_Point_1[1], Arc_Point_2[0], Arc_Point_2[1], Arc_Point_3[0], Arc_Point_3[1], Arc_Point_1[1], 57, L_Border, 2);
                    break;
                }
                case 3:         //贝塞尔二次补线
                {
                    Get_Three_Point(L_Line, R_Line, L_Border, R_Border, R_Statics, L_Start_Point, R_Start_Point, 2, 2, 0);
                    Quadratic_BezierCurve(Arc_Point_3[0], Arc_Point_3[1], Arc_Point_2[0], Arc_Point_2[1], Arc_Point_1[0], Arc_Point_1[1], 0.01, Arc_Point_1[1], 57, L_Border, 2);
                    break;
                }
                case 4:         //贝塞尔三次补线
                {
                    Get_Three_Point(L_Line, R_Line, L_Border, R_Border, R_Statics, L_Start_Point, R_Start_Point, 2, 2, 0);
                    Cubic_BezierCurve(Arc_Point_1[0], Arc_Point_1[1], Arc_Point_2[0], Arc_Point_2[1], Arc_Point_3[0], Arc_Point_3[1], 0.01, Arc_Point_1[1], 57, L_Border, R_Line);
                    break;
                }
                case 5:         //拉格朗日差值法
                {
                    break;
                }
            }
            X_Meet = Arc_Point_1[0];
            Y_Meet = Arc_Point_1[1];
            break;
        }
        case R_Arc_Patching_State_4:
        {
            switch(R_Circle_Flag)
            {
                case 2:         //三点求圆
                {
                    Get_Three_Point(L_Line, R_Line, L_Border, R_Border, L_Statics, L_Start_Point, R_Start_Point, 2, 4, 1);
                    Three_Point_Arc_Patching(Arc_Point_1[0], Arc_Point_1[1], Arc_Point_2[0], Arc_Point_2[1], Arc_Point_3[0], Arc_Point_3[1], Arc_Point_3[1], Arc_Point_1[1], L_Border, 2);
                    break;
                }
                case 3:         //贝塞尔二次补线
                {
                    Get_Three_Point(L_Line, R_Line, L_Border, R_Border, L_Statics, L_Start_Point, R_Start_Point, 2, 4, 0);
                    Quadratic_BezierCurve(Arc_Point_1[0], Arc_Point_1[1], Arc_Point_2[0], Arc_Point_2[1], Arc_Point_3[0], Arc_Point_3[1], 0.01, Arc_Point_1[1], 57, L_Border, 1);
                    break;
                }
                case 4:         //贝塞尔三次补线
                {
                    Get_Three_Point(L_Line, R_Line, L_Border, R_Border, L_Statics, L_Start_Point, R_Start_Point, 2, 4, 0);
                    Cubic_BezierCurve(Arc_Point_1[0], Arc_Point_1[1], Arc_Point_2[0], Arc_Point_2[1], Arc_Point_3[0], Arc_Point_3[1], 0.01, Arc_Point_1[1], 57, L_Border, L_Line);
                    break;
                }
                case 5:         //拉格朗日差值法
                {
                    break;
                }
            }
            X_Meet = Arc_Point_3[0];
            Y_Meet = Arc_Point_3[1];
            break;
        }
    }

    switch(unilateral_patrol_flag)  //单边巡线
    {
        case 0:
        {
            for(i = 2; i < point_num + 2; i++)
            {
                c_line[i] = (l_border[i] + r_border[i]) / 2;
            }
            break;
        }
        case L_Unilateral_Patrol: // 单边巡左线，已适配
        {
            uint8 L_Staright[60] = { 0, 0, 0, 6, 7, 7, 7, 8, 8, 9,
                                     9, 10, 10, 10, 11, 11,12,12,13,13,
                                    13,14,14,15,16,16,16,17,17,18,
                                    18,18,19,19,20,20,21,22,22,23,
                                    23,24,25,25,25,26,26,27,27,28,
                                    28,28,29,29,30,30,31,32,0,0};
            for(i = 2; i < point_num + 2; i++)
            {
                c_line[i] = l_border[i] + L_Staright[i];
            }
            break;
        }
        case R_Unilateral_Patrol: // 单边巡右线，已适配
        {
            uint8 R_Staright[60] = { 0, 0, 0, 7, 8, 8, 9, 9, 10, 10,
                                     11, 11, 12, 12, 13, 13,14,14,15,15,
                                    16,16,17,17,18,18,19,19,20,20,
                                    21,21,22,22,23,23,23,23,24,24,
                                    25,25,25,26,27,27,28,28,28,29,
                                    29,30,31,31,31,32,32,32,0,0};
            for(i = 2; i < point_num + 2; i++)
            {
                c_line[i] = r_border[i] - R_Staright[i];
            }
            break;
        }
        case L_Arc_Unilateral_Patrol_2:
        {
            uint8 L_Arc_1[60] = { 0, 0, 39, 39, 39, 39, 39, 39, 39, 39,
                    39, 39, 39, 39, 39, 39,39,39,39,14,
                   16,17,19,20,21,22,24,19,21,19,
                   19,18,18,18,18,18,18,18,19,20,
                   21,20,21,22,22,22,22,23,24,24,
                   24,24,24,24,24,24,25,25,0,0};
            for(i = 2; i < point_num + 2; i++)
            {
                c_line[i] = l_border[i] + L_Arc_1[i];
            }
            break;
        }
        case R_Arc_Unilateral_Patrol_2:
        {
            uint8 R_Arc_1[60] =  { 0, 0, 39, 39, 39, 39, 39,39, 39, 39,
                    39, 39, 39, 39, 39, 39,39,39,39,39,
                   15,17,19,20,22,23,23,24,25,26,
                   23,22,21,22,22,22,22,23,23,24,
                   23,23,24,24,24,25,25,25,26,27,
                   27,28,28,28,28,29,29,30,0,0};
            for(i = 2; i < point_num + 2; i++)
            {
                c_line[i] = r_border[i] - R_Arc_1[i];
            }
            break;
        }
        case L_Arc_Unilateral_Patrol_4:
        {
            uint8 L_Arc_2[60] = { 0, 0, 39, 39, 39, 39, 39,39, 39, 39,
                    39, 39, 39, 39, 39, 39,39,39,39,39,
                   15,17,19,20,22,23,23,24,25,26,
                   23,22,21,22,22,22,22,23,23,24,
                   23,23,24,24,24,25,25,25,26,27,
                   27,28,28,28,28,29,29,30,0,0};
            for(i = 2; i < point_num + 2; i++)
            {
                c_line[i] = l_border[i] + L_Arc_2[i];
            }
            break;
        }
        case R_Arc_Unilateral_Patrol_4:
        {
            uint8 R_Arc_2[60] =  { 0, 0, 0, 7, 8, 8, 9, 9, 10, 10,
                    11, 11, 12, 12, 13, 13,14,14,15,15,
                   16,16,17,17,18,18,19,19,20,20,
                   21,21,22,22,23,23,23,23,24,24,
                   25,25,25,26,27,27,28,28,28,29,
                   29,30,31,31,31,32,32,32,0,0};
            for(i = 2; i < point_num + 2; i++)
            {
                c_line[i] = r_border[i] - R_Arc_2[i];
            }
            break;
        }
    }
    Triangular_Filter(C_Line, Y_Meet + 1, 57);
}


float Last_Curvature_Value = 0;     //防止
/**
* 函数功能：      计算中线曲率，作为控制速度的一个数据
* 特殊说明：      无
* 形  参：        uint8 *line      中线
*                 uint8 start      爬线相遇点的Y值，或最大有效行
*                 uint8 end        爬线起始行（或图像最底端）
*
* 示例：          Calculate_Curvature_2(C_Line, Y_Meet, L_Start_Point[1]);
* 返回值：        Last_Curvature_Value      所得曲率
*/
float Calculate_Curvature_2(uint8 *line, uint8 start, uint8 end)
{
    int16 Sum_Difference_Value = 0;     //用于X坐标差值的累积
    int16 Sum_Weight = 0;               //用于权值相加
    uint8 i = 0;

    uint8 Base_Curvature_Weight[60] = {0, 0, 1, 0, 1, 0, 1, 0, 1, 0,
                                       1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
                                       1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
                                       1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
                                       1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
                                       1, 0, 1, 0, 1, 0, 1, 0, 1, 0};       //固定权值，0,1,0,1间隔加权
    uint8 Trends_Curvature_Weight[19] = {4, 5, 5, 6, 6, 7, 7, 8, 8, 8, 8, 8, 7, 7, 6, 6, 5, 5, 4};      //动态权值，高权值放中间

    uint8 Start_Line = (uint8)((float)(Y_Meet - 2) / 18.0f * 16.0f + 24.0f);    //动态权替换固定权起始行，我图像为60行，最低从24行向上替换，最高为40行向上替换，Y_Meet最大值为20

    //将动态权值赋值给固定权值数组
    for(i = 0; i < 19; i ++)
    {
        Base_Curvature_Weight[Start_Line - i] = Trends_Curvature_Weight[i];
    }

    for(i = start; i < end - 1; i++)    //求差值和并求出权值
    {
        Sum_Difference_Value += (int16)(My_ABS((int)line[i] - (int)line[i + 1]) * (int)Base_Curvature_Weight[i]);
        Sum_Weight += (int16)Base_Curvature_Weight[i];
    }

    if(Sum_Weight != 0)     //防止权值为0，但好像不会出现，当时不知道为啥要写这个
    {
        Last_Curvature_Value = (float)Sum_Difference_Value / (float)Sum_Weight;     //求加权平均值，归一化放在了另一个函数里
        return Last_Curvature_Value;
    }
    else
    {
        return Last_Curvature_Value;
    }
}


//uint8 Curvature_Weight[60] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
//                              1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
//                              1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
//                              1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
//                              1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
//                              1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
//float Last_Curvature_Value = 0;
//float Calculate_Curvature_2(uint8 *line, uint8 start, uint8 end)
//{
//    int16 Sum_Difference_Value = 0;
//    int16 Sum_Weight = 0;
//    uint8 i = 0;
//
//    for(i = start; i < end - 1; i++)
//    {
//        Sum_Difference_Value += (int16)(My_ABS((int8)line[i] - (int8)line[i + 1]) * (int8)Curvature_Weight[i]);
//        Sum_Weight += (int16)Curvature_Weight[i];
//    }
//
//    if(Sum_Weight != 0)
//    {
//        Last_Curvature_Value = (float)Sum_Difference_Value / (float)Sum_Weight;
////        ips200_show_float(100, 200, Last_Curvature_Value, 4, 4);
//        return Last_Curvature_Value;
//    }
//    else
//    {
//        return Last_Curvature_Value;
//    }
//}

float Speed_Value[7] = {0.0f, 0.166f, 0.333f, 0.5f, 0.666f, 0.833f, 1.0f};  //速度采用了归一化，将1.0均分为六段

float Speed_Vague_Array[4][4] = { {0, 1, 2, 3},
                                  {1, 2, 3, 4},
                                  {3, 4, 5, 6},
                                  {5, 6, 6, 6}};    //映射数组

int16 Y_Meet_Count = 0;     //累积满足相遇点条件的次数，比如由弯道进入直道时，车身未转正时Y_Meet已经趋近于2了，此时加速会导致车轨迹不稳
                            //那我们就进行一定的缓冲，当Y_Meet满足我们设定的条件一定次数后，再去执行相应的加速程序，可确保车身转正再加速
/**
* 函数功能：      使用相遇点和曲率映射速度，参照模糊PID思想
* 特殊说明：      无
* 形  参：        float sensitivity                //曲率
*                 float speed_min_proportion       //速度最小比率（设定最小速度与最大速度的比值），为0~1之间
*                 float Speed_max_proportion       //速度最大比率，通常为1
*                 uint8 y_meet                     //爬线相遇点的Y坐标
*                 uint8 min_y_meet                 //爬线相遇点的最小值，看得越远值越小
*                 uint8 max_y_meet                 //爬线相遇点的最大值，看得越远值越大
*
* 示例：          Calculate_Curvature_2(C_Line, Y_Meet, L_Start_Point[1]);
* 返回值：        Last_Curvature_Value      所得曲率
*/
uint16 max_speed_temp = 0;
uint16 min_speed_temp = 0;
float Speed_Mapping(float sensitivity, float speed_min_proportion, float Speed_max_proportion, uint8 y_meet, uint8 min_y_meet, uint8 max_y_meet)
{
    // 移植注释：Pe177 - 变量声明但从未引用，注释此行
    // uint16 i = 0;
    float VH = 0, VE = 0;
    float X2Y = 0;
    float X1Y = 0;
    float Y2X = 0;
    float Y1X = 0;
    // 移植注释：Pe177 - 变量声明但从未引用，注释此行
    // uint16 Mid_Speed = (uint16)((Min_Speed+Max_Speed)/2);
    uint8 View_Effictive_Line = max_y_meet - min_y_meet;

//    //弯道、环岛、十字时设定最大速度为35（这是我们组控制位给的一个值，换成自己组的）
//    if(Element_State == L_Turn ||  Element_State == R_Turn || Element_State == L_Circle || Element_State == R_Circle || Element_State == Cross)
//    {
//        Y_Meet_Count = 0;
//        max_speed_temp = Mid_Speed;
//    }
//    //下面就是所说的缓冲代码，弯道到直道切换时，爬线相遇点Y趋近于2，当满足一千次以后（计算一千张图像），将最大速度设定为40
//    //另一方面更重要的是为了防止误判，导致乱加减速
//    //冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！
//    //冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！
//    //冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！冲！
//    else if(Y_Meet <= 4 && max_speed_temp == Mid_Speed)
//    {
//        Y_Meet_Count ++;
//        if(Y_Meet_Count >= 1000)
//        {
//            Y_Meet_Count = 0;
//            max_speed_temp = Max_Speed;
//        }
//    }
//    //一次不满足就重新来，比较严格，可以不用或者判定放缓一些
//    else if(Y_Meet != 2 && max_speed_temp == Mid_Speed)
//    {
//        max_speed_temp = Mid_Speed;
//        Y_Meet_Count = 0;
//    }
//    //直道到弯道的缓冲（作用于弯道在图像最远端，还未识别到弯道时），这个缓冲时间短一些，减速快些，这两个计数值根据自己情况调整
//    else if(Y_Meet != 2 && max_speed_temp == Max_Speed)
//    {
//        Y_Meet_Count --;
//        if(Y_Meet_Count <= -350)
//        {
//            Y_Meet_Count = 0;
//            max_speed_temp = Mid_Speed;
//        }
//    }

    //获取左右两侧边线落在边框上的个数
    Get_L_Border_Point_Num();
    Get_R_Border_Point_Num();

    //当识别出元素是直道，爬线相遇点位于顶端，左右两侧几乎没有落在边框上的点，并且中线曲率很小时，直接返回最大速度开冲！
    //此处判定十分严格，不会误判，不满足时才执行下方速度拟合部分代码
    if(Element_State == 2 && Y_Meet <= 3 && L_Border_Point_Num <= 2 && R_Border_Point_Num <= 2 && sensitivity <= 0.2f)
    {
        return Max_Speed;
    }
    //下面使用曲率和爬线相遇点Y坐标拟合速度
    //与模糊PID原理相同，不过多注释
    VH = ((float)(Y_Meet - 2) * 3.0f / (float)View_Effictive_Line);
    VE = sensitivity * 3.0f;

    int8 VH1 = (int8)VH;
    if (VH1 > VH)
    {
      VH--;
    }
    int8 VH2 = VH1 + 1;

    int8 VE1 = (int8)VE;
    if (VE1 > VE)
    {
      VE1--;
    }
    int8 VE2 = VE1 + 1;

    if (VH1 > 3)
    {
      VH1 = 3;
    }

    if (VH2 > 3)
    {
      VH2 = 3;
    }

    if (VE1 > 3)
    {
      VE1 = 3;
    }

    if (VE2 > 3)
    {
      VE2 = 3;
    }

    X2Y = (Speed_Vague_Array[VH1][VE2] - Speed_Vague_Array[VH1][VE1]) * (VE - VE1) + Speed_Vague_Array[VH1][VE1];

    X1Y = (Speed_Vague_Array[VH2][VE2] - Speed_Vague_Array[VH2][VE1]) * (VE - VE1) + Speed_Vague_Array[VH2][VE1];

    Y2X = (Speed_Vague_Array[VH2][VE1] - Speed_Vague_Array[VH1][VE1]) * (VH - VH1) + Speed_Vague_Array[VH1][VE1];

    Y1X = (Speed_Vague_Array[VH2][VE2] - Speed_Vague_Array[VH1][VE2]) * (VH - VH1) + Speed_Vague_Array[VH1][VE2];

    float Speed_approximation = (X2Y + X1Y + Y2X + Y1X) / 4.0;

    int8 Speed_1 = (int8)Speed_approximation;
    if (Speed_1 > Speed_approximation)
    {
        Speed_1--;
    }
    int8 Speed_2 = Speed_1 + 1;

    return (1.0f - ((Speed_Value[Speed_2] - Speed_Value[Speed_1]) * (Speed_approximation - Speed_1) + Speed_Value[Speed_1])) * (float)(max_speed_temp - Min_Speed) + (float)Min_Speed;   //返回拟合好的速度
}


float Last_Sensitivity = 0;
float Temp_Sensitivity = 0;
float Speed_Proportion_Array[100] = {0};    //用于滑动均值滤波
/**
* 函数功能：      进行速度控制使用相遇点和曲率映射速度，参照模糊PID思想
* 特殊说明：      无
* 形  参：        uint8 Auto_Control_Flag           是否要车辆自行控制速度
*                                                   //当自动控制标志位为1时，速度根据中线曲率和相遇点进行拟合
*                                                   //当自动控制标志位为0时，速度由手动输入
*                 uint16 Manual_Control_Value       //手动控制速度
*
* 示例：          Fitting_Speed_Control(1, 0);
* 返回值：        无
*/
void Fitting_Speed_Control(uint8 Auto_Control_Flag, uint16 Manual_Control_Value, uint8 element, uint8 element_state, uint8 start, uint8 end)
{
    uint8 i = 0;
    if(Auto_Control_Flag == 1)
    {
        float Temp_Speed_Proportion = 0;
        //计算中线曲率，用于速度控制
        Last_Sensitivity = Sensitivity;
        Temp_Sensitivity = Calculate_Curvature_2(C_Line, Y_Meet, L_Start_Point[1]) / 1.0f;

        //计算最小速度比率
        Speed_Min_Proportion = (float)Min_Speed / (float)Max_Speed;

        //Stretch_Coefficient：减速拉伸系数，此参数越大，减速距离越短，最大值为1
        //Stretch_Coefficient 等于手动设定一个曲率最小阈值，小于这个阈值时直接将曲率设为0
        if(Temp_Sensitivity <= Stretch_Coefficient)
        {
            Sensitivity = 0.0f;
        }
        else    //将（阈值~1.0）之间的数进行拉伸，可以代入几个实际的数理解下这部分原理
        {
            float Temp_Float_Num = (Temp_Sensitivity - Stretch_Coefficient) * (1.0f / (1.0f - Stretch_Coefficient));
            Sensitivity = Limit_Float(Temp_Float_Num * 0.3f + Last_Sensitivity * 0.7f, 0.01f, 1.0f);    //曲率值滤波
        }

        standardized_curvature_ave = Sensitivity;

        Temp_Speed_Proportion = Speed_Mapping(Sensitivity, Speed_Min_Proportion, 1.0f, Y_Meet, 2, 20);  //计算速度
        Speed_Proportion = Limit_Float(Temp_Speed_Proportion, (float)Min_Speed, (float)Max_Speed);  //速度限幅

        //滑动均值滤波，防止速度突变
        float Speed_Proportion_Sum = 0;
        for(i = 0; i < 99; i ++)
        {
            Speed_Proportion_Array[i + 1] = Speed_Proportion_Array[i];
        }
        Speed_Proportion_Array[0] = Speed_Proportion;
        for(i = 0; i < 100; i ++)
        {
            Speed_Proportion_Sum += Speed_Proportion_Array[i];
        }
        Speed_Proportion = Speed_Proportion_Sum / 100.0f;
        Yao.Target_Speed = (int)Speed_Proportion;
    }
    else if(Auto_Control_Flag == 0)
    {
        Yao.Target_Speed = Manual_Control_Value;
    }
}

/******************** 模糊PID *********************/
float Dif_Effictive_Line = 40.0f;    //偏差值最大值
int16 View_Effictive_Line = 30;      //相遇点Y坐标的最大值（需要改）

float P_Value_L[7] = {400, 450, 470, 480, 510, 520, 550};    //这里没有等分，自己调一调车可摸出规律（需要改）

float Vague_Array[4][4] = { {0, 1, 2, 3},
                            {1, 2, 3, 4},
                            {3, 4, 5, 6},
                            {5, 6, 6, 6}};    //这个表原样抄下来

float f_Get_H_approximation(int16 i16_ViewH)
{
  float H_approximation;

  if (i16_ViewH < 0)
  {
    i16_ViewH = 0;
  }

  H_approximation = ((float)i16_ViewH * 3.0f / (float)View_Effictive_Line);    //*3.0是为了将结果放大三倍

  return H_approximation;
}

float f_Get_E_approximation(float i16_E)
{
  float E_approximation;

  if (i16_E < 0)
  {
    i16_E = -i16_E;
  }

  E_approximation = (float)i16_E * 40.0f * 3.0f/ Dif_Effictive_Line;    //*3.0与上述同理，还多乘了个四十，与Dif_Effictive_Line 的值对应上

  return E_approximation;
}

int16 Off_Line = 0;
int16 Now_Off_Line = 0;    //用于滤波
int16 Last_Off_Line = 0;    //用于滤波

// 第一个参数输入相遇点Y坐标
// 第二个参数输入归一化后的偏差值
float Get_P(int16 off_line, float dif_value)
{
//    float temp = (P_Value_L[6] - P_Value_L[0])/6;
//
//    P_Value_L[1] = P_Value_L[0] + temp * 1;
//    P_Value_L[2] = P_Value_L[0] + temp * 2;
//    P_Value_L[3] = P_Value_L[0] + temp * 3;
//    P_Value_L[4] = P_Value_L[0] + temp * 4;
//    P_Value_L[5] = P_Value_L[0] + temp * 5;

    Last_Off_Line = Now_Off_Line;
    Now_Off_Line = off_line;

    if(((Now_Off_Line - Last_Off_Line) >= 10) || ((Now_Off_Line - Last_Off_Line) <= -10) || Now_Off_Line >= 45)    //需要改
    {
        Off_Line = Last_Off_Line;
    }
    else
    {
        Off_Line = (int16)(0.3f * (float)(Now_Off_Line) + 0.7f * (float)(Last_Off_Line));
    }

    //下面这部分代入几个数据，结合整段代码计算几遍即可理解，只看的话比较吃力
    float VH = f_Get_H_approximation(off_line - 2);
    float VE = f_Get_E_approximation(dif_value);
    float X2Y = 0;
    float X1Y = 0;
    float Y2X = 0;
    float Y1X = 0;

    int8 VH1 = (int)VH;
    if (VH1 > VH)
    {
      VH--;
    }
    int8 VH2 = VH1 + 1;

    int8 VE1 = (int8)VE;
    if (VE1 > VE)
    {
      VE1--;
    }
    int8 VE2 = VE1 + 1;

    if (VH1 > 3)
    {
      VH1 = 3;
    }

    if (VH2 > 3)
    {
      VH2 = 3;
    }

    if (VE1 > 3)
    {
      VE1 = 3;
    }

    if (VE2 > 3)
    {
      VE2 = 3;
    }

    X2Y = (Vague_Array[VH1][VE2] - Vague_Array[VH1][VE1]) * (VE - VE1) + Vague_Array[VH1][VE1];

    X1Y = (Vague_Array[VH2][VE2] - Vague_Array[VH2][VE1]) * (VE - VE1) + Vague_Array[VH2][VE1];

    Y2X = (Vague_Array[VH2][VE1] - Vague_Array[VH1][VE1]) * (VH - VH1) + Vague_Array[VH1][VE1];

    Y1X = (Vague_Array[VH2][VE2] - Vague_Array[VH1][VE2]) * (VH - VH1) + Vague_Array[VH1][VE2];

    float P_approximation = (X2Y + X1Y + Y2X + Y1X) / 4.0;

    int8 P1 = (int8)P_approximation;
    if (P1 > P_approximation)
    {
      P1--;
    }
    int8 P2 = P1 + 1;
    return (P_Value_L[P2] - P_Value_L[P1]) * (P_approximation - P1) + P_Value_L[P1]; //返回p值
}

float Pure_KV = 37;
float Pure_Ld0 = 20;
float Pure_F_A_Distance = 2;        //前后轮轴距，这里两个像素
//纯跟踪控制
//float Pure_Pursuit_Control(uint8 element, uint8 y_meet, float speed)
//{
//    int8 Now_Position_X = 0;
//    int8 Now_Position_Y = 0;
//    int8 Target_Point_Y = 0;
//    int8 Target_Point_X = 0;
//
//    //先根据速度确定预瞄点的X,Y
//    Target_Point_Y = Y_Meet + 1;
//    Target_Point_X = C_Line[Target_Point_Y];
//    //确定车身当前位置，即爬线起始行中点
//    Now_Position_X = (L_Start_Point[0] + R_Start_Point[0]) / 2;
//    Now_Position_Y = (L_Start_Point[1] + R_Start_Point[1]) / 2;
//    //计算α的角度，这里直接调用逆透视代码中的求角度函数
////    float angle = I_Get_Turn_Point_Angle(Target_Point_X, Target_Point_Y, Now_Position_X, Now_Position_Y, Now_Position_X, Now_Position_Y - Pure_F_A_Distance);
//    ips200_show_float(0, 250, angle, 4, 4);
//    float Ld = Pure_KV * (speed - Speed_Min_Proportion) / (1.0f - Speed_Min_Proportion) + Pure_Ld0;
//    ips200_show_float(0, 260, Ld, 4, 4);
//    float Sigma = atanf((4 * sin(angle * 3.1415 / 180)) / Ld);
//    ips200_show_float(0, 270, Sigma, 4, 4);
//    float deviation_value = (Ld * Ld * Sigma) / (2 * Pure_F_A_Distance);
//    ips200_show_float(0, 280, deviation_value, 4, 4);
//
//    //根据车身位置与预瞄点位置确定圆弧轨迹
//    if(Target_Point_X - Now_Position_X == 0)
//    {
//        return 0.0f;
//    }
//    else if(Target_Point_X - Now_Position_X < 0)   //预瞄点位于当前位置左侧
//    {
//        return - deviation_value / 11.0f;
//    }
//    else if(Target_Point_X - Now_Position_X > 0)    //预瞄点位于当前位置右侧
//    {
//        return  deviation_value / 11.0f;
//    }
//
//    return deviation_value;
//}
/*
#define Derailment     1        //出赛道
#define Straightaway   2        //直道
#define L_Turn         3        //左转弯道
#define R_Turn         4        //右转弯道
#define L_Circle       5        //左环岛
#define R_Circle       6        //右环岛
#define Cross          7        //十字路口
#define Zebra          8        //斑马线
#define Ramp           9        //坡道
#define Three_Bif      10       //三岔
#define Barrier        11       //障碍
#define Disconnection  12       //断路
#define T_Way          13       //T路口
#define L_Garage       14       //左车库
#define R_Garage       15       //右车库
#define Zebra          16
*/
float Mid_Line_FFF = 0;
float Mid_Line_FF = 0;
float Mid_Line_F = 0;

float C_Variance = 0;
float Last_C_Variance = 0;

float Last_Deviation_Value = 0;

float Last_Position_conpensate = 0;
float Position_conpensate = 0;

float Cross_Dif_Correct = 0;
uint16 Circle_Temp_Num_2 = 0;
uint8 Small_S_Break_Flag = 0;

int Cross_State_4_Direction = 0;

//偏差值滑动均值滤波数组
float Deviation_Value_Array[20] = {0};
//用于相遇点滑动均值滤波数组
uint8 Y_Meet_Array[20] = {0};

uint8 Error_Line = 40;

void Get_Deviation_Value(uint8 element, uint8 element_state, uint8 start, uint8 end, uint8 *c_line, float power)//, uint8 position_conpensate_flag)
{
    uint8 i = 0;
    float Angle_Err = 0;
    int32 Weight_Sum = 0;
    int32 Sum = 0;
    float deviation_value = 0;
//    float Slope = 0;

    uint8 Oblique_Cross_Temp_Num = 0;
//    float Correct_Value = 0;

    C_Variance  = 0;

    if(Code_type == 2)
    {
        if(element == Cross)
        {
            switch(element_state)
            {
                case 1:     //未入十字
                {
                    uint8 weight_7_1[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 1, 2, 3, 3,   5, 5, 5, 5, 5, 6, 6, 6, 6, 6,
                                             7, 7, 7, 7, 8, 8, 8, 8, 9, 9,   10,10,10,10,10,10,10,10,10,10,   9, 8, 6, 5, 3, 1, 1, 1, 0, 0 };
                    for(i = start; i < end; i++)
                    {
    //                    if(L_Border[i] != 2 && R_Border[i] != 77)
    //                    {
                            Sum += weight_7_1[i] * C_Line[i];
                            Weight_Sum += weight_7_1[i];
    //                    }
                    }
                    break;
                }
                case 2:
                {
                    uint8 weight_7_2[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 1, 3, 5, 5, 5, 7, 8,   8, 8, 8, 9, 9,10,10,10,10,10,
                                            10,10,10,10,10,10,10,10,10,10,   10,10,10,10,10,10,10,10,10,10,   9, 6, 4, 3, 1, 1, 0, 0, 0, 0 };
                    for(i = start; i < end; i++)
                    {
                        if(L_Border[i] != 2 && R_Border[i] != 77)
                        {
                            Sum += weight_7_2[i] * C_Line[i];
                            Weight_Sum += weight_7_2[i];
                        }
                    }
                    break;
                }
                case 3:
                {
                    uint8 weight_7_3[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 1, 1, 1, 1, 2, 2, 3,   3, 3, 4, 5, 6, 6, 7, 7, 7, 7,
                                            10,10,10,10,10,10,10,10,10,10,    9, 8, 7, 6, 5, 5, 4, 4, 4, 4,   3, 3, 3, 1, 1, 1, 0, 0, 0, 0 };
                    for(i = start; i < end; i++)
                    {
                        if(L_Border[i] != 2 && R_Border[i] != 77)
                        {
                            Sum += weight_7_3[i] * C_Line[i];
                            Weight_Sum += weight_7_3[i];
                        }
                    }
                    break;
                }
    //            case 1:     //未入十字
    //            {
    //                uint8 weight_7_1[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 1, 2, 3, 3,      5, 5, 5, 5, 5, 6, 6, 6, 6, 6,
    //                                         7, 7, 7, 7, 7, 7, 7, 7, 8, 8,    8, 8, 8, 9, 9, 10,10,10,10,10,    10,10,10,10,10,10,10, 9, 0, 0 };
    //                for(i = start; i < end; i++)
    //                {
    //                    Sum += weight_7_1[i] * C_Line[i];
    //                    Weight_Sum += weight_7_1[i];
    //                }
    //                break;
    //            }
    //            case 2:
    //            {
    //                uint8 weight_7_2[60] = { 0, 0, 0, 0, 0, 0, 0, 8, 9, 9,    9, 9, 9, 9,10,10,10,10,10,10,   10, 9, 9, 9, 9, 8, 8, 8, 7, 7,
    //                                         6, 5, 4, 4, 3, 2, 2, 2, 2, 2,    2, 1, 2, 1, 2, 1, 2, 1, 2, 1,    2, 1, 2, 1, 2, 1, 2, 1, 0, 0 };
    //                for(i = start; i < end; i++)
    //                {
    //                    Sum += weight_7_2[i] * C_Line[i];
    //                    Weight_Sum += weight_7_2[i];
    //                }
    //                break;
    //            }
    //            case 3:
    //            {
    //                uint8 weight_7_3[60] = { 0, 0, 0, 0, 0, 0, 0, 8, 9, 9,    9, 9, 9, 9,10,10,10,10,10,10,   10, 9, 9, 9, 9, 8, 8, 8, 7, 7,
    //                                         6, 5, 4, 4, 3, 2, 2, 2, 2, 2,    2, 1, 2, 1, 2, 1, 2, 1, 2, 1,    2, 1, 2, 1, 2, 1, 2, 1, 0, 0 };
    //                for(i = start; i < end; i++)
    //                {
    //                    Sum += weight_7_3[i] * C_Line[i];
    //                    Weight_Sum += weight_7_3[i];
    //                }
    //                break;
    //            }
                case 4:
                {
                    uint8 weight_7_4[60] = { 0, 0, 0, 0, 0, 0, 7, 7, 7, 7,     8, 8, 8, 8, 8, 9, 9,10,10,10,    10,10,10,10,10,10,10,10,10,10,
                                            10,10,10,10,10, 8, 8, 8, 8, 9,     9, 9, 9, 8, 8, 7, 7, 5, 3, 2,   2, 2, 2, 1, 1, 1, 0, 0, 0, 0 };
                    for(i = start; i < end; i++)
                    {
                        Sum += weight_7_4[i] * C_Line[i];
                        Weight_Sum += weight_7_4[i];
                    }
                    break;
                }
                case 5:
                {
                    uint8 weight_7_5[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 1, 2, 3, 3,   5, 5, 5, 5, 5, 6, 6, 6, 6, 6,
                                             7, 7, 7, 7, 8, 8, 8, 8, 9, 9,   10,10,10,10,10,10,10,10,10,10,   9, 8, 6, 5, 3, 1, 1, 1, 0, 0 };
                    for(i = start; i < end; i++)
                    {
                        Sum += weight_7_5[i] * C_Line[i];
                        Weight_Sum += weight_7_5[i];
                    }
                    break;
                }
                case 6:
                {
                    uint8 weight_7_6[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 1, 2, 3, 3,   5, 5, 5, 5, 5, 6, 6, 6, 6, 6,
                                             7, 7, 7, 7, 8, 8, 8, 8, 9, 9,   10,10,10,10,10,10,10,10,10,10,   9, 8, 6, 5, 3, 1, 1, 1, 0, 0 };
                    for(i = start; i < end; i++)
                    {
                        Sum += weight_7_6[i] * C_Line[i];
                        Weight_Sum += weight_7_6[i];
                    }
                    break;
                }
                case 7:
                {
                    uint8 weight_7_7[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 1, 2, 3, 3,   5, 5, 5, 5, 5, 6, 6, 6, 6, 6,
                                             7, 7, 7, 7, 8, 8, 8, 8, 9, 9,   10,10,10,10,10,10,10,10,10,10,   9, 8, 6, 5, 3, 1, 1, 1, 0, 0 };
                    for(i = start; i < end; i++)
                    {
                        Sum += weight_7_7[i] * C_Line[i];
                        Weight_Sum += weight_7_7[i];
                    }
                    break;
                }
            }
        }
        else
        {
            //相遇点滑动均值滤波
            uint8 Y_Meet_Max = 0;
            uint8 Y_Meet_Min = 0;
            uint16 Y_Meet_Sum = 0;
            uint8 Y_Meet_Average = 0;

            Y_Meet_Min = Y_Meet_Array[0];
            Y_Meet_Max = Y_Meet_Array[0];
            for(i = 0; i < 19; i++)
            {
                Y_Meet_Array[i + 1] = Y_Meet_Array[i];
            }
            Y_Meet_Array[0] = start;

            for(i = 0; i < 20; i++)
            {
                if(Y_Meet_Array[i] < Y_Meet_Min)
                {
                    Y_Meet_Min = Y_Meet_Array[i];
                }
                if(Y_Meet_Array[i] > Y_Meet_Max)
                {
                    Y_Meet_Max = Y_Meet_Array[i];
                }
            }

            for(i = 0; i < 20; i++)
            {
                Y_Meet_Sum += (uint16)Y_Meet_Array[i];
            }
            Y_Meet_Sum = Y_Meet_Sum - (uint16)Y_Meet_Min - (uint16)Y_Meet_Max;
            Y_Meet_Average = (uint8)(Y_Meet_Sum / 18);

            if(Y_Meet_Average > 20)
            {
                Y_Meet_Average = 20;
            }
            else if(Y_Meet_Average < 2)
            {
                Y_Meet_Average = 2;
            }

    //        float Temp_Float = (0.9f * (float)(Y_Meet_Average - 2) + 1.8f * Sensitivity);
            float Temp_Float = (float)(Y_Meet_Average - 2);

            if(Temp_Float > 18.0f)
            {
                Temp_Float = 18.0f;
            }
            if(Temp_Float < 0)
            {
                Temp_Float = 0;
            }

            //固定权值
            uint8 Base_Weight[60] = {0, 0, 1, 0, 1, 0, 1, 0, 1, 0,      1, 0, 1, 0, 1, 0, 1, 0, 1, 0,      1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
                                     1, 0, 1, 0, 1, 0, 1, 0, 1, 0,      1, 0, 1, 0, 1, 0, 1, 0, 1, 0,      1, 0, 1, 0, 1, 0, 1, 1, 0, 0};
            //局部动态权值
            uint8 Trends_Weight[19] = {4, 5, 5, 6, 6, 7, 7, 8, 9, 10, 9, 8, 7, 7, 6, 6, 5, 5, 4};


//            uint8 Base_Weight[60] = {0};
//            uint8 Trends_Weight[10] = {4, 6, 6, 8, 10, 10, 8, 6, 6, 4};
            //局部动态权值
//            uint8 Trends_Weight[19] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

            //动态权重起始行只能落在24 - 40行之间(由下往上)
//            uint8 Start_Line = (uint8)(Temp_Float / 18.0f * 18.0f + 24.0f); // 20-30

            uint8 Start_Line;
            if(Error_Line != 0)
                Start_Line = Error_Line;
            else
                Start_Line = (uint8)(Temp_Float / 18.0f * 18.0f + 24.0f); // 20-30

            if(((element == L_Circle || element == R_Circle) && (element_state == 3 || element_state == 4)) || Element_State == Single_State)
                Start_Line = 50;
//            uint8 Start_Line = 50; // 100-50
//            ips200_draw_line(0, Start_Line, 79, Start_Line, RGB565_RED);     // 坐标 0,0 到 10,10 画一条红色的线

            //将动态权值赋值给固定权值数组
            for(i = 0; i < 19; i ++)
            {
                Base_Weight[Start_Line - i] = Trends_Weight[i];
            }

            for(i = Y_Meet_Average + 1; i < 57; i++)
            {
                Sum += (int32)Base_Weight[i] * (int32)C_Line[i];
                Weight_Sum += (int32)Base_Weight[i];
            }
        }
    }
/*
    else
    {
        switch(element)
        {
            case 0:
            {
    //            uint8 Weight_0[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 1, 1, 2, 3, 4, 5, 6, 7, 8,
    //                                   9,10,10,10,10,10,10,10,10,10,    10,10,10,10,10, 8, 7, 7, 5, 5,   3, 3, 3, 1, 1, 1, 0, 0, 0, 0 };

                uint8 Weight_0[60] = { 0, 0, 0, 0, 0, 0, 7, 7, 7, 7,     8, 8, 8, 8, 8, 9, 9,10,10,10,    10,10,10,10,10,10,10,10,10,10,
                                      10,10,10,10,10, 8, 8, 8, 8, 9,     9, 9, 9, 8, 8, 7, 7, 5, 3, 2,     2, 2, 2, 1, 1, 1, 0, 0, 0, 0 };
                for(i = start; i < end; i++)
                {
                    Sum += Weight_0[i] * C_Line[i];
                    Weight_Sum += Weight_0[i];
                }
                break;
            }
            case Straightaway:
            {
                uint8 Weight_2[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
                                       3, 3, 3, 7, 9,10,10,10,10,10,    10,10,10,10,10,10, 8, 7, 7, 5,   5, 3, 2, 1, 1, 1, 0, 0, 0, 0 };

                for(i = 28; i < end; i++)
                {
                    Sum += Weight_2[i] * C_Line[i];
                    Weight_Sum += Weight_2[i];
                }
                break;
            }
            case L_Turn:
            {
                uint8 Weight_3[60] = { 0, 0, 0, 0, 0, 0, 7, 7, 7, 7,     8, 8, 8, 8, 8, 9, 9,10,10,10,    10,10,10,10,10,10,10,10,10,10,
                                      10,10,10,10,10, 8, 8, 8, 8, 9,     9, 9, 9, 8, 8, 7, 7, 5, 3, 2,     2, 2, 2, 1, 1, 1, 0, 0, 0, 0 };

                for(i = start; i < end; i++)
                {
                    Sum += Weight_3[i] * C_Line[i];
                    Weight_Sum += Weight_3[i];
                }
                break;
            }
            case R_Turn:
            {
                uint8 Weight_4[60] = { 0, 0, 0, 0, 0, 0, 7, 7, 7, 7,    8, 8, 8, 8, 8, 9, 9,10,10,10,    10,10,10,10,10,10,10,10,10,10,
                                      10,10,10,10,10, 8, 8, 8, 8, 9,    9, 9, 9, 8, 8, 7, 7, 5, 3, 2,    2, 2, 2, 1, 1, 1, 0, 0, 0, 0 };

                for(i = start; i < end; i++)
                {
                    Sum += Weight_4[i] * C_Line[i];
                    Weight_Sum += Weight_4[i];
                }
                break;
            }
            case L_Circle:
            {
                switch(element_state)
                {
                    case 1:     //识别到圆环但未入环
                    {
                        uint8 Weight_5_1[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    1, 1, 1, 2, 2, 2, 3, 3, 3, 3,   3, 3, 4, 4, 4, 5, 5, 5, 5, 5,
                                                7, 7, 7, 7, 9,10,10,10,10,10,    10,10,10,10,10,10, 8, 7, 7, 5,   5, 3, 2, 1, 1, 1, 0, 0, 0, 0 };

                        for(i = 10; i < end; i++)
                        {
                            Sum += Weight_5_1[i] * C_Line[i];
                            Weight_Sum += Weight_5_1[i];
                        }
                        break;
                    }
                    case 2:
                    {
                        uint8 Weight_5_2[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    1, 5, 8, 8, 8, 9, 9,10,10,10,    10,10,10,10,10,10,10,10,10,10,
                                                10,10,10,10,10, 9, 9, 8, 8, 9,    9, 9, 9, 8, 8, 7, 7, 5, 3, 2,    2, 2, 2, 1, 0, 0, 0, 0, 0, 0 };

                        for(i = start; i < end; i++)
                        {
                            Sum += Weight_5_2[i] * C_Line[i];
                            Weight_Sum += Weight_5_2[i];
                        }
                        break;
                    }
                    case 3:     //圆环内
                    {
                        uint8 Weight_5_3[60] = { 0, 0, 0, 0, 0, 0, 7, 7, 7, 7,    8, 8, 8, 8, 8, 9, 9,10,10,10,    10,10,10,10,10,10,10,10,10,10,
                                                10,10,10,10,10, 9, 9, 8, 8, 9,    9, 9, 9, 8, 8, 7, 7, 5, 3, 2,    2, 2, 2, 1, 1, 1, 0, 0, 0, 0 };

                        for(i = start; i < end; i++)
                        {
                            Sum += Weight_5_3[i] * C_Line[i];
                            Weight_Sum += Weight_5_3[i];
                        }
                        break;
                    }
                    case 4:
                    {
                        uint8 Weight_5_4[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    1, 5, 8, 8, 8, 9, 9,10,10,10,    10,10,10,10,10,10,10,10,10,10,
                                                10,10,10,10,10, 9, 9, 8, 8, 9,    9, 9, 9, 8, 8, 7, 7, 5, 3, 2,    2, 2, 2, 1, 0, 0, 0, 0, 0, 0 };

                        for(i = start; i < end; i++)
                        {
                            Sum += Weight_5_4[i] * C_Line[i];
                            Weight_Sum += Weight_5_4[i];
                        }
                        break;
                    }
                    case 5:     //出圆环，单边巡线
                    {
                        uint8 Weight_5_5[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    1, 1, 1, 2, 2, 2, 3, 3, 3, 3,   3, 3, 4, 4, 4, 5, 5, 5, 5, 5,
                                                7, 7, 7, 7, 9,10,10,10,10,10,    10,10,10,10,10,10, 8, 7, 7, 5,   5, 3, 2, 1, 1, 1, 0, 0, 0, 0 };

                        for(i = 10; i < end; i++)
                        {
                            Sum += Weight_5_5[i] * C_Line[i];
                            Weight_Sum += Weight_5_5[i];
                        }
                        break;
                    }
                }
                break;
            }
            case R_Circle:
            {
                switch(element_state)
                {
                    case 1:     //识别到圆环但未入环
                    {
                        uint8 Weight_5_1[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    1, 1, 1, 2, 2, 2, 3, 3, 3, 3,   3, 3, 4, 4, 4, 5, 5, 5, 5, 5,
                                                7, 7, 7, 7, 9,10,10,10,10,10,    10,10,10,10,10,10, 8, 7, 7, 5,   5, 3, 2, 1, 1, 1, 0, 0, 0, 0 };

                        for(i = 10; i < end; i++)
                        {
                            Sum += Weight_5_1[i] * C_Line[i];
                            Weight_Sum += Weight_5_1[i];
                        }
                        break;
                    }
                    case 2:
                    {
                        uint8 Weight_5_2[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    1, 5, 8, 8, 8, 9, 9,10,10,10,    10,10,10,10,10,10,10,10,10,10,
                                                10,10,10,10,10, 9, 9, 8, 8, 9,    9, 9, 9, 8, 8, 7, 7, 5, 3, 2,    2, 2, 2, 1, 0, 0, 0, 0, 0, 0 };

                        for(i = start; i < end; i++)
                        {
                            Sum += Weight_5_2[i] * C_Line[i];
                            Weight_Sum += Weight_5_2[i];
                        }
                        break;
                    }
                    case 3:     //圆环内
                    {
                        uint8 Weight_5_3[60] = { 0, 0, 0, 0, 0, 0, 7, 7, 7, 7,    8, 8, 8, 8, 8, 9, 9,10,10,10,    10,10,10,10,10,10,10,10,10,10,
                                                10,10,10,10,10, 9, 9, 8, 8, 9,    9, 9, 9, 8, 8, 7, 7, 5, 3, 2,    2, 2, 2, 1, 1, 1, 0, 0, 0, 0 };

                        for(i = start; i < end; i++)
                        {
                            Sum += Weight_5_3[i] * C_Line[i];
                            Weight_Sum += Weight_5_3[i];
                        }
                        break;
                    }
                    case 4:
                    {
                        uint8 Weight_5_4[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    1, 5, 8, 8, 8, 9, 9,10,10,10,    10,10,10,10,10,10,10,10,10,10,
                                                10,10,10,10,10, 9, 9, 8, 8, 9,    9, 9, 9, 8, 8, 7, 7, 5, 3, 2,    2, 2, 2, 1, 0, 0, 0, 0, 0, 0 };

                        for(i = start; i < end; i++)
                        {
                            Sum += Weight_5_4[i] * C_Line[i];
                            Weight_Sum += Weight_5_4[i];
                        }
                        break;
                    }
                    case 5:     //出圆环，单边巡线
                    {
                        uint8 Weight_5_5[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    1, 1, 1, 2, 2, 2, 3, 3, 3, 3,   3, 3, 4, 4, 4, 5, 5, 5, 5, 5,
                                                7, 7, 7, 7, 9,10,10,10,10,10,    10,10,10,10,10,10, 8, 7, 7, 5,   5, 3, 2, 1, 1, 1, 0, 0, 0, 0 };

                        for(i = 10; i < end; i++)
                        {
                            Sum += Weight_5_5[i] * C_Line[i];
                            Weight_Sum += Weight_5_5[i];
                        }
                        break;
                    }
                }
                break;
            }
            case Cross:
            {
                switch(element_state)
                {
                    case 1:     //未入十字
                    {
                        uint8 weight_7_1[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 1, 2, 3, 3,   5, 5, 5, 5, 5, 6, 6, 6, 6, 6,
                                                 7, 7, 7, 7, 8, 8, 8, 8, 9, 9,   10,10,10,10,10,10,10,10,10,10,   9, 8, 6, 5, 3, 1, 1, 1, 0, 0 };
                        for(i = start; i < end; i++)
                        {
                            Sum += weight_7_1[i] * C_Line[i];
                            Weight_Sum += weight_7_1[i];
                        }
                        break;
                    }
                    case 2:
                    {
                        uint8 weight_7_2[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 1, 3, 5, 5, 5, 7, 8,   8, 8, 8, 9, 9,10,10,10,10,10,
                                                10,10,10,10,10,10,10,10,10,10,   10,10,10,10,10,10,10,10,10,10,   9, 6, 4, 3, 1, 1, 0, 0, 0, 0 };
                        for(i = start; i < end; i++)
                        {
                            Sum += weight_7_2[i] * C_Line[i];
                            Weight_Sum += weight_7_2[i];
                        }
                        break;
                    }
                    case 3:
                    {
                        uint8 weight_7_3[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 1, 1, 1, 1, 2, 2, 3,   3, 3, 4, 5, 6, 6, 7, 7, 7, 7,
                                                10,10,10,10,10,10,10,10,10,10,    9, 8, 7, 6, 5, 5, 4, 4, 4, 4,   3, 3, 3, 1, 1, 1, 0, 0, 0, 0 };
                        for(i = start; i < end; i++)
                        {
                            Sum += weight_7_3[i] * C_Line[i];
                            Weight_Sum += weight_7_3[i];
                        }
                        break;
                    }
                    case 4:
                    {
                        uint8 weight_7_4[60] = { 0, 0, 0, 0, 0, 0, 7, 7, 7, 7,     8, 8, 8, 8, 8, 9, 9,10,10,10,    10,10,10,10,10,10,10,10,10,10,
                                                10,10,10,10,10, 8, 8, 8, 8, 9,     9, 9, 9, 8, 8, 7, 7, 5, 3, 2,   2, 2, 2, 1, 1, 1, 0, 0, 0, 0 };
                        for(i = start; i < end; i++)
                        {
                            Sum += weight_7_4[i] * C_Line[i];
                            Weight_Sum += weight_7_4[i];
                        }
                        break;
                    }
                    case 5:
                    {
                        uint8 weight_7_5[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 1, 2, 3, 3,   5, 5, 5, 5, 5, 6, 6, 6, 6, 6,
                                                 7, 7, 7, 7, 8, 8, 8, 8, 9, 9,   10,10,10,10,10,10,10,10,10,10,   9, 8, 6, 5, 3, 1, 1, 1, 0, 0 };
                        for(i = start; i < end; i++)
                        {
                            Sum += weight_7_5[i] * C_Line[i];
                            Weight_Sum += weight_7_5[i];
                        }
                        break;
                    }
                    case 6:
                    {
                        uint8 weight_7_6[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 1, 2, 3, 3,   5, 5, 5, 5, 5, 6, 6, 6, 6, 6,
                                                 7, 7, 7, 7, 8, 8, 8, 8, 9, 9,   10,10,10,10,10,10,10,10,10,10,   9, 8, 6, 5, 3, 1, 1, 1, 0, 0 };
                        for(i = start; i < end; i++)
                        {
                            Sum += weight_7_6[i] * C_Line[i];
                            Weight_Sum += weight_7_6[i];
                        }
                        break;
                    }
                    case 7:
                    {
                        uint8 weight_7_7[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 1, 2, 3, 3,   5, 5, 5, 5, 5, 6, 6, 6, 6, 6,
                                                 7, 7, 7, 7, 8, 8, 8, 8, 9, 9,   10,10,10,10,10,10,10,10,10,10,   9, 8, 6, 5, 3, 1, 1, 1, 0, 0 };
                        for(i = start; i < end; i++)
                        {
                            Sum += weight_7_7[i] * C_Line[i];
                            Weight_Sum += weight_7_7[i];
                        }
                        break;
                    }
                }
                break;
            }
            case L_Oblique_Cross:
            {
                uint8 weight_16[60] = {0, 0, 1, 2, 4, 6, 7, 9, 10, 10,    10,10,10,10,10,10,10,10,10,10,   10,10,10,10,10,10,10,10,10,10,
                                       9, 9, 9, 8, 8, 7, 7, 6, 5, 4,       3, 2, 1, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
                for(i = start; i < end; i++)
                {
                    if(L_Border[i] != 2 && R_Border[i] != 77)
                    {
                        Oblique_Cross_Temp_Num ++;
                        Sum += weight_16[i] * C_Line[i];
                        Weight_Sum += weight_16[i];
                    }
                }
                break;
            }
            case R_Oblique_Cross:
            {
                uint8 weight_17[60] = {0, 0, 1, 2, 4, 6, 7, 9, 10, 10,    10,10,10,10,10,10,10,10,10,10,   10,10,10,10,10,10,10,10,10,10,
                                       9, 9, 9, 8, 8, 7, 7, 6, 5, 4,       3, 2, 1, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
                for(i = start; i < end; i++)
                {
                    if(L_Border[i] != 2 && R_Border[i] != 77)
                    {
                        Oblique_Cross_Temp_Num ++;
                        Sum += weight_17[i] * C_Line[i];
                        Weight_Sum += weight_17[i];
                    }
                }
                break;
            }
            case Ramp:
            {
                switch(element_state)
                {
                    case 1:
                    {
                        uint8 Weight_9_1[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0, 0, 1,    1, 1, 3, 3, 5, 5, 5, 7, 7, 7,
                                                 9, 9, 9,10,10,10,10,10,10,10,    10,10,10,10,10,10,10,10,10, 8,    8, 8, 8, 7, 5, 3, 1, 0, 0, 0 };

                        for(i = start; i < end; i++)
                        {
                            Sum += Weight_9_1[i] * C_Line[i];
                            Weight_Sum += Weight_9_1[i];
                        }
                        break;
                    }
                    case 2:
                    {
                        uint8 Weight_9_2[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                 6, 6, 7, 7, 8, 9, 9, 9,10,10,    10,10,10,10,10,10,10,10,10,10,   10,10, 9, 9, 8, 8, 7, 0, 0, 0 };

                        for(i = start; i < end; i++)
                        {
                            Sum += Weight_9_2[i] * C_Line[i];
                            Weight_Sum += Weight_9_2[i];
                        }
                        break;
                    }
                    case 3:
                    {
                        uint8 Weight_9_3[60] = { 0, 0, 0, 1, 2, 4, 6, 7, 7, 8,     8, 8, 9, 9,10,10,10,10,10,10,   10,10,10,10,10,10,10,10,10,10,
                                                10,10,10,10,10,10,10,10,10,10,    10,10,10,10,10,10,10,10,10,10,   10,10, 9, 9, 8, 8, 7, 0, 0, 0 };

                        for(i = start; i < end; i++)
                        {
                            Sum += Weight_9_3[i] * C_Line[i];
                            Weight_Sum += Weight_9_3[i];
                        }
                        break;
                    }
                }
                break;
            }
            case Zebra:
            {
                uint8 Weight_8[60] = { 0, 0, 1, 7, 8, 9,10,10,10,10,    10,10,10,10,10,10,10,10,10,10,  10,10,10, 9, 9, 9, 8, 8, 8, 7,
                                       7, 7, 6, 6, 5, 5, 4, 4, 3, 3,     2, 2, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

                for(i = start; i < end; i++)
                {
                    Sum += Weight_8[i] * C_Line[i];
                    Weight_Sum += Weight_8[i];
                }
                break;
            }
            case Three_Bif:
            {
                break;
            }
            case Barrier:
            {
                break;
            }
            case Disconnection:
            {
                break;
            }
            case T_Way:
            {
                break;
            }
            case L_Garage:
            {
                break;
            }
            case R_Garage:
            {
                break;
            }
            case Small_S:
            {
                uint8 Weight_18[60] = { 0, 0, 0, 0, 0, 0, 7, 7, 7, 7,    8, 8, 8, 8, 8, 9, 9,10,10,10,    10,10,10,10,10,10,10,10,10,10,
                                       10,10,10,10,10, 8, 8, 8, 8, 9,    9, 9, 9, 8, 8, 8, 8, 8, 7, 7,     7, 6, 6, 5, 4, 3, 2, 0, 0, 0 };

                for(i = start; i < end; i++)
                {
                    Sum += Weight_18[i] * C_Line[i];
                    Weight_Sum += Weight_18[i];
                }
                break;
            }
        }
    }*/

    Angle_Err = (float)Sum / (float)Weight_Sum - 39.5f;

    Angle_Err = Angle_Err / 17.0f;


    Mid_Line_FFF = Mid_Line_FF;
    Mid_Line_FF = Mid_Line_F;
    Mid_Line_F = Angle_Err;

    Angle_Err = 0.50f * Mid_Line_FFF + 0.30f * Mid_Line_FF + 0.20f * Mid_Line_F;
//    Angle_Err = 0.80f * Angle_Err + 0.20f * Slope;

    deviation_value = Limit_Float(Angle_Err, -1.0, 1.0);

    if(deviation_value >= -1.0 && deviation_value <= 1.0)
    {
        Last_Deviation_Value = deviation_value;
    }
    else
    {
        deviation_value = Last_Deviation_Value;
    }

    //偏差值滑动均值滤波
    float Deviation_Value_Sum = 0;
    for(i = 0; i < 19; i++)
    {
        Deviation_Value_Array[i + 1] = Deviation_Value_Array[i];
    }
    Deviation_Value_Array[0] = deviation_value;
    for(i = 0; i < 20; i++)
    {
        Deviation_Value_Sum += Deviation_Value_Array[i];
    }
    deviation_value = Deviation_Value_Sum / 20.0f;

    if(Code_type == 2)
    {
        Deviation_Value = deviation_value;
//        if(element == L_Circle)
//        {
//            switch(element_state)
//            {
//                case 2:
//                {
//                    Deviation_Value = deviation_value;//Limit_Float(deviation_value, -0.95f, 0.95f);
//                    break;
//                }
//                case 3:
//                {
//                    Deviation_Value = Limit_Float(deviation_value, -0.5, 0.5f);
//                    break;
//                }
//                case 4:
//                {
//                    Deviation_Value = Limit_Float(deviation_value, -0.45f, 0.45f);// - 0.15f;
//                    break;
//                }
//                case 5:
//                {
//                    Deviation_Value = Limit_Float(deviation_value + 0.4f, -0.80f, 0.80f);
//                    break;
//                }
//                default:
//                {
//                    Deviation_Value = deviation_value;
//                    break;
//                }
//            }
//        }
//        else if(element == R_Circle)
//        {
//            switch(element_state)
//            {
//                case 2:
//                {
//                    Deviation_Value = deviation_value;//Limit_Float(deviation_value, -0.95f, 0.95f);
//                    break;
//                }
//                case 3:
//                {
//                    Deviation_Value = Limit_Float(deviation_value, -0.85f, 0.85f);
//                    break;
//                }
//                case 4:
//                {
//                    Deviation_Value = Limit_Float(deviation_value, -0.55f, 0.55f);// + 0.15f;
//                    break;
//                }
//                case 5:
//                {
//                    Deviation_Value = Limit_Float(deviation_value - 0.4f, -0.80f, 0.80f);
//                    break;
//                }
//                default:
//                {
//                    Deviation_Value = deviation_value;
//                    break;
//                }
//            }
//        }
//        else if(element == Cross && element_state == 1)
//        {
//            Deviation_Value = Limit_Float(deviation_value, -0.3f, 0.3f);
//        }
//        else if(element == Cross && (element_state == 2 || element_state == 3))
//        {
//            Deviation_Value = Limit_Float(deviation_value, -0.2f, 0.2f);
//        }
//        else if(element == Cross && element_state == 4)
//        {
////            if(deviation_value <= 0)
////            {
//                Deviation_Value = Limit_Float(deviation_value, -0.8f, 0.8f);
////            }
////            else
////            {
////                Deviation_Value = Limit_Float(deviation_value, 0.95f, 0.95f);
////            }
////            Deviation_Value = deviation_value;
//        }
//        else if(element == Cross && (element_state == 5 || element_state == 6 || element_state == 7))
//        {
//            if(Cross_State_4_Direction >= 100)
//            {
//                if(deviation_value > -0.2f)
//                {
//                    Deviation_Value = -0.2f;
//                }
//                else
//                {
//                    Deviation_Value = deviation_value;
//                }
//            }
//            else if(Cross_State_4_Direction <= -100)
//            {
//                if(deviation_value < 0.2f)
//                {
//                    Deviation_Value = 0.2f;
//                }
//                else
//                {
//                    Deviation_Value = deviation_value;
//                }
//            }
//            else
//            {
//                Deviation_Value = deviation_value;
//            }
//        }
//        else if(element == Small_S)
//        {
//            if(My_ABS_F(deviation_value) >= 0.5f)
//            {
//                Small_S_Break_Flag = 1;
//            }
//            Deviation_Value = Limit_Float(deviation_value, -0.3f, 0.3f);
//        }
//        else if(element == Ramp)
//        {
//            Deviation_Value = Limit_Float(deviation_value, -0.3f, 0.3f);
//        }
//        else
//        {
//            Deviation_Value = deviation_value;
//        }
    }
    else if(Code_type == 0)
    {
        if(element == L_Circle)
        {
            switch(element_state)
            {
                case 2:
                {
                    Deviation_Value = Limit_Float(deviation_value, -0.88f, 0.88f);
                    break;
                }
                case 3:
                {
                    Deviation_Value = Limit_Float(deviation_value, -0.8f, 0.8f);
                    break;
                }
                case 4:
                {
                    Deviation_Value = Limit_Float(deviation_value, -0.45f, 0.45f);// - 0.15f;
                    break;
                }
                case 5:
                {
                    Deviation_Value = Limit_Float(deviation_value + 0.3f, -0.80f, 0.80f);
                    break;
                }
                default:
                {
                    Deviation_Value = deviation_value;
                    break;
                }
            }
        }
        else if(element == R_Circle)
        {
            switch(element_state)
            {
                case 2:
                {
                    Deviation_Value = Limit_Float(deviation_value, -0.87f, 0.87f);
                    break;
                }
                case 3:
                {
                    Deviation_Value = Limit_Float(deviation_value, -0.8f, 0.8f);
                    break;
                }
                case 4:
                {
                    Deviation_Value = Limit_Float(deviation_value, -0.45f, 0.45f);// + 0.15f;
                    break;
                }
                case 5:
                {
                    Deviation_Value = Limit_Float(deviation_value - 0.3f, -0.80f, 0.80f);
                    break;
                }
                default:
                {
                    Deviation_Value = deviation_value;
                    break;
                }
            }
        }
        else if(element == Cross && (element_state == 1 || element_state == 2 || element_state == 3))
        {
            Deviation_Value = Limit_Float(deviation_value, -0.1f, 0.1f);
        }
        else if(element == Cross && element_state == 4)
        {
            Deviation_Value = Limit_Float(deviation_value, -0.7f, 0.7f);
        }
        else if(element == Cross && (element_state == 5 || element_state == 6 || element_state == 7))
        {
            if(Cross_State_4_Direction >= 100)
            {
                if(deviation_value > -0.1f)
                {
                    Deviation_Value = -0.05f;
                }
                else
                {
                    Deviation_Value = deviation_value;
                }
            }
            else if(Cross_State_4_Direction <= -100)
            {
                if(deviation_value < 0.1f)
                {
                    Deviation_Value = 0.05f;
                }
                else
                {
                    Deviation_Value = deviation_value;
                }
            }
            else
            {
                Deviation_Value = deviation_value;
            }
        }
        else if(element == L_Oblique_Cross && Oblique_Cross_Temp_Num <= 6)
        {
            Deviation_Value = 0.21f;
        }
        else if(element == R_Oblique_Cross && Oblique_Cross_Temp_Num <= 6)
        {
            Deviation_Value = -0.21f;
        }
        else if(element == Small_S)
        {
            if(My_ABS_F(deviation_value) >= 0.5f)
            {
                Small_S_Break_Flag = 1;
            }
            Deviation_Value = Limit_Float(deviation_value, -0.3f, 0.3f);
        }
        else if(element == Ramp)
        {
            Deviation_Value = Limit_Float(deviation_value, -0.3f, 0.3f);
        }
        else
        {
            Deviation_Value = deviation_value;
        }
    }
    else if(Code_type == 1)
    {
        if(element == L_Circle)
        {
            switch(element_state)
            {
                case 2:
                {
                    Deviation_Value = Limit_Float(deviation_value, -0.88f, 0.88f);
                    break;
                }
                case 3:
                {
                    Deviation_Value = Limit_Float(deviation_value, -0.67f, 0.67f);
                    break;
                }
                case 4:
                {
                    Deviation_Value = Limit_Float(deviation_value, -0.45f, 0.45f);// - 0.15f;
                    break;
                }
                case 5:
                {
                    Deviation_Value = Limit_Float(deviation_value + 0.35f, -0.80f, 0.80f);
                    break;
                }
                default:
                {
                    Deviation_Value = deviation_value;
                    break;
                }
            }
        }
        else if(element == R_Circle)
        {
            switch(element_state)
            {
                case 2:
                {
                    Deviation_Value = Limit_Float(deviation_value, -0.87f, 0.87f);
                    break;
                }
                case 3:
                {
                    Deviation_Value = Limit_Float(deviation_value, -0.67f, 0.67f);
                    break;
                }
                case 4:
                {
                    Deviation_Value = Limit_Float(deviation_value, -0.45f, 0.45f);// + 0.15f;
                    break;
                }
                case 5:
                {
                    Deviation_Value = Limit_Float(deviation_value - 0.35f, -0.80f, 0.80f);
                    break;
                }
                default:
                {
                    Deviation_Value = deviation_value;
                    break;
                }
            }
        }
        else if(element == Cross && (element_state == 1 || element_state == 2 || element_state == 3))
        {
            Deviation_Value = Limit_Float(deviation_value, -0.1f, 0.1f);
        }
        else if(element == Cross && element_state == 4)
        {
            Deviation_Value = Limit_Float(deviation_value, -0.9f, 0.9f);
        }
        else if(element == Cross && (element_state == 5 || element_state == 6 || element_state == 7))
        {
            if(Cross_State_4_Direction >= 100)
            {
                if(deviation_value > -0.1f)
                {
                    Deviation_Value = -0.1f;
                }
                else
                {
                    Deviation_Value = deviation_value;
                }
            }
            else if(Cross_State_4_Direction <= -100)
            {
                if(deviation_value < 0.1f)
                {
                    Deviation_Value = 0.1f;
                }
                else
                {
                    Deviation_Value = deviation_value;
                }
            }
            else
            {
                Deviation_Value = deviation_value;
            }
        }
        else if(element == L_Oblique_Cross && Oblique_Cross_Temp_Num <= 6)
        {
            Deviation_Value = 0.21f;
        }
        else if(element == R_Oblique_Cross && Oblique_Cross_Temp_Num <= 6)
        {
            Deviation_Value = -0.21f;
        }
        else if(element == Small_S)
        {
            if(My_ABS_F(deviation_value) >= 0.5f)
            {
                Small_S_Break_Flag = 1;
            }
            Deviation_Value = Limit_Float(deviation_value, -0.3f, 0.3f);
        }
        else if(element == Ramp)
        {
            Deviation_Value = Limit_Float(deviation_value, -0.3f, 0.3f);
        }
        else
        {
            Deviation_Value = deviation_value;
        }
    }
}

void Patching_Line_Process(uint8 slope_start, uint8 slope_end, uint8 start, uint8 end, uint8 *border)
{
    float Slope_Rate = 0;
    float Intercept = 0;
    uint8 i =0;
    Calculate_Slope_Intercept(slope_start, slope_end, border, &Slope_Rate, &Intercept);
    for(i = start; i < end; i++)
    {
        border[i] = (uint8)(Slope_Rate * (float)i + Intercept);
    }
}

//弃用
void Turn_Point_Patching_Line(uint8 element)
{
    switch(element)
    {
        case L_Circle:
        {
            break;
        }
        case R_Circle:
        {
            break;
        }
        case Zebra:
        {
            break;
        }
        case Small_S:
        {
            break;
        }
        default:
        {
            if((L_Up_Turn_Left_Point_Flag_1 + L_Right_Turn_Up_Point_Flag_1 >= 1) && (R_Up_Turn_Right_Point_Flag_1 + R_Left_Turn_Up_Point_Flag_1 >= 1))
            {
                switch(L_Up_Turn_Left_Point_Flag_1 + L_Right_Turn_Up_Point_Flag_1)
                {
                    case 1:
                    {
                        if(L_Up_Turn_Left_Point_Flag_1 == 1)
                        {
                            Patching_Line_Process(L_Up_Turn_Left_Point_1[1], Limit_u8(L_Up_Turn_Left_Point_1[1] + 5, 2, 57), Y_Meet, L_Up_Turn_Left_Point_1[1], L_Border);
                        }
                        else if(L_Right_Turn_Up_Point_Flag_1 == 1)
                        {
                            Patching_Line_Process(Limit_u8(L_Right_Turn_Up_Point_1[1] - 5, 2, 57), L_Right_Turn_Up_Point_1[1], L_Right_Turn_Up_Point_1[1], L_Start_Point[1], L_Border);
                        }
                        break;
                    }
                    case 2:
                    {
                        if((L_Right_Turn_Up_Point_1[1] - Y_Meet) >= (L_Start_Point[1] - L_Up_Turn_Left_Point_1[1]))
                        {
                            Patching_Line_Process(Limit_u8(L_Right_Turn_Up_Point_1[1] - 5, 2, 57), L_Right_Turn_Up_Point_1[1], L_Right_Turn_Up_Point_1[1], L_Up_Turn_Left_Point_1[1], L_Border);
                        }
                        else
                        {
                            Patching_Line_Process(L_Up_Turn_Left_Point_1[1], Limit_u8(L_Up_Turn_Left_Point_1[1] + 5, 2, 57), L_Right_Turn_Up_Point_1[1], L_Up_Turn_Left_Point_1[1], L_Border);
                        }
                    }
                }
                switch(R_Up_Turn_Right_Point_Flag_1 + R_Left_Turn_Up_Point_Flag_1)
                {
                    case 1:
                    {
                        if(R_Up_Turn_Right_Point_Flag_1 == 1)
                        {
                            Patching_Line_Process(R_Up_Turn_Right_Point_1[1], Limit_u8(R_Up_Turn_Right_Point_1[1] + 5, 2, 57), Y_Meet, R_Up_Turn_Right_Point_1[1], R_Border);
                        }
                        else if(R_Left_Turn_Up_Point_Flag_1 == 1)
                        {
                            Patching_Line_Process(Limit_u8(R_Left_Turn_Up_Point_1[1] - 5, 2, 57), R_Left_Turn_Up_Point_1[1], R_Left_Turn_Up_Point_1[1], R_Start_Point[1], R_Border);
                        }
                        break;
                    }
                    case 2:
                    {
                        if((R_Left_Turn_Up_Point[1] - Y_Meet) >= (R_Start_Point[1] - R_Up_Turn_Right_Point_1[1]))
                        {
                            Patching_Line_Process(Limit_u8(R_Left_Turn_Up_Point_1[1] - 5, 2, 57), R_Left_Turn_Up_Point_1[1], R_Left_Turn_Up_Point_1[1], R_Up_Turn_Right_Point_1[1], R_Border);
                        }
                        else
                        {
                            Patching_Line_Process(R_Up_Turn_Right_Point_1[1], Limit_u8(R_Up_Turn_Right_Point_1[1] + 5, 2, 57), R_Left_Turn_Up_Point_1[1], R_Up_Turn_Right_Point_1[1], R_Border);
                        }
                        break;
                    }
                }
            }
            break;
        }
    }
}

void Lost_Line(uint8 *l_start_point, uint8 *r_start_point)
{
    if(((r_start_point[0] - l_start_point[0]) <= 5) && (L_Statics <= 20) && (R_Statics <= 20))
    {
        if(r_start_point[0] <= 30 && l_start_point[0] <= 30)
        {
            while(1)
            {
                Start_Flag = 0;
                Deviation_Value = -1;
                Fitting_Speed_Control(0, Min_Speed, Element_State, 0, Y_Meet, 57);

                Process_Image();

//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);

                if(L_Statics >= 40 && R_Statics >= 40)
                {
                    break;
                }
            }
        }
        if(r_start_point[0] >= 50 && l_start_point[0] >= 50)
        {
            while(1)
            {
                Start_Flag = 0;
                Deviation_Value = 1;
                Fitting_Speed_Control(0, Min_Speed, Element_State, 0, Y_Meet, 57);

                Process_Image();

//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);
                if(L_Statics >= 40 && R_Statics >= 40)
                {
                    break;
                }
            }
        }
    }
}

uint8 Temp_Patching_Line_Flag = 0;
void Special_Get_Middle_Line(void)
{
    uint8 i = 0;

    if(Element_State == 0 || Element_State == Straightaway)
    {
        uint8 Max_Row_Difference = 0;
        uint8 Min_Row_Difference = 0;
        Max_Row_Difference = Row_Difference[0];
        Min_Row_Difference = Row_Difference[10];
        for(i = 0; i < 10; i ++)
        {
            if(Max_Row_Difference < Row_Difference[i])
            {
                Max_Row_Difference = Row_Difference[i];
            }
        }
        for(i = 10; i < 19; i ++)
        {
            if(Min_Row_Difference > Row_Difference[i])
            {
                Min_Row_Difference = Row_Difference[i];
            }
        }
        if(((Max_Row_Difference - Min_Row_Difference) >= 25) && (Max_Row_Difference >= 70))
        {
            Temp_Patching_Line_Flag = 1;
        }

        uint8 Temp_Num = 0;
        for(i = 3; i < 10; i ++)
        {
            if(Row_Difference[i] <= 40)
            {
                Temp_Num ++;
            }
        }
        if(Temp_Num == 7)
        {
            Temp_Patching_Line_Flag = 0;
        }
        if(L_Statics <= 90 && R_Statics <= 90)
        {
            Temp_Patching_Line_Flag = 0;
        }

        if(Temp_Patching_Line_Flag == 1)
        {
            float Slope_Rate = 0;
            float Intercept = 0;
            Calculate_Slope_Intercept(47, 57, L_Border, &Slope_Rate, &Intercept);
            for(i = 2; i < 47; i++)
            {
                L_Border[i] = (uint8)(Slope_Rate * (float)i + Intercept);
            }
            Calculate_Slope_Intercept(47, 57, R_Border, &Slope_Rate, &Intercept);
            for(i = 2; i < 47; i++)
            {
                R_Border[i] = (uint8)(Slope_Rate * (float)i + Intercept);
            }
            Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);
        }
        else
        {
            if(L_Border_Point_Num >= 7 && R_Border_Point_Num < 7)
            {
                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, R_Patching);
            }
            else if(L_Border_Point_Num < 7 && R_Border_Point_Num >= 7)
            {
                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, L_Patching);
            }
            else if(L_Border_Point_Num < 7 && R_Border_Point_Num < 7)
            {
                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);
            }
            else
            {
                if(L_Border_Point_Num - R_Border_Point_Num > 5)
                {
                    Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, R_Patching);
                }
                else if(R_Border_Point_Num - L_Border_Point_Num > 5)
                {
                    Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, L_Patching);
                }
                else
                {
                    Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);
                }
            }
        }
    }
    else if(Element_State == L_Turn || Element_State == R_Turn)
    {
        if(L_Straight_Flag == 1 && R_Border_Point_Num >= 20)
        {
            Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, L_Patching);
        }
        else if(R_Straight_Flag == 1 && L_Border_Point_Num >= 20)
        {
            Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, R_Patching);
        }
    }

    if(Temp_Patching_Line_Flag == 1)
    {
        Image_Count_Flag = 1;
        if(Image_Count >= 150)
        {
            Image_Count_Flag = 0;
            Image_Count = 0;
        }
    }
    else
    {
        Image_Count_Flag = 0;
        Image_Count = 0;
    }
}

void Derailment_Process(void)
{
    Element_State = Derailment;
    Fitting_Speed_Control(1, Derailment_Speed, Element_State, 0, Y_Meet, 57);
//    Fitting_Speed_Control(1, 0, Element_State, 0, Y_Meet, 57);

    Lost_Line(L_Start_Point, R_Start_Point);
}
void Convention_Process(void)
{
    Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);
//    Special_Get_Middle_Line();
    Get_Deviation_Value(0, 0, Y_Meet, Image_Y - 3, C_Line, 1.0);
    Fitting_Speed_Control(1, ConvenTion_Speed, Element_State, 0, Y_Meet, 57);
//    Fitting_Speed_Control(1, 0, Element_State, 0, Y_Meet, 57);

    Lost_Line(L_Start_Point, R_Start_Point);
}

void Straightaway_Process(void)
{
    //当出现其他元素之前，循环由此函数接管
    //持续检测元素，当元素不是0或2时，退出检定，当是0或2是，保持最高速
    uint8 Element = 0;
    while(1)
    {
        Process_Image();

        Get_Information();
        Element = Element_Judgement(Find_Line_Image, L_Statics, R_Statics, L_Grow_Dir, R_Grow_Dir, L_Line, R_Line, X_Meet, Y_Meet, L_Border, R_Border);

        Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);
//        Special_Get_Middle_Line();
        Get_Deviation_Value(Straightaway, 0, Y_Meet, Image_Y - 3, C_Line, 2.0);

        Fitting_Speed_Control(1, Straightaway_Speed, Element_State, 0, Y_Meet, 57);
//        Fitting_Speed_Control(1, 0, Element_State, 0, Y_Meet, 57);

        Lost_Line(L_Start_Point, R_Start_Point);

//        TFT180_Show(Show_Flag);
        IPS200_Show(Show_Flag);
//        DisplayBorderDistances();

        if(Element != 2 && Element != 0)
        {
            break;
        }
    }
}

uint8 Last_Turn = 0;
void L_Turn_Process(void)
{
    uint8 Element = 0;
//    buzzer_flag = 1;
    Image_Count = 0;
    Image_Count_Flag = 1;

    while(1)
    {
        Process_Image();

        Get_Information();
        Element = Element_Judgement(Find_Line_Image, L_Statics, R_Statics, L_Grow_Dir, R_Grow_Dir, L_Line, R_Line, X_Meet, Y_Meet, L_Border, R_Border);
//        Special_Get_Middle_Line();
        Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);

        if(Z_Yaw <= -120)
        {
            Get_Deviation_Value(L_Turn, 0, Y_Meet + 1, Image_Y - 3, C_Line, 2.0);
        }
        else
        {
            Get_Deviation_Value(L_Turn, 0, Y_Meet + 1, Image_Y - 3, C_Line, 1.0);
        }
        Fitting_Speed_Control(0, Min_Speed, Element_State, 0, Limit_u8(Y_Meet, 14, 57), 57);

        Lost_Line(L_Start_Point, R_Start_Point);
//        TFT180_Show(Show_Flag);
        IPS200_Show(Show_Flag);

        if(Element != 3 && Element != 0)
        {
//            buzzer_flag = 1;
            Image_Count_Flag = 0;
            Image_Count = 0;
//            Zebra_Flag = 1;
            break;
        }
        if(Element == 3)
        {
            Image_Count = 0;
        }
        if(Image_Count >= 75)
        {
            buzzer_flag = 1;
            Image_Count_Flag = 0;
            Image_Count = 0;
//            Zebra_Flag = 1;
            break;
        }
    }
}

void R_Turn_Process(void)
{
    uint8 Element = 0;
//    buzzer_flag = 1;
    Image_Count = 0;
    Image_Count_Flag = 1;

    while(1)
    {
        Process_Image();

        Get_Information();
        Element = Element_Judgement(Find_Line_Image, L_Statics, R_Statics, L_Grow_Dir, R_Grow_Dir, L_Line, R_Line, X_Meet, Y_Meet, L_Border, R_Border);
//        Special_Get_Middle_Line();
        Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);

        Get_Deviation_Value(R_Turn, 0, Y_Meet + 1, Image_Y - 3, C_Line, 1.0);

        Fitting_Speed_Control(0, Min_Speed, Element_State, 0, Limit_u8(Y_Meet, 14, 57), 57);

        Lost_Line(L_Start_Point, R_Start_Point);
//        TFT180_Show(Show_Flag);
        IPS200_Show(Show_Flag);
        if(Element != 4 && Element != 0)
        {
//            buzzer_flag = 1;
            Image_Count_Flag = 0;
            Image_Count = 0;
            break;
        }
        if(Element == 4)
        {
            Image_Count = 0;
        }
        if(Image_Count >= 75)
        {
            buzzer_flag = 1;
            Image_Count_Flag = 0;
            Image_Count = 0;
            break;
        }
    }
}

float Circle_Temp_Dif = 0;
uint8 Circle_Temp_Num = 0;
void L_Circle_Process(uint8 Process_Mode)
{
    uint8 i = 0;
    uint8 L_Circle_State = 1;
    uint8 State_1_Flag = 0;
    uint8 State_2_Flag = 0;
    uint8 Break_Flag = 0;

    Z_Yaw = 0;
    Image_Count = 0;

    Temp_Patching_Line_Flag = 0;
    Image_Count_Flag = 1;
//对补线相关参数进行清零
    if(Process_Mode != 1)
    {
        Arc_Point_1[0] = 0;
        Arc_Point_2[0] = 0;
        Arc_Point_3[0] = 0;
        Arc_Point_1[1] = 0;
        Arc_Point_2[1] = 0;
        Arc_Point_3[1] = 0;

        Last_Arc_Point_1[0] = 0;
        Last_Arc_Point_2[0] = 0;
        Last_Arc_Point_3[0] = 0;
        Last_Arc_Point_1[1] = 0;
        Last_Arc_Point_2[1] = 0;
        Last_Arc_Point_3[1] = 0;

        Arc_Point_1_Flag = 0;
        Arc_Point_2_Flag = 0;
        Arc_Point_3_Flag = 0;

        Circlr_State_2_Flag = 0;
        Stop_Find_1_2_Flag = 0;
    }
    buzzer_flag = 1;
    while(1)
    {
//        ips200_show_string(0, 36*8, "L_Arc:");
//        ips200_show_uint( 50, 36*8, L_Circle_State, 2 );
//        ips200_show_uint( 70, 36*8, L_Right_Turn_Up_Point_Flag_1, 2 );
//        ips200_show_uint( 90, 36*8, L_Arc_Turn_Point[0][1], 2 );
        if(Element_Break_Flag == 1)
        {
            Zebra_Count_Flag = 0;
            Zebra_Flag = 1;

            Element_Break_Flag = 0;
            break;
        }
        Element_State = L_Circle;
        switch(L_Circle_State)
        {
            case 1: // 预入环
            {
                uint8 Temp_Num = 0;
                Process_Image();

                Get_L_Right_Turn_Up_Point_1(2);
                Get_L_Arc_Turn_Point();

                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, R_Unilateral_Patrol);
                Get_Deviation_Value(L_Circle, 1, Y_Meet, Image_Y - 3, C_Line, 1.0);
                Fitting_Speed_Control(0, L_Circle_State_1_Speed, Element_State, 1, Y_Meet, 57);

                switch(State_1_Flag)
                {
                    case 0: // 判断距离
                    {
                        for(i = 57; i > 47; i--)
                        {
                            if(L_Border[i] == 2)
                            {
                                Temp_Num ++;
                            }
                        }
                        if(Temp_Num >= 7) // 近距，进入下个状态
                        {
                            State_1_Flag = 1;
                        }
                        break;
                    }
                    case 1: // 近距判断入环条件
                    {
                        for(i = 57; i > 37; i--) // 判断距离
                        {
                            if(L_Border[i] != 2)
                            {
                                Temp_Num ++;
                            }
                        }
                        if(L_Arc_Turn_Point_Flag == 1)  // 找到圆弧拐点
                        {
                            if(Temp_Num >= 12 && L_Right_Turn_Up_Point_Flag_1 == 1 && L_Right_Turn_Up_Point_1[1] >= 8 && L_Arc_Turn_Point[0][1] >= 30)
                            {
                                // 此时在合适位置，进入圆环
                                buzzer_flag = 1;
                                L_Circle_State = 2;
                                Circle_Temp_Num_2 = 0;
                                Image_Count = 0;
                            }
                        }
                        else
                        {
                            if(Temp_Num >= 12 && L_Right_Turn_Up_Point_Flag_1 == 1 && L_Right_Turn_Up_Point_1[1] >= 8)
                            {
                                buzzer_flag = 1;
                                L_Circle_State = 2;
                                Circle_Temp_Num_2 = 0;
                                Image_Count = 0;
                            }
                        }
                        // 暂时不懂,感觉xy混淆严重
//                        if(L_Right_Turn_Up_Point_Flag_1 == 1)
//                        {
//                            Temp_Num = 0;
//                            for(i = L_Right_Turn_Up_Point_1[1]; i < L_Start_Point[1]; i++)
//                            {
//                                if(L_Border[i] == 2)
//                                {
//                                    Temp_Num ++;
//                                }
//                            }
//                            if(L_Right_Turn_Up_Point_1[1] >= 15 && (((float)Temp_Num / (float)(L_Start_Point[0] - L_Right_Turn_Up_Point_1[1])) >= 0.85f))
//                            {
//                                buzzer_flag = 1;
//                                L_Circle_State = 2;
//                                Circle_Temp_Num_2 = 0;
//                                Image_Count = 0;
//                            }
//                        }
                        break;
                    }
                }
//                if(Image_Count >= 225)
//                {
//                    buzzer_flag = 1;
//                    L_Circle_State = 2;
//                    Circle_Temp_Num_2 = 0;
//                    Image_Count = 0;
//                }
                break;
            }
            case 2: // 入环
            {
                IMU_JF_Flag = 1;
                Process_Image();

                if(Process_Mode == 1)
                {
                    Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, L_Arc_Unilateral_Patrol_2);
                }
                else
                {
                    Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, L_Arc_Patching_State_2, 0);
                }
//                Get_Deviation_Value(L_Circle, 2, 20, L_Line[0][1], C_Line, 1.0);
                Get_Deviation_Value(L_Circle, 2, 20, L_Line[0][1], C_Line, 1.0);

                Fitting_Speed_Control(0, L_Circle_State_2_Speed, Element_State, 2, Y_Meet, 57);

                uint8 Num = 0;
                for(i = 57; i > 0; i--)
                {
                    if(R_Line[i][0] == X_Border_Max - 2)
                    {
                        Num ++;
                    }
                }
                if(Num >= 35)
                {
                    State_2_Flag = 1;
                    break;
                }
                if(State_2_Flag == 1)
                {
                    if(Num <= 12)
                    {
                        buzzer_flag = 1;
                        L_Circle_State = 3;

                        Image_Count = 0;
                        break;
                    }
                }
                if(Image_Count >= 125)
                {
                    buzzer_flag = 1;
                    L_Circle_State = 3;

                    Image_Count = 0;
                }
                break;
            }
            case 3: // 环中
            {
                Process_Image();

                Get_R_Up_Turn_Right_Point_1(2);

                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, L_Arc_Unilateral_Patrol_2);
//                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);

                Get_Deviation_Value(L_Circle, 3, Y_Meet, Image_Y - 3, C_Line, 1.0);

                Fitting_Speed_Control(0, L_Circle_State_3_Speed, Element_State, 3, Y_Meet, 57);

                if(R_Up_Turn_Right_Point_Flag_1 == 1 && Y_Meet >= 15)
                {
                    buzzer_flag = 1;
                    L_Circle_State = 4;

                    Image_Count = 0;
                }
                if(My_ABS_F(Z_Yaw) >= 320)
                {
                    buzzer_flag = 1;
                    L_Circle_State = 5;
                    Image_Count = 0;
                }
                break;
            }
            case 4: // 预出环
            {
                Process_Image();

                Get_L_Right_Turn_Up_Point_1(3);
                Get_R_Intercept_And_Slope(R_Line[R_Statics][1] + 1, Image_Y - 10);

                if(Process_Mode == 1)
                {
                    Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, L_Arc_Unilateral_Patrol_4);
                }
                else
                {
                    Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, L_Arc_Patching_State_4, 0);
                }

                Get_Deviation_Value(L_Circle, 4, Y_Meet, Image_Y - 3, C_Line, 1.0);

                Fitting_Speed_Control(0, L_Circle_State_4_Speed, Element_State, 4, Y_Meet, 57);

                if(R_Straight_Flag == 1 && L_Right_Turn_Up_Point_Flag_1 == 1 && Y_Meet <= 15)
                { // 通过图像判断预出环完成
                    buzzer_flag = 1;
                    L_Circle_State = 5;
                    Image_Count = 0;
                }
                if(Image_Count >= 125)
                { // 超时判断
                    buzzer_flag = 1;
                    L_Circle_State = 5;
                    Image_Count = 0;
                }
                if(My_ABS_F(Z_Yaw) >= 320)
                { // 积分判断
                    buzzer_flag = 1;
                    L_Circle_State = 5;
                    Image_Count = 0;
                }
                break;
            }
            case 5: // 出环
            {
                IMU_JF_Flag = 0;
                // 移植注释：Pe177 - 变量声明但从未引用，注释此行
                // uint8 Num = 0;
                Process_Image();

                Get_L_Right_Turn_Up_Point_1(2);

                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, R_Unilateral_Patrol);
                Get_Deviation_Value(L_Circle, 5, Y_Meet, Image_Y - 3, C_Line, 1.0f);

                Fitting_Speed_Control(0, L_Circle_State_5_Speed, Element_State, 5, Y_Meet, 57);

//                for(i = 57; i > 47; i--)
//                {
//                    if(L_Border[i] == 2)
//                    {
//                        Num ++;
//                    }
//                }
//                ips200_show_uint( 0, 35*8, L_Right_Turn_Up_Point_Flag_1, 5 );
                if(L_Right_Turn_Up_Point_Flag_1 && L_Right_Turn_Up_Point_1[1] >= 50)
                {
                    buzzer_flag = 1;
                    Break_Flag = 1;
                    Image_Count = 0;
                }
                if(Image_Count >= 125)
                {
                    buzzer_flag = 1;
                    Break_Flag = 1;
                    Image_Count = 0;
                }
            }
        }
        Lost_Line(L_Start_Point, R_Start_Point);

//        TFT180_Show(Show_Flag);
        IPS200_Show(Show_Flag);
//        if(Other_Show_Flag == 1)
//        {
//            tft180_show_uint(0,61,State_1_Flag,1);
//            tft180_show_uint(0,71,L_Circle_State,1);
//        }
//        if(Other_Show_Flag == 2)
//        {
//            ips200_show_uint(0,61,State_1_Flag,1);
//            ips200_show_uint(0,71,L_Circle_State,1);
//        }

        if(Break_Flag == 1)
        {
            Image_Count_Flag = 0;
            Image_Count = 0;
            Circle_Temp_Num = 0;
//            L_Circle_Count++;

            Zebra_Flag = 1;
            break;
        }
    }
}

void R_Circle_Process(uint8 Process_Mode)
{
    uint8 i = 0;
    uint8 R_Circle_State = 1;
    uint8 State_1_Flag = 0;
    uint8 State_2_Flag = 0;
    uint8 Break_Flag = 0;

    Image_Count_Flag = 1;
    Z_Yaw = 0;
    Image_Count = 0;

    Temp_Patching_Line_Flag = 0;
//对补线相关参数进行清零
    if(Process_Mode != 1)
    {
        Arc_Point_1[0] = 0;
        Arc_Point_2[0] = 0;
        Arc_Point_3[0] = 0;
        Arc_Point_1[1] = 0;
        Arc_Point_2[1] = 0;
        Arc_Point_3[1] = 0;

        Last_Arc_Point_1[0] = 0;
        Last_Arc_Point_2[0] = 0;
        Last_Arc_Point_3[0] = 0;
        Last_Arc_Point_1[1] = 0;
        Last_Arc_Point_2[1] = 0;
        Last_Arc_Point_3[1] = 0;

        Arc_Point_1_Flag = 0;
        Arc_Point_2_Flag = 0;
        Arc_Point_3_Flag = 0;

        Circlr_State_2_Flag = 0;
        Stop_Find_1_2_Flag = 0;
    }
    buzzer_flag = 1;

    while(1)
    {
//        ips200_show_string(0, 36*8, "R_Arc:");
//        ips200_show_uint( 50, 36*8, R_Circle_State, 2 );
        if(Element_Break_Flag == 1)
        {
            Zebra_Count_Flag = 0;
            Zebra_Flag = 1;

            Element_Break_Flag = 0;
            Image_Count_Flag = 0;
            break;
        }
        Element_State = R_Circle;
        switch(R_Circle_State)
        {
            case 1:
            {
                uint8 Temp_Num = 0;
                Process_Image();

                Get_R_Left_Turn_Up_Point_1(2);
                Get_R_Arc_Turn_Point();

                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, L_Patching);
                Get_Deviation_Value(R_Circle, 1, Y_Meet, Image_Y - 3, C_Line, 1.0);

                Fitting_Speed_Control(0, R_Circle_State_1_Speed, Element_State, 1, Y_Meet, 57);

                switch(State_1_Flag)
                {
                    case 0:
                    {
                        for(i = 57; i > 47; i--)
                        {
                            if(R_Border[i] == 77)
                            {
                                Temp_Num ++;
                            }
                        }
                        if(Temp_Num >= 7)
                        {
                            State_1_Flag = 1;
                        }
                        break;
                    }
                    case 1:
                    {
                        for(i = 57; i > 37; i--)
                        {
                            if(R_Border[i] != 77)
                            {
                                Temp_Num ++;
                            }
                        }
                        if(R_Arc_Turn_Point_Flag == 1)
                        {
                            if(Temp_Num >= 12 && R_Left_Turn_Up_Point_Flag_1 == 1 && R_Left_Turn_Up_Point_1[1] >= 8 && R_Arc_Turn_Point[0][1] >= 30)
                            {
                                buzzer_flag = 1;
                                R_Circle_State = 2;
                                Circle_Temp_Num_2 = 0;
                                Image_Count = 0;
                            }
                        }
                        else if(Temp_Num >= 12 && R_Left_Turn_Up_Point_Flag_1 == 1 && R_Left_Turn_Up_Point_1[1] >= 8)
                        {
                            buzzer_flag = 1;
                            R_Circle_State = 2;
                            Circle_Temp_Num_2 = 0;
                            Image_Count = 0;
                        }
//                        if(R_Left_Turn_Up_Point_Flag_1 == 1)
//                        {
//                            Temp_Num = 0;
//                            for(i = R_Left_Turn_Up_Point_1[1]; i < R_Start_Point[1]; i++)
//                            {
//                                if(R_Border[i] == 77)
//                                {
//                                    Temp_Num ++;
//                                }
//                            }
//                            if(R_Left_Turn_Up_Point_1[1] >= 15 && (((float)Temp_Num / (float)(R_Start_Point[0] - R_Left_Turn_Up_Point_1[1])) >= 0.85f))
//                            {
//                                buzzer_flag = 1;
//                                R_Circle_State = 2;
//                                Circle_Temp_Num_2 = 0;
//                                Image_Count = 0;
//                            }
//                        }
                        break;
                    }
                }
                if(Image_Count >= 225)
                {
                    buzzer_flag = 1;
                    R_Circle_State = 2;
                    Circle_Temp_Num_2 = 0;
                    Image_Count = 0;
                }
                break;
            }
            case 2:
            {
                IMU_JF_Flag = 1;
                Process_Image();

                if(Process_Mode == 1)
                {
                    Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, R_Arc_Unilateral_Patrol_2);
                }
                else
                {
                    Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, R_Arc_Patching_State_2, 0);
                }
                Get_Deviation_Value(R_Circle, 2, 20, R_Line[0][1], C_Line, 1.0);

                Fitting_Speed_Control(0, R_Circle_State_2_Speed, Element_State, 2, Y_Meet, 57);

                uint8 Num = 0;
                for(i = 57; i > 0; i--)
                {
                    if(L_Line[i][0] == X_Border_Min + 2)
                    {
                        Num ++;
                    }
                }
                if(Num >= 35)
                {
                    State_2_Flag = 1;
                    break;
                }
                if(State_2_Flag == 1)
                {
                    if(Num <= 12)
                    {
                        buzzer_flag = 1;
                        R_Circle_State = 3;

                        Image_Count_Flag = 0;
                        Image_Count = 0;
                        break;
                    }
                }
                if(Image_Count >= 125)
                {
                    buzzer_flag = 1;
                    R_Circle_State = 3;
                    Image_Count = 0;
                }
                break;
            }
            case 3:
            {
                Process_Image();
                Get_L_Up_Turn_Left_Point_1(2);

                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, R_Arc_Unilateral_Patrol_2);
//                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);

                Get_Deviation_Value(R_Circle, 3, Y_Meet, Image_Y - 3, C_Line, 1.0);

                Fitting_Speed_Control(0, R_Circle_State_3_Speed, Element_State, 3, Y_Meet, 57);

                if(L_Up_Turn_Left_Point_Flag_1 == 1 && Y_Meet >= 15)
                {
                    buzzer_flag = 1;
                    R_Circle_State = 4;
                    Image_Count = 0;
                }
                if(My_ABS_F(Z_Yaw) >= 320)
                {
                    buzzer_flag = 1;
                    R_Circle_State = 5;
                    Image_Count = 0;
                }
                break;
            }
            case 4:
            {
                Process_Image();
                Get_R_Left_Turn_Up_Point_1(3);
                Get_L_Intercept_And_Slope(L_Line[L_Statics][1] + 1, Image_Y - 10);

                if(Process_Mode == 1)
                {
                    Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, R_Arc_Unilateral_Patrol_4);
                }
                else
                {
                    Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, R_Arc_Patching_State_4, 0);
                }

                Get_Deviation_Value(R_Circle, 4, Y_Meet, Image_Y - 3, C_Line, 1.0);

                Fitting_Speed_Control(0, R_Circle_State_4_Speed, Element_State, 4, Y_Meet, 57);

                if(L_Straight_Flag == 1 && R_Left_Turn_Up_Point_Flag_1 == 1 && Y_Meet <= 15)
                {
                    buzzer_flag = 1;
                    R_Circle_State = 5;
                    Image_Count = 0;
                }
                if(Image_Count >= 125)
                {
                    buzzer_flag = 1;
                    R_Circle_State = 5;
                    Image_Count = 0;
                }
                if(My_ABS_F(Z_Yaw) >= 320)
                {
                    buzzer_flag = 1;
                    R_Circle_State = 5;
                    Image_Count = 0;
                }
                break;
            }
            case 5:
            {
                IMU_JF_Flag = 0;
                // 移植注释：Pe177 - 变量声明但从未引用，注释此行
                // uint8 Num = 0;
                Process_Image();

                Get_R_Left_Turn_Up_Point_1(2);

                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, L_Unilateral_Patrol);
                Get_Deviation_Value(R_Circle, 5, Y_Meet, Image_Y - 3, C_Line, 1.0f);
                Fitting_Speed_Control(0, R_Circle_State_5_Speed, Element_State, 5, Y_Meet, 57);

//                for(i = 57; i > 47; i--)
//                {
//                    if(R_Border[i] == 77)
//                    {
//                        Num ++;
//                    }
//                }
//                ips200_show_uint( 0, 35*8, R_Left_Turn_Up_Point_Flag_1, 3 );
                if(R_Left_Turn_Up_Point_Flag_1 && R_Left_Turn_Up_Point_1[1] >= 50)
                {
                    buzzer_flag = 1;
                    Break_Flag = 1;
                    Image_Count = 0;
                }
                if(Image_Count > 125)
                {
                    buzzer_flag = 1;
                    Break_Flag = 1;
                    Image_Count = 0;
                }

            }
        }
        Lost_Line(L_Start_Point, R_Start_Point);

//        TFT180_Show(Show_Flag);
        IPS200_Show(Show_Flag);
//        if(Other_Show_Flag == 1)
//        {
//            tft180_show_uint(0,61,State_1_Flag,1);
//            tft180_show_uint(0,71,R_Circle_State,1);
//        }
//        if(Other_Show_Flag == 2)
//        {
//            ips200_show_uint(0,61,State_1_Flag,1);
//            ips200_show_uint(0,71,R_Circle_State,1);
//        }

        if(Break_Flag == 1)
        {
            Image_Count_Flag = 0;
            Image_Count = 0;
            Circle_Temp_Num = 0;
//            R_Circle_Count ++;

            Zebra_Flag = 1;
            break;
        }
    }
}

//十字根据横向生长点修正车身
float Last_Cross_Dif_Correct = 0;
float Now_Cross_Dif_Correct = 0;
float Cross_Get_Dif_Correct(void)
{
    uint8 i = 0;
    int16 L_Temp_Num = 0;
    int16 R_Temp_Num = 0;
    int16 Sum = 0;
    int16 Dif = 0;

    Last_Cross_Dif_Correct = Now_Cross_Dif_Correct;
    for(i = 0; i < L_Statics - 20; i ++)
    {
        if(L_Grow_Dir[i] == -3 || L_Grow_Dir[i] == 3)
        {
            L_Temp_Num ++;
        }
    }
    for(i = 0; i < R_Statics - 20; i ++)
    {
        if(R_Grow_Dir[i] == -3 || R_Grow_Dir[i] == 3)
        {
            R_Temp_Num ++;
        }
    }
    Dif = L_Temp_Num - R_Temp_Num;
    Sum = L_Temp_Num + R_Temp_Num;
    Now_Cross_Dif_Correct = 0.3 * Limit_Float((float)Dif / (float)Sum, -0.5f, 0.5f) + 0.7 * Last_Cross_Dif_Correct;
    return Now_Cross_Dif_Correct;
}
void Cross_Get_Middle_Line_old(uint8 side)
{
    uint8 i = 0;
    uint8 Max_Row_Difference = 0;
    uint8 Min_Row_Difference = 0;

    Get_Row_Difference(L_Border, R_Border);

    Max_Row_Difference = Row_Difference[0];
    Min_Row_Difference = Row_Difference[10];
    for(i = 0; i < 10; i ++)
    {
        if(Max_Row_Difference < Row_Difference[i])
        {
            Max_Row_Difference = Row_Difference[i];
        }
    }
    for(i = 10; i < 19; i ++)
    {
        if(Min_Row_Difference > Row_Difference[i])
        {
            Min_Row_Difference = Row_Difference[i];
        }
    }
    if(((Max_Row_Difference - Min_Row_Difference) >= 25) && (Max_Row_Difference >= 70))
    {
        Temp_Patching_Line_Flag = 1;
    }

//    uint8 Temp_Num = 0;
//    for(i = 3; i < 8; i ++)
//    {
//        if(Row_Difference[i] <= 40)
//        {
//            Temp_Num ++;
//        }
//    }
//    if(Temp_Num == 5)
//    {
//        Temp_Patching_Line_Flag = 0;
//    }
//    if(L_Statics <= 90 && R_Statics <= 90)
//    {
//        Temp_Patching_Line_Flag = 0;
//    }
    if(Row_Difference[14] >= 70)
    {
        Temp_Patching_Line_Flag = 0;
    }

    if(Temp_Patching_Line_Flag == 1)
    {
        float Slope_Rate = 0;
        float Intercept = 0;
        if(side == 1){ // 左侧群主补线
            Calculate_Slope_Intercept(47, 57, L_Border, &Slope_Rate, &Intercept);
            for(i = 2; i < 47; i++)
                L_Border[i] = (uint8)(Slope_Rate * (float)i + Intercept);
        }
        else if(side == 2){ // 右侧
            Calculate_Slope_Intercept(47, 57, R_Border, &Slope_Rate, &Intercept);
            for(i = 2; i < 47; i++)
                R_Border[i] = (uint8)(Slope_Rate * (float)i + Intercept);
        }
        else{ // 双侧
            Calculate_Slope_Intercept(47, 57, L_Border, &Slope_Rate, &Intercept);
            for(i = 2; i < 47; i++)
                L_Border[i] = (uint8)(Slope_Rate * (float)i + Intercept);
            Calculate_Slope_Intercept(47, 57, R_Border, &Slope_Rate, &Intercept);
            for(i = 2; i < 47; i++)
                R_Border[i] = (uint8)(Slope_Rate * (float)i + Intercept);
            Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);
        }
    }
}
#define L_MAX    5
#define R_MAX    5
void Cross_Get_Middle_Line(uint8 element_state)
{
    uint8 i = 0;
//    uint8 num = 0;
//    uint8 num1 = 0;
//    uint8 num2 = 0;
    float Slope_Rate = 0;
    float Intercept = 0;
//    uint8 L_flag = 0;
//    uint8 R_flag = 0;

    if(L_Up_Turn_Left_Point_Flag_1 && L_Right_Turn_Up_Point_Flag_1)  // 左侧两拐点都找到
    {
        if(element_state == 1 || element_state == 5)  // 四个拐点都可能找到的状态
        {
            Calculate_Slope_Intercept_break(L_Right_Turn_Up_Point_1[1]-10, L_Start_Point[1], L_Right_Turn_Up_Point_1[1],L_Up_Turn_Left_Point_1[1], L_Border, &Slope_Rate, &Intercept);
            for(i = L_Start_Point[1]; i >= L_Right_Turn_Up_Point_1[1]-5; i--)
            {
                L_Border[i] = (uint8)func_limit_ab((Slope_Rate * (float)i + Intercept), 3,76);
            }
            Slope_Rate = 0;
            Intercept = 0;
        }
        else if(element_state == 2 || element_state ==6)
        {
                Calculate_Slope_Intercept(L_Right_Turn_Up_Point_1[1]-10, L_Right_Turn_Up_Point_1[1], L_Border, &Slope_Rate, &Intercept);
                for(i = L_Start_Point[1]; i >= L_Right_Turn_Up_Point_1[1]-10; i--)
                {
                    L_Border[i] = (uint8)func_limit_ab((Slope_Rate * (float)i + Intercept), 3,76);
                }
                Slope_Rate = 0;
                Intercept = 0;
        }
    }
    else if(L_Up_Turn_Left_Point_Flag_1 || L_Right_Turn_Up_Point_Flag_1) // 只找到一个
    {
        if(L_Right_Turn_Up_Point_Flag_1) // 只找到远点
        {
                Calculate_Slope_Intercept(L_Right_Turn_Up_Point_1[1]-10, L_Right_Turn_Up_Point_1[1], L_Border, &Slope_Rate, &Intercept);
                for(i = L_Start_Point[1]; i < L_Right_Turn_Up_Point_1[1]-10; i--)
                {
                    L_Border[i] = (uint8)func_limit_ab((Slope_Rate * (float)i + Intercept), 3,76);
                }
        }
        else // 只找到近点
        {
            Calculate_Slope_Intercept(L_Up_Turn_Left_Point_1[1], L_Up_Turn_Left_Point_1[1]+10, L_Border, &Slope_Rate, &Intercept);
            for(i = L_Start_Point[1]; i >= 2; i--){
                L_Border[i] = (uint8)func_limit_ab((Slope_Rate * (float)i + Intercept), 3,76);
            }
        }
        Slope_Rate = 0;
        Intercept = 0;
    }
    else
    { // 一个都没找到
        Cross_Get_Middle_Line_old(1);
        Slope_Rate = 0;
        Intercept = 0;
    }

/********************right****************/
    if(R_Up_Turn_Right_Point_Flag_1 && R_Left_Turn_Up_Point_Flag_1) // 右侧两拐点都找到
    {
        if(element_state == 1 || element_state == 5)
        {
            Calculate_Slope_Intercept_break(R_Left_Turn_Up_Point_1[1]-10, R_Start_Point[1], R_Left_Turn_Up_Point_1[1], R_Up_Turn_Right_Point_1[1], R_Border, &Slope_Rate, &Intercept);
            for(i = R_Start_Point[1]; i >= R_Left_Turn_Up_Point_1[1]-10; i--)
            {
                R_Border[i] = (uint8)func_limit_ab((Slope_Rate * (float)i + Intercept), 3,76);
            }
            Slope_Rate = 0;
            Intercept = 0;
        }
        else if(element_state == 2 || element_state == 6)
        {
                Calculate_Slope_Intercept(R_Left_Turn_Up_Point_1[1]-10, R_Left_Turn_Up_Point_1[1], R_Border, &Slope_Rate, &Intercept);
                for(i = R_Start_Point[1]; i >= R_Left_Turn_Up_Point_1[1]-10; i--)
                {
                    R_Border[i] = (uint8)func_limit_ab((Slope_Rate * (float)i + Intercept), 3,76);
                }
                Slope_Rate = 0;
                Intercept = 0;
        }
    }
    else if(R_Up_Turn_Right_Point_Flag_1 || R_Left_Turn_Up_Point_Flag_1) // 只找到一个
    {
        if(R_Left_Turn_Up_Point_Flag_1) // 只找到远点
        {
                Calculate_Slope_Intercept(R_Left_Turn_Up_Point_1[1]-10, R_Left_Turn_Up_Point_1[1], R_Border, &Slope_Rate, &Intercept);
                for(i = R_Start_Point[1]; i > R_Left_Turn_Up_Point_1[1]-10; i--){
                    R_Border[i] = (uint8)func_limit_ab((Slope_Rate * (float)i + Intercept), 3,76);
                }
        }
        else // 只找到近点
        {
            Calculate_Slope_Intercept(R_Up_Turn_Right_Point_1[1], R_Start_Point[1], R_Border, &Slope_Rate, &Intercept);
            for(i = R_Start_Point[1]; i >= 2; i--)
            {
                R_Border[i] = (uint8)func_limit_ab((Slope_Rate * (float)i + Intercept), 3,76);
            }
        }
        Slope_Rate = 0;
        Intercept = 0;
    }
    else
    { // 一个都没找到
        Cross_Get_Middle_Line_old(2);
        Slope_Rate = 0;
        Intercept = 0;
    }

    Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);
}
//Z_Yaw
void Cross_Process(void)
{
    uint8 Break_Flag = 0;
    uint8 Cross_Process_State = 0;
    // 移植注释：Pe550 - Element 被赋值但从未读取，注释声明行
    // uint8 Element = 0;
    Cross_Process_State = 1;

//    Element = Element_State;

    // 移植注释：Pe550 - Element 被赋值但从未读取，注释赋值行
    // Element = Cross;

    Z_Yaw = 0;

    Cross_State_4_Direction = 0;

    Temp_Patching_Line_Flag = 0;
    while(1)
    {
//        ips200_show_string(0, 36*8, "Cross:");
//        ips200_show_uint( 50, 36*8, Cross_Process_State, 2 );
//        ips200_show_uint( 20, 36*8, IMU_JF_Flag, 2 );
//        ips200_show_float( 50, 36*8, imu660ra.eulerAngle.yaw, 3,2 );
//        ips200_show_float( 70, 36*8, Z_Yaw, 3, 2 );

        if(Element_Break_Flag == 1)
        {
            Zebra_Count_Flag = 0;
            Zebra_Flag = 1;

            Element_Break_Flag = 0;
            Image_Count_Flag = 0;
            break;
        }
        switch(Cross_Process_State)
        {
            case 1:
            {
                Process_Image();
                Get_Information();

                Cross_Get_Middle_Line(Cross_Process_State);
//                Cross_Get_Middle_Line_old(0);

                Get_Deviation_Value(Cross, 1, Y_Meet, Image_Y - 3, C_Line, 1.0);

//                Deviation_Value = 0.0f;

                Fitting_Speed_Control(0, Cross_Speed, Element_State, 1, Y_Meet, 57);

//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);

                if(L_Start_Point[0] == (X_Border_Min + 1) && R_Start_Point[0] == (X_Border_Max - 1))
                {
                    buzzer_flag = 1;
                    Cross_Process_State = 2;
                }
                break;
            }
            case 2:
            {
                Process_Image();
                Get_Information();
//                Get_L_Right_Turn_Up_Point_2(20,60);
//                Get_R_Left_Turn_Up_Point_2(20,60);

                Cross_Get_Middle_Line(Cross_Process_State);
//                Cross_Get_Middle_Line_old(0);

//                Deviation_Value = 0.0f;

                Get_Deviation_Value(Cross, 2, Y_Meet, Image_Y - 3, C_Line, 1.0);

                Fitting_Speed_Control(0, Cross_Speed, Element_State, 2, Y_Meet, 57);

//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);

                if(L_Right_Turn_Up_Point_Flag_1 == 0 && R_Left_Turn_Up_Point_Flag_1 == 0)
                {
                    buzzer_flag = 1;
                    Cross_Process_State = 3;
                }
                break;
            }
            case 3:
            {
                Process_Image();

                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);

                Get_Deviation_Value(Cross, 3, Y_Meet, Image_Y - 3, C_Line, 1.0);

                Fitting_Speed_Control(0, Cross_State_4_Speed, Element_State, 3, Y_Meet, 57);
//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);

                if(L_Start_Point[0] != (X_Border_Min + 1) || R_Start_Point[0] != (X_Border_Max - 1))
                {
                    buzzer_flag = 1;
                    Cross_Process_State = 4;
                    Image_Count_Flag = 1;
                    Image_Count = 0;
                }
                break;
            }
            case 4:
            {
                IMU_JF_Flag = 1;
                Process_Image();
                Get_Information();
                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);

                Get_Deviation_Value(Cross, 4, Y_Meet, Image_Y - 3, C_Line, 1.0);
                // 移植注释：Pe550 - Element 赋值后从未读取，对应声明已注释
                // Element = Element_Judgement(Find_Line_Image, L_Statics, R_Statics, L_Grow_Dir, R_Grow_Dir, L_Line, R_Line, X_Meet, Y_Meet, L_Border, R_Border);
                Fitting_Speed_Control(0, Cross_State_4_Speed, Element_State, 4, Y_Meet, 57);

//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);
                if(Deviation_Value <= -0.4f)
                {
                    Cross_State_4_Direction --;
                }
                else if(Deviation_Value >= 0.4f)
                {
                    Cross_State_4_Direction ++;
                }

                if(My_ABS_F(Z_Yaw) >= 230)
                {
                    buzzer_flag = 1;
                    Cross_Process_State = 5;
                    Image_Count = 0;
                    IMU_JF_Flag = 0;
                }
                break;
            }
            case 5:
            {
                Process_Image();

//                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, L_R_Patching, 0);
                Cross_Get_Middle_Line(Cross_Process_State);
                Get_Deviation_Value(Cross, 5, Y_Meet, Image_Y - 3, C_Line, 1.0);

                Fitting_Speed_Control(0, Cross_Speed, Element_State, 5, Y_Meet, 57);

//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);
                if(L_Start_Point[0] == (X_Border_Min + 1) && R_Start_Point[0] == (X_Border_Max - 1))
                {
//                    Cross_Count ++;
                    buzzer_flag = 1;
                    Cross_Process_State = 6;
                }
                if(Image_Count >= 125)
                {
//                    Cross_Count ++;
                    buzzer_flag = 1;
                    Break_Flag = 1;
                }
                break;
            }
            case 6:
            {
                Process_Image();
                Get_Information();
                Get_L_Right_Turn_Up_Point_1(2);
                Get_R_Left_Turn_Up_Point_1(2);
//                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, L_R_Patching, 0);
//                Cross_Get_Middle_Line(Cross_Process_State);
//                Turn_Point_Patching_Line(Element_State);

                Get_Deviation_Value(Cross, 6, Y_Meet, Image_Y - 3, C_Line, 1.0);

                Fitting_Speed_Control(0, Cross_Speed, Element_State, 6, Y_Meet, 57);

//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);

                if(L_Right_Turn_Up_Point_Flag_1 == 0 && R_Left_Turn_Up_Point_Flag_1 == 0)
                {
                    buzzer_flag = 1;
                    Cross_Process_State = 7;
                }
                if(Image_Count >= 125)
                {
                    buzzer_flag = 1;
                    Break_Flag = 1;
                }
                break;
            }
            case 7:
            {
                Process_Image();
                Get_Information();
                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);
                Turn_Point_Patching_Line(Element_State);

                Get_Deviation_Value(Cross, 7, Y_Meet, Image_Y - 3, C_Line, 1.0);

                Fitting_Speed_Control(1, Cross_Speed, Element_State, 7, Y_Meet, 57);

//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);

                if(L_Start_Point[0] != (X_Border_Min + 1) && R_Start_Point[0] != (X_Border_Max - 1))
                {
                    buzzer_flag = 1;
                    Break_Flag = 1;
                }
                if(Image_Count >= 125)
                {
                    buzzer_flag = 1;
                    Break_Flag = 1;
                }
                break;
            }
        }
        Lost_Line(L_Start_Point, R_Start_Point);
        if(Break_Flag == 1)
        {
            Image_Count = 0;
            Cross_Process_State = 0;
            break;
        }
    }
}
void Cross_Process_old(void)
{
    uint8 Break_Flag = 0;
    uint8 Cross_Process_State = 0;
    // 移植注释：Pe550 - Element 被赋值但从未读取，注释声明行
    // uint8 Element = 0;
    Cross_Process_State = 1;

//    Element = Element_State;

    // 移植注释：Pe550 - Element 被赋值但从未读取，注释赋值行
    // Element = Cross;

    Z_Yaw = 0;

    Cross_State_4_Direction = 0;

    Temp_Patching_Line_Flag = 0;
    while(1)
    {
        if(Element_Break_Flag == 1)
        {
            Zebra_Count_Flag = 0;
            Zebra_Flag = 1;

            Element_Break_Flag = 0;
            Image_Count_Flag = 0;
            break;
        }
        switch(Cross_Process_State)
        {
            case 1:
            {
                Process_Image();

                Cross_Get_Middle_Line_old(0);

                Get_Deviation_Value(Cross, 1, Y_Meet, Image_Y - 3, C_Line, 1.0);

//                Deviation_Value = 0.0f;

                Fitting_Speed_Control(0, Cross_Speed, Element_State, 1, Y_Meet, 57);

//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);

                if(L_Start_Point[0] == (X_Border_Min + 1) && R_Start_Point[0] == (X_Border_Max - 1))
                {
                    buzzer_flag = 1;
                    Cross_Process_State = 2;
                }
                break;
            }
            case 2:
            {
                Process_Image();
                Get_Information();
                Get_L_Right_Turn_Up_Point_1(2);
                Get_R_Left_Turn_Up_Point_1(2);

                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, L_R_Patching, 0);

//                Deviation_Value = 0.0f;

                Get_Deviation_Value(Cross, 2, Y_Meet, Image_Y - 3, C_Line, 1.0);

                Fitting_Speed_Control(0, Cross_Speed, Element_State, 2, Y_Meet, 57);

//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);

                if(L_Right_Turn_Up_Point_Flag_1 == 0 && R_Left_Turn_Up_Point_Flag_1 == 0)
                {
                    buzzer_flag = 1;
                    Cross_Process_State = 3;
                }
                break;
            }
            case 3:
            {
                Process_Image();

                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);

                Get_Deviation_Value(Cross, 3, Y_Meet, Image_Y - 3, C_Line, 1.0);

                Fitting_Speed_Control(0, Cross_State_4_Speed, Element_State, 3, Y_Meet, 57);
//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);

                if(L_Start_Point[0] != (X_Border_Min + 1) || R_Start_Point[0] != (X_Border_Max - 1))
                {
                    buzzer_flag = 1;
//                    Cross_Process_State = 4;
                    Break_Flag = 1;
                    Image_Count_Flag = 1;
                    Image_Count = 0;
                }
                break;
            }
            case 4:
            {
                IMU_JF_Flag = 1;
                Process_Image();
                Get_Information();
                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);

                Get_Deviation_Value(Cross, 4, Y_Meet, Image_Y - 3, C_Line, 1.0);
                // 移植注释：Pe550 - Element 赋值后从未读取，对应声明已注释
                // Element = Element_Judgement(Find_Line_Image, L_Statics, R_Statics, L_Grow_Dir, R_Grow_Dir, L_Line, R_Line, X_Meet, Y_Meet, L_Border, R_Border);
                Fitting_Speed_Control(0, Cross_State_4_Speed, Element_State, 4, Y_Meet, 57);

//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);
                if(Deviation_Value <= -0.4f)
                {
                    Cross_State_4_Direction --;
                }
                else if(Deviation_Value >= 0.4f)
                {
                    Cross_State_4_Direction ++;
                }

                if(My_ABS_F(Z_Yaw) >= 230)
                {
                    buzzer_flag = 1;
                    Cross_Process_State = 5;
                    Image_Count = 0;
                    IMU_JF_Flag = 0;
                }
                break;
            }
            case 5:
            {
                Process_Image();

                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, L_R_Patching, 0);
                Get_Deviation_Value(Cross, 5, Y_Meet, Image_Y - 3, C_Line, 1.0);

                Fitting_Speed_Control(0, Cross_Speed, Element_State, 5, Y_Meet, 57);

//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);
                if(L_Start_Point[0] == (X_Border_Min + 1) && R_Start_Point[0] == (X_Border_Max - 1))
                {
//                    Cross_Count ++;
//                    buzzer_flag = 1;
                    Cross_Process_State = 6;
                }
                if(Image_Count >= 125)
                {
//                    Cross_Count ++;
                    buzzer_flag = 2;
                    Break_Flag = 1;
                }
                break;
            }
            case 6:
            {
                Process_Image();
                Get_Information();
                Get_L_Right_Turn_Up_Point_1(2);
                Get_R_Left_Turn_Up_Point_1(2);
                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, L_R_Patching, 0);
                Turn_Point_Patching_Line(Element_State);

                Get_Deviation_Value(Cross, 6, Y_Meet, Image_Y - 3, C_Line, 1.0);

                Fitting_Speed_Control(0, Cross_Speed, Element_State, 6, Y_Meet, 57);

//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);

                if(L_Right_Turn_Up_Point_Flag_1 == 0 && R_Left_Turn_Up_Point_Flag_1 == 0)
                {
                    buzzer_flag = 1;
                    Cross_Process_State = 7;
                }
                if(Image_Count >= 125)
                {
                    buzzer_flag = 2;
                    Break_Flag = 1;
                }
                break;
            }
            case 7:
            {
                Process_Image();
                Get_Information();
                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);
                Turn_Point_Patching_Line(Element_State);

                Get_Deviation_Value(Cross, 7, Y_Meet, Image_Y - 3, C_Line, 1.0);

                Fitting_Speed_Control(1, Cross_Speed, Element_State, 7, Y_Meet, 57);
//                Fitting_Speed_Control(1, 0, Element_State, 7, Y_Meet, 57);

//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);

                if(L_Start_Point[0] != (X_Border_Min + 1) || R_Start_Point[0] != (X_Border_Max - 1))
                {
                    buzzer_flag = 1;
                    Break_Flag = 1;
                }
                if(Image_Count >= 125)
                {
                    buzzer_flag = 1;
                    Break_Flag = 1;
                }
                break;
            }
        }
        Lost_Line(L_Start_Point, R_Start_Point);
        if(Break_Flag == 1)
        {
            Image_Count = 0;
            Cross_Process_State = 0;
            break;
        }
    }
}

void Zebra_Process(void)
{
    buzzer_flag = 1;

    Image_Count = 0;
//    Image_Count_Flag = 1;

    Temp_Patching_Line_Flag = 0;
    while(1)
    {
        if(Element_Break_Flag == 1)
        {
            Zebra_Count_Flag = 0;
            Zebra_Flag = 1;

            Element_Break_Flag = 0;
            Image_Count_Flag = 0;
            break;
        }

        if(Zebra_Count_Flag == 0)
        {
            L_Statics = 0;
            R_Statics = 0;

            Adaptive_L_Statics = 0;
            Adaptive_R_Statics = 0;
            Binarization_L_Statics = 0;
            Binarization_R_Statics = 0;

            Copy_Zip_Image();
            Draw_Black_Box(Black_Box_Value, Find_Line_Image);
            if(Get_Start_Point(25, Find_Line_Image, Adaptive_L_Start_Point, Adaptive_R_Start_Point, 1, 78) == 1)
            {
                Dir_Labyrinth_5((uint16)Use_Num, Vague_Image, Adaptive_L_Line, Adaptive_R_Line, Adaptive_L_Grow_Dir, Adaptive_R_Grow_Dir, &Adaptive_L_Statics, &Adaptive_R_Statics, &Adaptive_X_Meet, &Adaptive_Y_Meet,
                                Adaptive_L_Start_Point[0], Adaptive_L_Start_Point[1], Adaptive_R_Start_Point[0], Adaptive_R_Start_Point[1], 0);
            }
            Thres_Record_Process();
            Black_Box_Value = (uint8)(0.45f * (float)sqrt(Adaptive_L_Thres_Min * Adaptive_L_Thres_Min + Adaptive_R_Thres_Min * Adaptive_R_Thres_Min)) + (uint8)(0.1f * (float)Black_Box_Value_1);      //0.45f即0.9 * 0.5，0.9为权重，0.5可自己调节
            Bilatreal_Line_Fitting();
            Get_Border(L_Statics, R_Statics, Image_Y - 3, 2, L_Border, R_Border, L_Line, R_Line);

            Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);
            Get_Deviation_Value(Straightaway, 0, Y_Meet, Image_Y - 3, C_Line, 2.0);

            if(Image_Count >= 40)
            {
                Zebra_Count_Flag = 1;
                Zebra_Flag = 0;

                Image_Count = 0;
                Image_Count_Flag = 0;

                //经过第一个斑马线后开启速度控制,在此之前保持低速
                Speed_Contral_Flag = 1;
                break;
            }
        }
        else if(Zebra_Count_Flag == 1)
        {
            Process_Image();
            Deviation_Value = 0.0f;
            if(R_Start_Point[0] - L_Start_Point[0] <= 20)
            {
//                system_delay_ms(500);

//                stop_flag=1;
                while(1)
                {
                    Yao.Target_Speed=0;
                    forward_target=0;
//                    balance_flag = 0;

                    if(Element_Break_Flag == 1)
                    {
                        Zebra_Count_Flag = 0;
                        Zebra_Flag = 1;

                        Element_Break_Flag = 0;
                        Image_Count_Flag = 0;
                        break;
                    }
                }
            }
            if(Image_Count >= 125)
            {
                Image_Count = 0;
                Image_Count_Flag = 0;
                break;
            }
        }
//        TFT180_Show(Show_Flag);
        IPS200_Show(Show_Flag);
    }
}

uint8 Ramp_State = 0;
void Ramp_Process(void)
{
//    buzzer_flag = 1;
    Ramp_State = 2;
    uint16 i = 0;
    uint8 Break_Flag = 0;

//    find_ramp_flag = 1;

    Image_Count_Flag = 1;

    Temp_Patching_Line_Flag = 0;
    while(1)
    {
        if(Element_Break_Flag == 1)
        {
            Zebra_Count_Flag = 0;
            Zebra_Flag = 1;

            Element_Break_Flag = 0;
//            find_ramp_flag = 0;
            Image_Count_Flag = 0;
            break;
        }

        switch(Ramp_State)
        {
            //识别到坡道
//            case 1:
//            {
//                Process_Image();
//
//                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);
//                Get_Deviation_Value(Ramp, 1, Y_Meet + 5, Image_Y - 3, C_Line, 2.2);
//
//                Fitting_Speed_Control(0, Ramp_State_1_Speed, Element_State, 1, Y_Meet, 57);
//
//                Lost_Line(L_Start_Point, R_Start_Point);
//
////                TFT180_Show(Show_Flag);
//                IPS200_Show(Show_Flag);
//
//                if(Image_Num % 10 == 0)
//                {
////                    Barrier_Distance = TOF_Get_Distance_mm();
//                }
//
//                if(dis_tof_mm >= 8000)
//                {
//                    i ++;
//                }
//                if(i >= 10)
//                {
//                    i = 0;
//                    Ramp_State = 2;
////                    buzzer_flag = 1;
//                }
//                if(Image_Count >= 500)
//                {
//                    Break_Flag = 1;
//                }
//                break;
//            }
            //上坡
            case 2:
            {
                Process_Image();

                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);
                Get_Deviation_Value(Ramp, 2, Y_Meet + 5, Image_Y - 3, C_Line, 2.2);

                Fitting_Speed_Control(0, Ramp_State_2_Speed, Element_State, 2, Y_Meet, 57);

                Lost_Line(L_Start_Point, R_Start_Point);

//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);

                if(Image_Num % 10 == 0)
                {
//                    Barrier_Distance = TOF_Get_Distance_mm();
                }
                if(dis_tof_mm <= 200)
                {
                    i ++;
                }
                if(i >= ramp_up_delay)
                {
                    i = 0;
                    Ramp_State = 3;
//                    buzzer_flag = 1;
                }
                if(Image_Count >= 300)
                {
                    Break_Flag = 1;
                }
                break;
            }
            //坡顶部不好识别。直接判断下坡
            //下坡
            case 3:
            {
                Process_Image();

                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);
                Get_Deviation_Value(Ramp, Ramp_State, Y_Meet + 5, Image_Y - 3, C_Line, 2.2);

                Fitting_Speed_Control(0, Ramp_State_2_Speed, Element_State, 3, Y_Meet, 57);

                Lost_Line(L_Start_Point, R_Start_Point);

//                TFT180_Show(Show_Flag);
                IPS200_Show(Show_Flag);

                if(Image_Num % 10 == 0)
                {
//                    Barrier_Distance = TOF_Get_Distance_mm();
                }
                if(dis_tof_mm >= ramp_end_dis)
                {
                    i ++;
                }
                if(i >= 10)
                {
                    i = 0;
                    Break_Flag = 1;
//                    buzzer_flag = 1;
                }
                if(Image_Count >= 500)
                {
                    Break_Flag = 1;
                }
                break;
            }
        }
        if(Break_Flag == 1)
        {
            Ramp_Flag = 0;
            Image_Count = 0;
            Break_Flag = 0;
//            find_ramp_flag = 0;
            break;
        }
    }
}
void Three_Bif_Process(void)
{

}
void Barrier_Process(void)
{

}
void Disconnection_Process(void)
{

}
void T_Way_Process(void)
{

}
void L_Garage_Process(void)
{

}
void R_Garage_Process(void)
{

}

void Small_S_Process(void)
{
    uint8 Element = 0;
//    buzzer_flag = 1;
    Image_Count = 0;
    Image_Count_Flag = 1;

    Temp_Patching_Line_Flag = 0;
    while(1)
    {
        Process_Image();
        Get_Information();
        Element = Element_Judgement(Find_Line_Image, L_Statics, R_Statics, L_Grow_Dir, R_Grow_Dir, L_Line, R_Line, X_Meet, Y_Meet, L_Border, R_Border);
        Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);
        Get_Deviation_Value(Small_S, 0, Y_Meet + 5, Image_Y - 3, C_Line, 2.0);

        Fitting_Speed_Control(1, 0, Element_State, 0, Y_Meet, 57);

        Lost_Line(L_Start_Point, R_Start_Point);

//        TFT180_Show(Show_Flag);
        IPS200_Show(Show_Flag);

        if(Small_S_Break_Flag == 1 || (Element != 2 && Element != 0 && Element != 18))
        {
            Small_S_Break_Flag = 0;
//            buzzer_flag = 1;
            Image_Count = 0;
            Image_Count_Flag = 0;

            break;
        }
    }
}

uint8 temp_flag_jump = 0;
uint8 flag_jump_2 = 0;
uint8 jump_line = 20;
uint8 jump_line_slowdown = 20;
void Jump_Process(void)
{
    while(1)
    {
        Process_Image();
        Get_Information();

        if(Y_Meet >= jump_line && temp_flag_jump == 0)
            flag_jump = 1;
//            ips200_show_uint( 190 , 10*8 , flag_jump , 2 );
        if(flag_jump == 0)
        {
            Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);

            Get_Deviation_Value(Straightaway, 0, Y_Meet, Image_Y - 3, C_Line, 2.0);

            if(Y_Meet >= jump_line_slowdown && temp_flag_jump == 0)
                flag_jump_2 = 1;

            Fitting_Speed_Control(0, Jump_speed, Element_State, 0, Y_Meet, 57);

            Lost_Line(L_Start_Point, R_Start_Point);
        }
//        else if(flag_jump == 1)
//            Fitting_Speed_Control(0, 0, Element_State, 0, Y_Meet, 57);

        IPS200_Show(Show_Flag);

        if(temp_flag_jump && !flag_jump)
        {
            flag_jump_2 = 0;
//            Jump_Control_Flag = 0;
            break;
        }
        if(Element_Break_Flag == 1)
        {
            flag_jump_2 = 0;
            flag_jump = 0;
            break;
        }
    }
}

void Single_Process(void);

uint16 Bypass_Count1 = 60;
uint16 Bypass_Count2 = 5;
uint16 Bypass_Count3 = 60;
uint8 Bypass_Line = 17;
float yaw_bypass = 0.8;
void Bypass_Process(void)
{
    uint8 elem = 0;
    float Record_Angle = 0;
    uint8 judge_line = 0;

    float sum_temp_angle = 0;

    uint8 Break_flag = 0;

    uint8 Bypass_State = 1;
    uint8 Bypass_State_temp = 1;

    Image_Count = 0;
    Image_Count_Flag = 0;

    while(1)
    {

//            ips200_show_uint( 190 , 10*8 , Bypass_State , 3 );
//            ips200_show_uint( 190 , 10*8 , L_Up_Turn_Right_Point_1[1] , 3 );
//            ips200_show_int( 190 , 11*8 , Image_Count , 4 );
//            ips200_show_float( 190 , 13*8 , imu660ra.eulerAngle.yaw , 3,3 );
//            ips200_show_float( 190 , 14*8 , Target_Yaw , 3,3 );
//        Fitting_Speed_Control(0, Jump_speed, Element_State, 0, Y_Meet, 57);
        switch(Bypass_State)
        {
            case 1: // 未到达指定行数，继续寻迹
                Fitting_Speed_Control(0, Jump_speed, Element_State, 0, Y_Meet, 57);

                turn_mode = 2;

                Process_Image();
                Get_Information();

                Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);

                Get_Deviation_Value(Straightaway, 0, Y_Meet, Image_Y - 3, C_Line, 2.0);

                elem = Element_Judgement(Find_Line_Image, L_Statics, R_Statics, L_Grow_Dir, R_Grow_Dir, L_Line, R_Line, X_Meet, Y_Meet, L_Border, R_Border);
                if((elem == Single_State/* || func_abs(L_Up_Turn_Right_Point_1[1] - R_Up_Turn_Left_Point_1[1]) >= 6*/)
                        && Single_Control_Flag != 0)
                {
                    Single_Process();
                    Break_flag = 1;
                }

                judge_line = R_Up_Turn_Left_Point_1[1] < R_Up_Turn_Left_Point_1[1] ? R_Up_Turn_Left_Point_1[1]:R_Up_Turn_Left_Point_1[1];

                if(judge_line >= Bypass_Line)
                {
                    Image_Count_Flag = 1;
                    imu660ra.eulerAngle.yaw = 0;
                    Record_Angle = imu660ra.eulerAngle.yaw;
//                    Target_Yaw = Record_Angle;
                    Bypass_State = 2;
                    turn_mode = 3;
                }
                break;
            case 2: // 到达，开始闭角度环绕行
//                turn_mode = 3;

                Copy_Zip_Image();
                Draw_Black_Box(Black_Box_Value, Find_Line_Image);

                if(Image_Count % 2 == 0)
                {
                    if(Image_Count <= (uint16)(Bypass_Count1/2))
                        Bypass_State_temp = 1;
                    else if((uint16)(Bypass_Count1/2) <= Image_Count && Image_Count <= (uint16)(Bypass_Count1))
                        Bypass_State_temp = 2;
                    else if((uint16)(Bypass_Count1) <= Image_Count && Image_Count <= (uint16)(Bypass_Count1 + Bypass_Count2))
                        Bypass_State_temp = 3;
                    else if((uint16)(Bypass_Count1 + Bypass_Count2) <= Image_Count && Image_Count <= (uint16)(Bypass_Count1 + Bypass_Count2 + Bypass_Count3/2))
                        Bypass_State_temp = 4;
                    else if((uint16)(Bypass_Count1 + Bypass_Count2 + Bypass_Count3/2) <= Image_Count && Image_Count <= (uint16)(Bypass_Count1 + Bypass_Count2 + Bypass_Count3))
                        Bypass_State_temp = 5;
                    else
                        Bypass_State_temp = 6;

                    switch(Bypass_State_temp)
                    {
                        case 1:
                            sum_temp_angle += func_abs(yaw_bypass);
                            sum_temp_angle = func_limit_ab(sum_temp_angle, 0, 45);
                            break;
                        case 2:
                            sum_temp_angle -= func_abs(yaw_bypass);
                            sum_temp_angle = func_limit_ab(sum_temp_angle, 0, 45);
                            break;
                        case 3:
                            sum_temp_angle = 0;
                            break;
                        case 4:
                            sum_temp_angle -= func_abs(yaw_bypass);
                            sum_temp_angle = func_limit_ab(sum_temp_angle, -45, 0);
                            break;
                        case 5:
                            sum_temp_angle += func_abs(yaw_bypass);
                            sum_temp_angle = func_limit_ab(sum_temp_angle, -45, 0);
                            break;
                        case 6:
                            sum_temp_angle = 0;
                            Image_Count_Flag = 0;
        //                    Fitting_Speed_Control(0, 0, Element_State, 0, Y_Meet, 57);
                            Bypass_State = 3;
                            break;
                        default:
                            break;
                    }
                }
                if(yaw_bypass > 0)
                    Target_Yaw = Record_Angle + sum_temp_angle;
                else
                    Target_Yaw = Record_Angle - sum_temp_angle;



//                if(yaw_bypass > 0 && Image_Count_Flag) // 正负号选择绕行方向
//                {
//                    if(Image_Count % 2 == 0 && Image_Count <= (uint16)(Bypass_Count1/2))
//                        sum_temp_angle += yaw_bypass;
//                    else if(Image_Count % 2 == 0 && Image_Count >= (uint16)(Bypass_Count1/2) && Image_Count <= (Bypass_Count1))
//                    {
//                        sum_temp_angle -= yaw_bypass;
//                        if(sum_temp_angle <= 0)
//                            sum_temp_angle = 0;
//                    }
//                    else if(Image_Count % 2 == 0 && Image_Count <= (uint16)(Bypass_Count1 + Bypass_Count2/2) && Image_Count <= (Bypass_Count1 + Bypass_Count2))
//                    {
//
//                    }
//                    sum_temp_angle = func_limit_ab(sum_temp_angle, -45, 45);
//                    Target_Yaw = Record_Angle + sum_temp_angle;
//                }
//                else if(yaw_bypass < 0 && Image_Count_Flag)
//                {
//                    if(Image_Count % 2 == 0 && Image_Count <= Bypass_Count)
//                        sum_temp_angle += yaw_bypass;
//                    else if(Image_Count % 2 == 0 && Image_Count >= Bypass_Count && Image_Count <= (Bypass_Count + Bypass_Count1))
//                    {
////                        Start_Flag = 0;
////                        if(Get_Start_Point(Image_Y - 3, Find_Line_Image, Adaptive_L_Start_Point, Adaptive_R_Start_Point, 1, 78) == 1 &&
////                           /*Adaptive_L_Start_Point[0] != 1 && Adaptive_R_Start_Point[0] != 78 &&*/
////                           Image_Count >= (int32)((Bypass_Count + Bypass_Count1) * 3 / 4))
////                        {
////                            Image_Count_Flag = 0;
////                            Bypass_State = 3;
////                        }
//                        sum_temp_angle -= yaw_bypass;
//                    }
//                    sum_temp_angle = func_limit_ab(sum_temp_angle, -45, 45);
//                    Target_Yaw = Record_Angle + sum_temp_angle;
//                }
//                if(Image_Count >= (Bypass_Count + Bypass_Count1)) // 超时退出
//                {
//                    Target_Yaw = Record_Angle;
//                    Image_Count_Flag = 0;
////                    Fitting_Speed_Control(0, 0, Element_State, 0, Y_Meet, 57);
//                    Bypass_State = 3;
//                }
                break;
            case 3: // 绕行即将完成，处理图像，正常则退出
                Process_Image();
                Get_Information();

                if(L_Statics >= 30 && R_Statics >= 30/* && Y_Meet <= 20*/)
                    Bypass_State = 4;


                break;
            case 4: // 饶行完成，继续寻迹
                Break_flag = 1;
                break;
            default:
                break;
        }

        IPS200_Show(Show_Flag);

        if(Break_flag == 1 || Element_Break_Flag == 1)
        {
            Element_Break_Flag = 0;
            Break_flag = 0;

            Image_Count = 0;
            Image_Count_Flag = 0;

            turn_mode = 2;
            break;
        }
    }

}

uint8 point_error = 0;
void Single_Get_Midline(void)
{
    uint8 i;
//    uint8 error_flag = 0;
    float Slope_Rate = 0;
    float Intercept = 0;

//    ips200_show_uint(0,23*8,point_error,2);
    point_error = 0;

    // 右侧新
    int retry_count = 0;

    do {
        if (R_Up_Turn_Left_Point_Flag_1) {
            Get_R_Right_Turn_Up_Point_1(1, R_Up_Turn_Left_Point_Position_1);
            if(R_Right_Turn_Up_Point_Flag_1)
            {
                Calculate_Slope_Intercept_Twodot(R_Up_Turn_Left_Point_1[0],R_Up_Turn_Left_Point_1[1],
                                                 R_Right_Turn_Up_Point_1[0],R_Right_Turn_Up_Point_1[1]
                                                 , &Slope_Rate, &Intercept);
                for(i = R_Right_Turn_Up_Point_1[1]; i <= R_Up_Turn_Left_Point_1[1]; i++)
                    R_Border[i] = (uint8)((i-Intercept)/Slope_Rate);
            }
            else
            {
                if(R_Up_Turn_Left_Point_1[1] <= 40)
                {
                    Calculate_Slope_Intercept(R_Up_Turn_Left_Point_1[1], R_Up_Turn_Left_Point_1[1] + 15, R_Border, &Slope_Rate, &Intercept);
                    for(i = 2; i < 57; i++)
                        R_Border[i] = (uint8)(Slope_Rate * (float)i + Intercept);
                }
                else
                {
                    R_Up_Turn_Left_Point_Flag_1 = 0;
                    Get_R_Up_Turn_Left_Point_1(1, R_Up_Turn_Left_Point_Position_1+4);
                    if(R_Up_Turn_Left_Point_Flag_1)
                    {
                        Calculate_Slope_Intercept(R_Up_Turn_Left_Point_1[1], R_Up_Turn_Left_Point_1[1] + 10, R_Border, &Slope_Rate, &Intercept);
                        for(i = 2; i < 57; i++)
                            R_Border[i] = (uint8)(Slope_Rate * (float)i + Intercept);
                    }
                }
            }
            break;
        } else {
            Get_R_Up_Turn_Left_Point_1(1, 50);
//            if (!R_Up_Turn_Left_Point_Flag_1) {
//                point_error = 1;
//            }
            retry_count++; // 增加重试计数
        }
    } while (retry_count < 3);

    Slope_Rate = 0;
    Intercept = 0;

    // zuo侧新
    do {
        if (L_Up_Turn_Right_Point_Flag_1) {
            Get_L_Left_Turn_Up_Point_1(1, L_Up_Turn_Right_Point_Position_1 + 5);

            if (L_Left_Turn_Up_Point_Flag_1) {
                Calculate_Slope_Intercept_Twodot(L_Left_Turn_Up_Point_1[0], L_Left_Turn_Up_Point_1[1],
                                                 L_Up_Turn_Right_Point_1[0], L_Up_Turn_Right_Point_1[1],
                                                &Slope_Rate, &Intercept);
                for (i = L_Left_Turn_Up_Point_1[1]; i <= L_Up_Turn_Right_Point_1[1]; i++)
                    L_Border[i] = (uint8)((i-Intercept)/Slope_Rate);
            } else {
                // 开始补线
                if (L_Up_Turn_Right_Point_1[1] <= 40) {
                    Calculate_Slope_Intercept(L_Up_Turn_Right_Point_1[1], L_Up_Turn_Right_Point_1[1] + 15,
                                            L_Border, &Slope_Rate, &Intercept);
                    for (i = 2; i < 57; i++)
                        L_Border[i] = (uint8)(Slope_Rate * (float)i + Intercept);
                } else {
                    L_Up_Turn_Right_Point_Flag_1 = 0;
                    Get_L_Up_Turn_Right_Point_1(1, L_Up_Turn_Right_Point_Position_1 + 4);

                    if (L_Up_Turn_Right_Point_Flag_1) {
                        Calculate_Slope_Intercept(L_Up_Turn_Right_Point_1[1], L_Up_Turn_Right_Point_1[1] + 10,
                                                L_Border, &Slope_Rate, &Intercept);
                        for (i = 2; i < 57; i++)
                            L_Border[i] = (uint8)(Slope_Rate * (float)i + Intercept);
                    }
                }
            }
            break;
        } else {
            Get_L_Up_Turn_Right_Point_1(1, 50); // 尝试重新获取左侧点

//            if (!L_Up_Turn_Right_Point_Flag_1) {
//                if (point_error == 1)
//                    point_error = 3;
//                else
//                    point_error = 2;
//
//            }
            retry_count++; // 增加重试计数
        }
    } while (retry_count < 3); // 最大重试3次
//    if(L_Up_Turn_Right_Point_Flag_1)
//    {
//        Get_L_Left_Turn_Up_Point_1(1, L_Up_Turn_Right_Point_Position_1+5);
//        if(L_Left_Turn_Up_Point_Flag_1)
//        {
//            Calculate_Slope_Intercept_Twodot(L_Up_Turn_Right_Point_1[0],L_Up_Turn_Right_Point_1[1],L_Left_Turn_Up_Point_1[0],L_Left_Turn_Up_Point_1[1]
//                    , &Slope_Rate, &Intercept);
//            for(i = L_Up_Turn_Right_Point_1[1]; i <= L_Left_Turn_Up_Point_1[1]; i++)
//                L_Border[i] = (uint8)(Slope_Rate * (float)i + Intercept);
//        }
//        else
//        {
//            // 开始补线
//            if(L_Up_Turn_Right_Point_Flag_1)
//            {
//                if(L_Up_Turn_Right_Point_1[1] <= 40)
//                {
//                    Calculate_Slope_Intercept(L_Up_Turn_Right_Point_1[1], L_Up_Turn_Right_Point_1[1] + 15, L_Border, &Slope_Rate, &Intercept);
//                    for(i = 2; i < 57; i++)
//                        L_Border[i] = (uint8)(Slope_Rate * (float)i + Intercept);
//                }
//                else
//                {
//                    L_Up_Turn_Right_Point_Flag_1 = 0;
//                    Get_L_Up_Turn_Right_Point_1(1, L_Up_Turn_Right_Point_Position_1+4);
//                    if(L_Up_Turn_Right_Point_Flag_1)
//                    {
//                        Calculate_Slope_Intercept(L_Up_Turn_Right_Point_1[1], L_Up_Turn_Right_Point_1[1] + 10, L_Border, &Slope_Rate, &Intercept);
//                        for(i = 2; i < 57; i++)
//                            L_Border[i] = (uint8)(Slope_Rate * (float)i + Intercept);
//                    }
//                }
//            }
//        }
//    }
//    else
//    {
//        if(point_error == 1)
//            point_error = 3;
//        else
//            point_error = 2;
//    }

    Get_Middle_Line(L_Border, R_Border, C_Line, Image_Y - 4, 0, 0);

}

uint8 flag_Single = 0;//判断单边桥
uint8 flag_Single_HighState = 0;
uint8 Single_state = 0;
uint8 Single_jg_line = 10;
uint16 Image_count_Single_End = 50;
void Single_Process(void)
{
    // 移植注释：Pe177 - 变量声明但从未引用，注释此行
    // uint8 i,j;
    uint8 Element = 0;
    Single_state = 1;
    uint8 temp = 0;
//    uint8 temp_flag = 0;
    // 移植注释：Pe177 - 变量声明但从未引用，注释此行
    // uint8 temp_dot = 0;

    Image_Count = 0;
    Image_Count_Flag = 0;

    flag_Single = 1;
    flag_Single_HighState = 0;

    Element = Single_State;

    Target_Yaw = imu660ra.eulerAngle.yaw;
    erect_Gyro_Pitch[0] = erect_Gyro_Pitch[0] * 2.0f / 3.0f;
    erect_Angle_Pitch[0] = erect_Angle_Pitch[0] * 2.0f / 3.0f;

    while(1)
    {
//        ips200_show_uint( 190 , 11*8 , flag_Single , 2 );
//        ips200_show_uint( 190 , 12*8 , Single_state , 2 );
//        ips200_show_uint( 190 , 13*8 , L_Straight_Flag , 2 );
//        ips200_show_uint( 200 , 13*8 , R_Straight_Flag , 3 );
        Process_Image();
        Get_Information();
        Element = Element_Judgement(Find_Line_Image, L_Statics, R_Statics, L_Grow_Dir, R_Grow_Dir, L_Line, R_Line, X_Meet, Y_Meet, L_Border, R_Border);
//        Lost_Line(L_Start_Point, R_Start_Point);

        switch(Single_state)
        {
            case 0: // 预判断阶段，以免过早判到

                if(Element == Ramp)
                {
                    Element_Break_Flag = 1;
                    Ramp_Process();
                }

                Fitting_Speed_Control(0, Single_speed_1, Element_State, 0, Y_Meet, 57);

                if(L_Up_Turn_Right_Point_Flag_1 && R_Up_Turn_Left_Point_Flag_1)
                {
                    temp = L_Up_Turn_Right_Point_1[1] > R_Up_Turn_Left_Point_1[1] ? L_Up_Turn_Right_Point_1[1] : R_Up_Turn_Left_Point_1[1];
                    if(temp >= Single_jg_line)
                    {
                        Single_state = 1;
                        flag_Single = 1;
                    }
                }

                break;
            case 1: // 第一阶段，判到单边桥减速等升到高位状态
                Fitting_Speed_Control(0, Single_speed_1, Element_State, 0, Y_Meet, 57);
                if(Single_Control_Flag == 1)
                {
                    Single_Get_Midline();
                    Get_Deviation_Value(Single_State, 0, Y_Meet, Image_Y - 3, C_Line, 2.0);
                }
                else if(Single_Control_Flag == 2)
                {
                    turn_mode = 3;
                    Deviation_Value = 0;
                }

                if(flag_Single_HighState == 1)
                    Single_state = 2;
                break;
            case 2: // 第二阶段，高位状态完成，给速度寻迹通过单边桥
                Fitting_Speed_Control(0, Single_speed_2, Element_State, 0, Y_Meet, 57);
                if(Single_Control_Flag == 1)
                {
                    Image_Count_Flag = 1;

                    if(Image_Count >= Image_count_Single_End)
                    {
                        if(R_Start_Point[0] - L_Start_Point[0] >= 40)
//                            temp_dot++;
//                        if(temp_dot >= 10)
                            Single_state = 4;
                    }

                    Single_Get_Midline();
                    Get_Deviation_Value(Single_State, 0, Y_Meet, Image_Y - 3, C_Line, 2.0);

                    if(R_Right_Turn_Up_Point_Flag_1 == 1 || L_Left_Turn_Up_Point_Flag_1 == 1)
                        Image_Count = 0;
//                    if(point_error == 3)
//                        temp_dot++;
//                    else
//                        temp_dot = 0;


                    // 纯定时退出
//                    if(/*temp_dot >= 30 && */Image_Count >= 300)
//                    {
//                        if(point_error == 3)
//                            Single_state = 4;
//                    }

                }
                else if(Single_Control_Flag == 2)
                {
                    turn_mode = 3;
                    Deviation_Value = 0;
                }
//                // 图像进入预退出状态
//                if( func_abs(L_Start_Point[0] - 40.0f) <= 5 && R_Straight_Flag )
//                {
//                    j = 0;
//                    for( i = L_Start_Point[1]; i >= 5; i-- )
//                    {
//                        if(L_Start_Point[0] - L_Border[i] >= 5)
//                            j++;
//                        if(j >= 10)
//                        {
//                            temp_flag = 1;
//                            break;
//                        }
//                    }
//                }
//                if( func_abs(R_Start_Point[0] - 40.0f) <= 5 && L_Straight_Flag )
//                {
//                    j = 0;
//                    for( i = R_Start_Point[1]; i >= 5; i-- )
//                    {
//                        if(R_Border[0] - R_Start_Point[i] >= 5)
//                            j++;
//                        if(j >= 10)
//                        {
//                            temp_flag = 1;
//                            break;
//                        }
//                    }
//                }
//                if(temp_flag && L_Straight_Flag && R_Straight_Flag)
//                    Single_state = 3;

                // 低速使用
//                if(
//                        (/*L_Up_Turn_Left_Point_Flag_1 + */L_Up_Turn_Right_Point_Flag_1/* + L_Right_Turn_Up_Point_Flag_1 + L_Left_Turn_Up_Point_Flag_1 */+
//                         R_Up_Turn_Left_Point_Flag_1/* + R_Up_Turn_Right_Point_Flag_1 + R_Right_Turn_Up_Point_Flag_1 + R_Left_Turn_Up_Point_Flag_1*/) == 0
////                        (R_Up_Turn_Left_Point_Flag_1 == 0) &&
////                        (L_Up_Turn_Right_Point_Flag_1 == 0)
//                        )
//                    temp_dot++;
//                else
//                    temp_dot = 0;
//                if(temp_dot >= 30)
//                    Single_state = 4;
                break;
            case 3:
                if(func_abs(stab_roll) <= 0.5)
                    Single_state = 4;

                break;
            case 4: // 第三阶段，通过单边桥，降低姿态继续寻迹
                Fitting_Speed_Control(0, Single_speed_1, Element_State, 0, Y_Meet, 57);

                Image_Count_Flag = 0;
                flag_Single_HighState = 2;
                Single_state = 5;
                break;
            case 5: // 第四阶段，降低姿态完成，退出单边桥寻迹状态
                if(flag_Single_HighState == 3)
                    flag_Single = 0;
                break;
        }

//        TFT180_Show(Show_Flag);
        IPS200_Show(Show_Flag);

        if(Single_state != 0 && flag_Single == 0 )
        {
            erect_Gyro_Pitch[0] = erect_Gyro_Pitch[0] * 3.0f / 2.0f;
            erect_Angle_Pitch[0] = erect_Angle_Pitch[0] * 3.0f / 2.0f;
            turn_mode = 2;
            break;
        }
        if(Element_Break_Flag == 1)
        {
            erect_Gyro_Pitch[0] = erect_Gyro_Pitch[0] * 3.0f / 2.0f;
            erect_Angle_Pitch[0] = erect_Angle_Pitch[0] * 3.0f / 2.0f;
            flag_Single = 0;
            flag_Single_HighState = 0;

            turn_mode = 2;

            Element_Break_Flag = 0;
            break;
        }
//        if(((L_Up_Turn_Left_Point_Flag_1 + R_Up_Turn_Right_Point_Flag_1 + L_Right_Turn_Up_Point_Flag_1 + R_Left_Turn_Up_Point_Flag_1) < 2)
//                && Single_state != 2 && Single_state != 3)
//        {
//            flag_Single = 0;
//            break;
//        }
    }
}

void Tracking(void)
{
    uint8 Element = 0;
    Process_Image();
    Get_Information();
    Element = Element_Judgement(Find_Line_Image, L_Statics, R_Statics, L_Grow_Dir, R_Grow_Dir, L_Line, R_Line, X_Meet, Y_Meet, L_Border, R_Border);

    switch(Element)
    {
        case 0:
        {
            Element_State = 0;
            Convention_Process();
            break;
        }
        case Derailment:
        {
            Element_State = Derailment;
            Derailment_Process();
            break;
        }
        case Straightaway:
        {
            Element_State = Straightaway;
            Straightaway_Process();
            break;
        }
        case L_Turn:
        {
            Element_State = L_Turn;
            L_Turn_Process();
            break;
        }
        case R_Turn:
        {
            Element_State = R_Turn;
            R_Turn_Process();
            break;
        }
        case L_Circle:
        {
            Element_State = L_Circle;
            L_Circle_Process(L_Circle_Flag);
            break;
        }
        case R_Circle:
        {
            Element_State = R_Circle;
            R_Circle_Process(R_Circle_Flag);
            break;
        }
        case Cross:
        {
            Element_State = Cross;
            Cross_Process_old();
            break;
        }
        case L_Oblique_Cross:
        {
            Element_State = L_Oblique_Cross;
            Cross_Process_old();
            break;
        }
        case R_Oblique_Cross:
        {
            Element_State = R_Oblique_Cross;
            Cross_Process_old();
            break;
        }
        case Zebra:
        {
            Element_State = Zebra;
            Zebra_Process();
            break;
        }
        case Ramp:
        {
            Element_State = Ramp;
            Ramp_Process();
            break;
        }
//        case Three_Bif:
//        {
//            Element_State = Three_Bif;
//            Three_Bif_Process();
//            break;
//        }
//        case Barrier:
//        {
//            Element_State = Barrier;
//            Barrier_Process();
//            break;
//        }
//        case Disconnection:
//        {
//            Element_State = Disconnection;
//            Disconnection_Process();
//            break;
//        }
//        case T_Way:
//        {
//            Element_State = T_Way;
//            T_Way_Process();
//            break;
//        }
//        case L_Garage:
//        {
//            Element_State = L_Garage;
//            L_Garage_Process();
//            break;
//        }
//        case R_Garage:
//        {
//            Element_State = R_Garage;
//            R_Garage_Process();
//            break;
//        }
//        case Small_S:
//        {
//            Element_State = Small_S;
//            Small_S_Process();
//            break;
//        }
        case Jump_State:
        {
            Element_State = Jump_State;
            if(Jump_Control_Flag == 1)
                Jump_Process();
            else if(Jump_Control_Flag == 2)
                Bypass_Process();
            break;
        }
        case Single_State:
        {
            Element_State = Single_State;
            Single_Process();
            break;
        }
    }
//    TFT180_Show(Show_Flag);
    IPS200_Show(Show_Flag);
}
//***************************************************************************************************************************************************************************




