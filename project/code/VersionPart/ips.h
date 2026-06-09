#ifndef CODE_IPS200_SHOW_H_
#define CODE_IPS200_SHOW_H_

#include "image.h"

/* 基础图像显示项 */
#define Show_Camera_Correct_Line            0   // 显示图像中心修正线
#define Show_Find_Line_Image                1   // 显示基础寻线二值图
#define Show_L_Line                         2   // 显示左侧线点集
#define Shop_R_Line                         3   // 显示右侧线点集
#define Show_M_Line                         5   // 显示中线点集
#define Show_All_Line                       6   // 同时显示左右线与中线
#define Show_L_Start_Point                  7   // 显示左起始点
#define Show_R_Start_Point                  8   // 显示右起始点
#define Show_All_Start_Point                9   // 同时显示左右起始点
#define Show_L_Statics                      10  // 显示左线统计数量
#define Show_R_Statics                      11  // 显示右线统计数量
#define Show_All_Statics                    12  // 同时显示左右线统计数量
#define Show_X_Meet                         13  // 显示 X 方向交点信息
#define Show_Y_Meet                         14  // 显示 Y 方向交点信息
#define Show_All_Meet                       15  // 同时显示 X/Y 交点信息
#define Show_L_Border                       16  // 显示左边界
#define Show_R_Border                       17  // 显示右边界
#define Show_All_Border                     18  // 同时显示左右边界与中线

/* 直角拐点显示项 */
#define Show_L_Up_Turn_Left_Point_1         19  // 显示左侧上转左拐点
#define Show_L_Right_Turn_Up_Point_1        20  // 显示左侧右转上拐点
#define Show_R_Up_Turn_Right_Point_1        21  // 显示右侧上转右拐点
#define Show_R_Left_Turn_Up_Point_1         22  // 显示右侧左转上拐点
#define Show_All_Stight_Turn_Point          23  // 显示全部直角与单边桥相关拐点

/* 圆弧拐点显示项 */
#define Show_L_Arc_Turn_Point               24  // 显示左侧圆弧拐点
#define Show_R_Arc_Turn_Point               25  // 显示右侧圆弧拐点
#define Show_All_Arc_Turn_Point             26  // 同时显示左右圆弧拐点

/* 斜率显示项 */
#define Show_R_Straightaway_Lope_Rate_A     27  // 显示右直道斜率 A 段
#define Show_R_Straightaway_Lope_Rate_B     28  // 显示右直道斜率 B 段
#define Show_R_Straightaway_Lope_Rate_C     29  // 显示右直道斜率 C 段
#define Show_L_Straightaway_Lope_Rate_A     30  // 显示左直道斜率 A 段
#define Show_L_Straightaway_Lope_Rate_B     31  // 显示左直道斜率 B 段
#define Show_L_Straightaway_Lope_Rate_C     32  // 显示左直道斜率 C 段
#define Show_All_Straightaway_Lope_Rate     33  // 同时显示左右直道斜率

/* 截距显示项 */
#define Show_L_Intercept                    34  // 显示左边界拟合截距
#define Show_R_Intercept                    35  // 显示右边界拟合截距
#define Show_All_Intercept                  36  // 同时显示左右拟合截距

/* 拟合线显示项 */
#define Show_L_Fitting_Line                 37  // 显示左拟合线
#define Show_R_Fitting_Line                 38  // 显示右拟合线
#define Show_All_Fitting_Line               39  // 同时显示左右拟合线

/* 方差显示项 */
#define Show_L_Variance                     40  // 显示左边界方差
#define Show_R_Variance                     41  // 显示右边界方差
#define Show_All_Variance                   42  // 同时显示左右边界方差

/* 状态量显示项 */
#define Show_Deviation_Value                43  // 显示当前偏差值
#define Show_Sensitivity                    44  // 显示图像灵敏度参数
#define Show_Element_State                  45  // 显示当前元素状态机状态
#define Show_Zebra_Flag                     46  // 显示斑马线标志位
#define Show_Stop_Flag                      47  // 显示停车标志位

/* 逆透视及附加图像显示项 */
#define Show_Inverse_Perspective_Image      48  // 显示逆透视图像
#define Show_Back_Inverse_Perspective_Image 49  // 显示反逆透视图像
#define Show_I_L_Line                       50  // 显示逆透视左线
#define Show_I_R_Line                       51  // 显示逆透视右线
#define Show_I_M_Line                       52  // 显示逆透视中线
#define Show_I_All_Line                     53  // 同时显示逆透视左右线与中线
#define Show_Vague_Image                    54  // 显示模糊处理图像
#define Show_Speed_Proportion               55  // 显示速度比例系数
#define Show_Adaptive_Thres_Average         56  // 显示自适应阈值平均值
#define Show_Barrier_Distance               57  // 显示障碍物距离信息

extern uint8 Show_Flag;    // 显示模式：2 时显示图像调试内容

/**
 * @brief  初始化 IPS200 屏幕显示参数
 * @return 无
 */
void IPS200_Show_Init(void);

/**
 * @brief  在指定点位绘制 X 形标记
 * @param  image    参考图像缓冲区
 * @param  point_y  标记点纵坐标
 * @param  point_x  标记点横坐标
 * @param  colour   标记颜色
 * @return 无
 */
void Draw_X_In_Point_IPS200(uint8(*image)[Image_X], uint8 point_y, uint8 point_x, uint16 colour);

/**
 * @brief  在指定点位绘制十字形标记
 * @param  image    参考图像缓冲区
 * @param  point_y  标记点纵坐标
 * @param  point_x  标记点横坐标
 * @param  colour   标记颜色
 * @return 无
 */
void Draw_10_In_Point_IPS200(uint8(*image)[Image_X], uint8 point_y, uint8 point_x, uint16 colour);

/**
 * @brief  根据显示标志绘制单项图像调试信息
 * @param  flag  显示项目编号
 * @return 无
 */
void IPS_Show(uint8 flag);

/**
 * @brief  刷新 IPS200 综合调试界面
 * @param  Show_Flag  当前显示模式
 * @return 无
 */
void IPS200_Show(uint8 Show_Flag);

/**
 * @brief  以表格形式显示左右边界与中线的距离差
 * @return 无
 */
void DisplayBorderDistances(void);

#endif /* CODE_IPS200_SHOW_H_ */
