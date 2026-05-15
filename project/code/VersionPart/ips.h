#ifndef CODE_IPS200_SHOW_H_
#define CODE_IPS200_SHOW_H_

#include "image.h"

#define Show_Camera_Correct_Line    0
#define Show_Find_Line_Image 1
#define Show_L_Line          2
#define Shop_R_Line          3
#define Show_M_Line          5
#define Show_All_Line        6
#define Show_L_Start_Point   7
#define Show_R_Start_Point   8
#define Show_All_Start_Point 9
#define Show_L_Statics       10
#define Show_R_Statics       11
#define Show_All_Statics     12
#define Show_X_Meet          13
#define Show_Y_Meet          14
#define Show_All_Meet        15
#define Show_L_Border        16
#define Show_R_Border        17
#define Show_All_Border      18

//直角拐点
#define     Show_L_Up_Turn_Left_Point_1            19       //左侧上转左
#define     Show_L_Right_Turn_Up_Point_1           20       //左侧右转上
#define     Show_R_Up_Turn_Right_Point_1           21       //右侧上转右
#define     Show_R_Left_Turn_Up_Point_1            22       //右侧左转上
#define     Show_All_Stight_Turn_Point           23
//圆弧形拐点
#define     Show_L_Arc_Turn_Point                24
#define     Show_R_Arc_Turn_Point                25
#define     Show_All_Arc_Turn_Point              26
//斜率
#define     Show_R_Straightaway_Lope_Rate_A      27
#define     Show_R_Straightaway_Lope_Rate_B      28
#define     Show_R_Straightaway_Lope_Rate_C      29
#define     Show_L_Straightaway_Lope_Rate_A      30
#define     Show_L_Straightaway_Lope_Rate_B      31
#define     Show_L_Straightaway_Lope_Rate_C      32
#define     Show_All_Straightaway_Lope_Rate      33
//截距
#define     Show_L_Intercept                     34
#define     Show_R_Intercept                     35
#define     Show_All_Intercept                   36
//拟合曲线
#define     Show_L_Fitting_Line                  37
#define     Show_R_Fitting_Line                  38
#define     Show_All_Fitting_Line                39
//方差
#define     Show_L_Variance                      40
#define     Show_R_Variance                      41
#define     Show_All_Variance                    42
//偏差
#define     Show_Deviation_Value                 43
//灵敏度
#define     Show_Sensitivity                     44
//元素
#define     Show_Element_State                   45
//斑马线标志位
#define     Show_Zebra_Flag                      46
//停车标志位
#define     Show_Stop_Flag                       47

//显示逆透视后图像
#define Show_Inverse_Perspective_Image              48
//显示反逆透视后图像
#define Show_Back_Inverse_Perspective_Image         49
#define Show_I_L_Line                          50
#define Show_I_R_Line                          51
#define Show_I_M_Line                          52
#define Show_I_All_Line                        53

#define Show_Vague_Image                       54

#define Show_Speed_Proportion                  55

#define Show_Adaptive_Thres_Average            56

#define Show_Barrier_Distance                  57

extern uint8 Show_Flag;    //2显示图像

void IPS200_Show_Init(void);

void Draw_X_In_Point_IPS200(uint8(*image)[Image_X], uint8 point_y, uint8 point_x, uint16 colour);

void Draw_10_In_Point_IPS200(uint8(*image)[Image_X], uint8 point_y, uint8 point_x, uint16 colour);

void IPS_Show(uint8 flag);

void IPS200_Show(uint8 Show_Flag);

void DisplayBorderDistances(void);

#endif /* CODE_IPS200_SHOW_H_ */
