#include "zf_common_headfile.h"
#include "ips.h"
#include "image.h"
#include "menu.h"
#include "imu660.h"
#include "Interrupt.h"
#include "small_driver_uart_control.h"

uint8 Other_Show_Flag = 2;

#define User_Red     0xF800
#define User_Green   0x07E0
#define User_Blue    0x001F

extern uint8 Ramp_Judge_Image_Flag;
extern uint8 L_Ramp_Flag;
extern uint8 R_Ramp_Flag;

extern float Ramp_Up_Slope_L;
extern float Ramp_Down_Slope_L;
extern float Ramp_Up_Slope_R;
extern float Ramp_Down_Slope_R;
extern uint8 L_Right_Turn_Up_Point_Flag_1_ramp;
extern uint8 R_Left_Turn_Up_Point_Flag_1_ramp;
extern uint8 L_Right_Turn_Up_Point_1_ramp[2];
extern uint8 R_Left_Turn_Up_Point_1_ramp[2];

extern uint8 Adaptive_Thres_Average;
extern uint8 Vague_Image[Image_Y][Image_X];
extern float Speed_Proportion;


extern uint8 Find_Line_Image[Image_Y][Image_X];
extern uint8 R_Border[Image_Y];
extern uint8 L_Border[Image_Y];

extern uint8 L_Line[(uint16)Use_Num][2];
extern uint8 R_Line[(uint16)Use_Num][2];
extern uint8 C_Line[(uint16)Use_Num];

extern uint16 L_Statics;
extern uint16 R_Statics;

extern uint8 L_Start_Point[2];
extern uint8 R_Start_Point[2];

extern uint8 Y_Meet;
extern uint8 X_Meet;

extern uint8 L_Up_Turn_Left_Point_1[2];
extern uint8 L_Right_Turn_Up_Point_1[2];
extern uint8 R_Up_Turn_Right_Point_1[2];
extern uint8 R_Left_Turn_Up_Point_1[2];
extern uint8 L_Up_Turn_Left_Point_Flag_1;
extern uint8 L_Right_Turn_Up_Point_Flag_1;
extern uint8 R_Up_Turn_Right_Point_Flag_1;
extern uint8 R_Left_Turn_Up_Point_Flag_1;

extern uint8 R_Up_Turn_Left_Point_1[2];
extern uint8 R_Right_Turn_Up_Point_1[2];
extern uint8 L_Up_Turn_Right_Point_1[2];
extern uint8 L_Left_Turn_Up_Point_1[2];
extern uint8 R_Up_Turn_Left_Point_1_2[2];
extern uint8 R_Right_Turn_Up_Point_1_2[2];
extern uint8 L_Up_Turn_Right_Point_1_2[2];
extern uint8 L_Left_Turn_Up_Point_1_2[2];
extern uint8 R_Up_Turn_Left_Point_Flag_1;
extern uint8 R_Right_Turn_Up_Point_Flag_1;
extern uint8 L_Up_Turn_Right_Point_Flag_1;
extern uint8 L_Left_Turn_Up_Point_Flag_1;
extern float L_Up_Turn_Right_Point_Angle_1;
extern float R_Up_Turn_Left_Point_Angle_1;

extern uint8 L_Arc_Turn_Point[3][2];
extern uint8 R_Arc_Turn_Point[3][2];
extern uint8 L_Arc_Turn_Point_Flag;
extern uint8 L_Arc_Turn_Point_Num;
extern uint8 R_Arc_Turn_Point_Flag;
extern uint8 R_Arc_Turn_Point_Num;

extern float R_Straightaway_Lope_Rate_A;
extern float R_Straightaway_Lope_Rate_B;
extern float R_Straightaway_Lope_Rate_C;
extern float R_Intercept;
extern float L_Straightaway_Lope_Rate_A;
extern float L_Straightaway_Lope_Rate_B;
extern float L_Straightaway_Lope_Rate_C;
extern float L_Intercept;

extern int16 L_Fitting_Line[];
extern int16 R_Fitting_Line[];

extern float L_Variance;
extern float R_Variance;

extern uint8 X_Border_Min;
extern uint8 X_Border_Max;
extern uint8 Y_Border_Min;
extern uint8 Y_Border_Max;

extern float Deviation_Value;

extern uint8 Element_State;

extern float Sensitivity;

extern uint8 Stop_Flag;

extern uint8 Zebra_Flag;
extern uint8 Ramp_Flag;
extern uint8 Small_S_Flag;

extern uint8 L_Border_Point_Num;
extern uint8 L_UP_Border_Point_Num;    //记录位于顶部黑框上的边界点个数
extern uint8 R_Border_Point_Num;
extern uint8 R_UP_Border_Point_Num;
extern uint8 Relative_Border_Point_Num;        //存储对位边界行的个数
extern uint8 Max_Row_Dif_Line_Num;

//extern uint8 I_L_Line[(uint16)Use_Num][2];
//extern uint8 I_R_Line[(uint16)Use_Num][2];
//extern uint8 I_C_Line[(uint16)Use_Num];

//extern uint16 I_L_Statics;
//extern uint16 I_R_Statics;

/**
 * @brief  初始化 IPS200 屏幕的颜色、字体、方向与接口模式
 * @return 无
 */
void IPS200_Show_Init(void)
{
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_set_font(IPS200_6X8_FONT);
    ips200_set_dir(IPS200_PORTAIT);
    ips200_init(IPS200_TYPE_SPI);
}

/**
 * @brief  在指定坐标附近绘制 X 形标记
 * @param  image    参考图像缓冲区，仅用于保持接口一致
 * @param  point_y  标记中心纵坐标
 * @param  point_x  标记中心横坐标
 * @param  colour   绘制颜色
 * @return 无
 */
void Draw_X_In_Point_IPS200(uint8(*image)[Image_X], uint8 point_y, uint8 point_x, uint16 colour)
{
    if(((point_y - 2) <= Y_Border_Max) && ((point_y - 2) >= Y_Border_Min) && ((point_x - 2) <= X_Border_Max) && ((point_x - 2) >= X_Border_Min))
    {
        ips200_draw_point(point_x - 2, point_y - 2, colour);
    }
    if(((point_y - 1) <= Y_Border_Max) && ((point_y - 1) >= Y_Border_Min) && ((point_x - 1) <= X_Border_Max) && ((point_x - 1) >= X_Border_Min))
    {
        ips200_draw_point(point_x - 1, point_y - 1, colour);
    }
    if(((point_y + 1) <= Y_Border_Max) && ((point_y + 1) >= Y_Border_Min) && ((point_x - 1) <= X_Border_Max) && ((point_x - 1) >= X_Border_Min))
    {
        ips200_draw_point(point_x - 1, point_y + 1, colour);
    }
    if(((point_y + 2) <= Y_Border_Max) && ((point_y + 2) >= Y_Border_Min) && ((point_x - 2) <= X_Border_Max) && ((point_x - 2) >= X_Border_Min))
    {
        ips200_draw_point(point_x - 2, point_y + 2, colour);
    }
    if(((point_y - 2) <= Y_Border_Max) && ((point_y - 2) >= Y_Border_Min) && ((point_x + 2) <= X_Border_Max) && ((point_x + 2) >= X_Border_Min))
    {
        ips200_draw_point(point_x + 2, point_y - 2, colour);
    }
    if(((point_y - 1) <= Y_Border_Max) && ((point_y - 1) >= Y_Border_Min) && ((point_x + 1) <= X_Border_Max) && ((point_x + 1) >= X_Border_Min))
    {
        ips200_draw_point(point_x + 1, point_y - 1, colour);
    }
    if(((point_y + 2) <= Y_Border_Max) && ((point_y + 2) >= Y_Border_Min) && ((point_x + 2) <= X_Border_Max) && ((point_x + 2) >= X_Border_Min))
    {
        ips200_draw_point(point_x + 2, point_y + 2, colour);
    }
    if(((point_y + 1) <= Y_Border_Max) && ((point_y + 1) >= Y_Border_Min) && ((point_x + 1) <= X_Border_Max) && ((point_x + 1) >= X_Border_Min))
    {
        ips200_draw_point(point_x + 1, point_y + 1, colour);
    }
}

/**
 * @brief  在指定坐标附近绘制十字形标记
 * @param  image    参考图像缓冲区，仅用于保持接口一致
 * @param  point_y  标记中心纵坐标
 * @param  point_x  标记中心横坐标
 * @param  colour   绘制颜色
 * @return 无
 */
void Draw_10_In_Point_IPS200(uint8(*image)[Image_X], uint8 point_y, uint8 point_x, uint16 colour)
{
    if(((point_y - 2) <= Y_Border_Max) && ((point_y - 2) >= Y_Border_Min) && ((point_x    ) <= X_Border_Max) && ((point_x    ) >= X_Border_Min))
    {
        ips200_draw_point(point_x    , point_y - 2, colour);
    }
    if(((point_y - 1) <= Y_Border_Max) && ((point_y - 1) >= Y_Border_Min) && ((point_x    ) <= X_Border_Max) && ((point_x    ) >= X_Border_Min))
    {
        ips200_draw_point(point_x    , point_y - 1, colour);
    }
    if(((point_y + 2) <= Y_Border_Max) && ((point_y + 2) >= Y_Border_Min) && ((point_x    ) <= X_Border_Max) && ((point_x    ) >= X_Border_Min))
    {
        ips200_draw_point(point_x    , point_y + 2, colour);
    }
    if(((point_y + 1) <= Y_Border_Max) && ((point_y + 1) >= Y_Border_Min) && ((point_x    ) <= X_Border_Max) && ((point_x    ) >= X_Border_Min))
    {
        ips200_draw_point(point_x    , point_y + 1, colour);
    }
    if(((point_y    ) <= Y_Border_Max) && ((point_y    ) >= Y_Border_Min) && ((point_x - 1) <= X_Border_Max) && ((point_x - 1) >= X_Border_Min))
    {
        ips200_draw_point(point_x - 1, point_y    , colour);
    }
    if(((point_y    ) <= Y_Border_Max) && ((point_y    ) >= Y_Border_Min) && ((point_x - 2) <= X_Border_Max) && ((point_x - 2) >= X_Border_Min))
    {
        ips200_draw_point(point_x - 2, point_y    , colour);
    }
    if(((point_y    ) <= Y_Border_Max) && ((point_y    ) >= Y_Border_Min) && ((point_x + 1) <= X_Border_Max) && ((point_x + 1) >= X_Border_Min))
    {
        ips200_draw_point(point_x + 1, point_y    , colour);
    }
    if(((point_y    ) <= Y_Border_Max) && ((point_y    ) >= Y_Border_Min) && ((point_x + 2) <= X_Border_Max) && ((point_x + 2) >= X_Border_Min))
    {
        ips200_draw_point(point_x + 2, point_y    , colour);
    }
}

/**
 * @brief  根据显示标志绘制指定调试图层
 * @param  flag  图层编号，取值见 ips.h 中的 Show_* 宏
 * @return 无
 */
void IPS_Show(uint8 flag)
{
    uint8 i = 0;
    switch (flag)
    {
        case Show_Camera_Correct_Line:
        {
            for(i = 0; i <  59; i ++)
            {
                ips200_draw_point(40, i, User_Green);
            }
            break;
        }
        case Show_Find_Line_Image:
        {
            ips200_show_gray_image(0,0,(const uint8*)Find_Line_Image,Image_X,Image_Y,Image_X,Image_Y,0);
            break;
        }
        case Show_Inverse_Perspective_Image:
        {
//            ips200_show_gray_image(0,0,(const uint8*)I_Perspective_Image,Image_X,Image_Y,Image_X,Image_Y,0);
            break;
        }
        case Show_Back_Inverse_Perspective_Image:
        {
//            ips200_show_gray_image(0,0,(const uint8*)Back_I_Perspective_Image,Image_X,Image_Y,Image_X,Image_Y,0);
            break;
        }
        case Show_Vague_Image:
        {
//            ips200_show_gray_image(0,0,(const uint8*)Vague_Image,Image_X,Image_Y,Image_X,Image_Y,0);
            break;
        }
        case Show_L_Line:
        {
            for (i = 0; i < L_Statics; i++)
            {
                // 移植注释：Pe186 - uint8类型 >= 0 恒为真，原写法保留如下
                // if(L_Line[i][0] <= 80 && L_Line[i][0] >= 0 && L_Line[i][1] >= 0 && L_Line[i][1] <= 60)
                if(L_Line[i][0] <= 80 && L_Line[i][1] <= 60)
                {
                    ips200_draw_point(L_Line[i][0], L_Line[i][1], User_Green);
                }
            }
            break;
        }
        case Shop_R_Line:
        {
            for (i = 0; i < R_Statics; i++)
            {
                // 移植注释：Pe186 - uint8类型 >= 0 恒为真，原写法保留如下
                // if(R_Line[i][0] <= 80 && R_Line[i][0] >= 0 && R_Line[i][1] >= 0 && R_Line[i][1] <= 60)
                if(R_Line[i][0] <= 80 && R_Line[i][1] <= 60)
                {
                    ips200_draw_point(R_Line[i][0], R_Line[i][1], User_Red);
                }
            }
            break;
        }
        case Show_M_Line:
        {
            for (i = 2; i < Image_Y - 2; i++)
            {
                // 移植注释：Pe186 - uint8类型 >= 0 恒为真，原写法保留如下
                // if(C_Line[i] <= 80 && C_Line[i] >= 0)
                if(C_Line[i] <= 80)
                {
                    ips200_draw_point(C_Line[i], i, User_Blue);
                }
            }
            break;
        }
        case Show_All_Line:
        {
            for (i = 0; i < L_Statics; i++)
            {
                // 移植注释：Pe186 - uint8类型 >= 0 恒为真，原写法保留如下
                // if(L_Line[i][0] <= 80 && L_Line[i][0] >= 0 && L_Line[i][1] >= 0 && L_Line[i][1] <= 60)
                if(L_Line[i][0] <= 80 && L_Line[i][1] <= 60)
                {
                    ips200_draw_point(L_Line[i][0], L_Line[i][1], User_Green);
                }
            }
            for (i = 0; i < R_Statics; i++)
            {
                // 移植注释：Pe186 - uint8类型 >= 0 恒为真，原写法保留如下
                // if(R_Line[i][0] <= 80 && R_Line[i][0] >= 0 && R_Line[i][1] >= 0 && R_Line[i][1] <= 60)
                if(R_Line[i][0] <= 80 && R_Line[i][1] <= 60)
                {
                    ips200_draw_point(R_Line[i][0], R_Line[i][1], User_Red);
                }
            }
            for (i = 2; i < Image_Y - 2; i++)
            {
                // 移植注释：Pe186 - uint8类型 >= 0 恒为真，原写法保留如下
                // if(C_Line[i] <= 80 && C_Line[i] >= 0)
                if(C_Line[i] <= 80)
                {
                    ips200_draw_point(C_Line[i], i, User_Blue);
                }
            }
            break;
        }
        case Show_I_L_Line:
        {
//            for (i = 0; i < I_L_Statics; i++)
//            {
//                if(I_L_Line[i][0] <= 80 && I_L_Line[i][0] >= 0 && I_L_Line[i][1] >= 0 && I_L_Line[i][1] <= 60)
//                {
//                    ips200_draw_point(I_L_Line[i][0], I_L_Line[i][1], User_Green);
//                }
//            }
            break;
        }
        case Show_I_R_Line:
        {
//            for (i = 0; i < I_R_Statics; i++)
//            {
//                if(I_R_Line[i][0] <= 80 && I_R_Line[i][0] >= 0 && I_R_Line[i][1] >= 0 && I_R_Line[i][1] <= 60)
//                {
//                    ips200_draw_point(I_R_Line[i][0], I_R_Line[i][1], User_Red);
//                }
//            }
            break;
        }
        case Show_I_M_Line:
        {
//            for (i = 2; i < Image_Y - 2; i++)
//            {
//                if(I_C_Line[i] <= 80 && I_C_Line[i] >= 0)
//                {
//                    ips200_draw_point(I_C_Line[i], i, User_Blue);
//                }
//            }
            break;
        }
        case Show_I_All_Line:
        {
//            for (i = 0; i < I_L_Statics; i++)
//            {
//                if(I_L_Line[i][0] <= 80 && I_L_Line[i][0] >= 0 && I_L_Line[i][1] >= 0 && I_L_Line[i][1] <= 60)
//                {
//                    ips200_draw_point(I_L_Line[i][0], I_L_Line[i][1], User_Green);
//                }
//            }
//            for (i = 0; i < I_R_Statics; i++)
//            {
//                if(I_R_Line[i][0] <= 80 && I_R_Line[i][0] >= 0 && I_R_Line[i][1] >= 0 && I_R_Line[i][1] <= 60)
//                {
//                    ips200_draw_point(I_R_Line[i][0], I_R_Line[i][1], User_Red);
//                }
//            }
//            for (i = 2; i < Image_Y - 2; i++)
//            {
////                if(I_C_Line[i] <= 80 && I_C_Line[i] >= 0)
////                {
////                    ips200_draw_point(I_C_Line[i], i, User_Blue);
////                }
//            }
            break;
        }
        case Show_Y_Meet:
        {
            ips200_show_string(0, 61, "Y_Meet:");
            ips200_show_int(43,61,Y_Meet,3);
            break;
        }
        case Show_X_Meet:
        {
            ips200_show_string(0, 70, "X_Meet:");
            ips200_show_int(43,70,X_Meet,3);
            break;
        }
        case Show_All_Meet:
        {
            ips200_show_string(0, 61, "Y_Meet:");
            ips200_show_int(43,61,Y_Meet,3);
            ips200_show_string(0, 70, "X_Meet:");
            ips200_show_int(43,70,X_Meet,3);
            break;
        }
        case Show_L_Start_Point:
        {
            ips200_show_string(0, 79, "L_ST_X:");
            ips200_show_int(43,79,L_Start_Point[0],3);
            break;
        }
        case Show_R_Start_Point:
        {
            ips200_show_string(0, 88, "R_ST_X:");
            ips200_show_int(43,88,R_Start_Point[0],3);
            break;
        }
        case Show_All_Start_Point:
        {
            ips200_show_string(0, 79, "L_ST_X:");
            ips200_show_int(43,79,L_Start_Point[0],3);
            ips200_show_string(0, 88, "R_ST_X:");
            ips200_show_int(43,88,R_Start_Point[0],3);
            break;
        }
        case Show_L_Statics:
        {
            ips200_show_string(0, 97, "L_NUM:");
            ips200_show_int(43,97,L_Statics,3);
            break;
        }
        case Show_R_Statics:
        {
            ips200_show_string(0, 106, "R_NUM:");
            ips200_show_int(43,106,R_Statics,3);
            break;
        }
        case Show_All_Statics:
        {
            ips200_show_string(0, 97, "L_NUM:");
            ips200_show_int(37,97,L_Statics,3);
            ips200_show_string(0, 106, "R_NUM:");
            ips200_show_int(37,106,R_Statics,3);
            break;
        }
        case Show_L_Border:
        {
            for (i = 0; i < Image_Y - 2; i++)
            {
                // 移植注释：Pe186 - uint8类型 >= 0 恒为真，原写法保留如下
                // if(L_Border[i] <= 80 && L_Border[i] >= 0)
                if(L_Border[i] <= 80)
                {
                    ips200_draw_point(L_Border[i], i, User_Green);
                }
            }
            break;
        }
        case Show_R_Border:
        {
            for (i = 0; i < Image_Y - 2; i++)
            {
                // 移植注释：Pe186 - uint8类型 >= 0 恒为真，原写法保留如下
                // if(R_Border[i] <= 80 && R_Border[i] >= 0)
                if(R_Border[i] <= 80)
                {
                    ips200_draw_point(R_Border[i], i, User_Red);
                }
            }
            break;
        }
        case Show_All_Border:
        {
            for (i = 0; i < Image_Y - 2; i++)
            {
                // 移植注释：Pe186 - uint8类型 >= 0 恒为真，原写法保留如下
                // if(L_Border[i] <= 80 && L_Border[i] >= 0)
                if(L_Border[i] <= 80)
                {
                    ips200_draw_point(L_Border[i], i, User_Green);
                }
            }
            for (i = 0; i < Image_Y - 2; i++)
            {
                // 移植注释：Pe186 - uint8类型 >= 0 恒为真，原写法保留如下
                // if(R_Border[i] <= 80 && R_Border[i] >= 0)
                if(R_Border[i] <= 80)
                {
                    ips200_draw_point(R_Border[i], i, User_Red);
                }
            }
            for (i = 2; i < Image_Y - 2; i++)
            {
                // 移植注释：Pe186 - uint8类型 >= 0 恒为真，原写法保留如下
                // if(C_Line[i] <= 80 && C_Line[i] >= 0)
                if(C_Line[i] <= 80)
                {
                    ips200_draw_point(C_Line[i], i, User_Blue);
                }
            }
            break;
        }
        case Show_L_Up_Turn_Left_Point_1:
        {
            if(L_Up_Turn_Left_Point_Flag_1 == 1)
            {
                Draw_X_In_Point_IPS200(Find_Line_Image, L_Up_Turn_Left_Point_1[1], L_Up_Turn_Left_Point_1[0], RGB565_PURPLE);
            }
            break;
        }
        case Show_L_Right_Turn_Up_Point_1:
        {
            if(L_Right_Turn_Up_Point_Flag_1 == 1)
            {
                Draw_X_In_Point_IPS200(Find_Line_Image, L_Right_Turn_Up_Point_1[1], L_Right_Turn_Up_Point_1[0], RGB565_PURPLE);
            }
            break;
        }
        case Show_R_Up_Turn_Right_Point_1:
        {
            if(R_Up_Turn_Right_Point_Flag_1 == 1)
            {
                Draw_X_In_Point_IPS200(Find_Line_Image, R_Up_Turn_Right_Point_1[1], R_Up_Turn_Right_Point_1[0], RGB565_PURPLE);
            }
            break;
        }
        case Show_R_Left_Turn_Up_Point_1:
        {
            if(R_Left_Turn_Up_Point_Flag_1 == 1)
            {
                Draw_X_In_Point_IPS200(Find_Line_Image, R_Left_Turn_Up_Point_1[1], R_Left_Turn_Up_Point_1[0], RGB565_PURPLE);
            }
            break;
        }
        case Show_All_Stight_Turn_Point:
        {
//            if(L_Up_Turn_Left_Point_Flag_1 == 1)
//            {
                Draw_X_In_Point_IPS200(Find_Line_Image, Y_Meet, X_Meet, RGB565_RED);
//            }
//            if(L_Right_Turn_Up_Point_Flag_1_ramp == 1)
//            {
//                Draw_X_In_Point_IPS200(Find_Line_Image, L_Right_Turn_Up_Point_1_ramp[1], L_Right_Turn_Up_Point_1_ramp[0], RGB565_GREEN);
//            }
                if(L_Right_Turn_Up_Point_Flag_1 == 1)
                {
                    Draw_X_In_Point_IPS200(Find_Line_Image, L_Right_Turn_Up_Point_1[1], L_Right_Turn_Up_Point_1[0], RGB565_GREEN);
                }
//            if(R_Up_Turn_Right_Point_Flag_1 == 1)
//            {
//                Draw_X_In_Point_IPS200(Find_Line_Image, R_Up_Turn_Right_Point_1[1], R_Up_Turn_Right_Point_1[0], RGB565_PURPLE);
//            }
            if(R_Left_Turn_Up_Point_Flag_1_ramp == 1)
            {
                Draw_X_In_Point_IPS200(Find_Line_Image, R_Left_Turn_Up_Point_1_ramp[1], R_Left_Turn_Up_Point_1_ramp[0], RGB565_GREEN);
            }
            // single
            if(R_Up_Turn_Left_Point_Flag_1)
            {
                Draw_X_In_Point_IPS200(Find_Line_Image, R_Up_Turn_Left_Point_1[1], R_Up_Turn_Left_Point_1[0], RGB565_YELLOW);
//                Draw_X_In_Point_IPS200(Find_Line_Image, R_Up_Turn_Left_Point_1_2[1], R_Up_Turn_Left_Point_1_2[0], RGB565_YELLOW);
            }
//            if(R_Left_Turn_Up_Point_Flag_1)
//            {
//                Draw_X_In_Point_IPS200(Find_Line_Image, R_Left_Turn_Up_Point_1[1], R_Left_Turn_Up_Point_1[0], RGB565_YELLOW);
////                Draw_X_In_Point_IPS200(Find_Line_Image, R_Left_Turn_Up_Point_1[1], R_Left_Turn_Up_Point_1[0], RGB565_YELLOW);
//            }
            if(L_Up_Turn_Right_Point_Flag_1)
            {
                Draw_X_In_Point_IPS200(Find_Line_Image, L_Up_Turn_Right_Point_1[1], L_Up_Turn_Right_Point_1[0], RGB565_YELLOW);
//                Draw_X_In_Point_IPS200(Find_Line_Image, L_Up_Turn_Right_Point_1_2[1], L_Up_Turn_Right_Point_1_2[0], RGB565_PURPLE);
            }
//            if(L_Right_Turn_Up_Point_Flag_1)
//            {
//                Draw_X_In_Point_IPS200(Find_Line_Image, L_Right_Turn_Up_Point_1[1], L_Right_Turn_Up_Point_1[0], RGB565_PURPLE);
////                Draw_X_In_Point_IPS200(Find_Line_Image, L_Right_Turn_Up_Point_1[1], L_Right_Turn_Up_Point_1[0], RGB565_PURPLE);
//            }
            if(L_Left_Turn_Up_Point_Flag_1)
            {
                Draw_X_In_Point_IPS200(Find_Line_Image, L_Left_Turn_Up_Point_1[1], L_Left_Turn_Up_Point_1[0], RGB565_PURPLE);
//                Draw_X_In_Point_IPS200(Find_Line_Image, R_Left_Turn_Up_Point_1[1], R_Left_Turn_Up_Point_1[0], RGB565_YELLOW);
            }
            if(R_Right_Turn_Up_Point_Flag_1)
            {
                Draw_X_In_Point_IPS200(Find_Line_Image, R_Right_Turn_Up_Point_1[1], R_Right_Turn_Up_Point_1[0], RGB565_PURPLE);
//                Draw_X_In_Point_IPS200(Find_Line_Image, L_Right_Turn_Up_Point_1[1], L_Right_Turn_Up_Point_1[0], RGB565_PURPLE);
            }
            break;
        }
        case Show_L_Arc_Turn_Point:
        {
            if(L_Arc_Turn_Point_Flag == 1)
            {
                for(i = 0; i < L_Arc_Turn_Point_Num; i++)
                {
                    Draw_10_In_Point_IPS200(Find_Line_Image, L_Arc_Turn_Point[i][1], L_Arc_Turn_Point[i][0], RGB565_RED);
                }
            }
            break;
        }
        case Show_R_Arc_Turn_Point:
        {
            if(R_Arc_Turn_Point_Flag == 1)
            {
                for(i = 0; i < R_Arc_Turn_Point_Num; i++)
                {
                    Draw_10_In_Point_IPS200(Find_Line_Image, R_Arc_Turn_Point[i][1], R_Arc_Turn_Point[i][0], RGB565_GREEN);
                }
            }
            break;
        }
        case Show_All_Arc_Turn_Point:
        {
            if(L_Arc_Turn_Point_Flag == 1)
            {
                for(i = 0; i < L_Arc_Turn_Point_Num; i++)
                {
                    Draw_10_In_Point_IPS200(Find_Line_Image, L_Arc_Turn_Point[i][1], L_Arc_Turn_Point[i][0], RGB565_RED);
                }
            }
            if(R_Arc_Turn_Point_Flag == 1)
            {
                for(i = 0; i < R_Arc_Turn_Point_Num; i++)
                {
                    Draw_10_In_Point_IPS200(Find_Line_Image, R_Arc_Turn_Point[i][1], R_Arc_Turn_Point[i][0], RGB565_GREEN);
                }
            }
            break;
        }
        case Show_L_Straightaway_Lope_Rate_A:
        {
            ips200_show_string(0, 115, "LLRA:");
            ips200_show_float(31,115,L_Straightaway_Lope_Rate_A,1,2);
            break;
        }
        case Show_L_Straightaway_Lope_Rate_B:
        {
            ips200_show_string(0, 124, "LLRB:");
            ips200_show_float(31,124,L_Straightaway_Lope_Rate_B,1,2);
            break;
        }
        case Show_L_Straightaway_Lope_Rate_C:
        {
            ips200_show_string(0, 133, "LLRC:");
            ips200_show_float(31,133,L_Straightaway_Lope_Rate_C,1,2);
            break;
        }
        case Show_R_Straightaway_Lope_Rate_A:
        {
            ips200_show_string(0, 142, "RLRA:");
            ips200_show_float(31,142,R_Straightaway_Lope_Rate_A,1,2);
            break;
        }
        case Show_R_Straightaway_Lope_Rate_B:
        {
            ips200_show_string(0, 151, "RLRB:");
            ips200_show_float(31,151,R_Straightaway_Lope_Rate_B,1,2);
            break;
        }
        case Show_R_Straightaway_Lope_Rate_C:
        {
            ips200_show_string(65, 61, "RLRC:");
            ips200_show_float(96,61,R_Straightaway_Lope_Rate_C,1,2);
            break;
        }
        case Show_All_Straightaway_Lope_Rate:
        {
            ips200_show_string(0, 115, "LLRA:");
            ips200_show_float(31,115,L_Straightaway_Lope_Rate_A,1,2);
            ips200_show_string(0, 124, "LLRB:");
            ips200_show_float(31,124,L_Straightaway_Lope_Rate_B,1,2);
            ips200_show_string(0, 133, "LLRC:");
            ips200_show_float(31,133,L_Straightaway_Lope_Rate_C,1,2);
            ips200_show_string(0, 142, "RLRA:");
            ips200_show_float(31,142,R_Straightaway_Lope_Rate_A,1,2);
            ips200_show_string(0, 151, "RLRB:");
            ips200_show_float(31,151,R_Straightaway_Lope_Rate_B,1,2);
            ips200_show_string(65, 61, "RLRC:");
            ips200_show_float(96,61,R_Straightaway_Lope_Rate_C,1,2);
            break;
        }
        case Show_L_Intercept:
        {
            ips200_show_string(65, 70, "L_IT:");
            ips200_show_float(96,70,L_Intercept,2,2);
            break;
        }
        case Show_R_Intercept:
        {
            ips200_show_string(65, 79, "R_IT:");
            ips200_show_float(96,79,R_Intercept,2,2);
            break;
        }
        case Show_All_Intercept:
        {
            ips200_show_string(65, 70, "L_IT:");
            ips200_show_float(96,70,L_Intercept,2,1);
            ips200_show_string(65, 79, "R_IT:");
            ips200_show_float(96,79,R_Intercept,2,1);
            break;
        }
        case Show_L_Fitting_Line:
        {
            for (i = 0; i < 18; i++)
            {
                if(L_Fitting_Line[i] <= 80 && L_Fitting_Line[i] >= 0)
                {
                    ips200_draw_point(L_Fitting_Line[i], i * 3 + 2, User_Red);
                }
            }
            break;
        }
        case Show_R_Fitting_Line:
        {
            for (i = 0; i < 18; i++)
            {
                if(R_Fitting_Line[i] <= 80 && R_Fitting_Line[i] >= 0)
                {
                    ips200_draw_point(R_Fitting_Line[i], i * 3 + 2, User_Green);
                }
            }
            break;
        }
        case Show_All_Fitting_Line:
        {
            for (i = 0; i < 18; i++)
            {
                if(L_Fitting_Line[i] <= 80 && L_Fitting_Line[i] >= 0)
                {
                    ips200_draw_point(L_Fitting_Line[i], i * 3 + 2, User_Red);
                }
            }
            for (i = 0; i < 18; i++)
            {
                if(R_Fitting_Line[i] <= 80 && R_Fitting_Line[i] >= 0)
                {
                    ips200_draw_point(R_Fitting_Line[i], i * 3 + 2, User_Green);
                }
            }
            break;
        }
        case Show_L_Variance:
        {
            ips200_show_string(65, 88, "LV:");
            ips200_show_float(84,88,L_Variance,4,1);
            break;
        }
        case Show_R_Variance:
        {
            ips200_show_string(65, 97, "RV:");
            ips200_show_float(84,97,R_Variance,4,1);
            break;
        }
        case Show_All_Variance:
        {
            ips200_show_string(65, 88, "LV:");
            ips200_show_float(84,88,L_Variance,4,1);
            ips200_show_string(65, 97, "RV:");
            ips200_show_float(84,97,R_Variance,4,1);
            break;
        }
        case Show_Deviation_Value:
        {
            ips200_show_string(81,0,"De_Va:");
            ips200_show_float(81,9,Deviation_Value,1,4);
            break;
        }
        case Show_Sensitivity:
        {
            ips200_show_string(81,18,"Sensi:");
            ips200_show_float(81,27,Sensitivity,3,3);
            break;
        }
        case Show_Element_State:
        {
            ips200_show_string(81,36,"Eleme:");
            ips200_show_uint(81,45,Element_State,2);
            break;
        }
        case Show_Stop_Flag:
        {
            ips200_show_string(65,106,"Stop:");
            ips200_show_uint(96,106,Stop_Flag,1);
            break;
        }
        case Show_Zebra_Flag:
        {
            ips200_show_string(65,115,"Zebra:");
            ips200_show_uint(96,115,Zebra_Flag,1);
            break;
        }
        case Show_Speed_Proportion:
        {
            ips200_show_string(65,124,"Speed:");
            ips200_show_float(96,124,Speed_Proportion,1,3);
            break;
        }
        case Show_Adaptive_Thres_Average:
        {
            ips200_show_string(65,133,"A_Thr:");
            ips200_show_uint(96,133,Adaptive_Thres_Average,3);
            break;
        }
        case Show_Barrier_Distance:
        {
            ips200_show_string(65,142,"B_Dis:");
//            ips200_show_int(96,142,Barrier_Distance,4);
            break;
        }
    }
}

float Last_a = 0;
float Last_b = 0;
uint8 Show_Flag = 1;//2显示

/**
 * @brief  刷新 IPS200 综合调试画面
 * @param  Show_Flag  当前显示模式，值为 2 时显示图像调试界面
 * @return 无
 */
void IPS200_Show(uint8 Show_Flag)
{
//    IPS_Show(Show_Find_Line_Image);
    if(Show_Flag == 2 && menu_mode)
    {
//        DisplayBorderDistances();

        Last_a = Last_b;
        Last_b = Deviation_Value;

//        IPS_Show(0);
//        IPS_Show(Show_Inverse_Perspective_Image);
//        IPS_Show(Show_I_All_Line);
//        IPS_Show(Show_Back_Inverse_Perspective_Image);
//        IPS_Show(Show_Vague_Image);

        IPS_Show(Show_Find_Line_Image);

//        IPS_Show(Show_All_Line);

        IPS_Show(Show_All_Border);
        IPS_Show(Show_All_Meet);
        IPS_Show(Show_All_Statics);
        IPS_Show(Show_All_Start_Point);

//        IPS_Show(Show_All_Straightaway_Lope_Rate);
//        IPS_Show(Show_All_Intercept);
//        IPS_Show(Show_All_Variance);

        IPS_Show(Show_All_Stight_Turn_Point);

//        IPS_Show(Show_All_Arc_Turn_Point);
//        IPS_Show(Show_All_Fitting_Line);

        IPS_Show(Show_Deviation_Value);
        IPS_Show(Show_Sensitivity);
        IPS_Show(Show_Element_State);

//        IPS_Show(Show_Stop_Flag);
        IPS_Show(Show_Zebra_Flag);

        IPS_Show(Show_Speed_Proportion);
        IPS_Show(Show_Adaptive_Thres_Average);

//        IPS_Show(Show_Barrier_Distance);
//        ips200_show_uint(0, 210, Image_Count, 6);
//        ips200_show_float(0, 220, Camera_pid.Kp, 2, 2);
//        ips200_show_uint(0, 230, forward_target, 2);
//        Wifi_Send_Data(Deviation_Value, Y_Meet, Speed_Proportion, Last_b - Last_a, 0, 0, 0, 0);

        /********************************************通用数据显示********************************************/

//                if(menu_mode)
//                {
                //euler角,角速度
                ips200_show_string(190 , 0*8 , "roll:");
                ips200_show_float( 190 , 1*8 , imu660ra.eulerAngle.roll  , 3 , 3 );
//                ips200_show_uint( 190 , 2*8 , imu660ra_gyro_x , 5 );
                ips200_show_string(190 , 3*8 , "pitch:");
                ips200_show_float( 190 , 4*8 , imu660ra.eulerAngle.pitch , 3 , 3 );
//                ips200_show_uint( 190 , 5*8 , imu660ra_gyro_y , 5 );

                ips200_show_uint( 190 , 7*8 , dis_tof_mm , 5 );
//                ips200_show_uint( 190 , 8*8 , dis_tof_mm , 5 );
//                ips200_show_float( 190 , 8*8 , IKParam.XLeft , 5 , 1 );
//                ips200_show_uint( 190 , 6*8 , flag_jump , 2 );
        //
//                ips200_show_uint( 0, 30*8, dis_tof_mm, 5 );
                ips200_show_float(  0, 20*8, Ramp_Up_Slope_L, 3,3 );
                ips200_show_float(  0, 21*8, Ramp_Down_Slope_L, 3,3 );
                ips200_show_float( 50, 20*8, Ramp_Up_Slope_R, 3,3 );
                ips200_show_float( 50, 21*8, Ramp_Down_Slope_R, 3,3 );
                ips200_show_uint( 0, 23*8, L_Ramp_Flag, 2 );
                ips200_show_uint( 50, 23*8, R_Ramp_Flag, 2 );
                ips200_show_uint( 100, 23*8, Ramp_Judge_Image_Flag, 2 );

//                ips200_show_float( 70, 36*8, stab_roll, 3, 2 );




        //
                ips200_show_int( 100, 38*8, motor_value.receive_left_speed_data, 5 );
                ips200_show_int( 130, 38*8, motor_value.receive_right_speed_data, 5 );
                ips200_show_int( 170, 38*8, Yao.Target_Speed, 4 );
                ips200_show_float( 200, 38*8, Yao.Target_height, 2, 2 );
//                ips200_show_float( 0 , 38*8 , Battery_voltage , 4 , 3 );
//                }

                /********************************************通用数据显示********************************************/


    }
}
// 测试函数，可以得到单边巡线数组
#define FONT_HEIGHT 8
#define FONT_WIDTH 6
#define MAX_Y 280
#define START_Y 160

/**
 * @brief  以数字矩阵方式显示中线与左右边界的距离差
 * @return 无
 */
void DisplayBorderDistances(void)
{
//    if (key_detect(KEY_3, KEY_SHORT_PRESS))
//    {
        /* 显示参数 */
        const uint16 x_max = 240;                      // 屏幕横向最大值
        const uint16 items_per_row = x_max / (2 * FONT_WIDTH); // 每行显示20个数据（240/(2 * 6)=20）

        /* 上半区：左边线差值 */
        uint16 base_y = START_Y;                        // 起始Y坐标
        for (uint8 i = 0; i < 60; i++)
        {
            // 计算显示位置
            uint16 row = i / items_per_row;             // 当前行（0-2）
            uint16 col = i % items_per_row;             // 当前列（0-19）
            uint16 x = col * 2 * FONT_WIDTH;            // X坐标（每数据占12像素）
            uint16 y = base_y + row * FONT_HEIGHT;      // Y坐标

            // 计算差值
            uint8 diff = C_Line[i] - L_Border[i];

            diff = (diff > 99) ? 0 : diff;

            // 显示两位数值
            ips200_show_uint(x, y, diff, 2);

            // 超过显示区域则停止
            if (y + FONT_HEIGHT > (START_Y + (MAX_Y - START_Y)/2)) break;
        }

        /* 下半区：右边线差值 */
        base_y = START_Y + (MAX_Y - START_Y)/2 + FONT_HEIGHT; // 下半区起始Y坐标
        for (uint8 i = 0; i < 60; i++)
        {
            // 计算显示位置
            uint16 row = i / items_per_row;             // 当前行（0-2）
            uint16 col = i % items_per_row;             // 当前列（0-19）
            uint16 x = col * 2 * FONT_WIDTH;            // X坐标
            uint16 y = base_y + row * FONT_HEIGHT;      // Y坐标

            // 计算差值
            uint8 diff = R_Border[i] - C_Line[i];

            // 显示两位数值
            ips200_show_uint(x, y, diff, 2);

            // 超过显示区域则停止
            if (y + FONT_HEIGHT > MAX_Y) break;
        }
//    }
}



