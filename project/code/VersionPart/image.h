#ifndef CODE_INVERSE_PROCESS_H_
#define CODE_INVERSE_PROCESS_H_

#define Image_X 80
#define Image_Y 60
#define Use_Num 250

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
#define Small_S        17
#define L_Oblique_Cross 18
#define R_Oblique_Cross 19
#define Jump_State      20
#define Single_State    21

extern uint8 Zebra_Flag;
extern uint8 Ramp_Flag;
extern uint8 Small_S_Flag;

extern float Deviation_Value;
extern uint8 Thres_Filiter_Flag_1;
extern uint8 IMU_JF_Flag;
extern float Z_Yaw;
extern uint8 Element_State;
extern float L_Straightaway_Lope_Rate_A;
extern float L_Straightaway_Lope_Rate_B;
extern float L_Straightaway_Lope_Rate_C;
extern float R_Straightaway_Lope_Rate_A;
extern float R_Straightaway_Lope_Rate_B;
extern float R_Straightaway_Lope_Rate_C;
extern float Sensitivity;
extern uint8 flag_Single;
extern uint8 flag_Single_HighState;
extern uint8 Single_state;
extern uint8 flag_jump_2;

extern uint8 buzzer_flag;
extern uint8 jump_line;
extern uint8 jump_line_slowdown;
extern float P_Value_L[7];
extern uint8 Y_Meet;
extern uint8 Element_Break_Flag;
extern uint8 Find_Line_Image[Image_Y][Image_X];
extern float Stretch_Coefficient;
extern uint16 Max_Speed;
extern uint16 Min_Speed;
extern uint16 Ramp_State_2_Speed;

extern uint16 Bypass_Count1;
extern uint16 Bypass_Count2;
extern uint16 Bypass_Count3;
extern uint8 Bypass_Line;
extern float yaw_bypass;
extern uint8 Error_Line;
extern uint8 Single_jg_line;
extern uint16 Image_count_Single_End;
extern uint16 ramp_judge_dis;
extern uint16 ramp_end_dis;
extern uint16 ramp_up_delay;
//extern uint8 L_No_Image_Border[60];
//extern uint8 R_No_Image_Border[60];
//存放边线的一维数组
//extern uint8 I_R_Border[Image_Y];
//extern uint8 I_L_Border[Image_Y];

//存放边线点的x，y坐标（二维数组）
//extern uint8 I_L_Line[(uint16)Use_Num][2] ;//左线
//extern uint8 I_R_Line[(uint16)Use_Num][2];//右线
//extern uint8 I_C_Line[(uint16)Use_Num];

//统计找到的边线点的个数
//extern uint16 I_L_Statics;//统计左边找到点的个数
//extern uint16 I_R_Statics;//统计右边找到点的个数

//extern uint8 I_Vague_Image[Image_Y][Image_X];

//extern uint8 I_Perspective_Image[Image_Y][Image_X];
//extern uint8 Back_I_Perspective_Image[Image_Y][Image_X];
//求逆透视图像
//void Get_Inverse_Perspective_Image(uint8(*source_image)[Image_X], uint8 (*target_image)[Image_X]);

//求逆透视的反逆透视图像
//void Get_Back_Inverse_Perspective_Image(uint8(*Source_Image)[Image_X], uint8(*target_image)[Image_X]);

//void I_Process_Image(void);

//float Get_Turn_Point_Angle(uint8 Ax, uint8 Ay, uint8 Bx, uint8 By, uint8 Cx, uint8 Cy);

//float I_Get_Turn_Point_Angle(uint8 Ax, uint8 Ay, uint8 Bx, uint8 By, uint8 Cx, uint8 Cy);
float Get_P(int16 off_line, float dif_value);
void Tracking(void);
float Limit_Float(float x, float a, float b);

#endif /* CODE_INVERSE_PROCESS_H_ */
