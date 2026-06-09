#ifndef CODE_INVERSE_PROCESS_H_
#define CODE_INVERSE_PROCESS_H_

#define Image_X 80
#define Image_Y 60
#define Use_Num 250

#define Derailment      1   // 出赛道状态
#define Straightaway    2   // 直道状态
#define L_Turn          3   // 左转弯道状态
#define R_Turn          4   // 右转弯道状态
#define L_Circle        5   // 左环岛状态
#define R_Circle        6   // 右环岛状态
#define Cross           7   // 十字路口状态
#define Zebra           8   // 斑马线状态
#define Ramp            9   // 坡道状态
#define Three_Bif       10  // 三岔状态
#define Barrier         11  // 障碍状态
#define Disconnection   12  // 断路状态
#define T_Way           13  // T 路口状态
#define L_Garage        14  // 左车库状态
#define R_Garage        15  // 右车库状态
#define Small_S         17  // 小 S 弯状态
#define L_Oblique_Cross 18  // 左斜入十字状态
#define R_Oblique_Cross 19  // 右斜入十字状态
#define Jump_State      20  // 跳跃状态
#define Single_State    21  // 单边桥状态

extern uint8 Zebra_Flag;                 // 斑马线处理开关
extern uint8 Ramp_Flag;                  // 坡道处理开关
extern uint8 Small_S_Flag;               // 小 S 弯处理开关

extern float Deviation_Value;            // 当前图像偏差值
extern uint8 Thres_Filiter_Flag_1;       // 阈值滤波开关
extern uint8 IMU_JF_Flag;                // IMU 积分/校正标志
extern float Z_Yaw;                      // 偏航角相关中间量
extern uint8 Element_State;              // 当前识别元素状态
extern float L_Straightaway_Lope_Rate_A; // 左直道 A 段斜率
extern float L_Straightaway_Lope_Rate_B; // 左直道 B 段斜率
extern float L_Straightaway_Lope_Rate_C; // 左直道 C 段斜率
extern float R_Straightaway_Lope_Rate_A; // 右直道 A 段斜率
extern float R_Straightaway_Lope_Rate_B; // 右直道 B 段斜率
extern float R_Straightaway_Lope_Rate_C; // 右直道 C 段斜率
extern float Sensitivity;                // 图像灵敏度参数
extern uint8 flag_Single;                // 单边桥流程标志
extern uint8 flag_Single_HighState;      // 单边桥高姿态标志
extern uint8 Single_state;               // 单边桥状态机状态
extern uint8 flag_jump_2;                // 跳跃二阶段标志

extern uint8 buzzer_flag;                // 蜂鸣器控制标志
extern uint8 jump_line;                  // 跳跃判定行号
extern uint8 jump_line_slowdown;         // 跳跃减速判定行号
extern float P_Value_L[7];               // 模糊控制参数表
extern uint8 Y_Meet;                     // Y 方向交点位置
extern uint8 Element_Break_Flag;         // 强制退出元素处理标志
extern uint8 Find_Line_Image[Image_Y][Image_X]; // 寻线结果图像
extern float Stretch_Coefficient;        // 图像拉伸系数
extern uint16 Max_Speed;                 // 速度上限
extern uint16 Min_Speed;                 // 速度下限
extern uint16 Ramp_State_2_Speed;        // 坡道阶段 2 目标速度

extern uint16 Bypass_Count1;             // 绕障阶段 1 计数
extern uint16 Bypass_Count2;             // 绕障阶段 2 计数
extern uint16 Bypass_Count3;             // 绕障阶段 3 计数
extern uint8 Bypass_Line;                // 绕障判定行
extern float yaw_bypass;                 // 绕障偏航补偿量
extern uint8 Error_Line;                 // 错误判定行号
extern uint8 Single_jg_line;             // 单边桥判定行号
extern uint16 Image_count_Single_End;    // 单边桥结束计数阈值
extern uint16 ramp_judge_dis;            // 坡道识别距离阈值
extern uint16 ramp_end_dis;              // 坡道结束距离阈值
extern uint16 ramp_up_delay;             // 上坡延时参数
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

/**
 * @brief  根据偏差和变化量计算图像控制输出
 * @param  off_line   当前离线偏差
 * @param  dif_value  偏差变化量
 * @return 计算后的控制量
 */
float Get_P(int16 off_line, float dif_value);

/**
 * @brief  执行图像寻迹与元素处理主流程
 * @return 无
 */
void Tracking(void);

/**
 * @brief  将浮点数限制在指定范围内
 * @param  x  输入值
 * @param  a  下限值
 * @param  b  上限值
 * @return 限幅后的结果
 */
float Limit_Float(float x, float a, float b);

#endif /* CODE_INVERSE_PROCESS_H_ */
