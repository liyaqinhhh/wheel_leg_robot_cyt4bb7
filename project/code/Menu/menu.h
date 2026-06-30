/*
 * menu.h
 *
 *  Created on: 2024年7月5日
 *      Author: LateRain
 */

#ifndef CODE_KEY_MENU_H_
#define CODE_KEY_MENU_H_

#define WRITE 0    // 将当前菜单参数写入 Flash
#define READ  1    // 从 Flash 读取菜单参数

extern uint8 menu_mode;        // 菜单模式：0=参数菜单，1=运行显示
extern uint8 flag_track;       // 菜单运行标志：用于暂停寻迹调参
extern uint8 screen_refresh;   // 屏幕刷新标志

extern uint16 Image_Gain;      // 摄像头增益参数
extern uint16 Image_EpTime;    // 摄像头曝光时间参数
extern uint8  Change_Control;  // 参数保存后是否重初始化摄像头

/**
 * @brief  检测指定按键是否进入目标状态
 * @param  key_n  目标按键编号
 * @param  state  目标按键状态
 * @return 1 表示检测到目标状态，0 表示未检测到
 */
int key_detect(key_index_enum key_n, key_state_enum state);

/**
 * @brief  在 IPS200 屏幕上显示运行状态与调试信息
 * @return 无
 */
void IPS200_Show1(void);

/**
 * @brief  将菜单参数写入 Flash 或从 Flash 恢复
 * @param  way  操作方式，取值为 WRITE 或 READ
 * @return 无
 */
void store_or_read_DATA(int way);

/**
 * @brief  切换参数调节步进单位
 * @return 无
 */
void switch_unit();

/**
 * @brief  对浮点类型菜单参数执行增减调节
 * @param  data  指向待调节参数的指针
 * @return 无
 */
void data_operate(float *data);

/**
 * @brief  处理菜单界面显示、光标切换与参数修改
 * @return 无
 */
void menu(void);

extern uint8 menu_show;               // 菜单显示开关
extern uint8 Jump_Control_Flag;       // 跳跃元素处理开关
extern uint8 Single_Control_Flag;     // 单边桥元素处理开关
extern uint8 L_Circle_Flag;           // 左环岛处理模式
extern uint8 R_Circle_Flag;           // 右环岛处理模式
extern uint16 ConvenTion_Speed;       // 常规巡线速度
extern uint16 Derailment_Speed;       // 出赛道时的保护速度
extern uint16 Straightaway_Speed;     // 直道速度
extern uint16 L_Turn_Speed;           // 左转弯道速度
extern uint16 R_Turn_SPeed;           // 右转弯道速度
extern uint16 Jump_speed;             // 跳跃动作速度
extern uint16 Single_speed_1;         // 单边桥切换姿态速度
extern uint16 Single_speed_2;         // 单边桥通过速度

extern uint16 L_Circle_State_1_Speed; // 左环岛预入环速度
extern uint16 L_Circle_State_2_Speed; // 左环岛入环速度
extern uint16 L_Circle_State_3_Speed; // 左环岛环中速度
extern uint16 L_Circle_State_4_Speed; // 左环岛预出环速度
extern uint16 L_Circle_State_5_Speed; // 左环岛出环速度

extern uint16 R_Circle_State_1_Speed; // 右环岛预入环速度
extern uint16 R_Circle_State_2_Speed; // 右环岛入环速度
extern uint16 R_Circle_State_3_Speed; // 右环岛环中速度
extern uint16 R_Circle_State_4_Speed; // 右环岛预出环速度
extern uint16 R_Circle_State_5_Speed; // 右环岛出环速度

extern uint16 Cross_Speed;            // 十字路口通过速度
extern uint16 Cross_State_4_Speed;    // 十字内部修正阶段速度
extern uint16 L_Oblique_Cross_Speed;  // 左斜入十字速度
extern uint16 R_Oblique_Cross_Speed;  // 右斜入十字速度
extern uint16 Small_S_Speed;          // 小 S 弯速度

#endif /* CODE_KEY_MENU_H_ */
