/*
 * menu.h
 *
 *  Created on: 2024年7月5日
 *      Author: LateRain
 */

#ifndef CODE_KEY_MENU_H_
#define CODE_KEY_MENU_H_

#define WRITE 0
#define READ  1

extern uint8 menu_mode;
extern uint8 flag_track;
extern uint8 screen_refresh;

extern uint16 Image_Gain;
extern uint16 Image_EpTime;
extern uint8  Change_Control;

int key_detect(key_index_enum key_n, key_state_enum state);

void store_or_read_DATA(int way);
void switch_unit();
void data_operate(float *data);
void menu(void);

extern uint8 menu_show ;
extern uint8 Jump_Control_Flag ;
extern uint8 Single_Control_Flag;
extern uint8 L_Circle_Flag ;
extern uint8 R_Circle_Flag;
extern uint16 ConvenTion_Speed;
extern uint16 Derailment_Speed  ;        //出赛道速度
extern uint16 Straightaway_Speed ;        //直道速度
extern uint16 L_Turn_Speed ;        //左转弯道速度
extern uint16 R_Turn_SPeed;        //右转弯道速度
extern uint16 Jump_speed;
extern uint16 Single_speed_1 ;     // 切换姿态时的速度
extern uint16 Single_speed_2 ;    // 通过单边桥的速度

extern uint16 L_Circle_State_1_Speed;        //左环岛预入环  1  速度
extern uint16 L_Circle_State_2_Speed;        //左环岛入环  2  速度
extern uint16 L_Circle_State_3_Speed;        //左环岛环中  3  速度
extern uint16 L_Circle_State_4_Speed ;        //左环岛预出环  4  速度
extern uint16 L_Circle_State_5_Speed;        //左环岛出环  5  速度

extern uint16 R_Circle_State_1_Speed;        //右环岛状态  1  速度
extern uint16 R_Circle_State_2_Speed;        //右环岛状态  2  速度
extern uint16 R_Circle_State_3_Speed;        //右环岛状态  3  速度
extern uint16 R_Circle_State_4_Speed ;        //右环岛状态  4  速度
extern uint16 R_Circle_State_5_Speed ;        //右环岛状态  5  速度

extern uint16 Cross_Speed ;        // 通过十字路口速度
//uint16 Cross_State_3_Speed               =       100;        // 十字中速度(积分中)
extern uint16 Cross_State_4_Speed;        // 十字中速度(积分中)
extern uint16 L_Oblique_Cross_Speed ;       //斜入十字——左
extern uint16 R_Oblique_Cross_Speed ;       //斜入十字——右
extern uint16 Small_S_Speed;       //小S弯

#endif /* CODE_KEY_MENU_H_ */
