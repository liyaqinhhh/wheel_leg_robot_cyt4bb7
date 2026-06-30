/*
 * Init.h
 * 系统初始化模块头文件
 *
 *  Created on: 2024年12月3日
 *      Author: 21912
 */

#ifndef CODE_CONTROLPART_INIT_H_
#define CODE_CONTROLPART_INIT_H_


// @brief  系统总初始化 - 依次初始化时钟、调试口、蜂鸣器、按键、无线串口、
//        TOF测距、IMU、ADC电压、IPS屏幕、Flash、卡尔曼滤波器、PID参数、
//        舵机、EKF、无刷驱动串口、GPS导航等全部外设和算法模块
void Init_All(void);

// @brief  蜂鸣器控制 - 根据赛道元素状态和错误标志控制蜂鸣器鸣响
void Buzzer_Control();

// @brief  TOF测距获取滤波后距离 - 使用滑动窗口均值滤波（窗口大小30），去除偏移量
// @return 滤波后的距离值（单位：mm）
uint16_t tof_dl1b_get_mm(void);


#endif /* CODE_CONTROLPART_INIT_H_ */
