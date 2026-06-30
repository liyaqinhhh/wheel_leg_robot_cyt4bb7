/*
 * Init.c
 *
 *  Created on: 2024年2月
 *      Author: 21912
 */
#include "zf_common_headfile.h"
#include "kalman.h"
#include "imu660.h"
#include "Interrupt.h"
#include "PID.h"
#include "menu.h"
#include "servo.h"
#include "ekf.h"
#include "small_driver_uart_control.h"
#include "ins_interface.h"
#include "gps_nav.h"

void Init_All(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    // 等待系统电源稳定
    system_delay_ms(100);

    //    gpio_init(P33_10, GPO, 0, GPO_PUSH_PULL);            // 蜂鸣器
    gpio_init(P19_3, GPO, GPIO_HIGH, GPO_PUSH_PULL);      // P19_3 拉高
    pwm_init(TCPWM_CH28_P10_0, 3000, 0);
    key_init(10);         // 按键初始化
    wireless_uart_init(); // 无线串口初始化
    lora3a22_init();//遥控初始化
    dl1b_init();
    imu660ra_init();                      // IMU初始化
                                          //  gyroOffset_init();
    adc_init(ADC0_CH06_P06_6, ADC_12BIT); // ADC初始化
    //                                       // mt9v03x_init();                       // 摄像头初始化
                                          //    uart_init (WIRELESS_UART_INDEX, WIRELESS_UART_BUAD_RATE, WIRELESS_UART_RX_PIN, WIRELESS_UART_TX_PIN);   // 无线串口初始化

    // IPS
    ips200_set_font(IPS200_6X8_FONT);             // 设置字体大小 6 * 8
    ips200_set_color(RGB565_BLACK, RGB565_WHITE); // 设置为彩色
    ips200_set_dir(IPS200_PORTAIT);               // 设置为竖屏
    ips200_init(IPS200_TYPE_SPI);           // IPS并口初始化

    flash_init();
    // Init_Nag();
    //     imu963ra_kalman_filter_init(&imu, 0.000001, 0.1, 4 / 1000.0f);
    imu963ra_kalman_filter_init(&imu, 0.0000003, 0.3, 4 / 1000.0f);

    //    wifi_spi_init_all();
    //    wifi_spi_init("team", "12345qwert",WIFI_SPI_STATION);
    //    detector_init(DETECTOR_WIFI_SPI);

    // 定时器初始化
    pit_ms_init(PIT_CH0, 1); // 设置定时器周期 1ms

    // PID参数与姿态偏移初始化

    imu660ra.offset_angle.pitch = 19.39; // 俯仰角零偏校准值
    imu660ra.offset_angle.roll = 6.39;   // 横滚角零偏校准值
    Yao.Target_Speed = 0;
    Yao.Target_height = 3;

    pid_para_init(&PID_all.Pid_Gyro_Pitch);
    pid_para_init(&PID_all.Pid_Angle_Pitch);
    pid_para_init(&PID_all.Pid_Speed_Pitch);
    pid_para_init(&PID_all.Pid_Gyro_Roll);
    pid_para_init(&PID_all.Pid_Angle_Roll);
    pid_para_init(&PID_all.Pid_Gyro_Yaw);
    pid_para_init(&PID_all.Pid_Angle_Yaw);

    // // 菜单
    if (menu_open && !flash_check(0, 1))        // Flash page 1 是空白页才能写入
    store_or_read_DATA(READ);

     system_delay_ms(100);
    //    mt9v03x_init();                                     // 摄像头初始化

    // 舵机PWM初始化
    servo_init();
    EKF_Init();

    small_driver_uart_init();

    // if (menu_open)
    // {
    //     store_or_read_DATA(READ);
    // }

    // 预留姿态角度校准
    // calibrate_state = 1;
    // calibrate_count = 0;
    // calibrate_sum = 0;
    // calibrate_offset = 0;
    // 等待系统电源稳定
    // system_delay_ms(100);

    // mt9v03x_init();                                     // 摄像头初始化

    // 初始化GPS导航模块驱动
    // Initialize GNSS driver for GPS navigation.
    gnss_init(TAU1201);
    gps_nav_init();  // 初始化GPS导航模块（航点+校准+状态机）

    // Initialize steering fusion placeholders and debug outputs.
    // Subject-1 background default:
    // - no fixed track
    // - pure vision localization + cone avoidance
    // steer_turn_init();

    // 全部 GPIO 初始化（调试用，已注释）
    //  {
    //      const gpio_pin_enum all_pins[] = {
    //          P00_0, P00_1, P00_2, P00_3,
    //          P01_0, P01_1,
    //          P02_0, P02_1, P02_2, P02_3, P02_4,
    //          P03_0, P03_1, P03_2, P03_3, P03_4,
    //          P04_0, P04_1,
    //          P05_0, P05_1, P05_2, P05_3, P05_4,
    //          P06_0, P06_1, P06_2, P06_3, P06_4, P06_5, P06_6, P06_7,
    //          P07_0, P07_1, P07_2, P07_3, P07_4, P07_5, P07_6, P07_7,
    //          P08_0, P08_1, P08_2, P08_3,
    //          P09_0, P09_1,
    //          P10_0, P10_1, P10_2, P10_3, P10_4,
    //          P11_0, P11_1, P11_2,
    //          P12_0, P12_1, P12_2, P12_3, P12_4, P12_5,
    //          P13_0, P13_1, P13_2, P13_3, P13_4, P13_5, P13_6, P13_7,
    //          P14_0, P14_1, P14_4, P14_5,
    //          P15_0, P15_1, P15_2, P15_3,
    //          P17_0, P17_1, P17_2, P17_3, P17_4,
    //          P18_0, P18_1, P18_2, P18_3, P18_4, P18_5, P18_6, P18_7,
    //          P19_0, P19_1, P19_2, P19_3, P19_4,
    //          P20_0, P20_1, P20_2, P20_3,
    //          P21_5, P21_6,
    //          P22_3, P22_4, P22_5, P22_6,
    //          P23_0, P23_1, P23_3, P23_4, P23_7,
    //      };
    //      for (int i = 0; i < sizeof(all_pins) / sizeof(all_pins[0]); i++)
    //      {
    //          gpio_init(all_pins[i], GPO, GPIO_HIGH, GPO_PUSH_PULL);
    //      }
    //  }

    // small_driver_uart_init();

    // INS 惯导初始化
    // ins_core_init();
    // ins_track_init();
    // ins_api_load_imu_bias();  // 从 Flash 加载陀螺仪零偏

    // cpu_wait_event_ready();
}

#define WINDOW_SIZE 30 // 滑动窗口大小，用于30次均值滤波
#define OFFSET_MM 10   // TOF距离偏移修正量(mm)

// 滑动窗口均值滤波器状态变量
static uint16_t tof_window[WINDOW_SIZE] = {0}; // 滑动窗口数据缓冲区
static uint8_t tof_index = 0;                  // 窗口索引
static uint32_t tof_sum = 0;                   // 窗口内数据累加和
static uint8_t tof_initialized = 0;            // 初始化完成标志

//-------------------------------------------------------------------------------------------------------------------
//  @brief      TOF测距传感器滑动窗口均值滤波
//  @param      无
//  @return     滤波后的距离值(mm)
//  @note       使用滑动窗口均值滤波，最大30次采样、偏移10mm修正
//-------------------------------------------------------------------------------------------------------------------
uint16_t tof_dl1b_get_mm(void)
{
    // 读取TOF测距距离
    dl1b_get_distance();
    uint16_t new_distance = dl1b_distance_mm;

    // 首次调用时填充整个窗口
    if (!tof_initialized)
    {
        for (uint8_t i = 0; i < WINDOW_SIZE; i++)
        {
            tof_window[i] = new_distance; // 首次采样值填充窗口
            tof_sum += new_distance;
        }
        tof_initialized = 1;
    }
    else
    {
        // 滑动窗口更新
        tof_sum -= tof_window[tof_index];     // 移除最旧的数据
        tof_window[tof_index] = new_distance; // 添加新的数据
        tof_sum += new_distance;              // 更新累加和
    }

    // 更新窗口索引（环形缓冲）
    tof_index = (tof_index + 1) % WINDOW_SIZE;

    // 计算均值并减去偏移
    uint16_t avg_distance = tof_sum / WINDOW_SIZE;
    return (avg_distance >= OFFSET_MM) ? (avg_distance - OFFSET_MM) : 0;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      tof __lw__
////  @param      无
////  @return     无
////  @note       简单均值滤波，10次采样滤波后返回mm
////-------------------------------------------------------------------------------------------------------------------
// uint16 tof_dl1b(void)
//{
//     uint8 i;
//     uint16 temp = 0;
//     for(i = 0 ; i <= 9 ; i++)
//     {
//         dl1b_get_distance();
//         temp += dl1b_distance_mm;
//     }
//     temp /= 10;
//     temp -= 10;
//     return temp;
// }
