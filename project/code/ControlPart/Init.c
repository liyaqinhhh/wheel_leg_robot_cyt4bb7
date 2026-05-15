 /*
     * Init.c
     *
     *  Created on: 2024�??2�??�??
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

    void Init_All(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    // 閫傚綋鐨勫欢鏃跺悗鍦ㄨ繘琛屽垵濮嬪寲
    system_delay_ms(100);

    //    gpio_init(P33_10, GPO, 0, GPO_PUSH_PULL);            // 铚傞福鍣?
    pwm_init(TCPWM_CH28_P10_0, 3000, 0);
    key_init(10); // 鎸夐�?
                  //  wireless_uart_init();                               // 鏃犵嚎涓插彛
                  //    lora3a22_init();
    dl1b_init();
    imu660rb_init();                      // 闄€铻轰�?
                                          //  gyroOffset_init();
    adc_init(ADC0_CH06_P06_6, ADC_12BIT); // 鐢垫�?
   // mt9v03x_init();                       // 鎬婚捇椋?
                                          //    uart_init (WIRELESS_UART_INDEX, WIRELESS_UART_BUAD_RATE, WIRELESS_UART_RX_PIN, WIRELESS_UART_TX_PIN);   // 鍒濆鍖栦覆�??

    // IPS
    ips200_set_font(IPS200_6X8_FONT);             // 璁剧疆瀛椾綋澶у皬涓?6 * 8鍍忕�?
    ips200_set_color(RGB565_BLACK, RGB565_WHITE); // 璁剧疆棰滆壊涓哄僵鑹?
    ips200_set_dir(IPS200_PORTAIT);               // 璁剧疆涓虹珫灞忔樉绀?
    ips200_init(IPS200_TYPE_SPI);                 // 鍙屾帓骞跺彛娆惧�?

    flash_init();
    // Init_Nag();
    //     imu963ra_kalman_filter_init(&imu, 0.000001, 0.1, 4 / 1000.0f);
    imu963ra_kalman_filter_init(&imu, 0.0000003, 0.3, 4 / 1000.0f);

    //    wifi_spi_init_all();
    //    wifi_spi_init("team", "12345qwert",WIFI_SPI_STATION);
    //    detector_init(DETECTOR_WIFI_SPI);

    // 瀹氭椂鍣?
    pit_ms_init(PIT_CH0, 1); // 璁剧疆鍛ㄦ湡涓�?1ms

    // 鍙傛暟鍒濆�??

    imu660ra.offset_angle.pitch = -7.4; // �??5�??.1
    imu660ra.offset_angle.roll = -2.1;  //
    Yao.Target_Speed = 0;
    Yao.Target_height = 3;

    pid_para_init(&PID_all.Pid_Gyro_Pitch);
    pid_para_init(&PID_all.Pid_Angle_Pitch);
    pid_para_init(&PID_all.Pid_Speed_Pitch);
    pid_para_init(&PID_all.Pid_Gyro_Roll);
    pid_para_init(&PID_all.Pid_Angle_Roll);
    pid_para_init(&PID_all.Pid_Gyro_Yaw);
    pid_para_init(&PID_all.Pid_Angle_Yaw);

    // 鑿滃�?
    if (menu_open)
        store_or_read_DATA(READ);
    // system_delay_ms(100);
    //    mt9v03x_init();                                     // 鎬婚捇椋?

    // 鑸垫満PWM鍒濆鍖?
    servo_init();
    EKF_Init();

    // 閫傚綋鐨勫欢鏃跺悗鍦ㄨ繘琛屽垵濮嬪寲
    // system_delay_ms(100);

    // mt9v03x_init();                                     // 鎬婚捇椋?

    // 涓€鍒囧噯澶囧氨�??鏃犲埛椹卞姩鍒濆鍖?
    // Initialize GNSS driver for steering fusion fallback.
    // This does not require waypoint planner yet.
    //gnss_init(TAU1201);

    // Initialize steering fusion placeholders and debug outputs.
    // Subject-1 background default:
    // - no fixed track
    // - pure vision localization + cone avoidance
    //steer_turn_init();

    small_driver_uart_init();

    // cpu_wait_event_ready();
}

#define WINDOW_SIZE 30 // 婊戝姩绐楀彛澶у皬锛屼笌�??0娆″潎鍊间竴鑷?
#define OFFSET_MM 10   // 鍋忕Щ閲忥紝涓庡師鍑芥暟涓€鑷?

// 闈欐€佸彉閲忥紝淇濆瓨婊戝姩绐楀彛鏁版嵁鍜岀储寮?
static uint16_t tof_window[WINDOW_SIZE] = {0}; // 绐楀彛鏁扮粍锛屽垵濮嬪寲�??
static uint8_t tof_index = 0;                  // 褰撳墠绱㈠紩
static uint32_t tof_sum = 0;                   // 绐楀彛鍐呮暟鎹殑鎬诲拰
static uint8_t tof_initialized = 0;            // 鍒濆鍖栨爣�??

//-------------------------------------------------------------------------------------------------------------------
//  @brief      TOF璺濈鑾峰彇锛堟粦鍔ㄧ獥鍙ｆ护娉級
//  @param      �??
//  @return     婊ゆ尝鍚庣殑璺濈锛堝崟浣嶏細mm�??
//  @note       浣跨敤婊戝姩绐楀彛婊ゆ尝锛岀獥鍙ｅぇ�??0锛屽噺鍘?0mm鍋忕Щ
//-------------------------------------------------------------------------------------------------------------------
uint16_t tof_dl1b_get_mm(void)
{
    // 鑾峰彇鏂拌窛绂绘暟鎹?
    dl1b_get_distance();
    uint16_t new_distance = dl1b_distance_mm;

    // 濡傛灉绐楀彛鏈垵濮嬪寲锛屽～鍏呭垵濮嬫暟�??
    if (!tof_initialized)
    {
        for (uint8_t i = 0; i < WINDOW_SIZE; i++)
        {
            tof_window[i] = new_distance; // 鐀��娆℃祴閲忓€煎～鍏?
            tof_sum += new_distance;
        }
        tof_initialized = 1;
    }
    else
    {
        // 鏇存柊婊戝姩绐楀�?
        tof_sum -= tof_window[tof_index];     // 绉婚櫎鏈€鏃ф暟鎹?
        tof_window[tof_index] = new_distance; // 娣诲姞鏂版暟�??
        tof_sum += new_distance;              // 鏇存柊鎬诲拰
    }

    // 鏇存柊绱㈠紩锛堝惊鐜娇鐀��獥鍙ｏ級
    tof_index = (tof_index + 1) % WINDOW_SIZE;

    // 璁＄畻骞冲潎鍊煎苟鍑忓幓鍋忕Щ
    uint16_t avg_distance = tof_sum / WINDOW_SIZE;
    return (avg_distance >= OFFSET_MM) ? (avg_distance - OFFSET_MM) : 0;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      tof __lw__
////  @param      �??
////  @return     �??
////  @note       鎺ユ敹鏁版嵁鍔犱�?10娆″潎鍊兼护娉?鍗曚綅锛歮m
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
