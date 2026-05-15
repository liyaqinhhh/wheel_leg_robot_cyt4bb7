/*
 * Buzzer.c
 *
 *  Created on: 2025年5月23日
 *      Author: Administrator
 */
#include "zf_common_headfile.h"
#include "image.h"

#define    BEEP_TIME_convent       100
#define    BEEP_TIME_cross         500
#define    BEEP_TIME_circle        200
#define    BEEP_TIME_zebra         2000

#define    BEEP_FREQ_NORMAL       3000
#define    BEEP_FREQ_ERROR        5000
//#define    BEEP_freq_cross         4000
//#define    BEEP_freq_circle        2000
//#define    BEEP_freq_zebra         1500



uint16 beep_count = 0;


void Buzzer_Control(void)
{
    static uint8 beep_open = 0;
    static uint16 current_freq = 0;
    static uint8 flag = 0;

    if(buzzer_flag)
    {
        beep_open = 1;

        if(buzzer_flag==2)
            current_freq = BEEP_FREQ_ERROR;
        else
            current_freq = BEEP_FREQ_NORMAL;

        if(buzzer_flag == 1)
        {
            switch(Element_State)
            {
                case 0:
                    beep_count++;
                    if(beep_count >= BEEP_TIME_convent)
                    {
                        beep_open = 0;
                        beep_count = 0;
                        buzzer_flag = 0;
                    }break;
                case Cross:
                    beep_count++;
                    if(beep_count >= BEEP_TIME_cross)
                    {
                        beep_open = 0;
                        beep_count = 0;
                        buzzer_flag = 0;
                    }break;
                case L_Circle:
                    beep_count++;
                    if(beep_count >= BEEP_TIME_circle)
                    {
                        beep_open = 0;
                        beep_count = 0;
                        buzzer_flag = 0;
                    }break;
                case R_Circle:
                    beep_count++;
                    if(beep_count >= BEEP_TIME_circle)
                    {
                        beep_open = 0;
                        beep_count = 0;
                        buzzer_flag = 0;
                    }break;
                case Zebra:
                    beep_count++;
                    if(beep_count >= BEEP_TIME_zebra)
                    {
                        beep_open = 0;
                        beep_count = 0;
                        buzzer_flag = 0;
                    }break;
                default:
                    beep_count++;
                    if(beep_count >= BEEP_TIME_convent)
                    {
                        beep_open = 0;
                        beep_count = 0;
                        buzzer_flag = 0;
                    }break;
            }
        }
        else if(buzzer_flag == 2)
        {
            if(flag == 1)
                beep_count = 0;
            beep_count++;
            if(beep_count >= BEEP_TIME_cross)
            {
                beep_open = 0;
                beep_count = 0;
                buzzer_flag = 0;
            }
        }
    }
    else
    {
        beep_open = 0;
        beep_count = 0;
    }

    if(beep_open)
    {
        pwm_init(TCPWM_CH28_P10_0, current_freq, 3000); // 移植：CYT4BB7 无 pwm_set_freq，用 pwm_init 同时设置频率和占空比
    }
    else
    {
        buzzer_flag = 0;
        pwm_set_duty(TCPWM_CH28_P10_0, 0);
    }
    flag = buzzer_flag;

}



////播放速度，值为四分音符的时长(ms)
//#define       SPEED                103
//
////音符与索引对应表，P：休止符，L：低音，M：中音，H：高音，下划线：升半音符号#
//#define P   0
////#define L1  1
//#define L1_1 2
////#define L2  3
//#define L2_1 4
////#define L3  5
////#define L4  6
//#define L4_1 7
////#define L5  8
//#define L5_1 9
//#define L6  10
//#define L6_1 11
//#define L7  12
//#define M1  13
//#define M1_1 14
//#define M2  15
//#define M2_1 16
//#define M3  17
//#define M4  18
//#define M4_1 19
//#define M5  20
//#define M5_1 21
//#define M6  22
//#define M6_1 23
//#define M7  24
//#define H1  25
//#define H1_1 26
//#define H2  27
//#define H2_1 28
//#define H3  29
//#define H4  30
//#define H4_1 31
//#define H5  32
//#define H5_1 33
//#define H6  34
//#define H6_1 35
//#define H7  36
//
//uint16 FreqTable[]={
//    0,
//    63628,63731,63835,63928,64021,64103,64185,64260,64331,64400,64463,64528,
//    64580,64633,64684,64732,64777,64820,64860,64898,64934,64968,65000,65030,
//    65058,65085,65110,65134,65157,65178,65198,65217,65235,65252,65268,65283,
//};
//
//uint16 On_My_Own[]={
////1
//    M4_1, 4,
//    L7 , 4,
//    M2 , 4,
//    M3 , 4,
//    L7 , 4,
//    M2 , 4,
//    M4_1, 4,
//    L7 , 4,
//
//    M2 , 4,
//    M3 , 4,
//    L7 , 4,
//    M2 , 4,
//    M4_1, 4,
//    L7 , 4,
//    M3 , 4,
//    L7 , 4,
////2
//    M4_1, 4,
//    L7 , 4,
//    M2 , 4,
//    M3 , 4,
//    L7 , 4,
//    M2 , 4,
//    M4_1, 4,
//    L7 , 4,
//
//    M2 , 4,
//    M3 , 4,
//    L7 , 4,
//    M2 , 4,
//    M4_1, 4,
//    L7 , 4,
//    M3 , 4,
//    L7 , 4,
////3
//    M4_1, 4,
//    L7 , 4,
//    M2 , 4,
//    M3 , 4,
//    L7 , 4,
//    M2 , 4,
//    M4_1, 4,
//    L7 , 4,
//
//    M2 , 4,
//    M3 , 4,
//    L7 , 4,
//    M2 , 4,
//    M4_1, 4,
//    L7 , 4,
//    M3 , 4,
//    L7 , 4,
////4
//    M2 , 4,
//    L4_1, 4,
//    L6 , 4,
//    M2 , 4,
//    L4_1, 4,
//    L6 , 4,
//    M2 , 4,
//    L4_1, 4,
//
//    M2 , 4,
//    L4_1, 4,
//    L6 , 4,
//    M2 , 4,
//    L4_1, 4,
//    L6 , 4,
//    M2 , 4,
//    L4_1, 4,
//
//    3000
//};
//
//uint16 FreqSelect,MusicSelect;
//uint16 music_count = 0;
//void Music()
//{
//    music_count++;
//    if(On_My_Own[MusicSelect]!=3000)    //如果不是停止标志位
//    {
//        FreqSelect=On_My_Own[MusicSelect];  //选择音符对应的频率
//        pwm_set_duty(TCPWM_CH28_P10_0, (uint32)(65268/65535*5000));
//        if(music_count >= SPEED/4*On_My_Own[MusicSelect+1])
//            MusicSelect += 2;
//    }
//    else    //如果是停止标志位
//    {
//        MusicSelect = 0;
//    }
//}




