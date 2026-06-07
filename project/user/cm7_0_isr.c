/*********************************************************************************************************************
* CYT4BB 开源库（CYT4BB Opensource Library），一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅 <https://www.gnu.org/licenses/>
*
* 文件名称：cm7_0_isr
* 公司名称：成都逐飞科技有限公司
* 版本信息：查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境：IAR 9.40.1
* 适用平台：CYT4BB
* 店铺链接：https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-9      pudding            first version
* 2024-5-14     pudding            增加12路PIT定时中断，增加参数注释说明
* 2025-2-4      pudding            优化中断处理逻辑
* 2025-2-4      pudding            增加串口接口
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "Interrupt.h"      // 引入从 TC264 移植过来的中断处理函数声明
#include "small_driver_uart_control.h"


// **************************** 定时器中断函数 ****************************
void pit0_ch0_isr()                     // 定时器通道 0 中断处理函数（1ms）
{
    pit_isr_flag_clear(PIT_CH0);

    // 移植说明：原 TC264 使用 IFX_INTERRUPT 宏注册中断，CYT4BB7 直接在此函数体内调用
    // 对应 TC264 的 cc60_pit_ch0_isr，产生 1ms 节拍，分频出 2/4/8/16/40ms 任务

    static uint8 Count_2ms  = 0;
    static uint8 Count_4ms  = 0;
    static uint8 Count_8ms  = 0;
    static uint8 Count_16ms = 0;
    static uint8 Count_40ms = 0;
    

    Count_2ms++;
    Count_4ms++;
    Count_8ms++;
    Count_16ms++;
    Count_40ms++;

    Interrupt_1ms();

    if(Count_2ms == 2){
        Count_2ms = 0;
        Interrupt_2ms();
       
    }
    if(Count_4ms == 4){
        Count_4ms = 0;
        Interrupt_4ms();
    }
    if(Count_8ms == 8){
        Count_8ms = 0;
        Interrupt_8ms();
    }
    if(Count_16ms == 16){
        Count_16ms = 0;
        Interrupt_16ms();
    }
    if(Count_40ms == 40){
        Count_40ms = 0;
        Interrupt_40ms();
    }
}

void pit0_ch1_isr()                     // 定时器通道 1 中断处理函数
{
    pit_isr_flag_clear(PIT_CH1);

}

void pit0_ch2_isr()                     // 定时器通道 2 中断处理函数
{
    pit_isr_flag_clear(PIT_CH2);

}

void pit0_ch10_isr()                    // 定时器通道 10 中断处理函数
{
    pit_isr_flag_clear(PIT_CH10);

}

void pit0_ch11_isr()                    // 定时器通道 11 中断处理函数
{
    pit_isr_flag_clear(PIT_CH11);

}

void pit0_ch12_isr()                    // 定时器通道 12 中断处理函数
{
    pit_isr_flag_clear(PIT_CH12);

}

void pit0_ch13_isr()                    // 定时器通道 13 中断处理函数
{
    pit_isr_flag_clear(PIT_CH13);

}

void pit0_ch14_isr()                    // 定时器通道 14 中断处理函数
{
    pit_isr_flag_clear(PIT_CH14);

}

void pit0_ch15_isr()                    // 定时器通道 15 中断处理函数
{
    pit_isr_flag_clear(PIT_CH15);

}

void pit0_ch16_isr()                    // 定时器通道 16 中断处理函数
{
    pit_isr_flag_clear(PIT_CH16);

}

void pit0_ch17_isr()                    // 定时器通道 17 中断处理函数
{
    pit_isr_flag_clear(PIT_CH17);

}

void pit0_ch18_isr()                    // 定时器通道 18 中断处理函数
{
    pit_isr_flag_clear(PIT_CH18);

}

void pit0_ch19_isr()                    // 定时器通道 19 中断处理函数
{
    pit_isr_flag_clear(PIT_CH19);

}

void pit0_ch20_isr()                    // 定时器通道 20 中断处理函数
{
    pit_isr_flag_clear(PIT_CH20);

}

void pit0_ch21_isr()                    // 定时器通道 21 中断处理函数
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}
// **************************** 定时器中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0 默认作为调试串口
void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))            // 串口0 接收中断
    {

#if DEBUG_UART_USE_INTERRUPT             // 如果启用 debug 串口中断
        debug_interrupr_handler();       // 调用 debug 串口接收处理函数，数据会被 debug 模块缓冲区接收
#endif                                   // 若修改了 DEBUG_UART_INDEX 宏，则需要将该处理放到对应的串口中断里

    }
    else                                 // 串口0 发送中断
    {



    }
}

void uart1_isr (void)
{
    if(uart_isr_mask(UART_1))            // 串口1 接收中断
    {
        wireless_module_uart_handler();  // 无线模块统一回调函数

    }
    else                                 // 串口1 发送中断
    {



    }
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))            // 串口2 接收中断
    {
        //gnss_uart_callback();            // GPS 模块回调函数
        uart_control_callback();
    }
    else                                 // 串口2 发送中断
    {



    }
}

void uart3_isr (void)
{
    if(uart_isr_mask(UART_3))            // 串口3 接收中断
    {
        // 移植说明：对应 TC264 的 uart2_rx_isr 中的 uart_control_callback()
        // SMALL_DRIVER_UART = UART_3（定义在 small_driver_uart_control.h）
        

    }
    else                                 // 串口3 发送中断
    {



    }
}

void uart4_isr (void)
{
    //num_1++;
    if(uart_isr_mask(UART_4))            // 串口4 接收中断
    {
        //uart_receiver_handler();         // 串口接收缓冲回调函数
        //uart_control_callback();
    }
    else                                 // 串口4 发送中断
    {



    }
}

void uart5_isr (void)
{
    if(uart_isr_mask(UART_5))            // 串口5 接收中断
    {



    }
    else                                 // 串口5 发送中断
    {



    }
}

void uart6_isr (void)
{
    if(uart_isr_mask(UART_6))            // 串口6 接收中断
    {



    }
    else                                 // 串口6 发送中断
    {



    }
}
// **************************** 串口中断函数 ****************************

// **************************** 外部中断函数 ****************************
void gpio_0_exti_isr()                  // 外部 GPIO_0 中断处理函数
{



}

void gpio_1_exti_isr()                  // 外部 GPIO_1 中断处理函数
{
    if(exti_flag_get(P01_0))            // P01_0 外部中断判断
    {



    }
    if(exti_flag_get(P01_1))            // P01_1 外部中断判断
    {



    }
}

void gpio_2_exti_isr()                  // 外部 GPIO_2 中断处理函数
{
    if(exti_flag_get(P02_0))
    {


    }
    if(exti_flag_get(P02_4))
    {


    }

}

void gpio_3_exti_isr()                  // 外部 GPIO_3 中断处理函数
{



}

void gpio_4_exti_isr()                  // 外部 GPIO_4 中断处理函数
{



}

void gpio_5_exti_isr()                  // 外部 GPIO_5 中断处理函数
{



}

void gpio_6_exti_isr()                  // 外部 GPIO_6 中断处理函数
{



}

void gpio_7_exti_isr()                  // 外部 GPIO_7 中断处理函数
{



}

void gpio_8_exti_isr()                  // 外部 GPIO_8 中断处理函数
{



}

void gpio_9_exti_isr()                  // 外部 GPIO_9 中断处理函数
{



}

void gpio_10_exti_isr()                 // 外部 GPIO_10 中断处理函数
{



}

void gpio_11_exti_isr()                 // 外部 GPIO_11 中断处理函数
{



}

void gpio_12_exti_isr()                 // 外部 GPIO_12 中断处理函数
{



}

void gpio_13_exti_isr()                 // 外部 GPIO_13 中断处理函数
{



}

void gpio_14_exti_isr()                 // 外部 GPIO_14 中断处理函数
{



}

void gpio_15_exti_isr()                 // 外部 GPIO_15 中断处理函数
{



}

void gpio_16_exti_isr()                 // 外部 GPIO_16 中断处理函数
{



}

void gpio_17_exti_isr()                 // 外部 GPIO_17 中断处理函数
{



}

void gpio_18_exti_isr()                 // 外部 GPIO_18 中断处理函数
{



}

void gpio_19_exti_isr()                 // 外部 GPIO_19 中断处理函数
{



}

void gpio_20_exti_isr()                 // 外部 GPIO_20 中断处理函数
{



}

void gpio_21_exti_isr()                 // 外部 GPIO_21 中断处理函数
{



}

void gpio_22_exti_isr()                 // 外部 GPIO_22 中断处理函数
{



}

void gpio_23_exti_isr()                 // 外部 GPIO_23 中断处理函数
{



}
// **************************** 外部中断函数 ****************************
