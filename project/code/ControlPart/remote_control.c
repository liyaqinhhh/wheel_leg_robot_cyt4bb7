/**
 * @file    remote_control.c
 * @brief   遥控器模块 -- 二进制帧协议解析 + 超时安全检测
 *
 * 通过 UART_1 无线透传模块接收 PCB 遥控器发送的控制帧，
 * 解析后映射到机器人控制变量 (速度/转向/模式/高度/急停)。
 *
 * 协议格式: [0x5A] [len] [type] [data...] [checksum]
 *   len    = 1(type) + len(data) + 1(checksum)
 *   checksum = len XOR type XOR data[0] XOR ... XOR data[n-1]
 *
 * 帧头识别分发:
 *   0x5A → 遥控帧 (本模块解析)
 *   'P'  → AI调参帧 (转发给 AI_Pid_Tuner_ProcessRx)
 *   其他 → 丢弃
 */

#include "zf_common_headfile.h"
#include "remote_control.h"
#include "yaokong.h"
#include "Interrupt.h"
#include "image.h"
#include "zf_device_wireless_uart.h"
#include "AI_Pid_Tuner.h"

/* ==================== 内部状态 ==================== */

static RemoteControl_t rc_data;            /* 遥控状态数据 */
static RC_ParserState  rc_state;           /* 解析器状态 */
static uint8_t         rc_rx_byte;         /* 当前接收字节 */
static uint8_t         rc_frame_len;       /* 当前帧长度字段 */
static uint8_t         rc_frame_type;      /* 当前帧类型字段 */
static uint8_t         rc_data_buf[RC_FRAME_MAX_LEN]; /* 数据缓冲区 */
static uint8_t         rc_data_idx;        /* 数据缓冲区写入索引 */
static uint8_t         rc_checksum_calc;   /* 计算中的校验值 */

/* 超时计数器 (以40ms为单位, 500ms/40ms ≈ 13) */
#define RC_TIMEOUT_COUNTS   13
static uint8_t rc_timeout_counter = 0;

/* ==================== 内部函数声明 ==================== */

static void rc_parser_reset(void);
static void rc_parser_feed(uint8_t byte);
static void rc_process_frame(void);
static void rc_handle_joystick(const uint8_t *data);
static void rc_handle_mode_switch(const uint8_t *data);
static void rc_handle_height(const uint8_t *data);
static void rc_handle_emergency(const uint8_t *data);
static void rc_handle_heartbeat(void);
static uint8_t rc_is_valid_turn_mode(uint8_t mode);

/* ==================== API 实现 ==================== */

/**
 * @brief  初始化遥控模块
 * @note   清零状态，复位解析器，应在 wireless_uart_init() 之后调用
 */
void remote_control_init(void)
{
    uint8_t i;

    /* 清零遥控状态 */
    for (i = 0; i < 4; i++)
        rc_data.joystick[i] = 0;
    rc_data.active         = 0;
    rc_data.last_rx_tick   = 0;
    rc_data.emergency_stop = 0;
    rc_data.frame_count    = 0;
    rc_data.error_count    = 0;

    /* 复位解析器 */
    rc_parser_reset();

    /* 复位超时计数器 */
    rc_timeout_counter = 0;
}

/**
 * @brief  处理 UART_1 接收数据
 * @note   在 Interrupt_40ms() 中调用
 *         读取 FIFO → 帧头识别分发 → 解析 → 映射
 *         0x5A → 遥控帧解析, 'P' → AI调参处理, 其他 → 丢弃
 */
void remote_control_process(void)
{
    uint8_t ch;
    uint32_t read_len;

    /* ---- 超时检测 ---- */
    if (rc_data.active)
    {
        rc_timeout_counter++;
        if (rc_timeout_counter >= RC_TIMEOUT_COUNTS)
        {
            /* 超时: 清零控制量, 触发安全停车 */
            rc_data.active = 0;
            Yao.Target_Speed = 0;
            Deviation_Value  = 0;
            rc_data.emergency_stop = 0;
            rc_timeout_counter = 0;
        }
    }

    /* ---- 逐字节读取 FIFO ---- */
    while (1)
    {
        read_len = wireless_uart_read_buffer(&ch, 1);
        if (read_len == 0)
            break;

        /* ---- 帧头识别分发 ---- */
        if (rc_state == RC_STATE_WAIT_HEADER)
        {
            if (ch == RC_FRAME_HEADER)
            {
                /* 0x5A → 遥控帧，进入状态机解析 */
                rc_parser_feed(ch);
            }
            else if (ch == 'P')
            {
                /* 'P' → AI调参帧，转发给AI调参模块 */
                AI_Pid_Tuner_ProcessRx();
            }
            /* 其他字节 → 丢弃 */
        }
        else
        {
            /* 解析器正在处理遥控帧，继续喂入 */
            rc_parser_feed(ch);
        }
    }
}

/**
 * @brief  查询遥控是否激活 (在超时窗口内)
 * @return 1=激活, 0=超时/未连接
 */
uint8_t remote_control_is_active(void)
{
    return rc_data.active;
}

/**
 * @brief  获取遥控状态数据 (调试用)
 * @return 指向内部 RemoteControl_t 的指针
 */
RemoteControl_t* remote_control_get_data(void)
{
    return &rc_data;
}

/* ==================== 解析器实现 ==================== */

/**
 * @brief  复位解析器状态
 */
static void rc_parser_reset(void)
{
    rc_state       = RC_STATE_WAIT_HEADER;
    rc_frame_len   = 0;
    rc_frame_type  = 0;
    rc_data_idx    = 0;
    rc_checksum_calc = 0;
}

/**
 * @brief  向解析器喂入一个字节
 * @param  byte 接收到的字节
 */
static void rc_parser_feed(uint8_t byte)
{
    switch (rc_state)
    {
    case RC_STATE_WAIT_HEADER:
        if (byte == RC_FRAME_HEADER)
        {
            rc_checksum_calc = 0;  /* 校验从len开始计算，header不参与 */
            rc_state = RC_STATE_WAIT_LENGTH;
        }
        break;

    case RC_STATE_WAIT_LENGTH:
        rc_frame_len = byte;
        rc_checksum_calc = byte;  /* len 参与校验 */
        if (rc_frame_len < RC_FRAME_MIN_LEN - 1 || rc_frame_len > RC_FRAME_MAX_LEN)
        {
            /* 长度异常，复位 */
            rc_data.error_count++;
            rc_parser_reset();
        }
        else
        {
            rc_state = RC_STATE_WAIT_TYPE;
        }
        break;

    case RC_STATE_WAIT_TYPE:
        rc_frame_type = byte;
        rc_checksum_calc ^= byte;  /* type 参与校验 */
        /* 计算数据长度 = len - 1(type) - 1(checksum) */
        {
            uint8_t data_len = rc_frame_len - 2;
            if (data_len == 0)
            {
                /* 无数据帧 (如心跳)，直接等待校验 */
                rc_state = RC_STATE_WAIT_CHECKSUM;
            }
            else
            {
                rc_data_idx = 0;
                rc_state = RC_STATE_WAIT_DATA;
            }
        }
        break;

    case RC_STATE_WAIT_DATA:
        rc_data_buf[rc_data_idx++] = byte;
        rc_checksum_calc ^= byte;  /* data 参与校验 */
        if (rc_data_idx >= (rc_frame_len - 2))
        {
            /* 数据接收完毕，等待校验 */
            rc_state = RC_STATE_WAIT_CHECKSUM;
        }
        break;

    case RC_STATE_WAIT_CHECKSUM:
        if (byte == (rc_checksum_calc & 0xFF))
        {
            /* 校验通过，处理帧 */
            rc_process_frame();
        }
        else
        {
            /* 校验失败 */
            rc_data.error_count++;
        }
        rc_parser_reset();
        break;

    default:
        rc_parser_reset();
        break;
    }
}

/**
 * @brief  处理已解析的完整帧
 */
static void rc_process_frame(void)
{
    /* 刷新超时计数器 */
    rc_timeout_counter = 0;
    rc_data.active = 1;
    rc_data.frame_count++;

    /* 按类型分发 */
    switch (rc_frame_type)
    {
    case RC_TYPE_JOYSTICK:
        rc_handle_joystick(rc_data_buf);
        break;

    case RC_TYPE_MODE_SWITCH:
        rc_handle_mode_switch(rc_data_buf);
        break;

    case RC_TYPE_HEIGHT:
        rc_handle_height(rc_data_buf);
        break;

    case RC_TYPE_EMERGENCY:
        rc_handle_emergency(rc_data_buf);
        break;

    case RC_TYPE_HEARTBEAT:
        rc_handle_heartbeat();
        break;

    default:
        /* 未知类型，忽略 */
        rc_data.error_count++;
        break;
    }
}

/* ==================== 帧处理函数 ==================== */

/**
 * @brief  处理摇杆帧 (type=0x01, data=4×int16=8B)
 * @param  data 指向8字节的摇杆数据
 *
 * 映射逻辑 (与原 yaokong_data_deal 一致):
 *   joystick[0] → 前后速度 (死区±200, 满值1000)
 *   joystick[1] → 左右转向 (死区±100, 限幅±1.0)
 */
static void rc_handle_joystick(const uint8_t *data)
{
    int16_t j0, j1;

    /* 解析4个int16摇杆值 */
    j0 = (int16_t)((uint16_t)data[1] << 8 | data[0]);  /* joystick[0] 前后 */
    j1 = (int16_t)((uint16_t)data[3] << 8 | data[2]);  /* joystick[1] 左右 */
    /* joystick[2]=data[5:4], joystick[3]=data[7:6] 预留，暂不使用 */

    rc_data.joystick[0] = j0;
    rc_data.joystick[1] = j1;

    /* 调用解耦后的摇杆映射函数 */
    yaokong_map_joystick(j0, j1);
}

/**
 * @brief  处理模式切换帧 (type=0x02, data=1B turn_mode)
 * @param  data 指向1字节的模式数据
 */
static void rc_handle_mode_switch(const uint8_t *data)
{
    uint8_t mode = data[0];
    if (rc_is_valid_turn_mode(mode))
    {
        turn_mode = mode;
    }
}

/**
 * @brief  处理高度调节帧 (type=0x03, data=1B direction)
 * @param  data 指向1字节的方向数据
 */
static void rc_handle_height(const uint8_t *data)
{
    switch (data[0])
    {
    case RC_HEIGHT_UP:
        Yao.Target_height += 0.5f;
        break;
    case RC_HEIGHT_DOWN:
        Yao.Target_height -= 0.5f;
        break;
    case RC_HEIGHT_HOLD:
    default:
        break;
    }
}

/**
 * @brief  处理急停帧 (type=0x04, data=1B stop)
 * @param  data 指向1字节的急停数据
 */
static void rc_handle_emergency(const uint8_t *data)
{
    if (data[0] == RC_EMERGENCY_STOP)
    {
        rc_data.emergency_stop = 1;
        flag_stop = 1;
        Yao.Target_Speed = 0;
        Deviation_Value  = 0;
    }
    else if (data[0] == RC_EMERGENCY_RESUME)
    {
        rc_data.emergency_stop = 0;
        flag_stop = 0;
    }
}

/**
 * @brief  处理心跳帧 (type=0x05, 无data)
 */
static void rc_handle_heartbeat(void)
{
    /* 心跳帧仅刷新超时计时器，已在 rc_process_frame() 中完成 */
}

/**
 * @brief  校验 turn_mode 是否有效
 * @param  mode 待校验的模式值
 * @return 1=有效, 0=无效
 *
 * 有效值: 0(关闭), 2(串级), 3(偏航闭环), 6(原地旋转)
 */
static uint8_t rc_is_valid_turn_mode(uint8_t mode)
{
    switch (mode)
    {
    case 0:  /* 关闭 */
    case 2:  /* 串级转向 */
    case 3:  /* 偏航角度闭环走直线 */
    case 6:  /* 原地旋转 */
        return 1;
    default:
        return 0;
    }
}
