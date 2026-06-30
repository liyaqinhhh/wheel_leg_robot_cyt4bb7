/**
 * @file    remote_control.h
 * @brief   遥控器模块头文件 -- 二进制帧协议解析 + 超时安全检测
 *
 * 通过 UART_1 无线透传模块接收 PCB 遥控器发送的控制帧，
 * 解析后映射到机器人控制变量 (速度/转向/模式/高度/急停)。
 *
 * 协议格式: [0x5A] [len] [type] [data...] [checksum]
 * 帧头 0x5A 区别于遥测 '$'=0x24 和 AI调参 'P'=0x50
 */

#ifndef REMOTE_CONTROL_H_
#define REMOTE_CONTROL_H_

#include "zf_common_typedef.h"

/* ==================== 帧协议常量 ==================== */

#define RC_FRAME_HEADER         0x5A    /* 帧头标识 */
#define RC_FRAME_MIN_LEN        4       /* 最小帧长度: header+len+type+checksum */
#define RC_FRAME_MAX_LEN        32      /* 最大帧长度 */
#define RC_TIMEOUT_MS           500     /* 遥控信号超时阈值 (ms) */

/* 命令类型定义 */
#define RC_TYPE_JOYSTICK        0x01    /* 摇杆数据, data=4×int16 (8B) */
#define RC_TYPE_MODE_SWITCH     0x02    /* 模式切换, data=uint8 turn_mode (1B) */
#define RC_TYPE_HEIGHT          0x03    /* 高度调节, data=uint8 dir (1B) */
#define RC_TYPE_EMERGENCY       0x04    /* 急停控制, data=uint8 stop (1B) */
#define RC_TYPE_HEARTBEAT       0x05    /* 心跳包, 无data (0B) */

/* 高度调节方向 */
#define RC_HEIGHT_HOLD          0       /* 保持当前高度 */
#define RC_HEIGHT_UP            1       /* 增高 */
#define RC_HEIGHT_DOWN          2       /* 降低 */

/* 急停控制值 */
#define RC_EMERGENCY_STOP       1       /* 急停 */
#define RC_EMERGENCY_RESUME     0       /* 恢复 */

/* 有效的 turn_mode 值集合 */
#define RC_VALID_TURN_MODES     0x0B    /* 位掩码: bit0=0, bit1=2, bit2=3, bit3=6 */

/* ==================== 解析器状态 ==================== */

typedef enum {
    RC_STATE_WAIT_HEADER = 0,   /* 等待帧头 0x5A */
    RC_STATE_WAIT_LENGTH,       /* 等待长度字节 */
    RC_STATE_WAIT_TYPE,         /* 等待类型字节 */
    RC_STATE_WAIT_DATA,         /* 等待数据字节 */
    RC_STATE_WAIT_CHECKSUM      /* 等待校验字节 */
} RC_ParserState;

/* ==================== 遥控状态结构体 ==================== */

typedef struct {
    int16_t  joystick[4];       /* 原始摇杆值 (来自遥控帧 type=0x01) */
    uint8_t  active;            /* 遥控激活标志: 1=在超时窗口内, 0=超时 */
    uint32_t last_rx_tick;      /* 最后收到有效帧的系统tick (ms) */
    uint8_t  emergency_stop;    /* 急停请求: 1=急停, 0=正常 */
    uint32_t frame_count;       /* 有效帧计数 (调试用) */
    uint32_t error_count;       /* 校验错误计数 (调试用) */
} RemoteControl_t;

/* ==================== API 接口 ==================== */

/**
 * @brief  初始化遥控模块
 * @note   清零状态，复位解析器，应在 wireless_uart_init() 之后调用
 */
void remote_control_init(void);

/**
 * @brief  处理 UART_1 接收数据
 * @note   在 Interrupt_40ms() 中调用，读取 FIFO → 帧头分发 → 解析 → 映射
 *         0x5A → 遥控帧解析, 'P' → AI调参处理, 其他 → 丢弃
 */
void remote_control_process(void);

/**
 * @brief  查询遥控是否激活 (在超时窗口内)
 * @return 1=激活, 0=超时/未连接
 */
uint8_t remote_control_is_active(void);

/**
 * @brief  获取遥控状态数据 (调试用)
 * @return 指向内部 RemoteControl_t 的指针
 */
RemoteControl_t* remote_control_get_data(void);

#endif /* REMOTE_CONTROL_H_ */
