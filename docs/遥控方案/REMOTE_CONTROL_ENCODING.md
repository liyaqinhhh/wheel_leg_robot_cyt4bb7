# 遥控功能编码协议与实现说明

## 1. 通信物理层

| 参数 | 值 | 说明 |
|------|-----|------|
| 接口 | UART_1 (P04_0 RX / P04_1 TX) | 与无线透传模块共用 |
| 波尔率 | 115200 | 与 wireless_uart 一致 |
| RTS 流控 | P22_6 | wireless_uart 已配置 |
| FIFO 深度 | 64 字节 | `WIRELESS_UART_BUFFER_SIZE` |
| 物理互斥 | 无线串口模块 / 遥控器无线模块 | 不会同时接入，插哪个用哪个 |

## 2. 二进制帧协议

### 2.1 帧格式

```
[Header] [Length] [Type] [Data...] [Checksum]
 0x5A     len     type   N字节      XOR校验
```

| 字段 | 偏移 | 长度 | 说明 |
|------|------|------|------|
| Header | 0 | 1B | 固定 `0x5A`（区别于遥测帧头 `'$'` = 0x24） |
| Length | 1 | 1B | 帧总长度（含 Header 和 Checksum） |
| Type | 2 | 1B | 命令类型编号 |
| Data | 3 | 变长 | 命令数据（小端序 int16） |
| Checksum | len-1 | 1B | `Length XOR Type XOR Data[0] XOR ... XOR Data[N-1]`（Header **不参与**校验） |

### 2.2 帧类型定义

| Type | 名称 | Data 长度 | Data 格式 | 控制输出 |
|------|------|----------|----------|---------|
| `0x01` | 摇杆数据 | 8B | 4×int16 小端: `[j0_L][j0_H][j1_L][j1_H][j2_L][j2_H][j3_L][j3_H]` | → `yaokong_map_joystick(j0, j1)` |
| `0x02` | 模式切换 | 1B | uint8 `turn_mode` | → `turn_mode`（仅允许 0/2/3/6） |
| `0x03` | 高度调节 | 1B | uint8 `dir`: 0=保持, 1=增高, 2=降低 | → `Yao.Target_height` ±0.5cm |
| `0x04` | 急停控制 | 1B | uint8 `stop`: 1=急停, 0=恢复 | → `flag_stop` |
| `0x05` | 心跳包 | 0B | 无数据 | → 刷新超时计时器 |

### 2.3 帧长度计算

| 帧类型 | 总长度 (Length 字段值) | 计算公式 |
|--------|----------------------|---------|
| 摇杆 0x01 | 12 (0x0C) | 1(header) + 1(len) + 1(type) + 8(data) + 1(chk) |
| 模式 0x02 | 6 | 1 + 1 + 1 + 1 + 1 |
| 高度 0x03 | 6 | 1 + 1 + 1 + 1 + 1 |
| 急停 0x04 | 6 | 1 + 1 + 1 + 1 + 1 |
| 心跳 0x05 | 5 | 1 + 1 + 1 + 0 + 1 |

### 2.4 校验和计算

```c
// 校验和 = Length XOR Type XOR Data[0] XOR ... XOR Data[N-1]
// Header (0x5A) 不参与校验
uint8_t checksum = frame_len ^ frame_type;
for (int i = 0; i < data_len; i++) {
    checksum ^= data[i];
}
```

### 2.5 帧示例

**摇杆帧** (joystick[0]=500, joystick[1]=-300, joystick[2]=0, joystick[3]=0):
```
5A 0C 01 F4 01 D4 FE 00 00 00 00 [chk]
│  │  │  └─┘ └─┘ └─┘ └─┘ └─┘ └─┘
│  │  │  j0=500  j1=-300 j2=0  j3=0
│  │  type=0x01
│  len=12
header=0x5A
```
- `chk = 0x0C ^ 0x01 ^ 0xF4 ^ 0x01 ^ 0xD4 ^ 0xFE ^ 0x00 ^ 0x00 ^ 0x00 ^ 0x00 = 0x2A`

**模式切换帧** (turn_mode=3):
```
5A 06 02 03 [chk]
│  │  │  │
│  │  │  turn_mode=3
│  │  type=0x02
│  len=6
header=0x5A
```
- `chk = 0x06 ^ 0x02 ^ 0x03 = 0x07`

**急停帧** (stop=1):
```
5A 06 04 01 [chk]
```
- `chk = 0x06 ^ 0x04 ^ 0x01 = 0x03`

**心跳帧**:
```
5A 05 05 [chk]
```
- `chk = 0x05 ^ 0x05 = 0x00`

## 3. 帧解析状态机

### 3.1 状态定义

```c
typedef enum {
    RC_PARSER_WAIT_HEADER    = 0,  // 等待帧头 0x5A
    RC_PARSER_WAIT_LENGTH    = 1,  // 等待长度字节
    RC_PARSER_WAIT_TYPE      = 2,  // 等待类型字节
    RC_PARSER_WAIT_DATA      = 3,  // 等待数据字节
    RC_PARSER_WAIT_CHECKSUM  = 4   // 等待校验和字节
} RC_ParserState;
```

### 3.2 状态转换

```
WAIT_HEADER ──收到0x5A──→ WAIT_LENGTH
WAIT_HEADER ──收到其他──→ WAIT_HEADER (丢弃)

WAIT_LENGTH ──存储len──→ WAIT_TYPE
WAIT_LENGTH ──len异常──→ WAIT_HEADER (复位)

WAIT_TYPE ──存储type──→ WAIT_DATA (计算data_len = len - 4)
WAIT_TYPE ──data_len<0──→ WAIT_HEADER (复位)

WAIT_DATA ──收齐N字节──→ WAIT_CHECKSUM
WAIT_DATA ──溢出──→ WAIT_HEADER (复位)

WAIT_CHECKSUM ──校验通过──→ 处理帧 → WAIT_HEADER
WAIT_CHECKSUM ──校验失败──→ WAIT_HEADER (丢弃)
```

### 3.3 解析器内部变量

| 变量 | 类型 | 说明 |
|------|------|------|
| `rc_state` | `RC_ParserState` | 当前状态 |
| `rc_frame_len` | `uint8_t` | 帧总长度 |
| `rc_frame_type` | `uint8_t` | 帧类型 |
| `rc_data_buf[RC_FRAME_MAX_LEN]` | `uint8_t[32]` | 数据缓冲区 |
| `rc_data_idx` | `uint8_t` | 数据写入索引 |
| `rc_checksum_calc` | `uint8_t` | 累积校验和 |

## 4. UART_1 接收分发

`remote_control_process()` 在 `Interrupt_40ms()` 中被调用，从 FIFO 逐字节读取并按帧头分发：

```
wireless_uart_read_buffer() → 逐字节读取
    ├── 字节 == 0x5A → 送入遥控帧解析器 (rc_parser_feed)
    ├── 字节 == 'P'  → 调用 AI_Pid_Tuner_ProcessRx('P')
    └── 其他         → 丢弃
```

**关键**: `remote_control_process()` 是 UART_1 接收的**唯一入口**，AI调参不再自行读 FIFO。

## 5. 各帧类型处理逻辑

### 5.1 摇杆帧 (Type=0x01) → `rc_handle_joystick()`

```c
// 小端序解析: int16 = data[高字节] << 8 | data[低字节]
int16_t j0 = (int16_t)((rc_data_buf[1] << 8) | rc_data_buf[0]);
int16_t j1 = (int16_t)((rc_data_buf[3] << 8) | rc_data_buf[2]);
int16_t j2 = (int16_t)((rc_data_buf[5] << 8) | rc_data_buf[4]);
int16_t j3 = (int16_t)((rc_data_buf[7] << 8) | rc_data_buf[6]);

// 存储原始值
rc_data.joystick[0] = j0;
rc_data.joystick[1] = j1;
rc_data.joystick[2] = j2;
rc_data.joystick[3] = j3;

// 映射到控制量 (仅使用 j0, j1)
yaokong_map_joystick(j0, j1);
```

### 5.2 模式切换帧 (Type=0x02) → `rc_handle_mode_switch()`

```c
uint8_t mode = rc_data_buf[0];
// 仅允许 turn_mode ∈ {0, 2, 3, 6}
switch (mode) {
    case 0: case 2: case 3: case 6:
        turn_mode = mode;  // 有效，更新
        break;
    default:
        // 无效值，忽略，保持当前模式
        break;
}
```

**有效 turn_mode 值**:

| 值 | 模式 |
|----|------|
| 0 | 平衡模式 |
| 2 | 转向模式2 |
| 3 | 转向模式3 |
| 6 | 转向模式6 |

### 5.3 高度调节帧 (Type=0x03) → `rc_handle_height()`

```c
uint8_t dir = rc_data_buf[0];
switch (dir) {
    case RC_HEIGHT_UP:     // 1
        Yao.Target_height += 0.5;  // 增高 0.5cm
        break;
    case RC_HEIGHT_DOWN:   // 2
        Yao.Target_height -= 0.5;  // 降低 0.5cm
        break;
    case RC_HEIGHT_HOLD:   // 0
    default:
        break;  // 保持当前高度
}
```

### 5.4 急停控制帧 (Type=0x04) → `rc_handle_emergency()`

```c
uint8_t stop = rc_data_buf[0];
if (stop == RC_EMERGENCY_STOP) {    // 1
    flag_stop = 1;
    Yao.Target_Speed = 0;
    Deviation_Value = 0;
    rc_data.emergency_stop = 1;
} else {                             // 0
    flag_stop = 0;
    rc_data.emergency_stop = 0;
}
```

### 5.5 心跳帧 (Type=0x05) → `rc_handle_heartbeat()`

```c
// 无数据处理，仅刷新超时计时器
// (超时刷新已在 rc_process_frame 中统一完成)
```

## 6. 摇杆→控制量映射 (`yaokong_map_joystick`)

### 6.1 速度映射 (joystick_1 → Yao.Target_Speed)

```
输入范围: int16_t, 典型值 ±1000

if (joystick_1 > 200)          → Yao.Target_Speed = 1000     (满速前进)
else if (joystick_1 < -200)    → Yao.Target_Speed = 0.2 × joystick_1  (比例后退)
else                           → Yao.Target_Speed = 0         (死区)
```

| joystick_1 值 | Target_Speed | 说明 |
|--------------|-------------|------|
| 1000 | 1000 | 满速前进 |
| 500 | 1000 | 满速前进（超过200即满速） |
| 200 | 0 | 死区边界 |
| 0 | 0 | 死区 |
| -200 | 0 | 死区边界 |
| -500 | -100 | 比例后退 (0.2 × -500) |
| -1000 | -200 | 比例后退 (0.2 × -1000) |

### 6.2 转向映射 (joystick_2 → Deviation_Value)

```
输入范围: int16_t, 典型值 ±1800

if (|joystick_2| > 100)        → Deviation_Value = -joystick_2 / 1800.0
else                           → Deviation_Value = 0         (死区)

Deviation_Value = func_limit_ab(Deviation_Value, -1.0, 1.0)  // 限幅 ±1.0
```

| joystick_2 值 | Deviation_Value | 说明 |
|--------------|----------------|------|
| 1800 | -1.0 | 满幅左转 |
| 900 | -0.5 | 半幅左转 |
| 100 | 0 | 死区边界 |
| 0 | 0 | 死区 |
| -100 | 0 | 死区边界 |
| -900 | 0.5 | 半幅右转 |
| -1800 | 1.0 | 满幅右转 |

## 7. 超时安全机制

### 7.1 实现方式

由于 SDK 无 `system_get_tick()` 函数，超时检测基于 40ms 中断计数：

```c
#define RC_TIMEOUT_COUNTS  13   // 13 × 40ms = 520ms ≈ 500ms

// 在 remote_control_process() 中:
if (rc_data.active) {
    rc_timeout_counter++;
    if (rc_timeout_counter >= RC_TIMEOUT_COUNTS) {
        // 超时！触发安全停车
        flag_stop = 1;
        Yao.Target_Speed = 0;
        Deviation_Value = 0;
        rc_data.active = 0;
        rc_data.emergency_stop = 1;
    }
}

// 收到有效帧时:
rc_timeout_counter = 0;   // 重置计数器
rc_data.active = 1;
```

### 7.2 超时行为

| 条件 | 动作 |
|------|------|
| 520ms 内无有效帧 | `flag_stop = 1`, `Target_Speed = 0`, `Deviation_Value = 0`, `active = 0` |
| 收到新帧 | `flag_stop = 0`, `active = 1`, 计数器清零 |
| 急停命令 (0x04, data=1) | 立即 `flag_stop = 1`，不等超时 |
| 急停恢复 (0x04, data=0) | `flag_stop = 0`，但若超时仍会再次触发 |

## 8. 速度环设定值接入

在 `Interrupt_16ms()` 速度外环中：

```c
// 原代码: 速度环设定值硬编码为 0
// 修改后:
float speed_setpoint = remote_control_is_active() ? Yao.Target_Speed : 0;
```

- **遥控激活** (`active=1`): 使用 `Yao.Target_Speed`（由摇杆映射设置）
- **遥控未激活** (`active=0`): 设定值为 0（纯平衡模式）

## 9. 常量定义汇总

```c
/* 帧协议常量 */
#define RC_FRAME_HEADER         0x5A    // 帧头标识
#define RC_FRAME_MAX_LEN        32      // 最大帧长度
#define RC_FRAME_MIN_LEN        5       // 最小帧长度 (header+len+type+0data+chk)

/* 帧类型 */
#define RC_TYPE_JOYSTICK        0x01    // 摇杆数据
#define RC_TYPE_MODE_SWITCH     0x02    // 模式切换
#define RC_TYPE_HEIGHT          0x03    // 高度调节
#define RC_TYPE_EMERGENCY       0x04    // 急停控制
#define RC_TYPE_HEARTBEAT       0x05    // 心跳包

/* 高度方向 */
#define RC_HEIGHT_HOLD          0       // 保持
#define RC_HEIGHT_UP            1       // 增高
#define RC_HEIGHT_DOWN          2       // 降低

/* 急停控制 */
#define RC_EMERGENCY_STOP       1       // 急停
#define RC_EMERGENCY_RESUME     0       // 恢复

/* 超时 */
#define RC_TIMEOUT_COUNTS       13      // 13 × 40ms ≈ 500ms

/* 有效 turn_mode 位掩码 */
#define RC_VALID_TURN_MODES     0x0B    // bit0=1(0), bit1=0, bit2=1(2), bit3=1(3), bit4=0, bit5=0, bit6=1(6)
```

## 10. 数据结构

```c
typedef struct {
    int16_t  joystick[4];       /* 原始摇杆值 (来自遥控帧) */
    uint8_t  active;            /* 遥控激活标志: 1=在超时窗口内, 0=超时 */
    uint32_t last_rx_tick;      /* 最后收到帧的系统tick (ms) */
    uint8_t  emergency_stop;    /* 急停请求: 1=急停, 0=正常 */
    uint8_t  frame_count;       /* 帧计数器 (调试用) */
    uint8_t  error_count;       /* 错误帧计数 (调试用) */
} RemoteControl_t;
```

## 11. API 接口

| 函数 | 原型 | 调用位置 | 说明 |
|------|------|---------|------|
| 初始化 | `void remote_control_init(void)` | `Init.c` | 清零状态，复位解析器，重置超时计数 |
| 处理接收 | `void remote_control_process(void)` | `Interrupt_40ms()` | 读FIFO→帧头分发→解析→映射→超时检测 |
| 查询激活 | `uint8_t remote_control_is_active(void)` | `Interrupt_16ms()` | 超时窗口内返回1，否则返回0 |
| 获取数据 | `RemoteControl_t* remote_control_get_data(void)` | 调试用 | 返回内部状态指针 |
| 摇杆映射 | `void yaokong_map_joystick(int16_t j1, int16_t j2)` | `rc_handle_joystick()` | 摇杆值→速度+转向控制量 |

## 12. Python 上位机测试脚本

位于 `tools/remote_control_test.py`，功能：

| 按键 | 功能 | 发送帧 |
|------|------|--------|
| W/A/S/D | 前后左右摇杆 | Type=0x01 |
| 0/2/3/6 | 切换 turn_mode | Type=0x02 |
| Q/E | 增高/降低 | Type=0x03 |
| Space | 急停/恢复 | Type=0x04 |
| 自动 | 每200ms发送心跳 | Type=0x05 |
| ESC | 退出(先停车) | Type=0x04 (stop=1) |

帧构建函数：
```python
def build_frame(frame_type, data):
    length = 1 + 1 + 1 + len(data) + 1  # header + len + type + data + chk
    frame = bytes([0x5A, length, frame_type]) + data
    checksum = length ^ frame_type
    for b in data:
        checksum ^= b
    frame += bytes([checksum])
    return frame
```

## 13. 文件变更清单

| 操作 | 文件路径 | 变更说明 |
|------|---------|---------|
| **新建** | `project/code/ControlPart/remote_control.c` | 遥控协议解析+超时检测+控制映射 |
| **新建** | `project/code/ControlPart/remote_control.h` | 遥控模块头文件（常量+结构体+API） |
| **修改** | `project/code/ControlPart/yaokong.c` | 修复双return bug，解耦LoRa，新增 `yaokong_map_joystick()` |
| **修改** | `project/code/ControlPart/yaokong.h` | 新接口声明，移除 `extern int velocity;` |
| **修改** | `project/code/ControlPart/Interrupt.c` | 40ms添加 `remote_control_process()`，16ms速度环接入遥控设定值，2ms改用 `AI_Pid_Tuner_SendData()` |
| **修改** | `project/code/ControlPart/Init.c` | 添加 `remote_control_init()` 调用 |
| **修改** | `project/user/main_cm7_0.c` | 移除硬编码零值，添加 `#include "remote_control.h"` |
| **修改** | `project/code/ControlPart/AI_Pid_Tuner.c` | `ProcessRx()` 改为接受 `first_byte` 参数 |
| **修改** | `project/code/ControlPart/AI_Pid_Tuner.h` | 更新 `ProcessRx()` 声明 |
| **新建** | `tools/remote_control_test.py` | Python上位机测试脚本 |
