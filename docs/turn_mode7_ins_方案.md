# turn_mode == 7：yaw_ins 惯导目标角转向模式

## 修改概述

新增 turn_mode == 7，将 `yaw_ins`（惯导目标航向角）作为 PID 目标进行闭环转向，并在屏幕显示转向 PWM 输出，菜单中添加惯导设置和 PID 调参。

## 修改文件清单

| 文件 | 改动 |
|------|------|
| `project/code/ControlPart/Interrupt.c` | 新增 `#include "Ins.h"`、`ins_open` 标志位、turn_mode==7 代码块 |
| `project/code/ControlPart/Interrupt.h` | 新增 `extern uint8 ins_open` |
| `project/code/Menu/menu.c` | 新增 `#include "Ins.h"`、屏幕显示 Outp_turn、菜单页1替换6行、Flash存储替换 |

## 详细改动

### 1. Interrupt.c

**头文件**：添加 `#include "Ins.h"`

**全局变量**：在 `flag_stop` 附近新增：
```c
uint8 ins_open = 0;       // 惯导转向开关，1=开启
```

**Interrupt_4ms()**：在 turn_mode==6 代码块之后新增：
```c
else if (turn_mode == 7)
{
    if (ins_open)
    {
        float yaw_ins_deg = (float)yaw_ins;
        if (yaw_ins_deg > 180.0f)
            yaw_ins_deg -= 360.0f;
        if (yaw_ins_deg < -180.0f)
            yaw_ins_deg += 360.0f;

        Yao.Outp_turn = Cascade_angle_Yaw_2(&PID_all.Pid_Angle_Yaw, erect_Angle_Yaw_2, imu660ra.eulerAngle.yaw, yaw_ins_deg);
        Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
    }
    else
    {
        Yao.Outp_turn = 0;
    }
}
```

### 2. Interrupt.h

新增 extern 声明：
```c
extern uint8 ins_open;
```

### 3. menu.c

**头文件**：添加 `#include "Ins.h"`

**IPS200_Show1()**：末尾新增屏幕显示：
```c
ips200_show_string(0, 96, "turn:");
ips200_show_float(60, 96, Yao.Outp_turn, 4, 3);
```

**菜单页1 display**：行5~10替换为 ins_open/ins_mode/yaw2 PID

**菜单页1 cursor switch**：case 5~10 替换为新变量的 data_operate 调用

**Flash WRITE**：buffer[17]~[22] 替换为 ins_open/ins_mode/erect_Angle_Yaw_2

**Flash READ**：buffer[17]~[22] 替换为对应的读取

## 菜单页1最终布局

| 行 | 标签 | 变量 |
|----|------|------|
| 0 | page: 222222222 | - |
| 1 | yaw-gyro-P: | erect_Gyro_Yaw[0] |
| 2 | yaw-gyro-I: | erect_Gyro_Yaw[1] |
| 3 | yaw-gyro-D: | erect_Gyro_Yaw[2] |
| 4 | yaw-gyro-IL: | erect_Gyro_Yaw[3] |
| 5 | ins_open: | ins_open |
| 6 | ins_mode: | ins_mode |
| 7 | yaw2-P: | erect_Angle_Yaw_2[0] |
| 8 | yaw2-I: | erect_Angle_Yaw_2[1] |
| 9 | yaw2-D: | erect_Angle_Yaw_2[2] |
| 10 | yaw2-IL: | erect_Angle_Yaw_2[3] |
| 11 | yaw_Angle-D: | erect_Angle_Yaw_4[2] |
| 12 | yaw_Angle-IL: | erect_Angle_Yaw_4[3] |
| 13~20 | Fuzzy-P1~P7 | fuzzy_mode + P_Value_L |

## 调用链路

```
yaw_ins (Ins.c:get_target, 0~360°)
    │
    ▼ ins_open == 1 ?
    │  ├─ 否 → Outp_turn = 0
    │  └─ 是 ↓
    ▼ 角度转换 if/if → [-180, 180]
Cascade_angle_Yaw_2(Pid_Angle_Yaw, erect_Angle_Yaw_2, imu_yaw, yaw_ins_deg)
    │ → Yao.Outp_turn (期望角速度)
    ▼
Cascade_gyro_Yaw (Interrupt_2ms 内环)
    │ → Yao.Outp_Gyro_Yaw → 电机差速
    ▼
IPS200 显示 Yao.Outp_turn
```

## 使用流程

1. 菜单页1设置 `ins_mode=0`，通过按键录点
2. 菜单页1设置 `ins_mode=1` 进入导航模式
3. 菜单页1设置 `ins_open=1` 开启惯导转向
4. 代码中将 `turn_mode` 设为 7
5. 菜单页1调节 `yaw2-P`/`yaw2-D` 微调转向响应

## 前置条件

1. `get_realtime_coordinate()` 调用需取消注释（Interrupt.c:336）
2. `ins_navigation()` 调用需取消注释（main_cm7_0.c:151）
3. 航路点已通过 `ins_mode=0` 录入到 Flash
