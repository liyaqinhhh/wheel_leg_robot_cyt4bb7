# INS 惯导移植项目 - 对齐文档

## 1. 项目概述

**项目名称**：惯导系统从四轮小车移植到两足轮腿机器人

**源工程**：`D:\05_open\procedure\惯导\4.4初步实现小车惯导\4.4初步实现小车惯导\new_ins`（基于TC264/AURIX平台）

**目标工程**：`d:\Yao_Port_4bb7_01 - backups\Yao_Port_4bb7_01\Seekfree_CYT4BB_Opensource_Library_`（基于CYT4BB7/Cortex-M7双核）

---

## 2. 源工程架构分析

### 2.1 核心模块

| 模块 | 文件 | 功能 |
|------|------|------|
| **INS核心** | `code/ins/Ins.c/h` | EKF航迹推算（2D平面），状态：x, y, yaw |
| **IMU驱动** | `code/ins/imu660.c/h` | IMU963RA驱动，含磁力计支持、校准算法 |
| **GNSS** | `code/ins/gnss.c/h` | GPS模块（本次移植不涉及） |
| **轨迹管理** | `code/control/track.c/h` | Flash坐标存取、轨迹记录/循迹 |
| **遥控** | `code/control/yaokong.c/h` | 遥控逻辑（需移除） |
| **菜单** | `code/system/menu.c/h` | 按键状态机（需移除） |
| **中断调度** | `code/system/interrupt.c/h` | 1/2/4/8/16/40ms任务调度 |

### 2.2 数据流

```
IMU数据 → date_handle() → INS_Input结构
编码器数据 → encoder_layer_update() → INS_Input结构
INS_Input → Ins_update() → INS_State (x, y, yaw)
INS_State → track_proc() → 循迹控制输出
```

### 2.3 定时任务分配

- **4ms**：IMU处理、编码器更新、PID计算、INS更新、轨迹处理
- **8ms**：按键扫描、遥控处理、INS导航任务
- **40ms**：INS显示、串口发送

---

## 3. 目标工程架构分析

### 3.1 已有资源

| 资源 | 文件 | 说明 |
|------|------|------|
| **IMU驱动** | `libraries/zf_device/zf_device_imu660ra.c/h` | IMU660RA驱动（SPI接口） |
| **编码器驱动** | `libraries/zf_driver/zf_driver_encoder.c/h` | 正交编码器接口 |
| **Flash驱动** | `libraries/zf_driver/zf_driver_flash.c/h` | 片上Flash读写（2KB页，512个uint32） |
| **定时器驱动** | `libraries/zf_driver/zf_driver_pit.c/h` | PIT定时器配置 |
| **主程序** | `project/user/main_cm7_0.c` | CM7_0核主程序 |

### 3.2 Flash接口对比

**源工程**：
- 使用seekfree_assistant库的flash_union_buffer
- 每页存储510个float（2040字节）

**目标工程**：
- 使用zf_driver_flash库
- 每页512个uint32（2048字节）
- flash_data_union联合体支持float/uint32/int32等类型

**兼容性**：接口高度相似，可直接适配。

### 3.3 IMU接口对比

**源工程（imu660_struct）**：
```c
typedef struct {
    euler_param offset_angle;    // 零偏角
    imu_param data_Raw;          // 原始数据
    imu_param data_Ripen;        // 处理后数据
    euler_param eulerAngle;      // 欧拉角
} imu660_struct;
```

**目标工程（imu660ra）**：
- 提供加速度计、陀螺仪原始数据
- 需要自行实现姿态解算（或复用源工程的date_handle）

---

## 4. 关键差异与挑战

### 4.1 平台差异

| 项目 | 源工程（TC264） | 目标工程（CYT4BB7） |
|------|-----------------|---------------------|
| 内核 | TriCore | Cortex-M7 双核 |
| 编译器 | Tasking | IAR |
| Flash布局 | 不同 | 192KB，96页 |
| 定时器 | GTM | PIT |

### 4.2 传感器差异

| 项目 | 源工程 | 目标工程 |
|------|--------|----------|
| IMU | IMU963RA（含磁力计） | IMU660RA（无磁力计） |
| 编码器 | 需确认 | 支持正交编码器 |

**关键问题**：目标工程无磁力计，INS航向角只能依赖陀螺仪积分，漂移问题需要通过ZUPT（零速更新）或其他方式补偿。

### 4.3 运动学差异

**源工程（四轮小车）**：
- 2D平面运动模型
- 差速转向：`ω = v * tan(steer_angle) / wheelbase`

**目标工程（轮腿机器人）**：
- 需要3D姿态补偿（俯仰角变化频繁）
- 转向控制逻辑不同（可能需要双轮差速或腿长差控制）

---

## 5. 移植方案（第一部分：架构适配层）

### 5.1 目录结构规划

在目标工程中创建以下目录：

```
project/code/ins/
├── ins_core/              # INS核心算法（从源工程移植）
│   ├── ins.c/h           # EKF航迹推算
│   ├── ins_interface.c/h # API接口层（新增）
│   └── ins_config.h      # 配置参数
├── ins_drivers/           # 传感器驱动适配层
│   ├── imu_adapter.c/h   # IMU数据适配
│   └── encoder_adapter.c/h # 编码器数据适配
└── ins_application/       # 应用层
    ├── track.c/h         # 轨迹管理（从源工程移植）
    └── track_interface.c/h # 轨迹API接口
```

### 5.2 需要移植的文件

| 源文件 | 目标文件 | 修改内容 |
|--------|----------|----------|
| `code/ins/Ins.c` | `project/code/ins/ins_core/ins.c` | 移除GNSS相关，适配新IMU接口 |
| `code/ins/Ins.h` | `project/code/ins/ins_core/ins.h` | 保持结构体定义 |
| `code/ins/imu660.c` | `project/code/ins/ins_drivers/imu_adapter.c` | 适配zf_device_imu660ra接口 |
| `code/control/track.c` | `project/code/ins/ins_application/track.c` | 适配zf_driver_flash接口 |
| `code/system/interrupt.c` | `project/user/cm7_0_isr.c` | 集成到现有中断系统 |

### 5.3 不需要移植的文件

- `code/control/yaokong.c/h` - 遥控逻辑（移除）
- `code/system/menu.c/h` - 菜单逻辑（移除）
- `code/ins/gnss.c/h` - GPS模块（不使用）

### 5.4 Flash适配策略

**方案**：直接使用目标工程的`zf_driver_flash`库，无需修改。

**接口映射**：
```c
// 源工程
flash_union_buffer[index].float_type = value;

// 目标工程
flash_union_buffer[index].float_type = value;  // 完全兼容
```

### 5.5 定时器适配策略

**方案**：使用目标工程的PIT定时器，配置相同的中断周期。

**需要配置**：
- PIT0：4ms周期（IMU/编码器/INS更新）
- PIT1：8ms周期（可选，用于其他任务）
- PIT2：40ms周期（可选，用于显示/通信）

### 5.6 浮点运算支持

**确认**：CYT4BB7的Cortex-M7核支持硬件FPU（单精度），无需额外配置。

---

## 6. 待确认问题

1. **IMU型号确认**：目标工程使用的是IMU660RA还是IMU963RA？源工程使用的是IMU963RA（含磁力计）。

2. **编码器接口确认**：轮腿机器人的编码器接口是什么类型？（正交编码器/霍尔传感器/其他）

3. **轮腿运动学模型**：请提供轮腿机器人的运动学参数：
   - 轮距（或腿间距）
   - 腿长范围
   - 俯仰角变化范围
   - 转向控制方式（差速/腿长差/其他）

4. **Flash存储需求**：轨迹点存储需要多少页？源工程配置为24页（约3000个点）。

5. **中断优先级**：INS更新任务的优先级如何设定？（建议最高优先级）

---

## 7. 下一步工作

确认上述问题后，将进入**第二部分：传感器数据融合优化**，重点解决：
- 无磁力计情况下的航向角漂移补偿
- 3D姿态补偿的坐标推算公式
- 轮腿运动学模型的集成
