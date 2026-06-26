# ALIGNMENT_IMU660RA_TO_RB.md — IMU660RA → IMU660RB 迁移对齐文档

**日期**: 2026-06-18
**任务**: 将项目中所有 IMU660RA 陀螺仪调用替换为 IMU660RB

---

## 1. 原始需求总结

用户要将原来的 **IMU660RA** 六轴陀螺仪换成 **IMU660RB**，需将项目中所有调用 IMU660RA 数据的部分替换为 IMU660RB。

---

## 2. 当前架构分析

### 2.1 硬件现状

| 资源 | IMU660RA | IMU660RB |
|------|----------|----------|
| SPI 总线 | SPI2 | SPI2 |
| SCL/CLK | P15_2 | P15_2 |
| SDA/MOSI | P15_1 | P15_1 |
| SDO/MISO | P15_0 | P15_0 |
| CS | P15_3 | P15_3 |
| I2C 地址 | 0x6A | 0x6A |

> ?? **关键发现**: RA 和 RB 共享完全相同的 SPI 引脚和 I2C 地址，意味着同一块板子上只能焊接其中一个芯片。目前板子上已焊接 IMU660RB。

### 2.2 软件现状 — 双驱动混合状态

当前代码存在"混合"状态：
- **初始化**: `Init.c` 中只调用了 `imu660rb_init()`（第32行）
- **但 RA 驱动函数仍在被大量调用**:
  - `imu660ra_get_gyro()` / `imu660ra_get_acc()` — 在 `imu660.c:date_handle()` 和 `kalman.c` 中调用
  - `imu660ra_acc_x/y/z` / `imu660ra_gyro_x/y/z` — 原始数据变量
  - `imu660ra` 结构体（`imu660_struct`）— 姿态解算核心

### 2.3 RA 被引用的文件全量清单

| 文件 | 引用方式 | 说明 |
|------|---------|------|
| `libraries/zf_device/zf_device_imu660ra.c` | RA 驱动实现 | 底层 SPI 读写 |
| `libraries/zf_device/zf_device_imu660ra.h` | RA 驱动头文件 | 宏定义、变量声明 |
| `libraries/zf_common/zf_common_headfile.h` | `#include "zf_device_imu660ra.h"` | 全局包含 |
| `project/code/ControlPart/imu660.h` | `extern imu660_struct imu660ra;` | 结构体声明 |
| `project/code/ControlPart/imu660.c` | **核心** — 数据采集、姿态解算 | `date_handle()`, `get_eulerAngle()`, `static_imu_test()` |
| `project/code/ControlPart/Init.c` | `imu660ra.offset_angle` 初始值 | pitch=0.1, roll=-3.9 |
| `project/code/ControlPart/kalman.c` | `imu660ra_get_acc/gyro`, `imu660ra_*_transition` | 卡尔曼滤波 |
| `project/code/ControlPart/Interrupt.c` | `imu660ra.eulerAngle`, `imu660ra.data_Raw`, `imu660ra_gyro_x` | PID 控制、遥测 |
| `project/code/ControlPart/PID.c` | `imu660ra.eulerAngle.roll` | 直立环 |
| `project/code/VersionPart/image.c` | `imu660ra.eulerAngle.yaw/pitch` | 图像处理/状态机 |
| `project/code/VersionPart/ips.c` | `imu660ra.eulerAngle.roll/pitch` | 屏幕显示 |
| `project/code/Menu/menu.c` | `imu660ra.eulerAngle`, `imu660ra.offset_angle` | 菜单显示和校准 |
| `project/code/ins/ins_auto_record.c` | `imu660ra.eulerAngle.yaw` | 自动打点 |
| `project/code/ins/ins_pure_pursuit.c` | 注释引用 | 注释中提及 |
| `project/code/ins/Ins.h` | 注释引用 | Doxygen 注释 |
| `docs/` 多个文档 | 文档引用 | 说明文档 |

---

## 3. IMU660RA 与 IMU660RB 的关键差异

| 参数 | IMU660RA | IMU660RB |
|------|----------|----------|
| 加速度计量程配置 | 枚举 `imu660ra_acc_sample_config` | 寄存器值 `IMU660RB_ACC_SAMPLE` |
| 陀螺仪量程配置 | 枚举 `imu660ra_gyro_sample_config` | 寄存器值 `IMU660RB_GYR_SAMPLE` |
| ±8G acc 转换因子 | `/4096` | `/4098` |
| ±2000dps gyro 转换因子 | `/16.4` | `/14.3` |
| 转换函数 | `imu660ra_acc_transition()`, `imu660ra_gyro_transition()` | `imu660rb_acc_transition()`, `imu660rb_gyro_transition()` |
| 初始化复杂度 | 需要 config_file 配置帧写入 | 直接寄存器配置（更简洁） |
| 陀螺仪采样率 | 200Hz (0xA9) | 由 CTRL2_G 寄存器配置 |
| 加速度计采样率 | 50Hz (0xA7) | 由 CTRL1_XL 寄存器配置 |

---

## 4. 边界条件确认

### 4.1 需要确认的问题

1. **`imu660ra` 结构体是否需要重命名？**
   - 当前 `imu660_struct imu660ra` 是姿态解算的核心结构体
   - 选项 A: 保持名称 `imu660ra`（最小改动，但命名语义不对）
   - 选项 B: 重命名为 `imu660rb`（语义正确，但需修改所有引用文件 ~10+ 个）
   - 建议：重命名为 `imu660rb` 更利于长期维护

2. **RB 已初始化的那部分代码怎么处理？**
   - `Interrupt.c:649` 已经使用 `imu660rb_gyro_z`（RB 的原始 Z 轴陀螺仪数据）
   - 迁移后：`imu660ra.data_Raw.gyro_z` → `imu660rb.data_Raw.gyro_z`

3. **校准参数是否需要重新标定？**
   - RA 的零偏校准: `gyro_x -= 1.8`, `gyro_y -= 2.5`, `gyro_z = 0`
   - RA 的 offset_angle: `pitch=0.1`, `roll=-3.9`
   - RA 的 gyro 偏移: `-19.0` (x), `+76.0` (y)
   - 换成 RB 芯片后，这些硬编码校准值**几乎肯定会变**

4. **转换因子变化**:
   - `date_handle()` 中硬编码的 `9.79 / 4096` 需要更新为 `9.79 / 4098`
   - `M_PI / 180 / 16.4f` 需要更新为 `M_PI / 180 / 14.3f`
   - 但更好的做法是统一使用 `imu660rb_acc_transition()` 和 `imu660rb_gyro_transition()`

5. **是否需要同时保留 RA 驱动的文件？**
   - 如果板子上不再有 IMU660RA，建议从 `zf_common_headfile.h` 中移除 `#include "zf_device_imu660ra.h"`

---

## 5. 歧义清单

| # | 歧义项 | 影响范围 |
|---|--------|---------|
| 1 | 结构体变量名 `imu660ra` 是否重命名 | 所有引用文件 |
| 2 | 硬编码校准值如何处理 | `imu660.c:date_handle()`, `Init.c` |
| 3 | `date_handle()` 中的转换因子是改用 RB 驱动函数还是硬编码修正 | `imu660.c` |
| 4 | RA 驱动文件是否删除/保留 | `libraries/zf_device/zf_device_imu660ra.c/h` |
| 5 | `kalman.c` 中直接调用 `imu660ra_*` 函数是否需要改为通过 `imu660_struct` 统一 | `kalman.c` |

---

## 6. 建议的迁移策略（待用户确认）

### 推荐方案: 全量迁移 + 统一使用 `imu660rb` 命名

1. **重命名结构体**: `imu660ra` → `imu660rb`（全局替换）
2. **替换驱动函数调用**:
   - `imu660ra_get_gyro()` → `imu660rb_get_gyro()`
   - `imu660ra_get_acc()` → `imu660rb_get_acc()`
   - `imu660ra_acc_transition()` → `imu660rb_acc_transition()`
   - `imu660ra_gyro_transition()` → `imu660rb_gyro_transition()`
3. **修正硬编码转换因子**: 统一使用 RB 驱动提供的转换函数
4. **保留校准值**: 先保持 RA 的校准值，但添加 `TODO` 注释标记需要重新标定
5. **移除 RA 头文件引用**: 从 `zf_common_headfile.h` 中移除
