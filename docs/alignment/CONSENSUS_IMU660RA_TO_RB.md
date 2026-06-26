# CONSENSUS_IMU660RA_TO_RB.md — 迁移共识文档

**日期**: 2026-06-18
**来源**: ALIGNMENT_IMU660RA_TO_RB.md + 用户确认

---

## 确认的决策

| # | 决策项 | 决定 |
|---|--------|------|
| 1 | 结构体命名 | ? **保持 `imu660ra`**，不重命名（最小改动） |
| 2 | 硬编码校准值 | ? **保留但加 TODO 标记**，提醒后续重新标定 |
| 3 | RA 驱动文件 | ? **保留不动**（zf_device_imu660ra.c/h 不做任何修改） |

## 迁移范围（待执行）

### 不修改的文件
- `libraries/zf_device/zf_device_imu660ra.c` — 保留不动
- `libraries/zf_device/zf_device_imu660ra.h` — 保留不动
- `libraries/zf_device/zf_device_imu660rb.c` — RB 驱动已存在，不动
- `libraries/zf_device/zf_device_imu660rb.h` — RB 驱动已存在，不动
- `libraries/zf_common/zf_common_headfile.h` — 保留两个 include，不动

### 需要修改的文件（仅上层应用代码）

| # | 文件 | 修改内容 |
|---|------|---------|
| 1 | `project/code/ControlPart/imu660.c` | **核心** — 替换 RA→RB 驱动函数、修正转换因子、保留校准值+TODO |
| 2 | `project/code/ControlPart/kalman.c` | 替换 `imu660ra_get_*` → `imu660rb_get_*`，`imu660ra_*_transition` → `imu660rb_*_transition` |
| 3 | `project/code/ControlPart/Interrupt.c` | 替换 `imu660ra_gyro_x` → `imu660rb_gyro_x` 等原始变量引用 |
| 4 | `project/code/ControlPart/Init.c` | 无需修改（已经调用 `imu660rb_init()`） |
| 5 | `project/code/ControlPart/PID.c` | 无需修改（只引用 `imu660ra.eulerAngle`，而结构体名不变） |
| 6 | `project/code/VersionPart/ips.c` | 无需修改（只引用 `imu660ra.eulerAngle`） |
| 7 | `project/code/VersionPart/image.c` | 无需修改（只引用 `imu660ra.eulerAngle`） |
| 8 | `project/code/Menu/menu.c` | 无需修改（只引用 `imu660ra.eulerAngle/offset_angle`） |
| 9 | `project/code/ins/ins_auto_record.c` | 无需修改（只引用 `imu660ra.eulerAngle.yaw`） |

### 关键：`imu660ra` 结构体中存储的姿态数据不变

因为结构体名保持 `imu660ra`，所以以下代码**无需修改**：
- `imu660ra.eulerAngle.yaw/pitch/roll`
- `imu660ra.data_Raw` / `imu660ra.data_Ripen`
- `imu660ra.offset_angle`

### 需要替换的驱动层符号

| 旧符号（RA 驱动导出） | 新符号（RB 驱动导出） |
|----------------------|----------------------|
| `imu660ra_get_gyro()` | `imu660rb_get_gyro()` |
| `imu660ra_get_acc()` | `imu660rb_get_acc()` |
| `imu660ra_acc_x/y/z` | `imu660rb_acc_x/y/z` |
| `imu660ra_gyro_x/y/z` | `imu660rb_gyro_x/y/z` |
| `imu660ra_acc_transition()` | `imu660rb_acc_transition()` |
| `imu660ra_gyro_transition()` | `imu660rb_gyro_transition()` |

### 需要修正的硬编码转换因子

`imu660.c:date_handle()` 中：
- `9.79 / 4096` → `9.79 / 4098`（RB 的 ±8G 量程因子）
- `M_PI / 180 / 16.4f` → `M_PI / 180 / 14.3f`（RB 的 ±2000dps 量程因子）
- 或者更好的做法：用 `imu660rb_acc_transition()` / `imu660rb_gyro_transition()` 替代手动计算

## 验收标准 (Given-When-Then)

**GIVEN** IMU660RB 硬件已焊接在板子上
**WHEN** 编译并烧录修改后的固件
**THEN**
1. 编译零错误零警告
2. 系统启动时 `imu660rb_init()` 成功初始化（自检通过）
3. `date_handle()` 能正确读取陀螺仪和加速度数据
4. 姿态解算（四元数滤波）正常输出 pitch/roll/yaw
5. PID 控制和 IMU 相关功能（直立、转向）正常运行

## 约束与风险

- ?? 校准值沿用 RA 的硬编码值，实际运行时可能需要重新标定
- ?? RB 的 gyro 量程因子 (`14.3`) 和 acc 量程因子 (`4098`) 需要验证是否与 RA 对应的默认配置一致
