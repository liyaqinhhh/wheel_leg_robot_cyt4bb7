# ALIGNMENT — GPS 导航框架

## 项目概述

为轮腿机器人（CYT4BB7 平台）开发 GPS 导航框架，基于现有 GNSS 驱动和 IMU 传感器，实现航点导航功能。

## 原始需求摘要

- 基于 `zf_device_gnss.c/h` 的 GNSS 驱动和 `GPS导航极简思路.pdf` 的导航思路
- 实现航点采集、存储、导航、到达检测的完整闭环
- 与现有 turn_mode==5 的 GPS 航向 PID 控制集成

## 关键决策确认

| 决策项 | 结论 | 备注 |
|--------|------|------|
| 转弯策略 | 简单直线转弯（非半径感知） | 到达航点后转向下一航点方向 |
| 航点数量上限 | 40 个 | 每航点 16 字节，1 页 Flash 足够 |
| 采点方式 | 先做手动采点 | 自动生成方案写入 MD 文档作为进阶 |
| 校准方式 | 开机停在第一个航点 | 其他校准方式写入 MD 文档作为扩展 |
| 起点约束 | 接受：每次开机必须停在第一个航点 | 面朝第二个航点方向 |
| Flash 存储 | 页 50 存航点数据，页 51 存元数据 | 避开现有分区（页 0/1/10/25/26~49） |

## 核心模块

### M1: 航点管理
- 手动采点：按键采集当前 GPS 坐标
- Flash 存储：40 航点存页 50，元数据存页 51
- 开机加载：读取 Flash 恢复航点

### M2: IMU-GPS 航向校准
- 起点约束法：开机停在第一个航点，面朝下一航点
- 计算偏移量：`offset = GPS_target_bearing - IMU_yaw`
- 扩展预留：运动方向法、双天线法（见 GPS_CALIBRATION_METHODS.md）

### M3: 漂移补偿
- IMU 陀螺仪漂移累积问题
- 行驶中用 GPS 方向（gnss.direction）周期性修正
- 低速/静止时不修正（GPS 方向不可靠）

### M4: 导航状态机
- 状态：IDLE → CALIBRATING → NAVIGATING → ARRIVED → COMPLETE
- IDLE：等待启动
- CALIBRATING：校准 IMU-GPS 偏移
- NAVIGATING：朝当前目标航点行驶
- ARRIVED：到达航点，切换下一航点
- COMPLETE：所有航点走完

### M5: 到达检测
- 距离判定：`get_two_points_distance() < threshold`
- 阈值可配置（默认 2 米）
- 到达后自动切换下一航点

## 边界与约束

1. **GPS 信号依赖**：无 GPS 信号时无法导航（不实现纯 IMU 推算）
2. **IMU 漂移**：长时间（> 5 分钟）运行后航向精度下降
3. **转弯策略**：简单原地转向，不考虑转弯半径
4. **速度限制**：导航时速度由 PID 控制，不单独处理
5. **单次路线**：不支持中途修改航点（需重新采点）

## 风险点

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| 页 0 冲突 | Ins.c 和 ins_track.c 都用页 0 | GPS 导航使用页 50/51，避开冲突 |
| IMU 漂移 | 长时间运行航向偏差大 | GPS 方向周期性修正 |
| GPS 多径 | 建筑物附近定位跳变 | 到达检测加距离滤波 |
| Flash 寿命 | 频繁擦写 | 仅在采点/保存时写入 |

## 参考文档

- `docs/导航方案/FLASH_STORAGE_GUIDE.md` — Flash 存储分区科普
- `docs/导航方案/GPS_AUTO_WAYPOINTS.md` — 航点自动生成方案（进阶）
- `docs/导航方案/GPS_CALIBRATION_METHODS.md` — 校准方法汇总（扩展）
- `libraries/zf_device/zf_device_gnss.c/h` — GNSS 驱动源码
- `project/code/ControlPart/Interrupt.c` — 现有 GPS 航向 PID 控制
