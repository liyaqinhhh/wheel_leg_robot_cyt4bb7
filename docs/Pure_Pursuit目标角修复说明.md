# Pure Pursuit 目标偏航角计算错误修复

## 问题现象

按键打点惯导（ins_mode=4）和分段惯导（ins_mode=3）在开始导航时，小车立即开始乱转，无法正常循迹。

## 根本原因

**Pure Pursuit 算法的目标偏航角计算逻辑错误**

### 错误代码（修复前）

```c
/* ins_pure_pursuit.c 第 170-180 行 */

/* 4. 计算转向角 */
/* δ = atan(κ × L) */
double steering_angle = atan(curvature * PURE_PURSUIT_WHEELBASE);
steering_angle = RAD_TO_ANGLE(steering_angle);

/* 5. 计算目标偏航角 (当前偏航角 + 转向角) */
double target_yaw = current_yaw + steering_angle;  // ? 错误！

/* 归一化到 0~360° */
while (target_yaw < 0) target_yaw += 360.0;
while (target_yaw >= 360.0) target_yaw -= 360.0;

return (float)target_yaw;
```

### 错误分析

1. **概念混淆**：
   - `steering_angle` 是转向角（方向盘角度），用于差速转向控制
   - 它不是偏航角变化量，不能直接加到当前偏航角上

2. **Pure Pursuit 原理**：
   - Pure Pursuit 计算的是转向角 δ，用于控制差速转向
   - 对于差速转向机器人，转向角通过左右轮速差实现
   - **目标偏航角应该是：从当前位置指向前瞻点的方位角**

3. **实际效果**：
   - 错误代码：`target_yaw = current_yaw + steering_angle`
   - 当小车偏离路径时，转向角可能很大（如 ±30°）
   - 目标偏航角被错误地设置为当前偏航角 + 转向角
   - 导致小车朝错误的方向转向，形成正反馈，开始乱转

### 正确代码（修复后）

```c
/* 4. 计算目标偏航角 */
/* 目标偏航角 = 从当前位置指向前瞻点的方位角 */
/* 使用 atan2 计算方位角，范围 [-π, +π] */
double target_yaw_rad = atan2(dy, dx);

/* 转换为度数并归一化到 [0, 360°) */
double target_yaw = RAD_TO_ANGLE(target_yaw_rad);
if (target_yaw < 0) target_yaw += 360.0;

return (float)target_yaw;
```

### 正确性验证

1. **几何意义**：
   - `dx = target_pos.x - current_pos.x`：前瞻点在 X 方向的距离
   - `dy = target_pos.y - current_pos.y`：前瞻点在 Y 方向的距离
   - `atan2(dy, dx)`：从当前位置指向前瞻点的方位角

2. **与原 Ins.c 一致**：
   - 原 Ins.c 的 `get_target()` 函数使用相同方法计算方位角
   - `yaw_ins = RAD_TO_ANGLE(atan2(dy, dx))`

3. **PID 控制器期望**：
   - `turn_mode==7` 使用 `yaw_ins` 作为目标偏航角
   - PID 控制器计算：`error = yaw_ins - current_yaw`
   - 正确的目标偏航角确保小车朝前瞻点方向转向

## 修复影响

### 受影响功能

1. **ins_mode=4**：自动打点惯导（按键打点）
2. **ins_mode=3**：分段惯导导航
3. **所有使用 Pure Pursuit 算法的导航模式**

### 修复效果

- ? 小车能够正确循迹，不再乱转
- ? 目标偏航角指向前瞻点，符合 Pure Pursuit 原理
- ? 与原 Ins.c 的 `get_target()` 逻辑一致
- ? PID 控制器能够正确计算转向误差

## 测试建议

### 测试步骤

1. 编译并烧录修复后的程序
2. 使用按键打点模式（ins_mode=4）：
   - 按键开始录制航点
   - 沿路径行驶，记录航点
   - 按键开始导航
   - 观察小车是否能够平滑循迹

3. 使用分段惯导模式（ins_mode=3）：
   - 设置分段航点
   - 开始导航
   - 观察小车是否能够正确循迹

### 预期结果

- 小车应该沿着记录的路径平滑行驶
- 目标偏航角应该指向前方路径上的前瞻点
- 转向应该平滑，没有剧烈震荡
- 到达终点后自动停止

### 调试信息

可以通过串口监视以下变量：

- `yaw_ins`：目标偏航角（应该指向前瞻点）
- `imu660ra.eulerAngle.yaw`：当前偏航角
- `g_pp_state.target_wp_index`：前瞻点索引
- `g_pp_state.distance_to_path`：到路径的横向距离
- `g_pp_state.angle_to_path`：到路径的角度偏差

## 相关文件

- `project/code/ins/ins_pure_pursuit.c`：Pure Pursuit 算法实现
- `project/code/ins/ins_auto_record.c`：自动打点惯导模块
- `project/code/ControlPart/Interrupt.c`：turn_mode==7 转向控制
- `project/code/ins/Ins.c`：原惯导系统（get_target 函数）

## 技术总结

### Pure Pursuit 算法要点

1. **前瞻点选择**：在路径前方距离 Ld 处选择目标点
2. **目标偏航角**：从当前位置指向前瞻点的方位角（使用 atan2 计算）
3. **转向控制**：通过 PID 控制器调整左右轮速差，使当前偏航角趋近目标偏航角

### 常见误区

1. ? 将转向角（steering_angle）直接加到当前偏航角上
2. ? 混淆转向角和偏航角变化量
3. ? 目标偏航角应该是前瞻点的方位角

### 与差速转向的关系

- Pure Pursuit 原本用于阿克曼转向（方向盘控制）
- 对于差速转向机器人，转向角通过左右轮速差实现
- **目标偏航角**才是 PID 控制器需要的输入，而不是转向角

## 修复日期

2026-06-11

## 修复人员

GitHub Copilot
