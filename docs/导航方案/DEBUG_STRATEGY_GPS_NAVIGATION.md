# GPS 导航调试策略

> 6A 流程 · Automate 阶段配套文档
> 配合 `TASKS_GPS_NAVIGATION.md` 使用

---

## 一、调试总原则

1. **分层验证**：从底层到顶层，每层独立验证后再组合
2. **串口为王**：所有中间变量通过无线串口（UART_1）输出，不依赖屏幕
3. **单变量控制**：每次只改一个参数/功能，确认效果后再改下一个
4. **安全优先**：调试时降低速度目标，随时准备 KEY_3 停车

---

## 二、分阶段调试流程

### 阶段 1：GPS 信号验证（不依赖任何导航代码）

**目标**：确认 GNSS 驱动正常工作，能输出有效经纬度

**前置条件**：仅启用 `gnss_init(TAU1201)`，while 循环中添加调试输出

**调试代码**（在 main_cm7_1.c while 循环中）：

```c
if (gnss_flag)
{
    gnss_data_parse();
    gnss_flag = 0;
    
    // 阶段1调试：每10帧输出一次（约1秒）
    static uint8 dbg_cnt = 0;
    if (++dbg_cnt >= 10)
    {
        dbg_cnt = 0;
        printf("GPS: state=%d, sat=%d, lat=%.6f, lng=%.6f, spd=%.1f, dir=%.1f\r\n",
               gnss.state, gnss.satellite_used,
               gnss.latitude, gnss.longitude,
               gnss.speed, gnss.direction);
    }
}
```

**验证清单**：

| # | 检查项 | 期望结果 | 失败排查 |
|---|--------|---------|---------|
| 1.1 | gnss.state | 室外=1（有效） | 检查天线连接、UART_2波特率、是否在室内 |
| 1.2 | gnss.satellite_used | ≥4 | 天线朝上、无遮挡；<4则定位精度差 |
| 1.3 | gnss.latitude/longitude | 合理值（如30.xxxx, 120.xxxx） | 全0→NMEA解析失败；检查FIFO缓冲区溢出 |
| 1.4 | gnss.speed | 静止≈0，移动时合理 | 始终0→RMC解析问题；始终大值→多径效应 |
| 1.5 | gnss.direction | 移动时0~360变化 | 始终0→速度太低（<5km/h无航向） |
| 1.6 | 输出频率 | 约10Hz（每100ms一次） | 频率低→检查gnss_init配置命令是否发送成功 |

**常见问题**：

- **gnss.state 始终为 0**：GPS模块未定位。到室外等待1-2分钟冷启动
- **经纬度全 0**：NMEA 校验和失败。用示波器/逻辑分析仪抓 UART_2 波形确认数据格式
- **输出频率只有 1Hz**：10Hz 配置命令未生效。检查 `gnss_init()` 中的 UART 发送是否成功

---

### 阶段 2：航点采集验证（T01 + T02 + T17）

**目标**：确认按键能正确采集GPS坐标并存入内存

**前置条件**：阶段1通过，已实现 gps_waypoint.h/c 和按键交互

**调试代码**：

```c
// 在 KEY_1 短按处理中添加
if (key_detect(KEY_1, KEY_SHORT_PRESS))
{
    if (gnss.state == 1)
    {
        uint8 result = gps_wp_add(gnss.latitude, gnss.longitude);
        printf("WP_ADD: result=%d, count=%d, lat=%.6f, lng=%.6f\r\n",
               result, gps_wp_get_count(), gnss.latitude, gnss.longitude);
    }
    else
    {
        printf("WP_ADD: FAIL - GPS invalid (state=%d)\r\n", gnss.state);
    }
}
```

**验证清单**：

| # | 检查项 | 期望结果 | 失败排查 |
|---|--------|---------|---------|
| 2.1 | KEY_1 采集 | count 递增，result=1 | result=0→航点已满(40)或坐标非法 |
| 2.2 | 连续采集 | 每次按KEY_1，count+1 | 重复按无反应→key_detect未正确识别 |
| 3.3 | 坐标合理性 | 相邻航点距离与实际移动一致 | 用手机GPS对比验证 |
| 2.4 | GPS无效时 | 不采集，输出FAIL | gnss.state检查生效 |
| 2.5 | 40个航点后 | result=0，拒绝添加 | gps_wp_add边界检查生效 |

**关键测试**：走到不同位置按 KEY_1 采集 3-5 个航点，记录每个航点的经纬度

---

### 阶段 3：方位角/距离计算验证（T09）

**目标**：确认 `get_two_points_azimuth` 和 `get_two_points_distance` 计算正确

**前置条件**：阶段2通过，已有≥2个航点

**调试代码**（在 while 循环中，gnss_flag 处理后）：

```c
// 阶段3调试：手动计算当前到航点0的方位角和距离
if (gps_wp_get_count() > 0)
{
    gps_waypoint_t *wp0 = &gps_wp_set.waypoints[0];  // 需include gps_waypoint.h
    float bearing = get_two_points_azimuth(gnss.latitude, gnss.longitude, wp0->latitude, wp0->longitude);
    float distance = get_two_points_distance(gnss.latitude, gnss.longitude, wp0->latitude, wp0->longitude);
    
    static uint8 dbg3_cnt = 0;
    if (++dbg3_cnt >= 10)
    {
        dbg3_cnt = 0;
        printf("CALC: bear=%.1f, dist=%.2f, gnss_dir=%.1f\r\n",
               bearing, distance, gnss.direction);
    }
}
```

**验证清单**：

| # | 检查项 | 期望结果 | 失败排查 |
|---|--------|---------|---------|
| 3.1 | 方位角范围 | [0, 360) | 超出范围→get_two_points_azimuth实现有误 |
| 3.2 | 距离合理性 | 与步测/手机GPS距离一致 | 差异大→Haversine公式参数单位错误 |
| 3.3 | 朝航点走 | 方位角趋近0°（正前方） | 不变→航点坐标错误 |
| 3.4 | 远离航点 | 距离递增 | 不变→当前GPS坐标未更新 |
| 3.5 | gnss.direction vs bearing | 移动时两者趋势一致 | 完全不同→一个有bug |

**关键测试**：采集一个航点后走 20 米，确认距离约 20m；朝航点走时方位角应接近 gnss.direction

---

### 阶段 4：校准验证（T05）

**目标**：确认起点校准计算出正确的 IMU-GPS 偏移量

**前置条件**：阶段3通过，已有≥2个航点

**调试代码**：

```c
// 在 gps_nav_start() 调用后输出校准结果
// 或在 gps_cal_startpoint() 实现中添加：
printf("CAL: imu_yaw=%.1f, gps_bearing=%.1f, offset=%.1f\r\n",
       imu660ra.eulerAngle.yaw, gps_bearing, gps_cal_offset_deg);
```

**验证清单**：

| # | 检查项 | 期望结果 | 失败排查 |
|---|--------|---------|---------|
| 4.1 | offset 范围 | [-90, +90] | 超出→机器人朝向与航点方向夹角>90°，需调整站位 |
| 4.2 | offset 方向 | 机器人面朝航点方向时 offset≈0 | 符号反→检查 yaw 正方向与 GPS 方位角方向是否一致 |
| 4.3 | 校准失败 | |offset|>90 时返回0 | 正常行为，调整站位重试 |

**关键测试**：
1. 采集2个航点（A→B，间距>10m）
2. 站在A点，面朝B点方向
3. 按 KEY_2 启动导航
4. 观察 offset 值：面朝B时 offset 应接近 0°

---

### 阶段 5：导航状态机验证（T08 + T09 + T10）

**目标**：确认状态机正确流转，双缓冲数据正确

**前置条件**：阶段4通过

**调试代码**：

```c
// 在 gps_nav_proc() 的每个状态入口添加
static const char *state_name[] = {"IDLE", "CAL", "NAV", "ARR", "DONE"};
printf("NAV: state=%s, wp=%d/%d, bear=%.1f, dist=%.1f, data_ready=%d\r\n",
       state_name[gps_nav_state],
       gps_wp_get_current_index(), gps_wp_get_count(),
       gps_steer_out.target_bearing_deg,
       gps_steer_out.distance_to_wp_m,
       gps_steer_out.data_ready);
```

**验证清单**：

| # | 检查项 | 期望结果 | 失败排查 |
|---|--------|---------|---------|
| 5.1 | IDLE→CAL | KEY_2后状态变为CAL | 不变→gps_nav_start()未调用或航点<2 |
| 5.2 | CAL→NAV | 校准成功后变为NAV | 回到IDLE→校准失败，检查offset |
| 5.3 | data_ready 周期 | NAV状态下每100ms置1 | 始终0→gnss_flag未触发gps_nav_proc |
| 5.4 | target_bearing_deg | 合理值[0,360) | 跳变→环形滤波未生效或首次未直赋值 |
| 5.5 | distance_to_wp_m | 逐渐减小 | 不变→GPS坐标未更新或航点错误 |
| 5.6 | NAV→ARR | 距离<2m时变为ARR | 不变→到达判定逻辑错误 |
| 5.7 | ARR→NAV | 切换到下一航点 | 直接COMPLETE→gps_wp_advance()返回0 |
| 5.8 | ARR→COMPLETE | 最后一个航点到达 | 正常行为 |

**关键测试**：采集3个航点（A→B→C），启动导航，观察状态流转 IDLE→CAL→NAV→ARR→NAV→ARR→COMPLETE

---

### 阶段 6：ISR 集成验证（T12 + T13）

**目标**：确认 turn_mode==5 时机器人能正确转向

**前置条件**：阶段5通过

**⚠️ 安全警告**：此阶段机器人开始运动！确保：
- 速度目标设为很低值（如 100-200）
- 手放在 KEY_3 上随时停车
- 在开阔场地测试

**调试代码**（在 Interrupt.c 的 turn_mode==5 分支中）：

```c
else if (turn_mode == 5)
{
    if (gps_steer_out.data_ready)
    {
        desired_yaw = gps_steer_out.target_bearing_deg
                    + gps_steer_out.imu_yaw_offset_deg;
        gps_steer_out.data_ready = 0;
    }
    
    // 调试输出（每50个4ms周期=200ms输出一次）
    static uint16 isr_dbg_cnt = 0;
    if (++isr_dbg_cnt >= 50)
    {
        isr_dbg_cnt = 0;
        printf("ISR5: yaw=%.1f, des=%.1f, out=%.0f, dr=%d\r\n",
               imu660ra.eulerAngle.yaw, desired_yaw,
               Yao.Outp_turn, gps_steer_out.data_ready);
    }
    
    Yao.Outp_turn = Cascade_angle_Yaw_4(&PID_all.Pid_turn2, erect_Angle_Yaw_3,
                      imu660ra.eulerAngle.yaw, desired_yaw);
    Yao.Outp_turn = limit(Yao.Outp_turn, 8000.0f);
}
```

**验证清单**：

| # | 检查项 | 期望结果 | 失败排查 |
|---|--------|---------|---------|
| 6.1 | desired_yaw 更新 | data_ready=1时更新 | 始终不变→双缓冲未写入或data_ready未置1 |
| 6.2 | 机器人转向 | 朝目标航点方向转 | 反向转→offset符号反或PID方向反 |
| 6.3 | 转向平滑 | 无剧烈抖动 | 抖动→PID参数不合适或滤波alpha太大 |
| 6.4 | 到达后停车 | COMPLETE时turn_mode=0 | 不停车→COMPLETE状态处理错误 |
| 6.5 | KEY_3 停车 | 立即停止 | 不停→gps_nav_stop()未正确重置 |

**关键测试**：
1. 采集2个航点（A→B，间距10-15m）
2. 站在A点，面朝任意方向
3. 按 KEY_2 启动导航
4. 观察机器人是否先转向B点方向，然后直线前进
5. 到达B点后应自动停车

---

### 阶段 7：Flash 持久化验证（T03）

**目标**：确认航点断电不丢失

**前置条件**：阶段2通过

**验证步骤**：

1. 采集 5 个航点
2. 按 KEY_2 长按保存到 Flash
3. 串口输出 `gps_wp_get_count()=5`
4. **断电重启**
5. 检查 `gps_wp_init()` 是否自动加载 → `gps_wp_get_count()=5`
6. 对比每个航点经纬度是否与保存前一致

**常见问题**：

| 问题 | 原因 | 解决 |
|------|------|------|
| 重启后 count=0 | Flash写入失败或魔数校验失败 | 检查 flash_buffer_clear + write_page 调用顺序 |
| 经纬度异常 | double↔uint32 拆合错误 | 单独测试 double_to_two_uint32 往返转换 |
| Flash写入卡死 | Flash页被其他模块占用 | 检查页50/51是否与Ins.c/ins_track冲突 |

---

## 三、调试变量速查表

| 变量 | 类型 | 位置 | 含义 | 观察时机 |
|------|------|------|------|---------|
| `gnss.state` | uint8 | zf_device_gnss.h | GPS有效性：0=无效, 1=有效 | 阶段1 |
| `gnss.latitude` | double | zf_device_gnss.h | 纬度（十进制度） | 阶段1 |
| `gnss.longitude` | double | zf_device_gnss.h | 经度（十进制度） | 阶段1 |
| `gnss.speed` | float | zf_device_gnss.h | 速度（km/h） | 阶段1 |
| `gnss.direction` | float | zf_device_gnss.h | 航向（度，0~360） | 阶段1 |
| `gnss.satellite_used` | uint8 | zf_device_gnss.h | 已用卫星数 | 阶段1 |
| `gnss_flag` | uint8 | zf_device_gnss.h | GPS数据就绪标志 | 阶段1 |
| `gps_wp_set.count` | uint8 | gps_waypoint.h | 当前航点数 | 阶段2 |
| `gps_wp_set.current_index` | uint8 | gps_waypoint.h | 当前目标航点索引 | 阶段5 |
| `gps_cal_offset_deg` | float | gps_calibration.h | IMU-GPS航向偏移量 | 阶段4 |
| `gps_nav_state` | enum | gps_nav.h | 导航状态机当前状态 | 阶段5 |
| `gps_steer_out.target_bearing_deg` | float | gps_nav.h | 滤波后目标方位角 | 阶段5 |
| `gps_steer_out.distance_to_wp_m` | float | gps_nav.h | 到目标航点距离 | 阶段5 |
| `gps_steer_out.data_ready` | uint8 | gps_nav.h | 双缓冲数据就绪标志 | 阶段5/6 |
| `imu660ra.eulerAngle.yaw` | float | imu660ra.h | IMU偏航角[-180,+180) | 阶段4/6 |
| `desired_yaw` | float | Interrupt.h | ISR中的目标偏航角 | 阶段6 |
| `turn_mode` | uint8 | Interrupt.h | 转向模式（5=GPS） | 阶段6 |
| `Yao.Outp_turn` | float | Interrupt.h | 转向PID输出 | 阶段6 |

---

## 四、常见故障诊断

### 故障 1：GPS 始终无信号

```
症状：gnss.state == 0, satellite_used == 0
排查路径：
  1. 检查天线是否连接 → 物理检查
  2. 检查 UART_2 是否有数据 → 在 uart2_isr 中加计数器
  3. 检查波特率 → 确认 gnss_init 中 115200
  4. 检查 NMEA 格式 → 串口直连GPS模块看原始数据
  5. 冷启动等待 → 室外首次定位需1-2分钟
```

### 故障 2：机器人转向方向反了

```
症状：应该左转却右转，或应该前进却后退
排查路径：
  1. 检查 gps_cal_offset_deg 符号 → offset正负是否与预期一致
  2. 检查 desired_yaw 计算 → target_bearing + offset 是否合理
  3. 检查 PID 方向 → Cascade_angle_Yaw_4 输出正负是否对应正确转向
  4. 检查 yaw 正方向 → 逆时针为正还是顺时针为正
  5. 临时修复：取反 offset 或取反 PID 输出
```

### 故障 3：desired_yaw 跳变导致机器人抖动

```
症状：机器人左右快速摆动
排查路径：
  1. 检查环形滤波 → angle_lpf_circular 是否正确处理±180°跨越
  2. 检查首次赋值 → 进入NAVIGATING时 lpf_bearing 是否直赋值
  3. 检查航点切换 → 切换航点后 lpf_bearing 是否重置
  4. 降低滤波系数 → GPS_NAV_LPF_ALPHA 从 0.3 降到 0.15
  5. 检查GPS更新频率 → gnss_flag 是否稳定10Hz
```

### 故障 4：到达航点后不停

```
症状：经过航点后继续前进
排查路径：
  1. 检查到达判定 → distance < GPS_NAV_ARRIVE_ENTER_M (2m)?
  2. 检查距离计算 → get_two_points_distance 返回值是否合理
  3. 检查 ARRIVED 状态 → 是否正确切换到下一航点或 COMPLETE
  4. 检查 COMPLETE 处理 → turn_mode 是否被设为 0
  5. 检查 turn_mode 被覆盖 → 是否有其他代码在修改 turn_mode
```

### 故障 5：GPS 坐标漂移严重

```
症状：静止时经纬度不断变化，距离读数不稳定
排查路径：
  1. 检查卫星数 → satellite_used < 6 时精度差
  2. 检查多径效应 → 是否在建筑物/金属结构附近
  3. 启用漂移修正 → gps_cal_drift_correction() 是否生效
  4. 增大到达阈值 → GPS_NAV_ARRIVE_ENTER_M 从 2m 增到 3m
  5. 增大滤波系数 → GPS_NAV_LPF_ALPHA 从 0.3 降到 0.15（更平滑）
```

---

## 五、调试输出模板

### 5.1 最小调试输出（阶段1-3）

```c
// 在 while 循环中，gnss_flag 处理后
static uint8 dbg_tick = 0;
if (++dbg_tick >= 10)  // 每10帧≈1秒
{
    dbg_tick = 0;
    printf("G:%d,%d,%.6f,%.6f,%.1f,%.1f|W:%d|C:%.1f\r\n",
           gnss.state, gnss.satellite_used,
           gnss.latitude, gnss.longitude,
           gnss.speed, gnss.direction,
           gps_wp_get_count(),
           gps_cal_offset_deg);
}
```

### 5.2 导航调试输出（阶段5-6）

```c
// 在 gps_nav_proc() 末尾
static uint8 nav_dbg_tick = 0;
if (++nav_dbg_tick >= 5)  // 每5帧≈0.5秒
{
    nav_dbg_tick = 0;
    printf("N:%d,WP:%d/%d,B:%.1f,D:%.1f,Y:%.1f,Des:%.1f,Off:%.1f\r\n",
           gps_nav_state,
           gps_wp_get_current_index(), gps_wp_get_count(),
           gps_steer_out.target_bearing_deg,
           gps_steer_out.distance_to_wp_m,
           imu660ra.eulerAngle.yaw,
           gps_steer_out.target_bearing_deg + gps_steer_out.imu_yaw_offset_deg,
           gps_steer_out.imu_yaw_offset_deg);
}
```

### 5.3 ISR 调试输出（阶段6，谨慎使用）

```c
// 在 turn_mode==5 分支中，每200ms输出一次
// ⚠️ printf 在 ISR 中不安全！仅调试时使用，正式版删除
static uint16 isr5_dbg = 0;
if (++isr5_dbg >= 50)  // 50×4ms=200ms
{
    isr5_dbg = 0;
    // 建议用全局变量传递，在main中printf
    // 而不是直接在ISR中调用printf
}
```

> ⚠️ **ISR 中禁止调用 printf**！正确做法：ISR 中将调试值写入全局变量，main while 循环中读取并输出。

---

## 六、调试安全守则

1. **速度限制**：调试导航时 `Yao.Target_Speed` 不超过 200
2. **随时停车**：手放在 KEY_3 上，异常时立即短按停车
3. **场地选择**：开阔平地，无障碍物，无悬崖/水域
4. **串口监控**：始终开启无线串口监控，观察状态变化
5. **逐步提速**：确认低速导航正确后，再逐步提高速度
6. **Flash 保护**：调试期间 `menu_open=0`，避免菜单误写 Flash
7. **备份参数**：修改 PID 参数前记录原始值
