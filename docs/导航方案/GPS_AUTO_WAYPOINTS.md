# GPS 航点自动生成方案（进阶功能）

> 本文档为进阶功能规划，当前阶段仅实现手动采点。自动生成作为未来扩展预留接口。

## 1. 核心思路

根据 PDF §9.2 的思路：**已知起点坐标 + 距离 + 方位角 → 反算目标点经纬度**。

用户只需提供：
- 起点经纬度（手动采集或从地图获取）
- 各段距离（米）
- 各段方位角（度，正北为 0°，顺时针）

系统自动计算出所有航点的 WGS84 坐标。

## 2. 数学公式

### 已知起点求终点

```
给定: 起点 (lat1, lng1), 距离 d (米), 方位角 θ (度)

lat2 = lat1 + (d × cos(θ)) / R
lng2 = lng1 + (d × sin(θ)) / (R × cos(lat1_rad))

其中:
  R = 6371000 米 (地球平均半径)
  lat1_rad = lat1 × π / 180
```

> ⚠️ 这是简化公式，适用于短距离（< 1km）。对于轮腿机器人场景完全足够。

### 精确公式（Vincenty / 大圆公式）

如需更高精度，可使用项目中已有的 `get_two_points_distance` 和 `get_two_points_azimuth` 的逆运算：

```c
// 现有函数：已知两点 → 求距离和方位角
double get_two_points_distance(lat1, lng1, lat2, lng2);  // 返回米
double get_two_points_azimuth(lat1, lng1, lat2, lng2);   // 返回度 [0,360)

// 需要实现的逆函数：已知起点+距离+方位角 → 求终点
void get_point_from_bearing_distance(lat1, lng1, bearing_deg, distance_m, &lat2, &lng2);
```

## 3. 输入格式设计

### 方案 A：代码内数组（最简单）

```c
// 在代码中直接定义路线描述
typedef struct {
    double distance_m;    // 本段距离（米）
    double bearing_deg;   // 本段方位角（度，正北=0，顺时针）
} GpsRouteSegment;

const GpsRouteSegment route_segments[] = {
    {15.0,  90.0},   // 向东 15 米
    {10.0,   0.0},   // 向北 10 米
    {15.0, 270.0},   // 向西 15 米
    {10.0, 180.0},   // 向南 10 米（回到起点附近）
};
```

### 方案 B：串口输入（推荐进阶）

通过逐飞助手或自定义串口协议，发送 JSON 格式的路线描述：

```json
{
  "start_lat": 30.123456,
  "start_lng": 120.654321,
  "segments": [
    {"distance": 15.0, "bearing": 90.0},
    {"distance": 10.0, "bearing": 0.0},
    {"distance": 15.0, "bearing": 270.0},
    {"distance": 10.0, "bearing": 180.0}
  ]
}
```

### 方案 C：地图工具导出（最精确）

使用 Google Earth / 高德地图 等工具在地图上标记航点，导出 KML 文件，再转换为 C 数组。

## 4. 实现步骤

```
Step 1: 实现 get_point_from_bearing_distance() 函数
Step 2: 定义路线描述结构体 GpsRouteSegment
Step 3: 实现路线解析函数 gps_route_generate_waypoints()
        - 输入：起点 + segments 数组
        - 输出：航点数组 gps_waypoints[]
        - 逐段累加计算每个航点坐标
Step 4: 将生成的航点写入 Flash（复用手动采点的存储接口）
Step 5: （可选）串口协议解析
```

## 5. 与手动采点的接口统一

无论手动采点还是自动生成，最终都写入相同的航点数组结构：

```c
typedef struct {
    double latitude;    // WGS84 纬度
    double longitude;   // WGS84 经度
} GpsWaypoint;

// 统一存储
GpsWaypoint gps_waypoints[40];
uint8 gps_waypoint_count;
```

手动采点：按键时读取 `gnss.latitude` / `gnss.longitude` 写入数组
自动生成：调用 `gps_route_generate_waypoints()` 填充数组

两者共享同一套 Flash 存储和导航逻辑。

## 6. 误差说明

| 误差来源 | 量级 | 说明 |
|----------|------|------|
| 简化公式 | < 0.1m (100m内) | 对轮腿机器人可忽略 |
| GPS 定位 | 1~3m | 单点定位精度 |
| 方位角测量 | 1~5° | 取决于双天线/罗盘精度 |
| 累积误差 | 随航点数增加 | 每段误差会累积 |

> 建议：路线总长 < 200m、航点数 < 20 时，自动生成精度可接受。
