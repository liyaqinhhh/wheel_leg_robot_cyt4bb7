# AI自动调节PID三环方案

## Context

用户希望通过串口通信实现AI自动化调节平衡车PID三环参数。当前项目使用CYT4BB7芯片，UART_0作为调试串口（115200波特率），已有printf重定向功能。

## 系统架构

```
┌─────────────────┐          UART_0 (115200)         ┌─────────────────┐
│                 │  ──────────────────────────────►  │                 │
│   CYT4BB7       │   实时数据 (角度/角速度/PID输出)    │   PC (Python)   │
│   单片机         │                                  │   AI调参算法     │
│                 │  ◄──────────────────────────────  │                 │
│                 │   参数更新 (Kp/Ki/Kd)              │                 │
└─────────────────┘                                  └─────────────────┘
```

## 第一步：单片机端 - 串口数据发送

### 修改文件：`project/user/main_cm7_0.c`

在主循环中添加数据发送，格式为JSON便于解析：

```c
// 数据发送函数
void send_pid_data(void)
{
    printf("{\"pitch\":%.2f,\"gyro\":%.2f,\"angle_out\":%.1f,\"gyro_out\":%.1f,\"speed_out\":%.1f}\n",
           aa1,                                    // 滤波后的角度
           y1,                                     // 滤波后的角速度
           Yao.Outp_Angle_Pitch,                   // 角度环输出
           Yao.Outp_Gyro_Pitch,                    // 角速度环输出
           Yao.Outp_Speed_Pitch);                  // 速度环输出
}
```

在主循环中定时发送（每50ms发一次）：

```c
// 在主循环中添加
static uint16_t send_cnt = 0;
send_cnt++;
if (send_cnt >= 50)  // 50ms发送一次
{
    send_cnt = 0;
    send_pid_data();
}
```

## 第二步：单片机端 - 串口参数接收

### 修改文件：`project/code/ControlPart/Interrupt.c`

添加串口接收解析函数：

```c
// 串口接收缓冲区
static char uart_rx_buf[128];
static uint8_t uart_rx_idx = 0;
static uint8_t uart_rx_flag = 0;

// 解析接收到的参数
void parse_pid_params(const char *buf)
{
    float kp_angle, ki_angle, kd_angle;
    float kp_gyro, ki_gyro, kd_gyro;
    float kp_speed, ki_speed, kd_speed;

    // 解析格式: "P:kp_a,ki_a,kd_a,kp_g,ki_g,kd_g,kp_s,ki_s,kd_s"
    if (sscanf(buf, "P:%f,%f,%f,%f,%f,%f,%f,%f,%f",
               &kp_angle, &ki_angle, &kd_angle,
               &kp_gyro, &ki_gyro, &kd_gyro,
               &kp_speed, &ki_speed, &kd_speed) == 9)
    {
        // 更新角度环参数
        erect_Angle_Pitch[0] = kp_angle;
        erect_Angle_Pitch[1] = ki_angle;
        erect_Angle_Pitch[2] = kd_angle;

        // 更新角速度环参数
        erect_Gyro_Pitch[0] = kp_gyro;
        erect_Gyro_Pitch[1] = ki_gyro;
        erect_Gyro_Pitch[2] = kd_gyro;

        // 更新速度环参数
        erect_Speed_Pitch[0] = kp_speed;
        erect_Speed_Pitch[1] = ki_speed;
        erect_Speed_Pitch[2] = kd_speed;

        // 复位PID积分项
        pid_para_init(&PID_all.Pid_Angle_Pitch);
        pid_para_init(&PID_all.Pid_Gyro_Pitch);
        pid_para_init(&PID_all.Pid_Speed_Pitch);

        printf("{\"status\":\"ok\",\"msg\":\"params updated\"}\n");
    }
}

// 在Interrupt_2ms中处理串口接收
void process_uart_rx(void)
{
    uint8_t ch;
    while (uart_query_byte(DEBUG_UART_INDEX, &ch))
    {
        if (ch == '\n' || ch == '\r')
        {
            if (uart_rx_idx > 0)
            {
                uart_rx_buf[uart_rx_idx] = '\0';
                parse_pid_params(uart_rx_buf);
                uart_rx_idx = 0;
            }
        }
        else if (uart_rx_idx < sizeof(uart_rx_buf) - 1)
        {
            uart_rx_buf[uart_rx_idx++] = ch;
        }
    }
}
```

## 第三步：PC端 - Python AI调参脚本

### 创建文件：`tools/pid_auto_tuner.py`

```python
import serial
import json
import time
import numpy as np
from bayes_opt import BayesianOptimization

# 串口配置
SERIAL_PORT = 'COM3'  # 根据实际修改
BAUD_RATE = 115200

# PID参数范围
PARAM_BOUNDS = {
    'kp_angle': (50, 500),
    'ki_angle': (0, 5),
    'kd_angle': (0, 50),
    'kp_gyro': (0.1, 2.0),
    'ki_gyro': (0, 0.5),
    'kd_gyro': (0, 1.0),
}

class PIDAutoTuner:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.data_buffer = []
        self.current_params = None

    def send_params(self, kp_a, ki_a, kd_a, kp_g, ki_g, kd_g):
        """发送PID参数到单片机"""
        cmd = f"P:{kp_a:.2f},{ki_a:.4f},{kd_a:.2f},{kp_g:.4f},{ki_g:.4f},{kd_g:.4f},0,0,0\n"
        self.ser.write(cmd.encode())
        time.sleep(0.1)

    def read_data(self, duration=5):
        """读取指定时间的数据"""
        data_list = []
        start_time = time.time()
        while time.time() - start_time < duration:
            line = self.ser.readline().decode().strip()
            if line:
                try:
                    data = json.loads(line)
                    data_list.append(data)
                except json.JSONDecodeError:
                    pass
        return data_list

    def evaluate_performance(self, data_list):
        """评估PID性能，返回代价（越小越好）"""
        if not data_list:
            return 1000  # 无数据返回最大代价

        angles = [d['pitch'] for d in data_list if 'pitch' in d]
        gyro_outputs = [d['gyro_out'] for d in data_list if 'gyro_out' in d]

        if not angles:
            return 1000

        # 计算评估指标
        angle_std = np.std(angles)           # 角度波动
        angle_mean = np.mean(np.abs(angles)) # 平均偏移
        gyro_std = np.std(gyro_outputs)      # 输出波动

        # 代价函数：角度波动小 + 偏移小 + 输出平滑
        cost = angle_std * 10 + angle_mean * 5 + gyro_std * 0.1
        return cost

    def objective(self, kp_angle, ki_angle, kd_angle, kp_gyro, ki_gyro, kd_gyro):
        """贝叶斯优化的目标函数"""
        # 发送参数
        self.send_params(kp_angle, ki_angle, kd_angle, kp_gyro, ki_gyro, kd_gyro)

        # 等待稳定
        time.sleep(2)

        # 采集数据
        data = self.read_data(duration=3)

        # 计算代价
        cost = self.evaluate_performance(data)
        print(f"Params: Kp_a={kp_angle:.1f}, Ki_a={ki_angle:.3f}, Kd_a={kd_angle:.1f}, "
              f"Kp_g={kp_gyro:.3f}, Ki_g={ki_gyro:.4f}, Kd_g={kd_gyro:.3f} -> Cost: {cost:.2f}")

        return -cost  # 贝叶斯优化最大化，所以取负

    def run_optimization(self, init_points=5, n_iter=25):
        """运行贝叶斯优化"""
        optimizer = BayesianOptimization(
            f=self.objective,
            pbounds=PARAM_BOUNDS,
            random_state=42,
        )

        optimizer.maximize(
            init_points=init_points,
            n_iter=n_iter,
        )

        print("\n========== 最优参数 ==========")
        print(optimizer.max)
        return optimizer.max

    def close(self):
        self.ser.close()

# 使用示例
if __name__ == '__main__':
    tuner = PIDAutoTuner(SERIAL_PORT, BAUD_RATE)

    try:
        # 先发送初始参数
        tuner.send_params(250, 0.5, 10, 0.95, 0, 0)

        # 运行优化
        best = tuner.run_optimization(init_points=5, n_iter=20)

        # 保存最优参数
        with open('best_pid_params.json', 'w') as f:
            json.dump(best['params'], f, indent=2)

    finally:
        tuner.close()
```

## 第四步：安装依赖

```bash
pip install pyserial bayesian-optimization numpy
```

## 实施步骤

### Step 1: 单片机端 - 数据发送（立即实现）
1. 修改 `main_cm7_0.c` - 在主循环中添加JSON格式数据发送
2. 编译烧录测试
3. 用串口助手验证数据格式

### Step 2: 单片机端 - 参数接收（后续实现）
1. 修改 `Interrupt.c` - 添加参数接收解析
2. 测试参数下发功能

### Step 3: PC端 - AI调参脚本（后续实现）
1. 安装Python依赖
2. 创建 `tools/pid_auto_tuner.py`
3. 运行自动调参

## 安全注意事项

1. **必须架起来测试**：车轮不能着地，防止飞车
2. **设置参数范围**：防止参数过大导致振荡
3. **手动停止**：按Ctrl+C可以随时停止
4. **先手动调好基础参数**：AI优化是在小范围内微调

## 关键文件

- `project/user/main_cm7_0.c` - 添加数据发送
- `project/code/ControlPart/Interrupt.c` - 添加参数接收
- `project/code/ControlPart/PID.c` - PID参数定义
- `tools/pid_auto_tuner.py` - PC端AI调参脚本（新建）
