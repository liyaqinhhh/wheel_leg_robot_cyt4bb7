#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI自动调节PID三环参数脚本
使用Claude API分析平衡车数据并自动调整PID参数
"""

import serial
import json
import time
import numpy as np
from anthropic import Anthropic

# ==================== 配置区域 ====================
SERIAL_PORT = 'COM28'  # 根据实际修改串口号
BAUD_RATE = 115200
CLAUDE_API_KEY = 'tp-cy98h5q3jncudr2vdq7jy3scgwqa0huasapirn144ccs60vr'  # 填入你的Claude API Key

# AI系统提示词
SYSTEM_PROMPT = """你是一个专业的PID控制工程师，你的任务是直接计算并输出新的PID参数值。

## 系统架构
平衡车使用三串级PID控制：
- 速度环（外环）→ 输出作为角度环设定值
- 角度环（中环）→ 输出作为角速度环设定值
- 角速度环（内环）→ 直接驱动电机

## 你的任务
根据单片机发来的实时数据，分析当前PID参数的性能，**直接计算并输出新的PID参数值**。
你的输出会被自动发送给单片机，无需人工确认。

## 数据说明
- pitch: 车体倾斜角度（度），理想值为0
- speed_out: 速度环输出，范围[-100, 100]
- angle_out: 角度环输出，范围[-12000, 12000]
- gyro_out: 角速度环输出，范围[-8000, 8000]，直接驱动电机
- motor_speed: 电机实际速度（编码器反馈）
- flag_main: 保护停转标志，1表示已触发保护停转（电机转速>3000或输出>7000）

## 特殊情况处理
当flag_main=1时，说明参数过大导致系统不稳定触发保护：
- 应该减小角度环KP（通常减小20-30%）
- 可能需要减小角速度环KP
- 在reason中说明是因保护停转而调整

## 分析要点
1. **稳定性**：pitch角波动是否过大？理想标准差<2度
2. **响应性**：受到扰动后能否快速恢复？
3. **平滑性**：gyro_out是否过于剧烈？会导致电机抖动
4. **零点偏移**：pitch均值不为0说明机械零点需要调整

## 输出格式
你必须且只能输出JSON格式，不要输出任何其他内容：
```json
{
  "params": {
    "kp_angle": 数值,
    "ki_angle": 数值,
    "kd_angle": 数值,
    "kp_gyro": 数值,
    "ki_gyro": 数值,
    "kd_gyro": 数值,
    "kp_speed": 数值,
    "ki_speed": 数值,
    "kd_speed": 数值,
    "offset_roll": 数值
  },
  "reason": "简短说明调整理由"
}
```

## 调参原则
- 角度环KP是主调参数，影响最大，范围[100, 400]
- 角速度环KP影响响应速度，过大会抖动，范围[0.1, 2.0]
- 速度环KP影响平衡点，过大会来回摆，范围[0.05, 0.3]
- KD项用于抑制振荡
- KI项用于消除稳态误差，但要谨慎使用
- offset_roll用于补偿机械安装误差，范围[-15, 5]
- 每次调整幅度不要超过当前值的30%，避免剧烈变化
"""


class AIPIDTuner:
    def __init__(self, serial_port, baudrate, claude_api_key):
        """初始化AI PID调参器"""
        self.ser = serial.Serial(serial_port, baudrate, timeout=1)
        self.client = Anthropic(api_key=claude_api_key)
        self.current_params = {
            'kp_angle': 215, 'ki_angle': 0, 'kd_angle': 20,
            'kp_gyro': 0.65, 'ki_gyro': 0, 'kd_gyro': 0,
            'kp_speed': 0.122, 'ki_speed': 0, 'kd_speed': 0,
            'offset_roll': -8.9
        }
        self.data_buffer = []
        self.history = []

    def collect_data(self, duration=5):
        """采集指定时间的实时数据"""
        data_list = []
        start = time.time()
        while time.time() - start < duration:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                try:
                    data = json.loads(line)
                    # 检查是否是状态消息
                    if 'status' in data:
                        print(f"单片机返回: {data}")
                        continue
                    data_list.append(data)
                except json.JSONDecodeError:
                    pass
        return data_list

    def analyze_with_ai(self, data_list):
        """使用Claude分析数据并直接获取新参数值"""
        # 计算统计数据
        pitches = [d['pitch'] for d in data_list]
        stats = {
            'pitch_mean': np.mean(pitches),
            'pitch_std': np.std(pitches),
            'pitch_max': np.max(np.abs(pitches)),
            'speed_out_mean': np.mean([d['speed_out'] for d in data_list]),
            'speed_out_std': np.std([d['speed_out'] for d in data_list]),
            'angle_out_mean': np.mean([d['angle_out'] for d in data_list]),
            'angle_out_std': np.std([d['angle_out'] for d in data_list]),
            'gyro_out_mean': np.mean([d['gyro_out'] for d in data_list]),
            'gyro_out_std': np.std([d['gyro_out'] for d in data_list]),
            'motor_speed_mean': np.mean([d['motor_speed'] for d in data_list]),
            'motor_speed_std': np.std([d['motor_speed'] for d in data_list]),
        }

        # 构建用户提示词
        user_prompt = f"""## 当前PID参数
- 角度环: KP={self.current_params['kp_angle']}, KI={self.current_params['ki_angle']}, KD={self.current_params['kd_angle']}
- 角速度环: KP={self.current_params['kp_gyro']}, KI={self.current_params['ki_gyro']}, KD={self.current_params['kd_gyro']}
- 速度环: KP={self.current_params['kp_speed']}, KI={self.current_params['ki_speed']}, KD={self.current_params['kd_speed']}
- 机械零点偏移: offset_roll={self.current_params['offset_roll']}

## 最近{len(data_list)/50:.1f}秒的统计数据
- pitch角: 均值={stats['pitch_mean']:.2f}°, 标准差={stats['pitch_std']:.2f}°, 最大偏移={stats['pitch_max']:.2f}°
- 速度环输出: 均值={stats['speed_out_mean']:.1f}, 标准差={stats['speed_out_std']:.1f}
- 角度环输出: 均值={stats['angle_out_mean']:.1f}, 标准差={stats['angle_out_std']:.1f}
- 角速度环输出: 均值={stats['gyro_out_mean']:.1f}, 标准差={stats['gyro_out_std']:.1f}
- 电机速度: 均值={stats['motor_speed_mean']:.1f}, 标准差={stats['motor_speed_std']:.1f}

请直接输出新的PID参数值。"""

        # 调用Claude API
        response = self.client.messages.create(
            model="claude-sonnet-4-20250514",
            max_tokens=512,
            system=SYSTEM_PROMPT,
            messages=[{"role": "user", "content": user_prompt}]
        )

        # 解析AI输出的参数
        ai_response = response.content[0].text
        json_start = ai_response.find('{')
        json_end = ai_response.rfind('}') + 1
        result = json.loads(ai_response[json_start:json_end])

        return result, stats

    def send_params(self, params):
        """发送参数到单片机"""
        cmd = f"P:{params['kp_angle']:.2f},{params['ki_angle']:.4f},{params['kd_angle']:.2f},"
        cmd += f"{params['kp_gyro']:.4f},{params['ki_gyro']:.4f},{params['kd_gyro']:.4f},"
        cmd += f"{params['kp_speed']:.4f},{params['ki_speed']:.4f},{params['kd_speed']:.4f},"
        cmd += f"{params['offset_roll']:.2f}\n"
        self.ser.write(cmd.encode())
        self.current_params = params.copy()
        print(f"已发送参数: {params}")

    def run_optimization_loop(self, iterations=10):
        """运行AI全自动优化循环（含安全保护）"""
        print("=" * 60)
        print("AI PID全自动调参系统启动")
        print("=" * 60)
        print(f"初始参数: {self.current_params}")
        print("AI将直接计算并应用新参数，无需人工干预")
        print("安全保护：电机转速>3000或输出>7000时自动停转")
        print("请确保平衡车已架起，车轮悬空！")
        print("=" * 60)

        for i in range(iterations):
            print(f"\n--- 第 {i+1}/{iterations} 轮 ---")

            # 1. 采集数据
            print("正在采集数据...")
            data = self.collect_data(duration=5)
            if len(data) < 10:
                print("数据不足，跳过本轮")
                continue

            print(f"采集到 {len(data)} 个数据点")

            # 2. 检查flag_main状态
            flag_main_values = [d.get('flag_main', 0) for d in data]
            if any(flag_main_values):
                print("\n" + "=" * 50)
                print("警告: 检测到保护停转！flag_main = 1")
                print("电机转速超过3000或输出超过7000")
                print("请按KEY_1复位后继续")
                print("=" * 50)

                # 等待用户复位
                while True:
                    data = self.collect_data(duration=2)
                    flag_main_values = [d.get('flag_main', 0) for d in data]
                    if not any(flag_main_values):
                        print("检测到复位成功，继续调参...")
                        break
                    else:
                        print("等待复位中... (按KEY_1复位)")
                        time.sleep(1)

                # AI分析停转原因并调整参数
                print("正在分析停转原因...")
                result, stats = self.analyze_with_ai(data)
                new_params = result['params']
                print(f"AI调整理由: {result['reason']}")
                print(f"调整后参数: {new_params}")

                # 应用调整后的参数
                self.send_params(new_params)
                time.sleep(3)
                continue

            # 3. AI分析并直接输出新参数
            print("正在请求AI分析并计算新参数...")
            result, stats = self.analyze_with_ai(data)

            new_params = result['params']
            print(f"AI调整理由: {result['reason']}")
            print(f"新参数: {new_params}")

            # 4. 自动应用新参数（无需人工确认）
            print("正在自动应用新参数...")
            self.send_params(new_params)
            time.sleep(3)  # 等待系统稳定

            # 5. 记录历史
            self.history.append({
                'iteration': i,
                'stats': stats,
                'old_params': self.current_params.copy(),
                'new_params': new_params,
                'reason': result['reason']
            })

            # 6. 检查是否达到目标（pitch标准差<1.5度）
            if stats['pitch_std'] < 1.5:
                print(f"\n已达到目标！pitch标准差={stats['pitch_std']:.2f}° < 1.5°")
                break

        # 保存最终参数
        final_params = {
            'erect_Gyro_Pitch': [self.current_params['kp_gyro'], self.current_params['ki_gyro'], self.current_params['kd_gyro'], 0],
            'erect_Angle_Pitch': [self.current_params['kp_angle'], self.current_params['ki_angle'], self.current_params['kd_angle'], 0],
            'erect_Speed_Pitch': [self.current_params['kp_speed'], self.current_params['ki_speed'], self.current_params['kd_speed'], 0],
            'offset_angle_roll': self.current_params['offset_roll']
        }
        with open('best_pid_params.json', 'w') as f:
            json.dump(final_params, f, indent=2)

        with open('ai_pid_tuning_history.json', 'w') as f:
            json.dump(self.history, f, indent=2, default=str)

        print("\n" + "=" * 60)
        print("调参完成")
        print(f"最终参数: {self.current_params}")
        print("参数已保存到 best_pid_params.json")
        print("历史记录已保存到 ai_pid_tuning_history.json")
        print("=" * 60)

    def close(self):
        """关闭串口连接"""
        self.ser.close()


if __name__ == '__main__':
    print("AI PID自动调参系统")
    print("请确保:")
    print("1. 平衡车已架起，车轮悬空")
    print("2. 无线串口模块已连接")
    print("3. 已设置正确的COM口号和API Key")
    print()

    # 检查配置
    if SERIAL_PORT == 'COM3':
        print("警告: 请修改 SERIAL_PORT 为实际的串口号")
    if CLAUDE_API_KEY == 'your-api-key-here':
        print("警告: 请修改 CLAUDE_API_KEY 为实际的API Key")
        exit(1)

    tuner = AIPIDTuner(SERIAL_PORT, BAUD_RATE, CLAUDE_API_KEY)

    try:
        # 运行优化循环
        tuner.run_optimization_loop(iterations=10)
    except KeyboardInterrupt:
        print("\n用户中断，正在保存当前状态...")
    finally:
        tuner.close()
        print("串口已关闭")
