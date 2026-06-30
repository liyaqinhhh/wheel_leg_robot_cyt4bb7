#!/usr/bin/env python3
"""
遥控器上位机测试脚本 — 轮腿机器人 CYT4BB7

通过串口向机器人发送二进制控制帧，支持:
  - WASD 键盘控制摇杆
  - 数字键切换 turn_mode
  - Q/E 调节腿高
  - Space 急停
  - 自动心跳保活

协议格式: [0x5A] [len] [type] [data...] [checksum]
  len = 1(type) + len(data) + 1(checksum)
  checksum = len XOR type XOR data[0] XOR ... XOR data[n-1]
"""

import serial
import struct
import threading
import time
import sys

# ==================== 帧协议常量 ====================
FRAME_HEADER    = 0x5A
TYPE_JOYSTICK   = 0x01
TYPE_MODE       = 0x02
TYPE_HEIGHT     = 0x03
TYPE_EMERGENCY  = 0x04
TYPE_HEARTBEAT  = 0x05

HEIGHT_HOLD     = 0
HEIGHT_UP       = 1
HEIGHT_DOWN     = 2

EMERGENCY_STOP  = 1
EMERGENCY_RESUME = 0

# ==================== 串口配置 ====================
SERIAL_PORT = "COM3"    # 修改为实际串口
SERIAL_BAUD = 115200

# ==================== 摇杆参数 ====================
JOYSTICK_CENTER   = 0
JOYSTICK_MIN      = -1000
JOYSTICK_MAX      = 1000
JOYSTICK_STEP     = 100
JOYSTICK_DEADZONE = 50

# ==================== 帧构建 ====================

def build_frame(frame_type: int, data: bytes = b"") -> bytes:
    """构建完整二进制帧: [0x5A][len][type][data...][checksum]"""
    length = 1 + len(data) + 1  # type + data + checksum
    frame = struct.pack("<BBB", FRAME_HEADER, length, frame_type) + data
    # checksum = len XOR type XOR data[0] XOR ... XOR data[n-1]
    checksum = length ^ frame_type
    for b in data:
        checksum ^= b
    frame += struct.pack("<B", checksum & 0xFF)
    return frame


def build_joystick_frame(j1: int, j2: int, j3: int = 1500, j4: int = 1500) -> bytes:
    """构建摇杆帧: 4×int16 (8字节)"""
    data = struct.pack("<hhhh", j1, j2, j3, j4)
    return build_frame(TYPE_JOYSTICK, data)


def build_mode_frame(turn_mode: int) -> bytes:
    """构建模式切换帧: 1×uint8"""
    return build_frame(TYPE_MODE, struct.pack("<B", turn_mode))


def build_height_frame(direction: int) -> bytes:
    """构建高度调节帧: 1×uint8"""
    return build_frame(TYPE_HEIGHT, struct.pack("<B", direction))


def build_emergency_frame(stop: int) -> bytes:
    """构建急停帧: 1×uint8"""
    return build_frame(TYPE_EMERGENCY, struct.pack("<B", stop))


def build_heartbeat_frame() -> bytes:
    """构建心跳帧: 无数据"""
    return build_frame(TYPE_HEARTBEAT)


# ==================== 串口管理 ====================

class SerialPort:
    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.ser = None
        self.lock = threading.Lock()

    def open(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            print(f"[串口] 已打开 {self.port} @ {self.baud}")
            return True
        except serial.SerialException as e:
            print(f"[串口] 打开失败: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[串口] 已关闭")

    def send(self, data: bytes):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.write(data)

    def read_response(self):
        """读取机器人返回的遥测数据"""
        if self.ser and self.ser.is_open:
            if self.ser.in_waiting > 0:
                return self.ser.read(self.ser.in_waiting)
        return None


# ==================== 遥控状态 ====================

class RemoteState:
    def __init__(self):
        self.j1 = JOYSTICK_CENTER  # 前后
        self.j2 = JOYSTICK_CENTER  # 左右
        self.turn_mode = 0
        self.running = True
        self.emergency = False

    def reset_joystick(self):
        self.j1 = JOYSTICK_CENTER
        self.j2 = JOYSTICK_CENTER


# ==================== 心跳线程 ====================

def heartbeat_thread(ser: SerialPort, state: RemoteState):
    """每200ms发送心跳帧保活"""
    while state.running:
        if not state.emergency:
            ser.send(build_heartbeat_frame())
        time.sleep(0.2)


# ==================== 接收线程 ====================

def receive_thread(ser: SerialPort, state: RemoteState):
    """接收机器人返回的遥测数据"""
    while state.running:
        data = ser.read_response()
        if data:
            try:
                text = data.decode("utf-8", errors="replace")
                # 只打印非二进制数据 (遥测文本)
                if text.startswith("$") or text.startswith("{"):
                    print(f"\r[遥测] {text.strip()}", end="", flush=True)
            except:
                pass
        time.sleep(0.01)


# ==================== 键盘输入 ====================

def get_key():
    """读取单个按键 (Windows)"""
    import msvcrt
    if msvcrt.kbhit():
        return msvcrt.getch().decode("utf-8", errors="replace").lower()
    return None


def print_help():
    print("=" * 50)
    print("  轮腿机器人遥控器上位机")
    print("=" * 50)
    print("  W/S  - 前进/后退 (摇杆Y轴)")
    print("  A/D  - 左转/右转 (摇杆X轴)")
    print("  0/2/3/6 - 切换 turn_mode")
    print("  Q/E  - 升高/降低腿高")
    print("  Space - 急停")
    print("  R    - 恢复 (解除急停)")
    print("  H    - 发送心跳")
    print("  Esc  - 退出")
    print("=" * 50)


# ==================== 主循环 ====================

def main():
    ser = SerialPort(SERIAL_PORT, SERIAL_BAUD)
    if not ser.open():
        print("请检查串口连接，退出...")
        return

    state = RemoteState()

    # 启动心跳线程
    hb_thread = threading.Thread(target=heartbeat_thread, args=(ser, state), daemon=True)
    hb_thread.start()

    # 启动接收线程
    rx_thread = threading.Thread(target=receive_thread, args=(ser, state), daemon=True)
    rx_thread.start()

    print_help()
    print("\n[就绪] 等待键盘输入...")

    try:
        while state.running:
            key = get_key()
            if key is None:
                time.sleep(0.02)
                continue

            # ---- 前后 ----
            if key == "w":
                state.j1 = min(state.j1 + JOYSTICK_STEP, JOYSTICK_MAX)
                ser.send(build_joystick_frame(state.j1, state.j2))
                print(f"\r[摇杆] 前后={state.j1} 左右={state.j2}        ", end="", flush=True)

            elif key == "s":
                state.j1 = max(state.j1 - JOYSTICK_STEP, JOYSTICK_MIN)
                ser.send(build_joystick_frame(state.j1, state.j2))
                print(f"\r[摇杆] 前后={state.j1} 左右={state.j2}        ", end="", flush=True)

            # ---- 左右 ----
            elif key == "a":
                state.j2 = max(state.j2 - JOYSTICK_STEP, JOYSTICK_MIN)
                ser.send(build_joystick_frame(state.j1, state.j2))
                print(f"\r[摇杆] 前后={state.j1} 左右={state.j2}        ", end="", flush=True)

            elif key == "d":
                state.j2 = min(state.j2 + JOYSTICK_STEP, JOYSTICK_MAX)
                ser.send(build_joystick_frame(state.j1, state.j2))
                print(f"\r[摇杆] 前后={state.j1} 左右={state.j2}        ", end="", flush=True)

            # ---- 模式切换 ----
            elif key in ("0", "2", "3", "6"):
                mode = int(key)
                state.turn_mode = mode
                ser.send(build_mode_frame(mode))
                print(f"\r[模式] turn_mode={mode}        ", end="", flush=True)

            # ---- 高度调节 ----
            elif key == "q":
                ser.send(build_height_frame(HEIGHT_UP))
                print(f"\r[高度] 升高        ", end="", flush=True)

            elif key == "e":
                ser.send(build_height_frame(HEIGHT_DOWN))
                print(f"\r[高度] 降低        ", end="", flush=True)

            # ---- 急停 ----
            elif key == " ":
                state.emergency = True
                state.reset_joystick()
                ser.send(build_emergency_frame(EMERGENCY_STOP))
                ser.send(build_joystick_frame(JOYSTICK_CENTER, JOYSTICK_CENTER))
                print(f"\r[急停] !!! 已触发急停 !!!        ", end="", flush=True)

            # ---- 恢复 ----
            elif key == "r":
                state.emergency = False
                ser.send(build_emergency_frame(EMERGENCY_RESUME))
                print(f"\r[恢复] 已解除急停        ", end="", flush=True)

            # ---- 手动心跳 ----
            elif key == "h":
                ser.send(build_heartbeat_frame())
                print(f"\r[心跳] 已发送        ", end="", flush=True)

            # ---- 退出 ----
            elif key == "\x1b":  # ESC
                print("\n[退出] 发送停车指令...")
                state.reset_joystick()
                ser.send(build_joystick_frame(JOYSTICK_CENTER, JOYSTICK_CENTER))
                ser.send(build_emergency_frame(EMERGENCY_STOP))
                time.sleep(0.1)
                state.running = False

    except KeyboardInterrupt:
        print("\n[中断] 退出...")
        state.running = False
    finally:
        ser.close()


if __name__ == "__main__":
    main()
