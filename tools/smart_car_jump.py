#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
跳跃诊断遥测采集工具 (Jump Telemetry Logger)
=============================================
功能：
  - 自动扫描/手动切换串口
  - 实时解析 $J 跳跃诊断遥测帧
  - 可视化显示实时数据曲线
  - 开始/暂停/结束测试
  - 自动保存日志到 doc/ 文件夹，文件名包含 jump 标识与时间戳

$J 帧格式 (CSV, \r\n 结束):
  $J,flag_jump,time_j,flag_jump_1,pitch,gyro_x_lpf,gx_raw,gyro_y,gyro_z,
     Outp_Gyro_Pitch,Outp_Angle_Pitch,
     servoLF,servoLR,servoRF,servoRR,speed_left,speed_right,
     y1_gyro_lpf,flag_stop,turn_mode

依赖: pyserial, tkinter (内置)
用法: python smart_car_jump.py
"""

import os
import sys
import time
import csv
import threading
import queue
from datetime import datetime
from collections import deque

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("请先安装 pyserial: pip install pyserial")
    sys.exit(1)

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

# ====== 全局变量 ======
SMART_CAR_JUMP = {
    "version": "1.0.0",
    "serial_port": None,
    "serial_thread": None,
    "recording": False,
    "paused": False,
    "data_buffer": [],  # list of dict
    "data_queue": queue.Queue(),
    "stop_event": threading.Event(),
    "log_dir": None,
    "csv_file": None,
    "csv_writer": None,
}

# 历史数据缓存（用于曲线绘制）
MAX_HISTORY = 500
history = {
    "time_j": deque(maxlen=MAX_HISTORY),
    "pitch": deque(maxlen=MAX_HISTORY),
    "gyro_x": deque(maxlen=MAX_HISTORY),
    "gyro_z": deque(maxlen=MAX_HISTORY),
    "outp_gyro": deque(maxlen=MAX_HISTORY),
    "outp_angle": deque(maxlen=MAX_HISTORY),
    "speed_left": deque(maxlen=MAX_HISTORY),
    "speed_right": deque(maxlen=MAX_HISTORY),
    "x_left": deque(maxlen=MAX_HISTORY),
    "y_left": deque(maxlen=MAX_HISTORY),
    "x_right": deque(maxlen=MAX_HISTORY),
    "y_right": deque(maxlen=MAX_HISTORY),
    "flag_jump": deque(maxlen=MAX_HISTORY),
    "flag_jump_1": deque(maxlen=MAX_HISTORY),
    "flag_stop": deque(maxlen=MAX_HISTORY),
}

# 数据字段定义
FIELD_NAMES = [
    "flag_jump", "time_j", "flag_jump_1",
    "pitch", "gyro_x_lpf", "gx_raw", "gyro_y", "gyro_z",
    "Outp_Gyro_Pitch", "Outp_Angle_Pitch",
    "servoLF", "servoLR", "servoRF", "servoRR",
    "speed_left", "speed_right",
    "y1_gyro_lpf", "flag_stop", "turn_mode"
]


def scan_serial_ports():
    """扫描可用串口"""
    ports = serial.tools.list_ports.comports()
    result = []
    for p in ports:
        result.append({
            "device": p.device,
            "description": p.description,
            "hwid": p.hwid,
        })
    return result


def parse_jump_frame(line):
    """解析 $J 帧，返回 dict 或 None"""
    line = line.strip()
    if not line.startswith("$J"):
        return None
    parts = line.split(",")
    if len(parts) < 20:
        return None
    try:
        return {
            "flag_jump": int(parts[1]),
            "time_j": int(parts[2]),
            "flag_jump_1": int(parts[3]),
            "pitch": float(parts[4]),
            "gyro_x_lpf": float(parts[5]),
            "gx_raw": float(parts[6]),
            "gyro_y": float(parts[7]),
            "gyro_z": float(parts[8]),
            "Outp_Gyro_Pitch": int(parts[9]),
            "Outp_Angle_Pitch": int(parts[10]),
            "servoLF": float(parts[11]),
            "servoLR": float(parts[12]),
            "servoRF": float(parts[13]),
            "servoRR": float(parts[14]),
            "speed_left": int(parts[15]),
            "speed_right": int(parts[16]),
            "y1_gyro_lpf": float(parts[17]),
            "flag_stop": int(parts[18]),
            "turn_mode": int(parts[19]),
        }
    except (ValueError, IndexError):
        return None


def get_doc_dir():
    """获取 doc 目录（脚本同级）"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    doc_dir = os.path.join(script_dir, "doc")
    if not os.path.exists(doc_dir):
        os.makedirs(doc_dir)
    return doc_dir


def generate_filename():
    """生成带时间戳的文件名"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"jump_test_{timestamp}.csv"


def serial_reader_thread(port_name, baudrate=115200):
    """串口读取线程"""
    ser = None
    try:
        ser = serial.Serial(port_name, baudrate, timeout=0.1)
        ser.reset_input_buffer()
        SMART_CAR_JUMP["serial_port"] = ser
        line_buf = ""

        while not SMART_CAR_JUMP["stop_event"].is_set():
            try:
                if ser.in_waiting:
                    raw = ser.read(ser.in_waiting)
                    try:
                        text = raw.decode("utf-8", errors="replace")
                    except Exception:
                        text = raw.decode("latin-1", errors="replace")
                    line_buf += text

                    while "\n" in line_buf:
                        idx = line_buf.index("\n")
                        line = line_buf[:idx].rstrip("\r")
                        line_buf = line_buf[idx + 1:]

                        if line.startswith("$J"):
                            data = parse_jump_frame(line)
                            if data and SMART_CAR_JUMP["recording"]:
                                if not SMART_CAR_JUMP["paused"]:
                                    SMART_CAR_JUMP["data_queue"].put(data)
                                    SMART_CAR_JUMP["data_buffer"].append(data)
                                    if SMART_CAR_JUMP["csv_writer"]:
                                        row = [data.get(f, "") for f in FIELD_NAMES]
                                        SMART_CAR_JUMP["csv_writer"].writerow(row)
                else:
                    time.sleep(0.001)
            except serial.SerialException as e:
                SMART_CAR_JUMP["data_queue"].put({"error": f"串口错误: {e}"})
                break
            except Exception as e:
                SMART_CAR_JUMP["data_queue"].put({"error": f"读取错误: {e}"})
    except serial.SerialException as e:
        SMART_CAR_JUMP["data_queue"].put({"error": f"无法打开串口 {port_name}: {e}"})
    finally:
        if ser and ser.is_open:
            ser.close()
        SMART_CAR_JUMP["serial_port"] = None


class JumpTelemetryApp:
    """跳跃遥测采集 GUI 应用"""

    def __init__(self, root):
        self.root = root
        self.root.title("跳跃诊断遥测采集工具 - Jump Telemetry Logger")
        self.root.geometry("1280x820")
        self.root.minsize(1024, 700)

        self.style = ttk.Style()
        self.style.theme_use("clam")

        self.update_timer = None
        self.record_count = 0

        self._build_ui()
        self._refresh_ports()
        self._start_update_loop()

    def _build_ui(self):
        """构建界面"""
        # ---- 顶部控制栏 ----
        control_frame = ttk.Frame(self.root, padding=5)
        control_frame.pack(fill=tk.X, side=tk.TOP)

        ttk.Label(control_frame, text="串口:").pack(side=tk.LEFT, padx=(0, 2))
        self.port_var = tk.StringVar(value="COM4")
        self.port_combo = ttk.Combobox(
            control_frame, textvariable=self.port_var, width=12, state="readonly"
        )
        self.port_combo.pack(side=tk.LEFT, padx=2)

        ttk.Button(
            control_frame, text="扫描端口", command=self._refresh_ports, width=10
        ).pack(side=tk.LEFT, padx=2)

        ttk.Label(control_frame, text="波特率:").pack(side=tk.LEFT, padx=(8, 2))
        self.baud_var = tk.StringVar(value="115200")
        baud_combo = ttk.Combobox(
            control_frame,
            textvariable=self.baud_var,
            width=8,
            values=["9600","19200","38400","57600","115200","230400","460800","921600"],
        )
        baud_combo.pack(side=tk.LEFT, padx=2)

        self.connect_btn = ttk.Button(
            control_frame, text="连接串口", command=self._toggle_connection, width=10
        )
        self.connect_btn.pack(side=tk.LEFT, padx=(8, 2))

        ttk.Separator(control_frame, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=8, pady=2
        )

        self.start_btn = ttk.Button(
            control_frame, text="开始测试", command=self._start_recording,
            width=11, state=tk.DISABLED
        )
        self.start_btn.pack(side=tk.LEFT, padx=2)

        self.pause_btn = ttk.Button(
            control_frame, text="暂停", command=self._toggle_pause,
            width=8, state=tk.DISABLED
        )
        self.pause_btn.pack(side=tk.LEFT, padx=2)

        self.stop_btn = ttk.Button(
            control_frame, text="结束测试", command=self._stop_recording,
            width=11, state=tk.DISABLED
        )
        self.stop_btn.pack(side=tk.LEFT, padx=2)

        ttk.Separator(control_frame, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=8, pady=2
        )

        self.status_label = ttk.Label(
            control_frame, text="未连接", foreground="gray"
        )
        self.status_label.pack(side=tk.LEFT, padx=4)

        self.count_label = ttk.Label(control_frame, text="帧数: 0")
        self.count_label.pack(side=tk.LEFT, padx=4)

        self.jump_state_label = ttk.Label(control_frame, text="", foreground="orange")
        self.jump_state_label.pack(side=tk.LEFT, padx=4)

        self.save_path_var = tk.StringVar(value="")
        ttk.Label(control_frame, text="保存:").pack(side=tk.LEFT, padx=(12, 2))
        self.save_path_entry = ttk.Entry(
            control_frame, textvariable=self.save_path_var, width=40, state="readonly"
        )
        self.save_path_entry.pack(side=tk.LEFT, padx=2)

        # ---- 主面板 ----
        main_paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        left_frame = ttk.Frame(main_paned, padding=5)
        main_paned.add(left_frame, weight=1)
        self._build_value_panel(left_frame)

        right_frame = ttk.Frame(main_paned, padding=5)
        main_paned.add(right_frame, weight=2)
        self._build_log_panel(right_frame)

        # 底部
        bottom_frame = ttk.Frame(self.root, padding=3)
        bottom_frame.pack(fill=tk.X, side=tk.BOTTOM)
        self.detail_label = ttk.Label(
            bottom_frame, text="就绪 - 请选择串口并点击连接串口"
        )
        self.detail_label.pack(side=tk.LEFT)

    def _build_value_panel(self, parent):
        nb = ttk.Notebook(parent)
        nb.pack(fill=tk.BOTH, expand=True)
        self.val_labels = {}

        # 页1: 跳跃状态
        f1 = ttk.Frame(nb, padding=8)
        nb.add(f1, text="跳跃状态")
        rows1 = [
            ("flag_jump", "跳跃使能", "0"),
            ("time_j", "跳跃计时", "0"),
            ("flag_jump_1", "放腿阶段", "0"),
            ("flag_stop", "停止标志", "1"),
            ("turn_mode", "转向模式", "7"),
        ]
        for i, (key, label, default) in enumerate(rows1):
            ttk.Label(f1, text=label+":", font=("",9,"bold")).grid(
                row=i, column=0, sticky=tk.W, padx=4, pady=3
            )
            lbl = ttk.Label(f1, text=default, font=("Consolas",12), foreground="#2c3e50")
            lbl.grid(row=i, column=1, sticky=tk.W, padx=8, pady=3)
            self.val_labels[key] = lbl

        # 页2: 姿态
        f2 = ttk.Frame(nb, padding=8)
        nb.add(f2, text="姿态/陀螺仪")
        rows2 = [
            ("pitch", "Pitch", "0.00"),
            ("gyro_x_lpf", "GyroX 滤波", "0.00"),
            ("gx_raw", "GyroX 原始", "0.00"),
            ("gyro_y", "GyroY", "0.00"),
            ("gyro_z", "GyroZ", "0.00"),
            ("y1_gyro_lpf", "y1 低通滤波", "0.00"),
        ]
        for i, (key, label, default) in enumerate(rows2):
            ttk.Label(f2, text=label+":", font=("",9,"bold")).grid(
                row=i, column=0, sticky=tk.W, padx=4, pady=3
            )
            lbl = ttk.Label(f2, text=default, font=("Consolas",12), foreground="#2c3e50")
            lbl.grid(row=i, column=1, sticky=tk.W, padx=8, pady=3)
            self.val_labels[key] = lbl

        # 页3: PID
        f3 = ttk.Frame(nb, padding=8)
        nb.add(f3, text="PID输出")
        rows3 = [
            ("Outp_Gyro_Pitch", "角速度环输出", "0"),
            ("Outp_Angle_Pitch", "角度环输出", "0"),
        ]
        for i, (key, label, default) in enumerate(rows3):
            ttk.Label(f3, text=label+":", font=("",9,"bold")).grid(
                row=i, column=0, sticky=tk.W, padx=4, pady=3
            )
            lbl = ttk.Label(f3, text=default, font=("Consolas",12), foreground="#e74c3c")
            lbl.grid(row=i, column=1, sticky=tk.W, padx=8, pady=3)
            self.val_labels[key] = lbl

        # 页4: 舵机角度
        f4 = ttk.Frame(nb, padding=8)
        nb.add(f4, text="舵机角度/轮速")
        rows4 = [
            ("servoLF", "前左 LF", "0.0"),
            ("servoLR", "后左 LB", "0.0"),
            ("servoRF", "前右 RF", "0.0"),
            ("servoRR", "后右 RB", "0.0"),
            ("speed_left", "左轮速", "0"),
            ("speed_right", "右轮速", "0"),
        ]
        for i, (key, label, default) in enumerate(rows4):
            ttk.Label(f4, text=label+":", font=("",9,"bold")).grid(
                row=i, column=0, sticky=tk.W, padx=4, pady=3
            )
            lbl = ttk.Label(f4, text=default, font=("Consolas",12), foreground="#16a085")
            lbl.grid(row=i, column=1, sticky=tk.W, padx=8, pady=3)
            self.val_labels[key] = lbl

    def _build_log_panel(self, parent):
        ttk.Label(parent, text="原始遥测日志 ($J 帧):", font=("",9,"bold")).pack(anchor=tk.W)
        self.log_text = scrolledtext.ScrolledText(
            parent, height=12, font=("Consolas",9), wrap=tk.NONE
        )
        self.log_text.pack(fill=tk.BOTH, expand=True, pady=(0,5))

        ttk.Label(parent, text="实时趋势图:", font=("",9,"bold")).pack(anchor=tk.W)
        self.canvas = tk.Canvas(parent, height=200, bg="#1a1a2e", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda e: self._draw_canvas())

    def _draw_canvas(self):
        c = self.canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10:
            return

        c.create_rectangle(0, 0, w, h, fill="#1a1a2e", outline="")
        grid_color = "#2d2d44"
        for i in range(1, 10):
            y = h * i / 10
            c.create_line(0, y, w, y, fill=grid_color, dash=(2,4))
        for i in range(1, 20):
            x = w * i / 20
            c.create_line(x, 0, x, h, fill=grid_color, dash=(2,4))

        datasets = [
            ("pitch", "#3498db", "Pitch"),
            ("gyro_x", "#e74c3c", "GyroX"),
            ("outp_gyro", "#2ecc71", "OutpGyro"),
            ("outp_angle", "#f39c12", "OutpAngle"),
        ]

        for ds_key, color, label in datasets:
            vals = list(history.get(ds_key, []))
            if len(vals) < 2:
                continue
            mn, mx = min(vals), max(vals)
            if abs(mx - mn) < 1e-6:
                mx = mn + 1.0
            points = []
            n = len(vals)
            for i, v in enumerate(vals):
                x = w * i / max(n - 1, 1)
                y = h - ((v - mn) / (mx - mn)) * (h - 20) - 10
                points.extend([x, y])
            if len(points) >= 4:
                c.create_line(*points, fill=color, width=1.5, smooth=True)

        legend_y = 10
        for ds_key, color, label in datasets:
            c.create_rectangle(10, legend_y, 22, legend_y+10, fill=color, outline="")
            c.create_text(28, legend_y+5, text=label, anchor=tk.W, fill="#cccccc", font=("",7))
            legend_y += 14

    def _refresh_ports(self):
        ports = scan_serial_ports()
        port_names = [p["device"] for p in ports]
        self.port_combo["values"] = port_names
        if port_names:
            if "COM4" in port_names:
                self.port_var.set("COM4")
            elif not self.port_var.get() or self.port_var.get() not in port_names:
                self.port_var.set(port_names[0])
        else:
            self.port_var.set("")
        info = (
            ", ".join(f"{p['device']}({p['description'][:20]})" for p in ports)
            if ports else "未检测到串口"
        )
        self.detail_label.config(text=f"可用串口: {info}")

    def _toggle_connection(self):
        if SMART_CAR_JUMP["serial_thread"] and SMART_CAR_JUMP["serial_thread"].is_alive():
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("提示", "请先选择串口")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("错误", "波特率无效")
            return

        self._append_log(f"[系统] 正在连接 {port} @ {baud}...")
        SMART_CAR_JUMP["stop_event"].clear()
        t = threading.Thread(
            target=serial_reader_thread, args=(port, baud), daemon=True
        )
        t.start()
        SMART_CAR_JUMP["serial_thread"] = t
        time.sleep(0.5)

        if SMART_CAR_JUMP["serial_port"] and SMART_CAR_JUMP["serial_port"].is_open:
            self.connect_btn.config(text="断开串口")
            self.status_label.config(text="已连接", foreground="green")
            self.start_btn.config(state=tk.NORMAL)
            self._append_log(f"[系统] 已连接 {port}")
        else:
            self._append_log(f"[系统] 连接 {port} 失败")

    def _disconnect(self):
        SMART_CAR_JUMP["stop_event"].set()
        if SMART_CAR_JUMP["serial_thread"]:
            SMART_CAR_JUMP["serial_thread"].join(timeout=2)
            SMART_CAR_JUMP["serial_thread"] = None
        self.connect_btn.config(text="连接串口")
        self.status_label.config(text="已断开", foreground="red")
        self.start_btn.config(state=tk.DISABLED)
        self.pause_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        self._append_log("[系统] 已断开串口")

    def _start_recording(self):
        doc_dir = get_doc_dir()
        filename = generate_filename()
        filepath = os.path.join(doc_dir, filename)
        try:
            f = open(filepath, "w", newline="", encoding="utf-8-sig")
            writer = csv.writer(f)
            writer.writerow(FIELD_NAMES)
            SMART_CAR_JUMP["csv_file"] = f
            SMART_CAR_JUMP["csv_writer"] = writer
        except Exception as e:
            messagebox.showerror("错误", f"无法创建文件: {e}")
            return

        SMART_CAR_JUMP["data_buffer"].clear()
        SMART_CAR_JUMP["recording"] = True
        SMART_CAR_JUMP["paused"] = False
        self.record_count = 0
        self.save_path_var.set(filepath)
        self.start_btn.config(state=tk.DISABLED)
        self.pause_btn.config(state=tk.NORMAL, text="暂停")
        self.stop_btn.config(state=tk.NORMAL)
        self.status_label.config(text="录制中", foreground="#3498db")
        self._append_log(f"[系统] 开始记录 -> {filepath}")
        for k in history:
            history[k].clear()

    def _toggle_pause(self):
        if SMART_CAR_JUMP["paused"]:
            SMART_CAR_JUMP["paused"] = False
            self.pause_btn.config(text="暂停")
            self.status_label.config(text="录制中", foreground="#3498db")
            self._append_log("[系统] 已恢复记录")
        else:
            SMART_CAR_JUMP["paused"] = True
            self.pause_btn.config(text="继续")
            self.status_label.config(text="已暂停", foreground="orange")
            self._append_log("[系统] 已暂停记录")

    def _stop_recording(self):
        SMART_CAR_JUMP["recording"] = False
        SMART_CAR_JUMP["paused"] = False
        if SMART_CAR_JUMP["csv_file"]:
            SMART_CAR_JUMP["csv_file"].close()
            SMART_CAR_JUMP["csv_file"] = None
            SMART_CAR_JUMP["csv_writer"] = None
        self.start_btn.config(state=tk.NORMAL)
        self.pause_btn.config(state=tk.DISABLED, text="暂停")
        self.stop_btn.config(state=tk.DISABLED)
        self.status_label.config(text="已停止", foreground="gray")
        filepath = self.save_path_var.get()
        count = len(SMART_CAR_JUMP["data_buffer"])
        self._append_log(f"[系统] 停止记录 - 共 {count} 帧 -> {filepath}")
        messagebox.showinfo("完成", f"数据已保存:\n{filepath}\n共 {count} 帧")

    def _append_log(self, msg):
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)

    def _update_ui(self):
        latest_data = None
        while not SMART_CAR_JUMP["data_queue"].empty():
            try:
                item = SMART_CAR_JUMP["data_queue"].get_nowait()
                if "error" in item:
                    self._append_log(f"[错误] {item['error']}")
                else:
                    latest_data = item
            except queue.Empty:
                break

        if latest_data:
            self.record_count += 1
            self.count_label.config(text=f"帧数: {self.record_count}")

            for k in history:
                if k in latest_data:
                    history[k].append(latest_data[k])
                elif k == "outp_gyro" and "Outp_Gyro_Pitch" in latest_data:
                    history[k].append(latest_data["Outp_Gyro_Pitch"])
                elif k == "outp_angle" and "Outp_Angle_Pitch" in latest_data:
                    history[k].append(latest_data["Outp_Angle_Pitch"])

            for key, lbl in self.val_labels.items():
                if key in latest_data:
                    val = latest_data[key]
                    lbl.config(
                        text=f"{val:.2f}" if isinstance(val, float) else str(val)
                    )

            if latest_data.get("flag_jump"):
                tj = latest_data.get("time_j", 0)
                if tj <= 75:
                    phase = "上升"
                elif tj <= 120:
                    phase = "收腿"
                elif latest_data.get("flag_jump_1"):
                    phase = "放腿"
                else:
                    phase = "缓冲"
                self.jump_state_label.config(
                    text=f"JUMP: {phase}", foreground="#e74c3c"
                )
            else:
                self.jump_state_label.config(text="", foreground="orange")

            self._draw_canvas()

            if int(self.log_text.index("end-1c").split(".")[0]) > 500:
                self.log_text.delete("1.0", "2.0")
            raw_line = ",".join(
                str(latest_data.get(f, "?")) for f in FIELD_NAMES
            )
            self._append_log(f"$J,{raw_line}")

        self.update_timer = self.root.after(50, self._update_ui)

    def _start_update_loop(self):
        self._update_ui()

    def on_close(self):
        SMART_CAR_JUMP["stop_event"].set()
        SMART_CAR_JUMP["recording"] = False
        if SMART_CAR_JUMP["csv_file"]:
            SMART_CAR_JUMP["csv_file"].close()
        if self.update_timer:
            self.root.after_cancel(self.update_timer)
        self.root.destroy()


def main():
    root = tk.Tk()
    app = JumpTelemetryApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
