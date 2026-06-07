#!/usr/bin/env python3
# -*- coding: gbk -*-
"""
智能车遥测数据采集工具 — Smart Car Telemetry Logger
=====================================================
功能：
  - 自动扫描 / 手动选择 COM 端口
  - 开始 / 暂停 / 结束 测试
  - 实时显示遥测日志
  - 解析 $T 协议数据帧，自动存为 CSV
  - 数据保存至 docs/ 目录，文件名含测试时间日期

协议格式（每行以 \\r\\n 结尾）：
  $T,tick,pitch,roll,yaw,gx,gy,gz,outp_turn,outp_gyro_pitch,
     target_yaw,turn_mode,deviation,kf_pitch,kf_roll,
     motor_l,motor_r,ax_linear,ay_linear,angle_Z

全局变量 smart_car_race 说明：
  该变量用于标识当前测试会话，方便外部脚本引用。
  类型: dict { 'running': bool, 'port': str, 'session': str, ... }

依赖（标准库 + pyserial）：
  pip install pyserial
"""

import sys
import os
import re
import csv
import time
import json
import threading
import datetime as dt
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

# ====================== 全局变量 smart_car_race ======================
smart_car_race = {
    "running": False,          # 是否正在采集
    "paused": False,           # 是否暂停
    "port": "COM4",            # 当前端口
    "baudrate": 115200,        # 波特率
    "session_start": None,     # 本次测试开始时间
    "csv_path": "",            # 当前 CSV 文件路径
    "frame_count": 0,          # 已接收帧数
    "error_count": 0,          # 解析错误帧数
}

# ====================== 串口工具函数 ======================
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("错误：缺少 pyserial 库，请执行: pip install pyserial")
    sys.exit(1)


def scan_ports():
    """扫描系统可用串口，返回列表 [(port, desc), ...]"""
    ports = []
    try:
        for p in serial.tools.list_ports.comports():
            ports.append((p.device, p.description))
    except Exception:
        pass
    # 确保 COM4 在列表中（即使未检测到）
    found = [p[0] for p in ports]
    if "COM4" not in found:
        ports.insert(0, ("COM4", "（手动指定）"))
    return ports


def format_timestamp():
    """返回当前时间字符串，用于文件名"""
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S")


# ====================== 数据解析器 ======================
# 字段顺序（与 C 端 sprintf 严格对应）
FIELD_NAMES = [
    "tick",
    "pitch", "roll", "yaw",
    "gyro_x", "gyro_y", "gyro_z",
    "outp_turn", "outp_gyro_pitch",
    "target_yaw", "turn_mode", "deviation",
    "kf_pitch", "kf_roll",
    "motor_l", "motor_r",
    "ax_linear", "ay_linear",
    "angle_Z",
]


def parse_telemetry_line(line: str):
    """
    解析 $T 协议行。
    返回: dict 或 None（解析失败）
    """
    line = line.strip()
    if not line.startswith("$T,"):
        return None
    # 去掉 $T, 前缀
    data_part = line[3:]
    parts = data_part.split(",")
    if len(parts) != len(FIELD_NAMES):
        return None
    record = {}
    for i, name in enumerate(FIELD_NAMES):
        try:
            if name in ("tick", "outp_turn", "outp_gyro_pitch",
                        "turn_mode", "motor_l", "motor_r"):
                record[name] = int(float(parts[i]))
            else:
                record[name] = float(parts[i])
        except ValueError:
            return None
    return record


# ====================== 主 GUI 应用 ======================
class TelemetryApp:
    """遥测采集器 GUI"""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("智能车遥测数据采集工具 — Smart Car Telemetry Logger")
        self.root.geometry("1000x700")
        self.root.minsize(800, 550)

        # 串口线程
        self.ser: serial.Serial | None = None
        self.reader_thread: threading.Thread | None = None
        self.stop_event = threading.Event()

        # CSV 写入
        self.csv_file = None
        self.csv_writer = None

        # 数据缓冲（实时日志用）
        self.log_lines = []          # 最多保留 500 行
        self.max_log_lines = 500

        # ---- 构建 UI ----
        self._build_ui()

        # 初始扫描端口
        self._refresh_ports()
        # 默认选择 COM4
        idx = self._find_port_index("COM4")
        if idx >= 0:
            self.port_combo.current(idx)

        # 窗口关闭处理
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- UI 构建 ----------
    def _build_ui(self):
        """搭建完整界面"""
        root = self.root
        # 全局样式
        style = ttk.Style(root)
        style.theme_use("clam")

        # 主容器
        main_frame = ttk.Frame(root, padding=8)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # ---------- 顶栏：端口设置 ----------
        top_bar = ttk.LabelFrame(main_frame, text="串口设置", padding=6)
        top_bar.pack(fill=tk.X, pady=(0, 6))

        ttk.Label(top_bar, text="端口:").pack(side=tk.LEFT, padx=(0, 4))
        self.port_var = tk.StringVar(value="COM4")
        self.port_combo = ttk.Combobox(top_bar, textvariable=self.port_var,
                                       width=14, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(0, 6))

        ttk.Button(top_bar, text="扫描端口", width=9,
                   command=self._refresh_ports).pack(side=tk.LEFT, padx=(0, 10))

        self.manual_port_var = tk.StringVar(value="")
        ttk.Label(top_bar, text="手动输入:").pack(side=tk.LEFT, padx=(0, 4))
        self.manual_entry = ttk.Entry(top_bar, textvariable=self.manual_port_var,
                                      width=10)
        self.manual_entry.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(top_bar, text="切换", width=5,
                   command=self._manual_switch_port).pack(side=tk.LEFT)

        # 波特率
        ttk.Label(top_bar, text="  波特率:").pack(side=tk.LEFT, padx=(16, 4))
        self.baud_var = tk.StringVar(value="115200")
        baud_combo = ttk.Combobox(top_bar, textvariable=self.baud_var,
                                  width=9, values=["9600", "115200", "460800"],
                                  state="readonly")
        baud_combo.pack(side=tk.LEFT)

        # ---------- 中栏左：控制按钮 + 状态 ----------
        mid_frame = ttk.Frame(main_frame)
        mid_frame.pack(fill=tk.X, pady=(0, 6))

        ctrl_frame = ttk.LabelFrame(mid_frame, text="采集控制", padding=6)
        ctrl_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        self.btn_start = ttk.Button(ctrl_frame, text="?  开始测试",
                                    width=14, command=self.start_test)
        self.btn_start.pack(pady=2)

        self.btn_pause = ttk.Button(ctrl_frame, text="?  暂停",
                                    width=14, command=self.toggle_pause,
                                    state=tk.DISABLED)
        self.btn_pause.pack(pady=2)

        self.btn_stop = ttk.Button(ctrl_frame, text="■  结束测试",
                                   width=14, command=self.stop_test,
                                   state=tk.DISABLED)
        self.btn_stop.pack(pady=2)

        # 状态面板
        status_frame = ttk.LabelFrame(mid_frame, text="运行状态", padding=6)
        status_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.status_port = ttk.Label(status_frame, text="端口: COM4 (未连接)")
        self.status_port.pack(anchor=tk.W)

        self.status_state = ttk.Label(status_frame, text="状态: 空闲")
        self.status_state.pack(anchor=tk.W)

        self.status_frames = ttk.Label(status_frame, text="接收帧: 0  |  错误: 0")
        self.status_frames.pack(anchor=tk.W)

        self.status_file = ttk.Label(status_frame, text="保存: 未开始")
        self.status_file.pack(anchor=tk.W)

        # 单独的遥测使能提示
        tip_text = ("提示：C 端遥测由 telemetry_enable 变量控制(1=开启, 0=关闭)\n"
                    "      可在 Interrupt_40ms 中修改，默认已开启。")
        ttk.Label(status_frame, text=tip_text, foreground="gray").pack(
            anchor=tk.W, pady=(4, 0))

        # ---------- 主区域：实时日志 ----------
        log_frame = ttk.LabelFrame(main_frame, text="实时遥测日志（最新在上）", padding=4)
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.log_text = scrolledtext.ScrolledText(
            log_frame, wrap=tk.NONE, font=("Consolas", 10),
            bg="#1e1e1e", fg="#d4d4d4", insertbackground="white")
        self.log_text.pack(fill=tk.BOTH, expand=True)
        # 只读
        self.log_text.configure(state=tk.DISABLED)

        # 添加列标题提示
        self._append_log(
            "等待连接 — 遥测格式: $T,tick,pitch,roll,yaw,gx,gy,gz,"
            "outp_turn,outp_gyro_pitch,target_yaw,turn_mode,deviation,"
            "kf_pitch,kf_roll,motor_l,motor_r,ax_linear,ay_linear,angle_Z")

        # ---------- 底栏 ----------
        bottom_bar = ttk.Frame(main_frame)
        bottom_bar.pack(fill=tk.X, pady=(6, 0))

        ttk.Button(bottom_bar, text="打开数据文件夹",
                   command=self._open_data_folder).pack(side=tk.RIGHT, padx=4)
        ttk.Button(bottom_bar, text="清空日志",
                   command=self._clear_log).pack(side=tk.RIGHT, padx=4)

        ttk.Label(bottom_bar,
                  text="数据自动保存至 project/code/docs/ 目录",
                  foreground="gray").pack(side=tk.LEFT)

    # ---------- 端口操作 ----------
    def _refresh_ports(self):
        """扫描端口并更新下拉框"""
        ports = scan_ports()
        values = [f"{p[0]} — {p[1][:40]}" for p in ports]
        self.port_combo["values"] = values
        self._port_map = {f"{p[0]} — {p[1][:40]}": p[0] for p in ports}

    def _get_selected_port(self) -> str:
        """获取当前选择的端口号"""
        sel = self.port_var.get()
        return self._port_map.get(sel, "COM4")

    def _find_port_index(self, port_name: str) -> int:
        """查找端口在下拉框中的索引"""
        values = list(self.port_combo["values"])
        for i, v in enumerate(values):
            if v.startswith(port_name + " "):
                return i
        return -1

    def _manual_switch_port(self):
        """手动输入端口号切换"""
        manual = self.manual_port_var.get().strip().upper()
        if not manual:
            messagebox.showwarning("提示", "请输入端口号，如 COM5")
            return
        if not manual.startswith("COM"):
            manual = "COM" + manual
        # 检查是否已在列表中
        idx = self._find_port_index(manual)
        if idx >= 0:
            self.port_combo.current(idx)
        else:
            # 添加到列表
            values = list(self.port_combo["values"])
            new_entry = f"{manual} — （手动输入）"
            values.insert(0, new_entry)
            self.port_combo["values"] = values
            self._port_map[new_entry] = manual
            self.port_combo.current(0)
        self.status_port.config(text=f"端口: {manual} (手动切换)")

    # ---------- 测试控制 ----------
    def start_test(self):
        """开始测试：打开串口，启动读取线程，创建 CSV 文件"""
        if smart_car_race["running"]:
            messagebox.showinfo("提示", "测试已在运行中")
            return

        port = self._get_selected_port()
        baud = int(self.baud_var.get())

        # 打开串口
        try:
            self.ser = serial.Serial(port, baud, timeout=0.5)
        except Exception as e:
            messagebox.showerror("串口错误", f"无法打开 {port}:\n{e}")
            return

        # 清空缓冲区
        self.ser.reset_input_buffer()

        # 创建 CSV 文件
        docs_dir = self._get_docs_dir()
        os.makedirs(docs_dir, exist_ok=True)
        session_ts = format_timestamp()
        csv_name = f"telemetry_turn_test_{session_ts}.csv"
        csv_path = os.path.join(docs_dir, csv_name)

        try:
            self.csv_file = open(csv_path, "w", newline="", encoding="utf-8-sig")
            self.csv_writer = csv.writer(self.csv_file)
            # 写表头
            self.csv_writer.writerow(FIELD_NAMES)
            self.csv_file.flush()
        except Exception as e:
            messagebox.showerror("文件错误", f"无法创建 CSV:\n{e}")
            self.ser.close()
            self.ser = None
            return

        # 更新全局变量
        smart_car_race["running"] = True
        smart_car_race["paused"] = False
        smart_car_race["port"] = port
        smart_car_race["session_start"] = session_ts
        smart_car_race["csv_path"] = csv_path
        smart_car_race["frame_count"] = 0
        smart_car_race["error_count"] = 0

        # 启动读取线程
        self.stop_event.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop,
                                              daemon=True)
        self.reader_thread.start()

        # 更新 UI
        self._set_ui_state("running")
        self.status_port.config(text=f"端口: {port} (已连接 @ {baud})")
        self.status_file.config(text=f"保存: {os.path.basename(csv_path)}")
        self._append_log(f"===== 测试开始 [{session_ts}] 端口={port} 波特率={baud} =====")

    def toggle_pause(self):
        """暂停 / 继续"""
        if not smart_car_race["running"]:
            return
        if smart_car_race["paused"]:
            smart_car_race["paused"] = False
            self.btn_pause.config(text="?  暂停")
            self.status_state.config(text="状态: 采集中")
            self._append_log("===== 继续采集 =====")
        else:
            smart_car_race["paused"] = True
            self.btn_pause.config(text="?  继续")
            self.status_state.config(text="状态: 已暂停")
            self._append_log("===== 暂停采集 =====")

    def stop_test(self):
        """结束测试：关闭串口，关闭 CSV，更新统计"""
        self.stop_event.set()
        smart_car_race["running"] = False
        smart_car_race["paused"] = False

        # 等待读取线程结束
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=2.0)

        # 关闭串口
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

        # 关闭 CSV
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

        # 更新 UI
        self._set_ui_state("stopped")
        self.status_state.config(text="状态: 空闲")
        self.status_port.config(text=f"端口: {smart_car_race['port']} (已断开)")

        fc = smart_car_race["frame_count"]
        ec = smart_car_race["error_count"]
        csv_path = smart_car_race["csv_path"]
        self._append_log(
            f"===== 测试结束 ====="
            f"  有效帧: {fc}  错误帧: {ec}  文件: {os.path.basename(csv_path)}")

        # 弹出统计
        messagebox.showinfo("测试完成",
                            f"数据已保存至:\n{csv_path}\n\n"
                            f"有效帧数: {fc}\n错误帧数: {ec}")

    # ---------- 串口读取线程 ----------
    def _reader_loop(self):
        """后台线程：持续读取串口数据并解析"""
        buf = ""
        while not self.stop_event.is_set():
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                    chunk = self.ser.read(self.ser.in_waiting)
                    try:
                        text = chunk.decode("utf-8", errors="replace")
                    except Exception:
                        text = chunk.decode("latin-1", errors="replace")
                    buf += text
                    # 按行分割
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        if not smart_car_race["paused"]:
                            self.root.after(0, self._process_line, line.strip())
                else:
                    time.sleep(0.01)
            except (serial.SerialException, OSError):
                self.root.after(0, self._on_serial_error)
                break
            except Exception:
                time.sleep(0.02)

    def _process_line(self, line: str):
        """在主线程中处理一行数据（更新 UI + 写 CSV）"""
        if not line:
            return

        record = parse_telemetry_line(line)
        if record is None:
            # 非 $T 行，判断是否为有用日志
            if line and len(line) > 3:
                # 可能是不完整的行，静默忽略
                pass
            return

        # 解析成功
        smart_car_race["frame_count"] += 1

        # 写 CSV
        if self.csv_writer:
            row = [record.get(name, "") for name in FIELD_NAMES]
            self.csv_writer.writerow(row)
            # 每 50 帧刷盘一次
            if smart_car_race["frame_count"] % 50 == 0:
                self.csv_file.flush()

        # 更新日志显示（显示全部19个字段）
        fc = smart_car_race["frame_count"]
        summary = (
            f"[{fc:5d}] tick={record['tick']:4d} "
            f"pitch={record['pitch']: 6.1f} roll={record['roll']: 6.1f} yaw={record['yaw']: 7.1f} "
            f"gx={record['gyro_x']: 6.1f} gy={record['gyro_y']: 6.1f} gz={record['gyro_z']: 6.1f} "
            f"ot={record['outp_turn']: 5d} ogp={record['outp_gyro_pitch']: 5d} "
            f"ty={record['target_yaw']: 6.1f} tm={record['turn_mode']:1d} dv={record['deviation']: 6.3f} "
            f"kf_pit={record['kf_pitch']: 6.1f} kf_rol={record['kf_roll']: 6.1f} "
            f"mL={record['motor_l']: 5d} mR={record['motor_r']: 5d} "
            f"axL={record['ax_linear']: 6.2f} ayL={record['ay_linear']: 6.2f} "
            f"aZ={record['angle_Z']: 7.1f}"
        )
        self._append_log(summary)

        # 更新状态栏（每 10 帧）
        if fc % 10 == 0:
            self.status_frames.config(
                text=f"接收帧: {fc}  |  错误: {smart_car_race['error_count']}")

    def _on_serial_error(self):
        """串口异常处理"""
        if smart_car_race["running"]:
            self._append_log("!!!!! 串口连接丢失，自动停止 !!!!!")
            self.stop_test()

    # ---------- UI 辅助 ----------
    def _set_ui_state(self, state: str):
        """根据状态启用/禁用按钮"""
        if state == "running":
            self.btn_start.config(state=tk.DISABLED)
            self.btn_pause.config(state=tk.NORMAL, text="?  暂停")
            self.btn_stop.config(state=tk.NORMAL)
            self.status_state.config(text="状态: 采集中")
            self.port_combo.config(state=tk.DISABLED)
            self.manual_entry.config(state=tk.DISABLED)
        else:
            self.btn_start.config(state=tk.NORMAL)
            self.btn_pause.config(state=tk.DISABLED, text="?  暂停")
            self.btn_stop.config(state=tk.DISABLED)
            self.port_combo.config(state="readonly")
            self.manual_entry.config(state=tk.NORMAL)

    def _append_log(self, text: str):
        """向日志区域追加文本（线程安全）"""
        self.log_lines.insert(0, text)
        if len(self.log_lines) > self.max_log_lines:
            self.log_lines = self.log_lines[:self.max_log_lines]

        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.insert("1.0", "\n".join(self.log_lines) + "\n")
        self.log_text.configure(state=tk.DISABLED)
        # 不自动滚到底部（最新在上）

    def _clear_log(self):
        """清空日志"""
        self.log_lines.clear()
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _get_docs_dir(self) -> str:
        """获取 docs 目录绝对路径"""
        # 脚本所在目录的上级的 docs 目录
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # tools/ → 项目根目录
        project_root = os.path.dirname(script_dir)
        docs_dir = os.path.join(project_root, "docs")
        return docs_dir

    def _open_data_folder(self):
        """打开数据文件夹"""
        docs_dir = self._get_docs_dir()
        if not os.path.exists(docs_dir):
            os.makedirs(docs_dir, exist_ok=True)
        os.startfile(docs_dir)

    def _on_close(self):
        """窗口关闭时的清理"""
        if smart_car_race["running"]:
            if messagebox.askyesno("确认", "测试仍在进行中，确定退出？"):
                self.stop_test()
            else:
                return
        self.root.destroy()


# ====================== 入口 ======================
def main():
    root = tk.Tk()
    app = TelemetryApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
