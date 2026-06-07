#!/usr/bin/env python3
# -*- coding: gbk -*-
"""
INS (Inertial Navigation System) 惯导路径可视化采集工具
========================================================
功能：
  - 自动扫描/手动选择 COM 端口
  - 开始记录/结束记录/开始循迹/结束循迹
  - 实时 XY 坐标路径图（记录路径虚线+航点标记 / 循迹路径实线）
  - 实时显示遥测日志
  - 数据保存至 docs/ 目录，文件名 ins_YYYYMMDD_HHMMSS.csv

协议：
  $I,x,y,ins_mode,dis_ins,yaw_ins,n,target,flag_save  (INS实时帧, 40ms)
  $W,index,x,y,n                                       (航点帧, 录点瞬间)

全局变量 smart_car_ins:
  dict { 'running':bool, 'port':str, 'session':str, 'mode':str, ... }
"""

import sys, os, re, csv, time, threading, datetime as dt
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from collections import deque

# ---- matplotlib ----
try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# ---- pyserial ----
try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

# ====================== 全局变量 smart_car_ins ======================
smart_car_ins = {
    "running": False,
    "paused": False,
    "port": "COM4",
    "baudrate": 115200,
    "session_start": None,
    "csv_path": "",
    "frame_count": 0,
    "error_count": 0,
    "mode": "idle",          # idle / recording / navigating
    "record_waypoints": [],  # [(x,y,index), ...] recorded waypoints
    "nav_path": [],          # [(x,y), ...] navigation real-time path
    "current_x": 0.0,
    "current_y": 0.0,
    "current_ins_mode": 0,
    "current_target": 0,
    "current_n": 0,
    "current_yaw_ins": 0.0,
    "current_dis_ins": 0.0,
}

# ====================== 端口扫描 ======================
def scan_ports():
    ports = []
    if HAS_SERIAL:
        try:
            for p in serial.tools.list_ports.comports():
                ports.append((p.device, p.description))
        except Exception:
            pass
    found = [p[0] for p in ports]
    if "COM4" not in found:
        ports.insert(0, ("COM4", "(manual)"))
    return ports

def format_timestamp():
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S")

# ====================== 协议解析 ======================
# $I type:  x, y, ins_mode, dis_ins, yaw_ins, n, target, flag_save
I_FIELDS = ["x", "y", "ins_mode", "dis_ins", "yaw_ins", "n", "target", "flag_save"]
# $W type:  index, x, y, n
W_FIELDS = ["wp_index", "wp_x", "wp_y", "wp_n"]

def parse_ins_line(line):
    line = line.strip()
    if not line:
        return None
    if line.startswith("$I,"):
        parts = line[3:].split(",")
        if len(parts) != len(I_FIELDS):
            return None
        rec = {}
        for i, name in enumerate(I_FIELDS):
            try:
                if name in ("ins_mode", "n", "target", "flag_save"):
                    rec[name] = int(float(parts[i]))
                else:
                    rec[name] = float(parts[i])
            except ValueError:
                return None
        rec["type"] = "I"
        return rec
    elif line.startswith("$W,"):
        parts = line[3:].split(",")
        if len(parts) != len(W_FIELDS):
            return None
        rec = {}
        for i, name in enumerate(W_FIELDS):
            try:
                if name in ("wp_index", "wp_n"):
                    rec[name] = int(float(parts[i]))
                else:
                    rec[name] = float(parts[i])
            except ValueError:
                return None
        rec["type"] = "W"
        return rec
    return None

# ====================== 主 GUI ======================
class InsLoggerApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("INS 惯导路径可视化采集工具")
        self.root.geometry("1200x800")
        self.root.minsize(1000, 650)

        if not HAS_SERIAL:
            messagebox.showwarning("missing dependency", "pyserial not installed. Run: pip install pyserial")
        if not HAS_MPL:
            messagebox.showwarning("missing dependency", "matplotlib not installed. Run: pip install matplotlib")

        # serial
        self.ser = None
        self.reader_thread = None
        self.stop_event = threading.Event()

        # CSV
        self.csv_file = None
        self.csv_writer = None

        # log lines (max 400)
        self.log_lines = deque(maxlen=400)

        # build UI
        self._build_ui()

        # initial port scan
        self._refresh_ports()
        idx = self._find_port_index("COM4")
        if idx >= 0:
            self.port_combo.current(idx)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- UI ----------
    def _build_ui(self):
        root = self.root
        style = ttk.Style(root)
        style.theme_use("clam")

        main = ttk.Frame(root, padding=6)
        main.pack(fill=tk.BOTH, expand=True)

        # ---- top bar: port ----
        top_bar = ttk.LabelFrame(main, text="Serial Port", padding=4)
        top_bar.pack(fill=tk.X, pady=(0,4))

        ttk.Label(top_bar, text="Port:").pack(side=tk.LEFT, padx=(0,4))
        self.port_var = tk.StringVar(value="COM4")
        self.port_combo = ttk.Combobox(top_bar, textvariable=self.port_var, width=14, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(0,6))
        ttk.Button(top_bar, text="Scan", width=5, command=self._refresh_ports).pack(side=tk.LEFT, padx=(0,10))

        self.manual_port_var = tk.StringVar()
        ttk.Label(top_bar, text="Manual:").pack(side=tk.LEFT, padx=(0,4))
        ttk.Entry(top_bar, textvariable=self.manual_port_var, width=8).pack(side=tk.LEFT, padx=(0,4))
        ttk.Button(top_bar, text="Switch", width=6, command=self._manual_switch).pack(side=tk.LEFT)

        ttk.Label(top_bar, text="  Baud:").pack(side=tk.LEFT, padx=(16,4))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(top_bar, textvariable=self.baud_var, width=9,
                     values=["9600","115200","460800"], state="readonly").pack(side=tk.LEFT)

        # ---- mid: left controls + status, right plot area ----
        mid = ttk.Frame(main)
        mid.pack(fill=tk.BOTH, expand=True, pady=(0,4))

        # left panel
        left = ttk.Frame(mid, width=300)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0,6))
        left.pack_propagate(False)

        # control buttons
        ctrl = ttk.LabelFrame(left, text="Control", padding=4)
        ctrl.pack(fill=tk.X, pady=(0,4))

        self.btn_record = ttk.Button(ctrl, text="Start Record", width=15, command=self.start_record)
        self.btn_record.pack(pady=2)
        self.btn_stop_rec = ttk.Button(ctrl, text="Stop Record", width=15, command=self.stop_record, state=tk.DISABLED)
        self.btn_stop_rec.pack(pady=2)

        ttk.Separator(ctrl, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=4)

        self.btn_nav = ttk.Button(ctrl, text="Start Navigation", width=15, command=self.start_navigation, state=tk.DISABLED)
        self.btn_nav.pack(pady=2)
        self.btn_stop_nav = ttk.Button(ctrl, text="Stop Navigation", width=15, command=self.stop_navigation, state=tk.DISABLED)
        self.btn_stop_nav.pack(pady=2)

        ttk.Separator(ctrl, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=4)

        self.btn_pause = ttk.Button(ctrl, text="Pause", width=15, command=self.toggle_pause, state=tk.DISABLED)
        self.btn_pause.pack(pady=2)

        # status
        status = ttk.LabelFrame(left, text="Status", padding=4)
        status.pack(fill=tk.X, pady=(0,4))

        self.lbl_port = ttk.Label(status, text="Port: COM4 (disconnected)")
        self.lbl_port.pack(anchor=tk.W)
        self.lbl_state = ttk.Label(status, text="State: idle")
        self.lbl_state.pack(anchor=tk.W)
        self.lbl_mode = ttk.Label(status, text="INS Mode: --")
        self.lbl_mode.pack(anchor=tk.W)
        self.lbl_frames = ttk.Label(status, text="Frames: 0  |  Errors: 0")
        self.lbl_frames.pack(anchor=tk.W)
        self.lbl_file = ttk.Label(status, text="File: --")
        self.lbl_file.pack(anchor=tk.W)
        self.lbl_wp = ttk.Label(status, text="Waypoints: 0")
        self.lbl_wp.pack(anchor=tk.W)
        self.lbl_pos = ttk.Label(status, text="Pos: (0.0, 0.0)")
        self.lbl_pos.pack(anchor=tk.W)
        self.lbl_target = ttk.Label(status, text="Target idx: --  dist: --  yaw: --")
        self.lbl_target.pack(anchor=tk.W)

        # right: plot + log
        right = ttk.Frame(mid)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # matplotlib figure
        plot_frame = ttk.LabelFrame(right, text="XY Path", padding=2)
        plot_frame.pack(fill=tk.BOTH, expand=True, pady=(0,4))

        self.fig = Figure(figsize=(6, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title("INS Navigation Path")
        self.ax.grid(True, linestyle=":", alpha=0.6)
        self.ax.set_aspect("equal")
        self.ax.axhline(y=0, color="gray", linewidth=0.5)
        self.ax.axvline(x=0, color="gray", linewidth=0.5)

        # plot artists (updated in draw_plot)
        self.art_record_line = None
        self.art_record_pts = None
        self.art_nav_line = None
        self.art_current_pos = None
        self.art_target_marker = None

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # log area
        log_frame = ttk.LabelFrame(right, text="Log (newest on top)", padding=2)
        log_frame.pack(fill=tk.BOTH, expand=False)
        log_frame.configure(height=120)

        self.log_text = scrolledtext.ScrolledText(
            log_frame, wrap=tk.NONE, font=("Consolas", 9),
            bg="#1e1e1e", fg="#d4d4d4", height=6)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.configure(state=tk.DISABLED)

        # ---- bottom bar ----
        bottom = ttk.Frame(main)
        bottom.pack(fill=tk.X, pady=(4,0))
        ttk.Button(bottom, text="Open Data Folder", command=self._open_folder).pack(side=tk.RIGHT, padx=4)
        ttk.Button(bottom, text="Clear Log", command=self._clear_log).pack(side=tk.RIGHT, padx=4)
        ttk.Label(bottom, text="Data saved to docs/ with ins_ prefix", foreground="gray").pack(side=tk.LEFT)

        self._update_plot()

    # ---------- port ----------
    def _refresh_ports(self):
        ports = scan_ports()
        values = [f"{p[0]}  {p[1][:40]}" for p in ports]
        self.port_combo["values"] = values
        self._port_map = {f"{p[0]}  {p[1][:40]}": p[0] for p in ports}

    def _get_port(self):
        sel = self.port_var.get()
        return self._port_map.get(sel, "COM4")

    def _find_port_index(self, name):
        for i, v in enumerate(self.port_combo["values"]):
            if v.startswith(name + " "):
                return i
        return -1

    def _manual_switch(self):
        manual = self.manual_port_var.get().strip().upper()
        if not manual:
            messagebox.showwarning("Tip", "Enter port, e.g. COM5")
            return
        if not manual.startswith("COM"):
            manual = "COM" + manual
        idx = self._find_port_index(manual)
        if idx >= 0:
            self.port_combo.current(idx)
        else:
            vals = list(self.port_combo["values"])
            entry = f"{manual}  (manual)"
            vals.insert(0, entry)
            self.port_combo["values"] = vals
            self._port_map[entry] = manual
            self.port_combo.current(0)
        self.lbl_port.config(text=f"Port: {manual} (manual switch)")

    # ---------- record / nav ----------
    def _open_serial(self):
        port = self._get_port()
        baud = int(self.baud_var.get())
        try:
            self.ser = serial.Serial(port, baud, timeout=0.5)
            self.ser.reset_input_buffer()
            return True, port, baud
        except Exception as e:
            messagebox.showerror("Serial Error", f"Cannot open {port}:\n{e}")
            return False, port, baud

    def _start_session(self, tag):
        if smart_car_ins["running"]:
            messagebox.showinfo("Tip", "Session already running")
            return False
        ok, port, baud = self._open_serial()
        if not ok:
            return False

        docs_dir = self._docs_dir()
        os.makedirs(docs_dir, exist_ok=True)
        ts = format_timestamp()
        csv_name = f"ins_{tag}_{ts}.csv"
        csv_path = os.path.join(docs_dir, csv_name)
        try:
            self.csv_file = open(csv_path, "w", newline="", encoding="utf-8-sig")
            self.csv_writer = csv.writer(self.csv_file)
            # headers: type, x, y, ins_mode, dis_ins, yaw_ins, n, target, flag_save
            self.csv_writer.writerow(["type","x","y","ins_mode","dis_ins","yaw_ins","n","target","flag_save","comment"])
            self.csv_file.flush()
        except Exception as e:
            messagebox.showerror("File Error", f"Cannot create CSV:\n{e}")
            self.ser.close()
            self.ser = None
            return False

        smart_car_ins["running"] = True
        smart_car_ins["paused"] = False
        smart_car_ins["port"] = port
        smart_car_ins["session_start"] = ts
        smart_car_ins["csv_path"] = csv_path
        smart_car_ins["frame_count"] = 0
        smart_car_ins["error_count"] = 0

        self.stop_event.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

        self.lbl_port.config(text=f"Port: {port} (connected @ {baud})")
        self.lbl_file.config(text=f"File: {csv_name}")
        self._append_log(f"===== Session started [{ts}] port={port} =====")
        return True

    def start_record(self):
        if self._start_session("rec"):
            smart_car_ins["mode"] = "recording"
            smart_car_ins["record_waypoints"] = []
            self._set_ui_recording()
            self.lbl_state.config(text="State: recording waypoints...")
            self.lbl_mode.config(text="INS Mode: 0 (record)")

    def stop_record(self):
        self._stop_session("recording stopped")

    def start_navigation(self):
        if self._start_session("nav"):
            smart_car_ins["mode"] = "navigating"
            smart_car_ins["nav_path"] = []
            self._set_ui_navigating()
            self.lbl_state.config(text="State: navigating...")
            self.lbl_mode.config(text="INS Mode: 1 (navigate)")

    def stop_navigation(self):
        self._stop_session("navigation stopped")

    def _stop_session(self, msg):
        self.stop_event.set()
        smart_car_ins["running"] = False
        smart_car_ins["paused"] = False
        smart_car_ins["mode"] = "idle"

        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=2.0)

        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

        self._set_ui_idle()
        self.lbl_state.config(text="State: idle")
        self.lbl_mode.config(text="INS Mode: --")
        self.lbl_port.config(text=f"Port: {smart_car_ins['port']} (disconnected)")

        fc = smart_car_ins["frame_count"]
        ec = smart_car_ins["error_count"]
        self._append_log(f"===== {msg} =====")
        self._append_log(f"  Valid frames: {fc}  Errors: {ec}")
        self._append_log(f"  Waypoints: {len(smart_car_ins['record_waypoints'])}")

    def toggle_pause(self):
        if smart_car_ins["paused"]:
            smart_car_ins["paused"] = False
            self.btn_pause.config(text="Pause")
            self._append_log("===== Resumed =====")
        else:
            smart_car_ins["paused"] = True
            self.btn_pause.config(text="Resume")
            self._append_log("===== Paused =====")

    # ---------- serial reader ----------
    def _reader_loop(self):
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
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        if not smart_car_ins["paused"]:
                            self.root.after(0, self._process, line.strip())
                else:
                    time.sleep(0.01)
            except (serial.SerialException, OSError):
                self.root.after(0, self._serial_error)
                break
            except Exception:
                time.sleep(0.02)

    def _process(self, line):
        if not line:
            return
        rec = parse_ins_line(line)
        if rec is None:
            return

        smart_car_ins["frame_count"] += 1
        fc = smart_car_ins["frame_count"]

        # CSV write
        if self.csv_writer and rec["type"] == "I":
            row = [rec["type"]] + [rec.get(f, "") for f in I_FIELDS] + [""]
            self.csv_writer.writerow(row)
            if fc % 50 == 0:
                self.csv_file.flush()

        if rec["type"] == "I":
            # update current position
            smart_car_ins["current_x"] = rec["x"]
            smart_car_ins["current_y"] = rec["y"]
            smart_car_ins["current_ins_mode"] = rec["ins_mode"]
            smart_car_ins["current_target"] = rec["target"]
            smart_car_ins["current_n"] = rec["n"]
            smart_car_ins["current_yaw_ins"] = rec["yaw_ins"]
            smart_car_ins["current_dis_ins"] = rec["dis_ins"]

            # add to nav path
            if smart_car_ins["mode"] == "navigating":
                smart_car_ins["nav_path"].append((rec["x"], rec["y"]))

            # log (every 5th)
            if fc % 5 == 0:
                self._append_log(
                    f"[{fc:4d}] x={rec['x']: 7.1f} y={rec['y']: 7.1f} "
                    f"mode={rec['ins_mode']} dis={rec['dis_ins']: 5.1f} "
                    f"yaw={rec['yaw_ins']: 6.1f} n={rec['n']} tgt={rec['target']}")

            # update status
            mode_names = {0:"RECORD", 1:"NAVIGATE", 2:"SEG_EDIT", 3:"SEG_RUN"}
            self.lbl_mode.config(text=f"INS Mode: {mode_names.get(rec['ins_mode'], str(rec['ins_mode']))}")
            self.lbl_pos.config(text=f"Pos: ({rec['x']:.1f}, {rec['y']:.1f})")
            self.lbl_target.config(
                text=f"Target: {rec['target']}  dist: {rec['dis_ins']:.1f}  yaw: {rec['yaw_ins']:.1f}")

            # update plot every 3 frames
            if fc % 3 == 0:
                self._update_plot()

        elif rec["type"] == "W":
            # waypoint recorded
            wp = (rec["wp_x"], rec["wp_y"], rec["wp_index"])
            # avoid duplicates
            existing = [w[2] for w in smart_car_ins["record_waypoints"]]
            if rec["wp_index"] not in existing:
                smart_car_ins["record_waypoints"].append(wp)
                # sort by index
                smart_car_ins["record_waypoints"].sort(key=lambda x: x[2])
            self._append_log(f"[WP] index={rec['wp_index']} x={rec['wp_x']:.1f} y={rec['wp_y']:.1f} total={rec['wp_n']}")
            self.lbl_wp.config(text=f"Waypoints: {rec['wp_n']}")
            if self.csv_writer:
                row = [rec["type"], rec["wp_x"], rec["wp_y"], "", "", "", rec["wp_n"], rec["wp_index"], "", "waypoint"]
                self.csv_writer.writerow(row)
                self.csv_file.flush()
            self._update_plot()

        # status bar
        if fc % 10 == 0:
            self.lbl_frames.config(text=f"Frames: {fc}  |  Errors: {smart_car_ins['error_count']}")

    def _serial_error(self):
        self._append_log("!!!!! Serial connection lost !!!!!")
        self._stop_session("connection lost")

    # ---------- plot ----------
    def _update_plot(self):
        self.ax.clear()
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title("INS Navigation Path")
        self.ax.grid(True, linestyle=":", alpha=0.6)
        self.ax.set_aspect("equal")
        self.ax.axhline(y=0, color="gray", linewidth=0.5)
        self.ax.axvline(x=0, color="gray", linewidth=0.5)

        wps = smart_car_ins["record_waypoints"]
        nav = smart_car_ins["nav_path"]

        all_x, all_y = [], []

        # ---- recording path (dashed blue + numbered markers) ----
        if wps:
            wpx = [w[0] for w in wps]
            wpy = [w[1] for w in wps]
            self.ax.plot(wpx, wpy, "b--", linewidth=1.2, alpha=0.7, label="Record Path")
            self.ax.scatter(wpx, wpy, c="blue", s=30, marker="o", zorder=5, alpha=0.9)
            for i, (wx, wy, widx) in enumerate(wps):
                self.ax.annotate(str(widx), (wx, wy), textcoords="offset points",
                                 xytext=(4, 4), fontsize=7, color="darkblue")
            all_x.extend(wpx)
            all_y.extend(wpy)

        # ---- navigation path (solid red) ----
        if nav:
            npx = [p[0] for p in nav]
            npy = [p[1] for p in nav]
            self.ax.plot(npx, npy, "r-", linewidth=1.5, alpha=0.8, label="Navigate Path")
            all_x.extend(npx)
            all_y.extend(npy)

        # ---- current position (green dot) ----
        cx = smart_car_ins["current_x"]
        cy = smart_car_ins["current_y"]
        if cx != 0.0 or cy != 0.0:
            self.ax.scatter([cx], [cy], c="lime", s=80, marker="o", zorder=10,
                            edgecolors="black", linewidth=0.8, label="Current")
            all_x.append(cx)
            all_y.append(cy)

        # ---- target marker (red X on recording path) ----
        tgt = smart_car_ins["current_target"]
        if 0 <= tgt < len(wps):
            tx, ty, _ = wps[tgt]
            self.ax.scatter([tx], [ty], c="red", s=100, marker="X", zorder=10,
                            linewidths=1.5, label=f"Target [{tgt}]")
            all_x.append(tx)
            all_y.append(ty)

        # ---- auto zoom ----
        if all_x:
            x_min, x_max = min(all_x), max(all_x)
            y_min, y_max = min(all_y), max(all_y)
            x_margin = max((x_max - x_min) * 0.15, 0.5)
            y_margin = max((y_max - y_min) * 0.15, 0.5)
            self.ax.set_xlim(x_min - x_margin, x_max + x_margin)
            self.ax.set_ylim(y_min - y_margin, y_max + y_margin)
        else:
            self.ax.set_xlim(-1, 1)
            self.ax.set_ylim(-1, 1)

        self.ax.legend(loc="upper left", fontsize=8)
        self.canvas.draw()

    # ---------- UI helpers ----------
    def _set_ui_recording(self):
        self.btn_record.config(state=tk.DISABLED)
        self.btn_stop_rec.config(state=tk.NORMAL)
        self.btn_nav.config(state=tk.DISABLED)
        self.btn_stop_nav.config(state=tk.DISABLED)
        self.btn_pause.config(state=tk.NORMAL, text="Pause")
        self.port_combo.config(state=tk.DISABLED)
        self.lbl_state.config(text="State: recording...")

    def _set_ui_navigating(self):
        self.btn_record.config(state=tk.DISABLED)
        self.btn_stop_rec.config(state=tk.DISABLED)
        self.btn_nav.config(state=tk.DISABLED)
        self.btn_stop_nav.config(state=tk.NORMAL)
        self.btn_pause.config(state=tk.NORMAL, text="Pause")
        self.port_combo.config(state=tk.DISABLED)
        self.lbl_state.config(text="State: navigating...")

    def _set_ui_idle(self):
        self.btn_record.config(state=tk.NORMAL)
        self.btn_stop_rec.config(state=tk.DISABLED)
        self.btn_nav.config(state=tk.NORMAL if smart_car_ins["record_waypoints"] else tk.DISABLED)
        self.btn_stop_nav.config(state=tk.DISABLED)
        self.btn_pause.config(state=tk.DISABLED, text="Pause")
        self.port_combo.config(state="readonly")
        self.lbl_state.config(text="State: idle")

    def _append_log(self, text):
        self.log_lines.appendleft(text)
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.insert("1.0", "\n".join(self.log_lines) + "\n")
        self.log_text.configure(state=tk.DISABLED)

    def _clear_log(self):
        self.log_lines.clear()
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _docs_dir(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        return os.path.join(project_root, "docs")

    def _open_folder(self):
        d = self._docs_dir()
        os.makedirs(d, exist_ok=True)
        os.startfile(d)

    def _on_close(self):
        if smart_car_ins["running"]:
            if messagebox.askyesno("Confirm", "Session is active. Really quit?"):
                self._stop_session("user quit")
            else:
                return
        self.root.destroy()

# ====================== entry ======================
def main():
    root = tk.Tk()
    app = InsLoggerApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
