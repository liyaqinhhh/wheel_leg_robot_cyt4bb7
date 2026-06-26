#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Calibration: Kalman Filter Diagnostics Logger
=================================================
$K frame: $K,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,kf_roll,kf_pitch,Xk0,Xk1,Pk0,Pk1,Q0,R0
Usage: python smart_car_angle.py
"""

import os, sys, time, csv, threading, queue
from datetime import datetime
from collections import deque

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pip install pyserial"); sys.exit(1)

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

SMART_CAR_ANGLE = {
    "serial_port": None, "serial_thread": None,
    "recording": False, "paused": False,
    "data_buffer": [], "data_queue": queue.Queue(),
    "stop_event": threading.Event(),
    "csv_file": None, "csv_writer": None,
}

MAX_HISTORY = 500
history = {
    "tick": deque(maxlen=MAX_HISTORY),
    "kf_roll": deque(maxlen=MAX_HISTORY), "kf_pitch": deque(maxlen=MAX_HISTORY),
    "acc_x": deque(maxlen=MAX_HISTORY), "acc_y": deque(maxlen=MAX_HISTORY),
    "gyro_x": deque(maxlen=MAX_HISTORY),
    "Xk0": deque(maxlen=MAX_HISTORY), "Xk1": deque(maxlen=MAX_HISTORY),
}

FIELDS = ["acc_x","acc_y","acc_z","gyro_x","gyro_y","gyro_z",
          "kf_roll","kf_pitch","Xk0","Xk1","Pk0","Pk1","Q0","R0"]

def scan_ports():
    return [{"device":p.device,"description":p.description} for p in serial.tools.list_ports.comports()]

def parse_kf(line):
    line = line.strip()
    if not line.startswith("$K,"): return None
    parts = line.split(",")
    if len(parts) < 15: return None
    try:
        return {
            "acc_x":int(parts[1]),"acc_y":int(parts[2]),"acc_z":int(parts[3]),
            "gyro_x":int(parts[4]),"gyro_y":int(parts[5]),"gyro_z":int(parts[6]),
            "kf_roll":float(parts[7]),"kf_pitch":float(parts[8]),
            "Xk0":float(parts[9]),"Xk1":float(parts[10]),
            "Pk0":float(parts[11]),"Pk1":float(parts[12]),
            "Q0":float(parts[13]),"R0":float(parts[14]),
        }
    except: return None

def get_doc_dir():
    d = os.path.join(os.path.dirname(os.path.abspath(__file__)), "doc")
    os.makedirs(d, exist_ok=True)
    return d

def gen_filename():
    return f"angle_kalman_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

def reader_thread(port, baud=115200):
    ser = None
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        ser.reset_input_buffer()
        SMART_CAR_ANGLE["serial_port"] = ser
        buf = ""
        while not SMART_CAR_ANGLE["stop_event"].is_set():
            try:
                if ser.in_waiting:
                    raw = ser.read(ser.in_waiting)
                    try: text = raw.decode("utf-8", errors="replace")
                    except: text = raw.decode("latin-1", errors="replace")
                    buf += text
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.rstrip("\r")
                        if line.startswith("$K"):
                            d = parse_kf(line)
                            if d and SMART_CAR_ANGLE["recording"] and not SMART_CAR_ANGLE["paused"]:
                                SMART_CAR_ANGLE["data_queue"].put(d)
                                SMART_CAR_ANGLE["data_buffer"].append(d)
                                if SMART_CAR_ANGLE["csv_writer"]:
                                    SMART_CAR_ANGLE["csv_writer"].writerow([d.get(f,"") for f in FIELDS])
                else: time.sleep(0.001)
            except serial.SerialException as e:
                SMART_CAR_ANGLE["data_queue"].put({"error":f"Serial: {e}"}); break
            except Exception as e:
                SMART_CAR_ANGLE["data_queue"].put({"error":f"Read: {e}"})
    except serial.SerialException as e:
        SMART_CAR_ANGLE["data_queue"].put({"error":f"Open {port}: {e}"})
    finally:
        if ser and ser.is_open: ser.close()
        SMART_CAR_ANGLE["serial_port"] = None

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Kalaman Filter Diagnostics - Smart Car Angle")
        self.root.geometry("1100x780"); self.root.minsize(900, 600)
        ttk.Style().theme_use("clam")
        self.timer = None; self.cnt = 0; self._tick = 0
        self.val = {}
        self._build(); self._refresh_ports(); self._loop()

    def _build(self):
        cf = ttk.Frame(self.root, padding=5); cf.pack(fill=tk.X, side=tk.TOP)
        ttk.Label(cf, text="Port:").pack(side=tk.LEFT, padx=(0,2))
        self.port_var = tk.StringVar(value="COM4")
        self.port_combo = ttk.Combobox(cf, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=2)
        ttk.Button(cf, text="Scan", command=self._refresh_ports, width=6).pack(side=tk.LEFT, padx=2)
        self.man_var = tk.StringVar()
        self.man_entry = ttk.Entry(cf, textvariable=self.man_var, width=8)
        self.man_entry.pack(side=tk.LEFT, padx=2)
        ttk.Button(cf, text="Switch", command=self._man_switch, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Label(cf, text=" Baud:").pack(side=tk.LEFT, padx=(8,2))
        self.baud_var = tk.StringVar(value="115200")
        self.baud_cb = ttk.Combobox(cf, textvariable=self.baud_var, width=8,
            values=["9600","19200","38400","57600","115200","230400","460800","921600"])
        self.baud_cb.pack(side=tk.LEFT, padx=2)
        self.conn_btn = ttk.Button(cf, text="Connect", command=self._toggle_conn, width=8)
        self.conn_btn.pack(side=tk.LEFT, padx=(8,2))
        ttk.Separator(cf, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8, pady=2)
        self.start_btn = ttk.Button(cf, text="Start", command=self._start, width=10, state=tk.DISABLED)
        self.start_btn.pack(side=tk.LEFT, padx=2)
        self.pause_btn = ttk.Button(cf, text="Pause", command=self._pause, width=8, state=tk.DISABLED)
        self.pause_btn.pack(side=tk.LEFT, padx=2)
        self.stop_btn = ttk.Button(cf, text="Stop", command=self._stop, width=10, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=2)
        ttk.Separator(cf, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8, pady=2)
        self.st_lbl = ttk.Label(cf, text="Disconnected", foreground="gray")
        self.st_lbl.pack(side=tk.LEFT, padx=4)
        self.cnt_lbl = ttk.Label(cf, text="Frames: 0")
        self.cnt_lbl.pack(side=tk.LEFT, padx=4)
        self.path_var = tk.StringVar()
        ttk.Label(cf, text="Save:").pack(side=tk.LEFT, padx=(12,2))
        ttk.Entry(cf, textvariable=self.path_var, width=35, state="readonly").pack(side=tk.LEFT, padx=2)

        mp = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        mp.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        lf = ttk.Frame(mp, padding=5); mp.add(lf, weight=1); self._vals(lf)
        rf = ttk.Frame(mp, padding=5); mp.add(rf, weight=2); self._logs(rf)

        bf = ttk.Frame(self.root, padding=3); bf.pack(fill=tk.X, side=tk.BOTTOM)
        self.det_lbl = ttk.Label(bf, text="Ready"); self.det_lbl.pack(side=tk.LEFT)

    def _vals(self, p):
        nb = ttk.Notebook(p); nb.pack(fill=tk.BOTH, expand=True)
        for tab_name, rows in [
            ("Raw IMU", [("acc_x","Acc X","0"),("acc_y","Acc Y","0"),("acc_z","Acc Z","0"),
                         ("gyro_x","Gyro X","0"),("gyro_y","Gyro Y","0"),("gyro_z","Gyro Z","0")]),
            ("KF Output", [("kf_roll","KF Roll","0.000"),("kf_pitch","KF Pitch","0.000"),
                          ("Xk0","Xk[0] rad","0"),("Xk1","Xk[1] rad","0")]),
            ("KF Params", [("Pk0","Pk[0]","0"),("Pk1","Pk[1]","0"),("Q0","Q[0]","0"),("R0","R[0]","0")]),
        ]:
            f = ttk.Frame(nb, padding=8); nb.add(f, text=tab_name)
            for i,(k,lb,d) in enumerate(rows):
                ttk.Label(f, text=lb+":", font=("",9,"bold")).grid(row=i,column=0,sticky=tk.W,padx=4,pady=3)
                lbl = ttk.Label(f, text=d, font=("Consolas",12), foreground="#2c3e50")
                lbl.grid(row=i,column=1,sticky=tk.W,padx=8,pady=3)
                self.val[k] = lbl

    def _logs(self, p):
        ttk.Label(p, text="Raw $K Frames:", font=("",9,"bold")).pack(anchor=tk.W)
        self.log = scrolledtext.ScrolledText(p, height=12, font=("Consolas",9), wrap=tk.NONE)
        self.log.pack(fill=tk.BOTH, expand=True, pady=(0,5))
        ttk.Label(p, text="Real-time Plot:", font=("",9,"bold")).pack(anchor=tk.W)
        self.canvas = tk.Canvas(p, height=200, bg="#1a1a2e", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda e: self._draw())

    def _draw(self):
        c = self.canvas; c.delete("all")
        w,h = c.winfo_width(), c.winfo_height()
        if w<10 or h<10: return
        c.create_rectangle(0,0,w,h,fill="#1a1a2e",outline="")
        for i in range(1,10):
            c.create_line(0,h*i/10,w,h*i/10,fill="#2d2d44",dash=(2,4))
        for dk,cl,lb in [("kf_roll","#3498db","KF_R"),("kf_pitch","#e74c3c","KF_P"),
                         ("Xk0","#2ecc71","Xk0"),("Xk1","#f39c12","Xk1")]:
            vs = list(history.get(dk,[]))
            if len(vs)<2: continue
            mn,mx = min(vs),max(vs)
            if abs(mx-mn)<1e-6: mx=mn+1.0
            pts=[]; n=len(vs)
            for i,v in enumerate(vs):
                pts.extend([w*i/max(n-1,1), h-((v-mn)/(mx-mn))*(h-20)-10])
            if len(pts)>=4: c.create_line(*pts, fill=cl, width=1.5, smooth=True)
        ly=10
        for dk,cl,lb in [("kf_roll","#3498db","KF_R"),("kf_pitch","#e74c3c","KF_P"),
                         ("Xk0","#2ecc71","Xk0"),("Xk1","#f39c12","Xk1")]:
            c.create_rectangle(10,ly,22,ly+10,fill=cl,outline="")
            c.create_text(28,ly+5,text=lb,anchor=tk.W,fill="#ccc",font=("",7))
            ly+=14

    def _refresh_ports(self):
        ports = scan_ports()
        names = [p["device"] for p in ports]
        self.port_combo["values"] = names
        if names:
            if "COM4" in names: self.port_var.set("COM4")
            elif not self.port_var.get() or self.port_var.get() not in names:
                self.port_var.set(names[0])
        else: self.port_var.set("")
        info = ", ".join(f"{p['device']}({p['description'][:20]})" for p in ports) if ports else "No ports"
        self.det_lbl.config(text=f"Available: {info}")

    def _man_switch(self):
        m = self.man_var.get().strip().upper()
        if not m: return
        if not m.startswith("COM"): m = "COM"+m
        vs = list(self.port_combo["values"])
        if m not in vs: vs.insert(0,m); self.port_combo["values"]=vs
        self.port_combo.set(m)

    def _toggle_conn(self):
        if SMART_CAR_ANGLE["serial_thread"] and SMART_CAR_ANGLE["serial_thread"].is_alive():
            self._disco()
        else: self._connect()

    def _connect(self):
        port = self.port_var.get().strip()
        if not port: messagebox.showwarning("Warning","Select port"); return
        try: baud = int(self.baud_var.get())
        except: messagebox.showerror("Error","Invalid baud"); return
        self._log(f"[System] Connecting {port}@{baud}...")
        SMART_CAR_ANGLE["stop_event"].clear()
        t = threading.Thread(target=reader_thread, args=(port,baud), daemon=True)
        t.start(); SMART_CAR_ANGLE["serial_thread"] = t; time.sleep(0.5)
        if SMART_CAR_ANGLE["serial_port"] and SMART_CAR_ANGLE["serial_port"].is_open:
            self.conn_btn.config(text="Disconnect")
            self.st_lbl.config(text="Connected", foreground="green")
            self.start_btn.config(state=tk.NORMAL)
            self._log(f"[System] Connected {port}")
        else: self._log(f"[System] Failed {port}")

    def _disco(self):
        SMART_CAR_ANGLE["stop_event"].set()
        if SMART_CAR_ANGLE["serial_thread"]:
            SMART_CAR_ANGLE["serial_thread"].join(timeout=2)
            SMART_CAR_ANGLE["serial_thread"] = None
        self.conn_btn.config(text="Connect")
        self.st_lbl.config(text="Disconnected", foreground="red")
        self.start_btn.config(state=tk.DISABLED)
        self.pause_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        self._log("[System] Disconnected")

    def _start(self):
        fp = os.path.join(get_doc_dir(), gen_filename())
        try:
            f = open(fp, "w", newline="", encoding="utf-8-sig")
            w = csv.writer(f); w.writerow(FIELDS)
            SMART_CAR_ANGLE["csv_file"]=f; SMART_CAR_ANGLE["csv_writer"]=w
        except Exception as e:
            messagebox.showerror("Error", f"Cannot create: {e}"); return
        SMART_CAR_ANGLE["data_buffer"].clear()
        SMART_CAR_ANGLE["recording"] = True
        SMART_CAR_ANGLE["paused"] = False
        self.cnt = 0; self._tick = 0
        self.path_var.set(fp)
        self.start_btn.config(state=tk.DISABLED)
        self.pause_btn.config(state=tk.NORMAL, text="Pause")
        self.stop_btn.config(state=tk.NORMAL)
        self.st_lbl.config(text="Recording", foreground="#3498db")
        self._log(f"[System] Start -> {fp}")
        for k in history: history[k].clear()

    def _pause(self):
        if SMART_CAR_ANGLE["paused"]:
            SMART_CAR_ANGLE["paused"] = False
            self.pause_btn.config(text="Pause")
            self.st_lbl.config(text="Recording", foreground="#3498db")
            self._log("[System] Resumed")
        else:
            SMART_CAR_ANGLE["paused"] = True
            self.pause_btn.config(text="Continue")
            self.st_lbl.config(text="Paused", foreground="orange")
            self._log("[System] Paused")

    def _stop(self):
        SMART_CAR_ANGLE["recording"] = False
        SMART_CAR_ANGLE["paused"] = False
        if SMART_CAR_ANGLE["csv_file"]:
            SMART_CAR_ANGLE["csv_file"].close()
            SMART_CAR_ANGLE["csv_file"] = None
            SMART_CAR_ANGLE["csv_writer"] = None
        self.start_btn.config(state=tk.NORMAL)
        self.pause_btn.config(state=tk.DISABLED, text="Pause")
        self.stop_btn.config(state=tk.DISABLED)
        self.st_lbl.config(text="Stopped", foreground="gray")
        fp = self.path_var.get(); cnt = len(SMART_CAR_ANGLE["data_buffer"])
        self._log(f"[System] Done - {cnt} frames -> {fp}")
        messagebox.showinfo("Done", f"Saved:\n{fp}\nFrames: {cnt}")

    def _log(self, msg):
        self.log.insert(tk.END, msg+"\n"); self.log.see(tk.END)

    def _loop(self):
        latest = None
        while not SMART_CAR_ANGLE["data_queue"].empty():
            try:
                item = SMART_CAR_ANGLE["data_queue"].get_nowait()
                if "error" in item: self._log(f"[Error] {item['error']}")
                else: latest = item
            except queue.Empty: break
        if latest:
            self.cnt += 1; self._tick += 1
            self.cnt_lbl.config(text=f"Frames: {self.cnt}")
            for k in history:
                if k in latest: history[k].append(latest[k])
            for k,lbl in self.val.items():
                if k in latest:
                    v = latest[k]
                    if isinstance(v, float):
                        lbl.config(text=f"{v:.6f}" if abs(v)<10 and abs(v)>1e-6 else f"{v:.3f}")
                    else: lbl.config(text=str(v))
            self._draw()
            if int(self.log.index("end-1c").split(".")[0]) > 500:
                self.log.delete("1.0", "2.0")
            raw = ','.join(str(latest.get(f,'?')) for f in FIELDS)
            self._log('[{0}] $K,{1}'.format(self._tick, raw))
        self.timer = self.root.after(50, self._loop)

    def on_close(self):
        SMART_CAR_ANGLE["stop_event"].set()
        SMART_CAR_ANGLE["recording"] = False
        if SMART_CAR_ANGLE["csv_file"]: SMART_CAR_ANGLE["csv_file"].close()
        if self.timer: self.root.after_cancel(self.timer)
        self.root.destroy()

def main():
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
