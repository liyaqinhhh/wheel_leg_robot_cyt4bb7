# -*- coding: utf-8 -*-
"""
Ins Auto Telemetry - Navigation Data Collection and Visualization
==================================================================
Features:
  1. Auto-scan/manual switch serial port
  2. Parse wireless serial data ($I, $W, $P frames)
  3. Real-time trajectory visualization (grid, draggable canvas)
  4. Distinguish collection mode (ins_mode=0/4) and navigation mode (ins_mode=1/5)
  5. Real-time log display
  6. Auto-save data to doc folder

Data Frame Formats:
  $I: Navigation telemetry - ins_mode,realtime_x,realtime_y,target_x,target_y,yaw_ins,dis_ins
  $W: Waypoint marker - wp_x,wp_y
  $P: Pure Pursuit details - target_yaw,current_wp_x,current_wp_y

Created on: 2026-06-11
Author: GitHub Copilot
"""

import serial
import serial.tools.list_ports
import threading
import time
import os
from datetime import datetime
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import numpy as np

# ==================== Global Variables ====================
smart_car_auto = {
    'serial_port': None,
    'serial_connected': False,
    'collecting': False,
    'navigating': False,
    'ins_mode': 4,
    'current_pos': (0.0, 0.0),
    'target_pos': (0.0, 0.0),
    'yaw_ins': 0.0,
    'dis_ins': 0.0,
    'wp_current': 0,
    'wp_count': 0,
    'trajectory': [],
    'waypoints': [],
    'pp_state': None,
}

# ==================== Data Parser ====================
class DataParser:
    """Serial data parser"""
    
    @staticmethod
    def parse_line(line):
        """Parse a line of data, return (frame_type, data_dict) or (None, None)"""
        try:
            line = line.strip()
            if not line or len(line) < 3:
                return None, None
            
            # $I Navigation telemetry frame
            # Format: $I,mode,realtime_x,realtime_y,target_x,target_y,yaw_ins,dis_ins
            if line.startswith('$I,'):
                parts = line[3:].split(',')
                if len(parts) >= 7:
                    return 'I', {
                        'ins_mode': int(parts[0]),
                        'realtime_x': float(parts[1]),
                        'realtime_y': float(parts[2]),
                        'target_x': float(parts[3]),
                        'target_y': float(parts[4]),
                        'yaw_ins': float(parts[5]),
                        'dis_ins': float(parts[6]),
                    }
            
            # $W Waypoint marker frame
            elif line.startswith('$W,'):
                parts = line[3:].split(',')
                if len(parts) >= 2:
                    return 'W', {
                        'wp_x': float(parts[0]),
                        'wp_y': float(parts[1]),
                    }
            
            # $P Pure Pursuit details frame
            elif line.startswith('$P,'):
                parts = line[3:].split(',')
                if len(parts) >= 3:
                    return 'P', {
                        'target_yaw': float(parts[0]),
                        'current_wp_x': float(parts[1]),
                        'current_wp_y': float(parts[2]),
                    }
            
            return None, None
            
        except Exception as e:
            return None, None


# ==================== Main Application ====================
class InsAutoApp:
    """Ins Auto Telemetry Main Application"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Ins Auto Telemetry System")
        self.root.geometry("1400x900")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Data
        self.parser = DataParser()
        self.serial_thread = None
        self.running = True
        self.plot_update_interval = 500  # ms (incremental draw, can be slower)
        
        # Create UI
        self.create_widgets()
        
        # Scan ports
        self.scan_ports()
        
        # Start plot update
        self.update_plot()
    
    def create_widgets(self):
        """Create UI widgets"""
        # ==================== Left Control Panel ====================
        left_frame = ttk.Frame(self.root, padding="10")
        left_frame.pack(side=tk.LEFT, fill=tk.Y)
        
        # --- Serial Control ---
        port_frame = ttk.LabelFrame(left_frame, text="Serial Control", padding="10")
        port_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(port_frame, text="Port:").grid(row=0, column=0, sticky=tk.W)
        self.port_combo = ttk.Combobox(port_frame, width=15, state='readonly')
        self.port_combo.grid(row=0, column=1, padx=5)
        
        self.scan_btn = ttk.Button(port_frame, text="Scan", command=self.scan_ports, width=8)
        self.scan_btn.grid(row=0, column=2, padx=5)
        
        self.connect_btn = ttk.Button(port_frame, text="Connect", command=self.toggle_connection, width=8)
        self.connect_btn.grid(row=1, column=0, columnspan=3, pady=5)
        
        # --- Status Display ---
        status_frame = ttk.LabelFrame(left_frame, text="Status Info", padding="10")
        status_frame.pack(fill=tk.X, pady=5)
        
        self.status_labels = {}
        status_items = [
            ('ins_mode', 'INS Mode'),
            ('pos', 'Current Pos'),
            ('target', 'Target Pos'),
            ('yaw', 'Target Yaw'),
            ('distance', 'Distance'),
            ('wp', 'Waypoint'),
        ]
        
        for i, (key, label) in enumerate(status_items):
            ttk.Label(status_frame, text=f"{label}:").grid(row=i, column=0, sticky=tk.W)
            self.status_labels[key] = ttk.Label(status_frame, text="--", width=20)
            self.status_labels[key].grid(row=i, column=1, sticky=tk.W, padx=5)
        
        # --- Pure Pursuit Status ---
        pp_frame = ttk.LabelFrame(left_frame, text="Pure Pursuit Status", padding="10")
        pp_frame.pack(fill=tk.X, pady=5)
        
        self.pp_labels = {}
        pp_items = [
            ('lookahead', 'Lookahead'),
            ('curvature', 'Curvature'),
            ('target_yaw', 'Target Yaw'),
            ('dist_to_path', 'Lateral Err'),
            ('angle_to_path', 'Angle Err'),
        ]
        
        for i, (key, label) in enumerate(pp_items):
            ttk.Label(pp_frame, text=f"{label}:").grid(row=i, column=0, sticky=tk.W)
            self.pp_labels[key] = ttk.Label(pp_frame, text="--", width=15)
            self.pp_labels[key].grid(row=i, column=1, sticky=tk.W, padx=5)
        
        # --- Control Buttons ---
        ctrl_frame = ttk.LabelFrame(left_frame, text="Control", padding="10")
        ctrl_frame.pack(fill=tk.X, pady=5)
        
        self.collect_btn = ttk.Button(ctrl_frame, text="Start Collect", command=self.toggle_collect, width=15)
        self.collect_btn.pack(pady=5)
        
        self.navigate_btn = ttk.Button(ctrl_frame, text="Start Navigate", command=self.toggle_navigate, width=15)
        self.navigate_btn.pack(pady=5)
        
        self.clear_btn = ttk.Button(ctrl_frame, text="Clear Trajectory", command=self.clear_trajectory, width=15)
        self.clear_btn.pack(pady=5)
        
        # --- Legend ---
        legend_frame = ttk.LabelFrame(left_frame, text="Legend", padding="10")
        legend_frame.pack(fill=tk.X, pady=5)
        
        legend_text = """
Collection Mode (ins_mode=0/4):
  - Dashed line trajectory
  - Waypoints: Green squares

Navigation Mode (ins_mode=1/5):
  - Solid line trajectory
  - Target: Red triangle
  - Lookahead: Blue dot

Current Position: Blue circle
"""
        ttk.Label(legend_frame, text=legend_text, justify=tk.LEFT).pack()
        
        # ==================== Right Display Area ====================
        right_frame = ttk.Frame(self.root, padding="10")
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # --- Trajectory Plot ---
        plot_frame = ttk.LabelFrame(right_frame, text="Real-time Trajectory", padding="5")
        plot_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Create matplotlib figure
        self.fig = Figure(figsize=(10, 8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.setup_plot()
        
        # Embed into tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Add toolbar (draggable, zoomable)
        toolbar_frame = ttk.Frame(plot_frame)
        toolbar_frame.pack(fill=tk.X)
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()
        
        # --- Log Display ---
        log_frame = ttk.LabelFrame(right_frame, text="Log Info", padding="5")
        log_frame.pack(fill=tk.X, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=8, width=80, 
                                                   wrap=tk.WORD, font=('Consolas', 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # Configure log color tags
        self.log_text.tag_config('info', foreground='black')
        self.log_text.tag_config('waypoint', foreground='green')
        self.log_text.tag_config('navigate', foreground='blue')
        self.log_text.tag_config('pp', foreground='purple')
        self.log_text.tag_config('error', foreground='red')
    
    def setup_plot(self):
        """Initialize trajectory plot with persistent artists for fast incremental updates"""
        self.ax.clear()
        self.ax.set_xlabel('X (encoder pulses)', fontsize=10)
        self.ax.set_ylabel('Y (encoder pulses)', fontsize=10)
        self.ax.set_title('Ins Real-time Trajectory', fontsize=12)
        self.ax.grid(True, linestyle='--', alpha=0.7)
        self.ax.set_axisbelow(True)
        self.ax.xaxis.set_major_locator(plt.MultipleLocator(100))
        self.ax.yaxis.set_major_locator(plt.MultipleLocator(100))
        self.ax.set_xlim(-200, 200)
        self.ax.set_ylim(-200, 200)
        self.ax.set_aspect('equal', adjustable='box')

        # Persistent artists ¡ª updated via set_data/set_offsets (no clear needed)
        self._collect_line, = self.ax.plot([], [], 'g--', linewidth=1.5, alpha=0.7, label='Collection')
        self._nav_line, = self.ax.plot([], [], 'b-', linewidth=2, alpha=0.8, label='Navigation')
        self._wp_scatter = self.ax.scatter([], [], c='green', s=100, marker='s',
                                           edgecolors='darkgreen', linewidths=2, zorder=5)
        self._target_scatter = self.ax.scatter([], [], c='red', s=150, marker='^',
                                               edgecolors='darkred', linewidths=2, zorder=6, label='Target')
        self._current_scatter = self.ax.scatter([], [], c='blue', s=200, marker='o',
                                                edgecolors='navy', linewidths=2, zorder=7, label='Current')
        self._lookahead_scatter = self.ax.scatter([], [], c='cyan', s=80, marker='o',
                                                  alpha=0.6, zorder=4, label='Lookahead')
        self._wp_annotations = []
        self.ax.legend(loc='upper right', fontsize=8)
    
    def scan_ports(self):
        """Scan available serial ports"""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo['values'] = port_list
        
        if port_list:
            # Prefer COM4
            if 'COM4' in port_list:
                self.port_combo.set('COM4')
            else:
                self.port_combo.set(port_list[0])
            self.log_message(f"Scanned {len(port_list)} ports: {', '.join(port_list)}", 'info')
        else:
            self.log_message("No serial ports found", 'error')
    
    def toggle_connection(self):
        """Toggle serial connection state"""
        if smart_car_auto['serial_connected']:
            self.disconnect_serial()
        else:
            self.connect_serial()
    
    def connect_serial(self):
        """Connect to serial port"""
        port = self.port_combo.get()
        if not port:
            messagebox.showerror("Error", "Please select a port")
            return
        
        try:
            smart_car_auto['serial_port'] = serial.Serial(
                port=port,
                baudrate=115200,
                timeout=0.1
            )
            smart_car_auto['serial_connected'] = True
            self.connect_btn.config(text="Disconnect")

            # Clear buffer
            smart_car_auto['serial_port'].reset_input_buffer()

            self.log_message(f"Connected to {port}", 'info')
            
            # Start receive thread
            self.serial_thread = threading.Thread(target=self.receive_data, daemon=True)
            self.serial_thread.start()
            
        except Exception as e:
            messagebox.showerror("Connection Failed", str(e))
            self.log_message(f"Connection failed: {e}", 'error')
    
    def disconnect_serial(self):
        """Disconnect serial port"""
        smart_car_auto['serial_connected'] = False
        if smart_car_auto['serial_port'] and smart_car_auto['serial_port'].is_open:
            smart_car_auto['serial_port'].close()
        self.connect_btn.config(text="Connect")
        self.log_message("Disconnected", 'info')
    
    def receive_data(self):
        """Serial data receive thread"""
        buf = ""
        while smart_car_auto['serial_connected'] and self.running:
            try:
                if smart_car_auto['serial_port'] and smart_car_auto['serial_port'].is_open and smart_car_auto['serial_port'].in_waiting > 0:
                    chunk = smart_car_auto['serial_port'].read(
                        smart_car_auto['serial_port'].in_waiting
                    )
                    try:
                        text = chunk.decode('utf-8', errors='replace')
                    except Exception:
                        text = chunk.decode('latin-1', errors='replace')
                    buf += text
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        line_stripped = line.strip()
                        if line_stripped:
                            self.root.after(0, self.process_line, line_stripped)
                else:
                    time.sleep(0.01)
            except (serial.SerialException, OSError):
                if smart_car_auto['serial_connected']:
                    self.root.after(0, self.log_message, "Serial disconnected!", 'error')
                    self.root.after(0, self.disconnect_serial)
                break
            except Exception:
                time.sleep(0.02)
    
    def process_line(self, line):
        """Process a line of data"""
        frame_type, data = self.parser.parse_line(line)
        
        if frame_type == 'I':
            smart_car_auto['current_pos'] = (data['realtime_x'], data['realtime_y'])
            smart_car_auto['target_pos'] = (data['target_x'], data['target_y'])
            smart_car_auto['yaw_ins'] = data['yaw_ins']
            smart_car_auto['dis_ins'] = data['dis_ins']
            smart_car_auto['ins_mode'] = data['ins_mode']
            smart_car_auto['trajectory'].append({
                'x': data['realtime_x'],
                'y': data['realtime_y'],
                'mode': data['ins_mode'],
                'timestamp': time.time()
            })
        
        elif frame_type == 'W':
            if smart_car_auto['ins_mode'] != 5:
                smart_car_auto['ins_mode'] = 4  # recording
            wp = {'x': data['wp_x'], 'y': data['wp_y']}
            smart_car_auto['waypoints'].append(wp)
            wp_cnt = len(smart_car_auto['waypoints'])
            self.log_message(
                f"[Waypoint] #{wp_cnt} ({data['wp_x']:.1f}, {data['wp_y']:.1f})",
                'waypoint'
            )
        
        elif frame_type == 'P':
            smart_car_auto['ins_mode'] = 5  # navigating
            smart_car_auto['pp_state'] = data
            self.log_message(
                f"[PP] Yaw: {data['target_yaw']:.1f} WP: ({data['current_wp_x']:.1f}, {data['current_wp_y']:.1f})",
                'pp'
            )
    
    def get_mode_string(self, mode):
        """Get mode string"""
        mode_map = {
            0: "Record",
            1: "Follow",
            4: "AutoRecord",
            5: "AutoNav",
        }
        return mode_map.get(mode, f"Mode{mode}")
    
    def update_plot(self):
        """Update trajectory plot incrementally - set_data/set_offsets, no clear()"""
        if not self.running:
            return

        try:
            current_xlim = self.ax.get_xlim()
            current_ylim = self.ax.get_ylim()

            all_x = []
            all_y = []

            # ©¤©¤ Trajectory lines: split by mode ©¤©¤
            collect_x, collect_y = [], []
            nav_x, nav_y = [], []
            for point in smart_car_auto['trajectory']:
                if point['mode'] in [0, 4]:
                    collect_x.append(point['x'])
                    collect_y.append(point['y'])
                else:
                    nav_x.append(point['x'])
                    nav_y.append(point['y'])

            self._collect_line.set_data(collect_x, collect_y)
            self._nav_line.set_data(nav_x, nav_y)
            all_x.extend(collect_x + nav_x)
            all_y.extend(collect_y + nav_y)

            # ©¤©¤ Waypoints ©¤©¤
            if smart_car_auto['waypoints']:
                wp_x = [wp['x'] for wp in smart_car_auto['waypoints']]
                wp_y = [wp['y'] for wp in smart_car_auto['waypoints']]
                self._wp_scatter.set_offsets(np.column_stack((wp_x, wp_y)))
                all_x.extend(wp_x)
                all_y.extend(wp_y)
                # Rebuild annotations (cheap: only waypoint count)
                for ann in self._wp_annotations:
                    ann.remove()
                self._wp_annotations.clear()
                for i, (x, y) in enumerate(zip(wp_x, wp_y)):
                    ann = self.ax.annotate(f"#{i+1}", (x, y),
                                           textcoords="offset points", xytext=(5, 5), fontsize=8)
                    self._wp_annotations.append(ann)
            else:
                self._wp_scatter.set_offsets(np.empty((0, 2)))
                for ann in self._wp_annotations:
                    ann.remove()
                self._wp_annotations.clear()

            # ©¤©¤ Target (red triangle) ©¤©¤
            if smart_car_auto['ins_mode'] in [1, 5]:
                tx, ty = smart_car_auto['target_pos']
                self._target_scatter.set_offsets([[tx, ty]])
                all_x.append(tx)
                all_y.append(ty)
            else:
                self._target_scatter.set_offsets(np.empty((0, 2)))

            # ©¤©¤ Current position (blue circle) ©¤©¤
            cx, cy = smart_car_auto['current_pos']
            self._current_scatter.set_offsets([[cx, cy]])
            all_x.append(cx)
            all_y.append(cy)

            # ©¤©¤ Lookahead (cyan dot) ©¤©¤
            if smart_car_auto['pp_state'] and smart_car_auto['ins_mode'] == 5:
                pp = smart_car_auto['pp_state']
                self._lookahead_scatter.set_offsets([[pp['current_wp_x'], pp['current_wp_y']]])
                all_x.append(pp['current_wp_x'])
                all_y.append(pp['current_wp_y'])
            else:
                self._lookahead_scatter.set_offsets(np.empty((0, 2)))

            # ©¤©¤ Auto-expand axis limits ©¤©¤
            if all_x and all_y:
                data_xmin, data_xmax = min(all_x), max(all_x)
                data_ymin, data_ymax = min(all_y), max(all_y)
                dx = max((data_xmax - data_xmin) * 0.1, 50)
                dy = max((data_ymax - data_ymin) * 0.1, 50)

                new_xmin = min(current_xlim[0], data_xmin - dx)
                new_xmax = max(current_xlim[1], data_xmax + dx)
                new_ymin = min(current_ylim[0], data_ymin - dy)
                new_ymax = max(current_ylim[1], data_ymax + dy)

                self.ax.set_xlim(new_xmin, new_xmax)
                self.ax.set_ylim(new_ymin, new_ymax)
                self.ax.set_aspect('equal', adjustable='box')

            self.canvas.draw_idle()

        except Exception:
            pass

        self.update_status()
        self.root.after(self.plot_update_interval, self.update_plot)
    
    def update_status(self):
        """Update status display"""
        cx, cy = smart_car_auto['current_pos']
        self.status_labels['pos'].config(text=f"({cx:.1f}, {cy:.1f})")
        
        tx, ty = smart_car_auto['target_pos']
        self.status_labels['target'].config(text=f"({tx:.1f}, {ty:.1f})")
        
        self.status_labels['yaw'].config(text=f"{smart_car_auto['yaw_ins']:.1f} deg")
        self.status_labels['distance'].config(text=f"{smart_car_auto['dis_ins']:.1f}")
        
        wp_cnt = len(smart_car_auto['waypoints'])
        self.status_labels['wp'].config(text=f"{wp_cnt} waypoints")
        self.status_labels['ins_mode'].config(text=f"{smart_car_auto['ins_mode']}")
        
        # Pure Pursuit status
        if smart_car_auto['pp_state']:
            pp = smart_car_auto['pp_state']
            self.pp_labels['target_yaw'].config(text=f"{pp.get('target_yaw', 0):.1f} deg")
            self.pp_labels['lookahead'].config(text="--")
            self.pp_labels['curvature'].config(text="--")
            self.pp_labels['dist_to_path'].config(text="--")
            self.pp_labels['angle_to_path'].config(text="--")
    
    def log_message(self, message, tag='info'):
        """Add log message"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n", tag)
        self.log_text.see(tk.END)
    
    def toggle_collect(self):
        """Toggle collection state"""
        if not smart_car_auto['serial_connected']:
            messagebox.showwarning("Warning", "Please connect to serial port first")
            return
        
        smart_car_auto['collecting'] = not smart_car_auto['collecting']
        
        if smart_car_auto['collecting']:
            self.collect_btn.config(text="Stop Collect")
            self.log_message("Started collecting trajectory", 'info')
        else:
            self.collect_btn.config(text="Start Collect")
            self.log_message("Stopped collecting", 'info')
    
    def toggle_navigate(self):
        """Toggle navigation state"""
        if not smart_car_auto['serial_connected']:
            messagebox.showwarning("Warning", "Please connect to serial port first")
            return
        
        smart_car_auto['navigating'] = not smart_car_auto['navigating']
        
        if smart_car_auto['navigating']:
            self.navigate_btn.config(text="Stop Navigate")
            self.log_message("Started navigation", 'navigate')
        else:
            self.navigate_btn.config(text="Start Navigate")
            self.log_message("Stopped navigation", 'info')
    
    def clear_trajectory(self):
        """Clear trajectory data"""
        smart_car_auto['trajectory'].clear()
        smart_car_auto['waypoints'].clear()
        smart_car_auto['pp_state'] = None
        self.log_message("Cleared trajectory data", 'info')
    
    def on_closing(self):
        """Close window"""
        self.running = False
        self.disconnect_serial()
        self.root.destroy()


# ==================== Main Entry ====================
def main():
    """Main entry"""
    root = tk.Tk()
    app = InsAutoApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
