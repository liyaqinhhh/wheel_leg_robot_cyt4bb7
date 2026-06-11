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
  $I: Navigation telemetry - mode,realtime_x,realtime_y,target_x,target_y,yaw,wp_current,wp_count,found_wp,distance
  $W: Waypoint marker - wp_index,wp_x,wp_y,ins_mode
  $P: Pure Pursuit details - lookahead,curvature,target_yaw,target_wp_index,dist_to_path,angle_to_path,wp_x,wp_y

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
    'data_log': [],
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
            if line.startswith('$I,'):
                parts = line[3:].split(',')
                if len(parts) >= 10:
                    return 'I', {
                        'ins_mode': int(parts[0]),
                        'realtime_x': float(parts[1]),
                        'realtime_y': float(parts[2]),
                        'target_x': float(parts[3]),
                        'target_y': float(parts[4]),
                        'yaw_ins': float(parts[5]),
                        'wp_current': int(parts[6]),
                        'wp_count': int(parts[7]),
                        'found_wp': int(parts[8]),
                        'dis_ins': float(parts[9]),
                    }
            
            # $W Waypoint marker frame
            elif line.startswith('$W,'):
                parts = line[3:].split(',')
                if len(parts) >= 4:
                    return 'W', {
                        'wp_index': int(parts[0]),
                        'wp_x': float(parts[1]),
                        'wp_y': float(parts[2]),
                        'ins_mode': int(parts[3]),
                    }
            
            # $P Pure Pursuit details frame
            elif line.startswith('$P,'):
                parts = line[3:].split(',')
                if len(parts) >= 8:
                    return 'P', {
                        'lookahead': float(parts[0]),
                        'curvature': float(parts[1]),
                        'target_yaw': float(parts[2]),
                        'target_wp_index': int(parts[3]),
                        'distance_to_path': float(parts[4]),
                        'angle_to_path': float(parts[5]),
                        'current_wp_x': float(parts[6]),
                        'current_wp_y': float(parts[7]),
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
        self.plot_update_interval = 50  # ms
        
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
        
        self.save_btn = ttk.Button(ctrl_frame, text="Save Data", command=self.save_data, width=15)
        self.save_btn.pack(pady=5)
        
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
        """Initialize trajectory plot"""
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
        buffer = ""
        while smart_car_auto['serial_connected'] and self.running:
            try:
                if smart_car_auto['serial_port'].in_waiting > 0:
                    data = smart_car_auto['serial_port'].read(
                        smart_car_auto['serial_port'].in_waiting
                    ).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Process line by line
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.strip():
                            self.process_line(line.strip())
                
                time.sleep(0.01)  # 10ms check interval
                
            except Exception as e:
                if smart_car_auto['serial_connected']:
                    self.log_message(f"Receive error: {e}", 'error')
                break
    
    def process_line(self, line):
        """Process a line of data"""
        frame_type, data = self.parser.parse_line(line)
        
        if frame_type == 'I':
            # Navigation telemetry frame
            smart_car_auto['ins_mode'] = data['ins_mode']
            smart_car_auto['current_pos'] = (data['realtime_x'], data['realtime_y'])
            smart_car_auto['target_pos'] = (data['target_x'], data['target_y'])
            smart_car_auto['yaw_ins'] = data['yaw_ins']
            smart_car_auto['dis_ins'] = data['dis_ins']
            smart_car_auto['wp_current'] = data['wp_current']
            smart_car_auto['wp_count'] = data['wp_count']
            
            # Add trajectory point
            mode = data['ins_mode']
            if mode in [0, 1, 4, 5]:
                smart_car_auto['trajectory'].append({
                    'x': data['realtime_x'],
                    'y': data['realtime_y'],
                    'mode': mode,
                    'timestamp': time.time()
                })
            
            # Log
            mode_str = self.get_mode_string(mode)
            self.log_message(
                f"[{mode_str}] Pos: ({data['realtime_x']:.1f}, {data['realtime_y']:.1f}) "
                f"Target: ({data['target_x']:.1f}, {data['target_y']:.1f}) "
                f"Yaw: {data['yaw_ins']:.1f} Dist: {data['dis_ins']:.1f}",
                'navigate' if mode in [1, 5] else 'info'
            )
            
            # Save raw data
            smart_car_auto['data_log'].append({
                'timestamp': datetime.now().isoformat(),
                'frame': 'I',
                'data': data
            })
        
        elif frame_type == 'W':
            # Waypoint marker frame
            wp = {
                'x': data['wp_x'],
                'y': data['wp_y'],
                'index': data['wp_index'],
                'mode': data['ins_mode']
            }
            smart_car_auto['waypoints'].append(wp)
            
            self.log_message(
                f"[Waypoint] #{data['wp_index']} ({data['wp_x']:.1f}, {data['wp_y']:.1f})",
                'waypoint'
            )
            
            smart_car_auto['data_log'].append({
                'timestamp': datetime.now().isoformat(),
                'frame': 'W',
                'data': data
            })
        
        elif frame_type == 'P':
            # Pure Pursuit details frame
            smart_car_auto['pp_state'] = data
            
            self.log_message(
                f"[PP] Lookahead: {data['lookahead']:.1f} Curvature: {data['curvature']:.4f} "
                f"Yaw: {data['target_yaw']:.1f} Lateral: {data['distance_to_path']:.1f}",
                'pp'
            )
            
            smart_car_auto['data_log'].append({
                'timestamp': datetime.now().isoformat(),
                'frame': 'P',
                'data': data
            })
    
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
        """Update trajectory plot"""
        if not self.running:
            return
        
        try:
            # Save current view limits (user may have zoomed/panned)
            current_xlim = self.ax.get_xlim()
            current_ylim = self.ax.get_ylim()
            
            self.ax.clear()
            # Don't call setup_plot, do minimal setup here
            self.ax.set_xlabel('X (encoder pulses)', fontsize=10)
            self.ax.set_ylabel('Y (encoder pulses)', fontsize=10)
            self.ax.set_title('Ins Real-time Trajectory', fontsize=12)
            self.ax.grid(True, linestyle='--', alpha=0.7)
            self.ax.set_axisbelow(True)
            self.ax.xaxis.set_major_locator(plt.MultipleLocator(100))
            self.ax.yaxis.set_major_locator(plt.MultipleLocator(100))
            
            # Track actual data extent
            all_x = []
            all_y = []
            
            # Draw trajectory
            if smart_car_auto['trajectory']:
                collect_traj = []
                nav_traj = []
                
                for point in smart_car_auto['trajectory']:
                    if point['mode'] in [0, 4]:
                        collect_traj.append((point['x'], point['y']))
                    else:
                        nav_traj.append((point['x'], point['y']))
                
                if collect_traj:
                    xs, ys = zip(*collect_traj)
                    all_x.extend(xs)
                    all_y.extend(ys)
                    self.ax.plot(xs, ys, 'g--', linewidth=1.5, alpha=0.7, label='Collection')
                
                if nav_traj:
                    xs, ys = zip(*nav_traj)
                    all_x.extend(xs)
                    all_y.extend(ys)
                    self.ax.plot(xs, ys, 'b-', linewidth=2, alpha=0.8, label='Navigation')
            
            # Draw waypoints
            if smart_car_auto['waypoints']:
                for wp in smart_car_auto['waypoints']:
                    all_x.append(wp['x'])
                    all_y.append(wp['y'])
                    self.ax.scatter(wp['x'], wp['y'], c='green', s=100, marker='s',
                                   edgecolors='darkgreen', linewidths=2, zorder=5)
                    self.ax.annotate(f"#{wp['index']}", (wp['x'], wp['y']),
                                    textcoords="offset points", xytext=(5, 5), fontsize=8)
            
            # Draw current target
            if smart_car_auto['ins_mode'] in [1, 5]:
                tx, ty = smart_car_auto['target_pos']
                all_x.append(tx)
                all_y.append(ty)
                self.ax.scatter(tx, ty, c='red', s=150, marker='^',
                               edgecolors='darkred', linewidths=2, zorder=6, label='Target')
            
            # Draw current position
            cx, cy = smart_car_auto['current_pos']
            all_x.append(cx)
            all_y.append(cy)
            self.ax.scatter(cx, cy, c='blue', s=200, marker='o',
                           edgecolors='navy', linewidths=2, zorder=7, label='Current')
            
            # Draw Pure Pursuit lookahead
            if smart_car_auto['pp_state'] and smart_car_auto['ins_mode'] == 5:
                pp = smart_car_auto['pp_state']
                all_x.append(pp['current_wp_x'])
                all_y.append(pp['current_wp_y'])
                self.ax.scatter(pp['current_wp_x'], pp['current_wp_y'],
                               c='cyan', s=80, marker='o', alpha=0.6, zorder=4, label='Lookahead')
            
            # Compute data-driven limits with padding, auto-expand only
            if all_x and all_y:
                data_xmin, data_xmax = min(all_x), max(all_x)
                data_ymin, data_ymax = min(all_y), max(all_y)
                dx = max((data_xmax - data_xmin) * 0.1, 50)
                dy = max((data_ymax - data_ymin) * 0.1, 50)
                data_xmin -= dx
                data_xmax += dx
                data_ymin -= dy
                data_ymax += dy
                
                new_xmin = min(current_xlim[0], data_xmin)
                new_xmax = max(current_xlim[1], data_xmax)
                new_ymin = min(current_ylim[0], data_ymin)
                new_ymax = max(current_ylim[1], data_ymax)
                
                self.ax.set_xlim(new_xmin, new_xmax)
                self.ax.set_ylim(new_ymin, new_ymax)
                self.ax.set_aspect('equal', adjustable='box')
            else:
                self.ax.set_xlim(current_xlim)
                self.ax.set_ylim(current_ylim)
                # No data yet, keep previous view or use default
                self.ax.set_xlim(current_xlim)
                self.ax.set_ylim(current_ylim)
            
            # Add legend
            self.ax.legend(loc='upper right', fontsize=8)
            
            # Refresh canvas
            self.canvas.draw_idle()
            
        except Exception as e:
            pass
        
        # Update status display
        self.update_status()
        
        # Schedule next update
        self.root.after(self.plot_update_interval, self.update_plot)
    
    def update_status(self):
        """Update status display"""
        mode = smart_car_auto['ins_mode']
        self.status_labels['ins_mode'].config(text=f"{mode} ({self.get_mode_string(mode)})")
        
        cx, cy = smart_car_auto['current_pos']
        self.status_labels['pos'].config(text=f"({cx:.1f}, {cy:.1f})")
        
        tx, ty = smart_car_auto['target_pos']
        self.status_labels['target'].config(text=f"({tx:.1f}, {ty:.1f})")
        
        self.status_labels['yaw'].config(text=f"{smart_car_auto['yaw_ins']:.1f} deg")
        self.status_labels['distance'].config(text=f"{smart_car_auto['dis_ins']:.1f}")
        
        wp_cur = smart_car_auto['wp_current']
        wp_total = smart_car_auto['wp_count']
        self.status_labels['wp'].config(text=f"{wp_cur}/{wp_total}")
        
        # Pure Pursuit status
        if smart_car_auto['pp_state']:
            pp = smart_car_auto['pp_state']
            self.pp_labels['lookahead'].config(text=f"{pp['lookahead']:.1f}")
            self.pp_labels['curvature'].config(text=f"{pp['curvature']:.4f}")
            self.pp_labels['target_yaw'].config(text=f"{pp['target_yaw']:.1f} deg")
            self.pp_labels['dist_to_path'].config(text=f"{pp['distance_to_path']:.1f}")
            self.pp_labels['angle_to_path'].config(text=f"{pp['angle_to_path']:.1f} deg")
    
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
            # Auto save
            self.save_data()
    
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
            # Auto save
            self.save_data()
    
    def clear_trajectory(self):
        """Clear trajectory data"""
        smart_car_auto['trajectory'].clear()
        smart_car_auto['waypoints'].clear()
        smart_car_auto['pp_state'] = None
        self.log_message("Cleared trajectory data", 'info')
    
    def save_data(self):
        """Save data to file"""
        if not smart_car_auto['trajectory'] and not smart_car_auto['data_log']:
            messagebox.showinfo("Info", "No data to save")
            return
        
        # Create doc folder
        doc_dir = os.path.join(os.path.dirname(__file__), '..', 'doc')
        os.makedirs(doc_dir, exist_ok=True)
        
        # Generate filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"ins_auto_{timestamp}.txt"
        filepath = os.path.join(doc_dir, filename)
        
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(f"Ins Auto Telemetry Data Record\n")
                f.write(f"Record Time: {datetime.now().isoformat()}\n")
                f.write(f"{'='*60}\n\n")
                
                # Trajectory data
                f.write(f"Trajectory Points: {len(smart_car_auto['trajectory'])}\n")
                f.write(f"Waypoints: {len(smart_car_auto['waypoints'])}\n\n")
                
                # Trajectory points
                f.write("Trajectory Points (x, y, mode, timestamp):\n")
                for point in smart_car_auto['trajectory']:
                    f.write(f"  {point['x']:.2f}, {point['y']:.2f}, {point['mode']}, {point['timestamp']}\n")
                
                f.write("\n")
                
                # Waypoints
                f.write("Waypoints (x, y, index, mode):\n")
                for wp in smart_car_auto['waypoints']:
                    f.write(f"  {wp['x']:.2f}, {wp['y']:.2f}, {wp['index']}, {wp['mode']}\n")
                
                f.write("\n")
                
                # Raw data log
                f.write("Raw Data Log:\n")
                for log in smart_car_auto['data_log']:
                    f.write(f"  [{log['timestamp']}] {log['frame']}: {log['data']}\n")
            
            self.log_message(f"Data saved to: {filepath}", 'info')
            messagebox.showinfo("Save Success", f"Data saved to:\n{filepath}")
            
        except Exception as e:
            messagebox.showerror("Save Failed", str(e))
            self.log_message(f"Save failed: {e}", 'error')
    
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
