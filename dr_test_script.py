#!/usr/bin/env python3
"""
dr_test_script_working.py -
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import subprocess
import sys
import time
import logging
import threading
import os

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class IntegratedGCSTestController:
    """Working test controller"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Integrated GCS Test Controller")
        self.root.geometry("1000x800")
        
        # Process tracking
        self.gcs_process = None
        self.vehicle_process = None
        self.mission_file = None
        
        # Connection mode variable
        self.connection_mode = tk.StringVar(value="localhost")
        
        self._create_interface()
        
        # Check for default mission file
        default_files = ["track_follow_test.json", "test_mission.json", "integration_test_mission.json"]
        for filename in default_files:
            if os.path.exists(filename):
                self.mission_file = filename
                self.mission_status_label.config(text=f"Default: {filename}", fg='#48bb78')
                self._log_status(f"Found default mission file: {filename}")
                break
    
    def _create_interface(self):
        """Create the test interface"""
        # Header
        header_frame = tk.Frame(self.root, bg='#2c5282', height=60)
        header_frame.pack(fill=tk.X)
        header_frame.pack_propagate(False)
        
        title_label = tk.Label(
            header_frame, 
            text="Integrated GCS Test Controller", 
            font=('Arial', 16, 'bold'), 
            fg='white', bg='#2c5282'
        )
        title_label.pack(pady=15)
        
        # Main content
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=10)
        
        # Instructions
        instructions_frame = tk.LabelFrame(main_frame, text="Instructions", 
                                         font=('Arial', 12, 'bold'), padx=10, pady=10)
        instructions_frame.pack(fill=tk.X, pady=(0, 10))
        
        instructions_text = """1. Select mission file   2. Start GCS   3. Start vehicle   4. In GCS Mission Control: Load mission → Start vehicle → Send mission start"""
        tk.Label(instructions_frame, text=instructions_text, font=('Arial', 10), 
                wraplength=900, justify=tk.LEFT).pack()
        
        # Control sections
        controls_frame = tk.Frame(main_frame)
        controls_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Left column
        left_col = tk.Frame(controls_frame)
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        # 1. Mission File
        mission_frame = tk.LabelFrame(left_col, text="1. Mission File", 
                                    font=('Arial', 11, 'bold'), padx=10, pady=10)
        mission_frame.pack(fill=tk.X, pady=(0, 10))
        
        tk.Button(mission_frame, text="Select Mission File", 
                 command=self._select_mission_file,
                 font=('Arial', 10), bg='#4299e1', fg='white', width=20).pack(pady=5)
        
        self.mission_status_label = tk.Label(mission_frame, text="No mission selected", 
                                           font=('Arial', 9), fg='#f56565', wraplength=200)
        self.mission_status_label.pack()
        
        # 2. GCS Control
        gcs_frame = tk.LabelFrame(left_col, text="2. Integrated GCS", 
                                font=('Arial', 11, 'bold'), padx=10, pady=10)
        gcs_frame.pack(fill=tk.X, pady=(0, 10))
        
        gcs_buttons = tk.Frame(gcs_frame)
        gcs_buttons.pack()
        
        tk.Button(gcs_buttons, text="Start GCS", 
                 command=self._start_integrated_gcs,
                 font=('Arial', 10, 'bold'), bg='#48bb78', fg='white', width=10).pack(side=tk.LEFT, padx=2)
        
        tk.Button(gcs_buttons, text="Stop GCS", 
                 command=self._stop_gcs,
                 font=('Arial', 10), bg='#f56565', fg='white', width=10).pack(side=tk.LEFT, padx=2)
        
        self.gcs_status_label = tk.Label(gcs_frame, text="Not Started", 
                                       font=('Arial', 9), fg='#f56565')
        self.gcs_status_label.pack(pady=5)
        
        # Right column
        right_col = tk.Frame(controls_frame)
        right_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # 3. Connection Mode
        connection_frame = tk.LabelFrame(right_col, text="3. Connection Mode", 
                                       font=('Arial', 11, 'bold'), padx=15, pady=15)
        connection_frame.pack(fill=tk.X, pady=(0, 10))
        
        tk.Radiobutton(connection_frame, text="Localhost (Same Machine)", 
                      variable=self.connection_mode, value="localhost",
                      font=('Arial', 10), 
                      command=self._on_connection_mode_change).pack(anchor=tk.W, pady=2)
        
        tk.Radiobutton(connection_frame, text="WiFi (Separate Machines)", 
                      variable=self.connection_mode, value="wifi",
                      font=('Arial', 10),
                      command=self._on_connection_mode_change).pack(anchor=tk.W, pady=2)
        
        # Description label
        self.connection_description = tk.Label(connection_frame, 
                                             text="Vehicle connects to 127.0.0.1:15000",
                                             font=('Arial', 9), fg='#666666', wraplength=200)
        self.connection_description.pack(anchor=tk.W, pady=(5, 0))
        
        # 4. Vehicle Control
        vehicle_frame = tk.LabelFrame(right_col, text="4. Vehicle Control", 
                                    font=('Arial', 11, 'bold'), padx=10, pady=10)
        vehicle_frame.pack(fill=tk.X)
        
        vehicle_buttons = tk.Frame(vehicle_frame)
        vehicle_buttons.pack()
        
        tk.Button(vehicle_buttons, text="Start Vehicle", 
                 command=self._start_vehicle,
                 font=('Arial', 10, 'bold'), bg='#38b2ac', fg='white', width=12).pack(side=tk.LEFT, padx=2)
        
        tk.Button(vehicle_buttons, text="Stop Vehicle", 
                 command=self._stop_vehicle,
                 font=('Arial', 10), bg='#f56565', fg='white', width=12).pack(side=tk.LEFT, padx=2)
        
        self.vehicle_status_label = tk.Label(vehicle_frame, text="Not Running", 
                                           font=('Arial', 9), fg='#f56565')
        self.vehicle_status_label.pack(pady=5)
        
        # Status summary
        summary_frame = tk.Frame(main_frame, bg='#f0f8ff', relief='ridge', bd=2)
        summary_frame.pack(fill=tk.X, pady=(10, 5))
        
        self.summary_label = tk.Label(summary_frame, 
                                    text="System Status: Ready to start testing", 
                                    font=('Arial', 11, 'bold'), bg='#f0f8ff', fg='#2c5282')
        self.summary_label.pack(pady=8)
        
        # Log area
        log_frame = tk.LabelFrame(main_frame, text="System Log", 
                                font=('Arial', 11, 'bold'), padx=5, pady=5)
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        log_content = tk.Frame(log_frame)
        log_content.pack(fill=tk.BOTH, expand=True)
        
        self.status_text = tk.Text(log_content, height=12, font=('Consolas', 9), 
                                  state=tk.DISABLED, bg='#f8f9fa', wrap=tk.WORD)
        status_scrollbar = tk.Scrollbar(log_content, orient=tk.VERTICAL)
        
        self.status_text.config(yscrollcommand=status_scrollbar.set)
        status_scrollbar.config(command=self.status_text.yview)
        
        self.status_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        status_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Initialize
        self._log_status("Test Controller initialized - WORKING VERSION")
        self._log_status("Vehicle will wait for GCS mission start command")
        self._update_summary()
    
    def _on_connection_mode_change(self):
        """Handle connection mode change"""
        mode = self.connection_mode.get()
        if mode == "localhost":
            description = "Vehicle connects to 127.0.0.1:15000"
        else:
            description = "Vehicle scans WiFi network"
        
        self.connection_description.config(text=description)
        self._log_status(f"Connection mode: {mode}")
    
    def _select_mission_file(self):
        """Select a mission file"""
        file_path = filedialog.askopenfilename(
            title="Select Mission File",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir="."
        )
        
        if file_path:
            self.mission_file = file_path
            filename = os.path.basename(file_path)
            self.mission_status_label.config(text=f"Selected: {filename}", fg='#48bb78')
            self._log_status(f"Mission file: {filename}")
            self._update_summary()
    
    def _start_integrated_gcs(self):
        """Start the integrated GCS system"""
        try:
            if self.gcs_process and self.gcs_process.poll() is None:
                messagebox.showinfo("GCS", "Integrated GCS is already running")
                return
            
            self._log_status("Starting Integrated GCS...")
            
            cmd = [sys.executable, "integrated_gcs_main.py"]
            self.gcs_process = subprocess.Popen(cmd)
            
            self.gcs_status_label.config(text="Running", fg='#48bb78')
            self._log_status(f"GCS started (PID: {self.gcs_process.pid})")
            self._log_status("Main GCS window should open...")
            
            threading.Thread(target=self._monitor_gcs_process, daemon=True).start()
            self._update_summary()
            
        except Exception as e:
            self._log_status(f"ERROR: Failed to start GCS: {e}")
            messagebox.showerror("Error", f"Failed to start GCS:\n{str(e)}")
    
    def _monitor_gcs_process(self):
        """Monitor GCS process"""
        if not self.gcs_process:
            return
        
        try:
            return_code = self.gcs_process.wait()
            if return_code != 0:
                self._log_status(f"WARNING: GCS exited with code: {return_code}")
            else:
                self._log_status("GCS exited normally")
            self.gcs_status_label.config(text="Stopped", fg='#f56565')
        except Exception as e:
            self._log_status(f"Error monitoring GCS: {e}")
        finally:
            self.gcs_process = None
            self._update_summary()
    
    def _stop_gcs(self):
        """Stop the GCS"""
        try:
            if not self.gcs_process or self.gcs_process.poll() is not None:
                messagebox.showinfo("GCS", "GCS is not running")
                return
            
            self._log_status("Stopping GCS...")
            self.gcs_process.terminate()
            
            try:
                self.gcs_process.wait(timeout=5)
                self._log_status("GCS stopped")
            except subprocess.TimeoutExpired:
                self.gcs_process.kill()
                self.gcs_process.wait()
                self._log_status("GCS force stopped")
            
            self.gcs_status_label.config(text="Stopped", fg='#f56565')
            self.gcs_process = None
            self._update_summary()
            
        except Exception as e:
            self._log_status(f"Error stopping GCS: {e}")
    
    def _start_vehicle(self):
        """Start vehicle simulation - will wait for GCS mission start command"""
        try:
            if not self.mission_file:
                messagebox.showwarning("Mission Required", "Please select a mission file first")
                return
            
            if self.vehicle_process and self.vehicle_process.poll() is None:
                messagebox.showinfo("Vehicle", "Vehicle is already running")
                return
            
            # Check if GCS is running first
            if not self.gcs_process or self.gcs_process.poll() is not None:
                messagebox.showwarning("GCS Required", "Please start the GCS first")
                return
            
            connection_mode = self.connection_mode.get()
            self._log_status(f"Starting vehicle ({connection_mode} mode)...")
            
            time.sleep(2)  # Wait for GCS to fully start
            
            cmd = [
                sys.executable, "auv_main.py",
                "--mission", self.mission_file,
                "--verbose",
                "--console-output"
                # No --auto-start: vehicle will wait for GCS command
            ]
            
            if connection_mode == "localhost":
                cmd.extend(["--mavlink-host", "127.0.0.1"])
                cmd.extend(["--mavlink-port", "15000"])
                self._log_status("Vehicle will connect to 127.0.0.1:15000")
            else:
                cmd.extend(["--mavlink-host", "192.168.86.25"])
                cmd.extend(["--mavlink-port", "15000"])
                self._log_status("Vehicle will use WiFi")
            
            self._log_status(f"Command: {' '.join(cmd)}")
            
            self.vehicle_process = subprocess.Popen(cmd)
            
            self.vehicle_status_label.config(text="Waiting for GCS", fg='#f6ad55')
            self._log_status(f"Vehicle started (PID: {self.vehicle_process.pid})")
            self._log_status("Vehicle is waiting for mission_start command from GCS")
            self._log_status("Use Mission Control tab in GCS to:")
            self._log_status("1. Load the same mission file")
            self._log_status("2. Start the vehicle simulation")
            self._log_status("3. Send mission start command")
            
            threading.Thread(target=self._monitor_vehicle_process, daemon=True).start()
            self._update_summary()
            
        except Exception as e:
            self._log_status(f"ERROR: Failed to start vehicle: {e}")
            messagebox.showerror("Error", f"Failed to start vehicle:\n{str(e)}")
    
    def _monitor_vehicle_process(self):
        """Monitor vehicle process without captured output"""
        if not self.vehicle_process:
            return
        
        try:
            return_code = self.vehicle_process.wait()
            if return_code != 0:
                self._log_status(f"WARNING: Vehicle exited with code: {return_code}")
                self.vehicle_status_label.config(text="Error", fg='#f56565')
            else:
                self._log_status("Vehicle exited normally")
                self.vehicle_status_label.config(text="Stopped", fg='#f56565')
                
        except Exception as e:
            self._log_status(f"Error monitoring vehicle: {e}")
        finally:
            self.vehicle_process = None
            self._update_summary()
    
    def _stop_vehicle(self):
        """Stop the vehicle"""
        try:
            if not self.vehicle_process or self.vehicle_process.poll() is not None:
                messagebox.showinfo("Vehicle", "Vehicle is not running")
                return
            
            self._log_status("Stopping vehicle...")
            self.vehicle_process.terminate()
            
            try:
                self.vehicle_process.wait(timeout=5)
                self._log_status("Vehicle stopped")
            except subprocess.TimeoutExpired:
                self.vehicle_process.kill()
                self.vehicle_process.wait()
                self._log_status("Vehicle force stopped")
            
            self.vehicle_status_label.config(text="Stopped", fg='#f56565')
            self.vehicle_process = None
            self._update_summary()
            
        except Exception as e:
            self._log_status(f"Error stopping vehicle: {e}")
    
    def _update_summary(self):
        """Update the summary status"""
        gcs_running = self.gcs_process and self.gcs_process.poll() is None
        vehicle_running = self.vehicle_process and self.vehicle_process.poll() is None
        
        if gcs_running and vehicle_running:
            status = "GCS and Vehicle running - Use Mission Control to start mission"
            color = '#48bb78'
        elif gcs_running:
            status = "GCS running - Start vehicle next"
            color = '#f6ad55'
        elif self.mission_file:
            status = "Mission ready - Start GCS next"
            color = '#4299e1'
        else:
            status = "Select mission file to begin"
            color = '#666666'
        
        self.summary_label.config(text=status, fg=color)
    
    def _log_status(self, message):
        """Add message to status log"""
        try:
            timestamp = time.strftime("%H:%M:%S")
            log_message = f"[{timestamp}] {message}\n"
            
            self.status_text.config(state=tk.NORMAL)
            self.status_text.insert(tk.END, log_message)
            self.status_text.see(tk.END)
            self.status_text.config(state=tk.DISABLED)
            
            print(log_message.strip())
            
        except Exception as e:
            print(f"Error logging: {e}")
    
    def run(self):
        """Run the test controller"""
        self._log_status("Test controller ready")
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
        self.root.mainloop()
    
    def _on_closing(self):
        """Handle window close"""
        try:
            if self.vehicle_process and self.vehicle_process.poll() is None:
                self.vehicle_process.terminate()
            if self.gcs_process and self.gcs_process.poll() is None:
                self.gcs_process.terminate()
        except:
            pass
        self.root.destroy()


if __name__ == "__main__":
    try:
        print("Starting WORKING Test Controller...")
        controller = IntegratedGCSTestController()
        controller.run()
        
    except KeyboardInterrupt:
        print("\nTest controller interrupted")
    except Exception as e:
        print(f"Test controller error: {e}")
        import traceback
        traceback.print_exc()