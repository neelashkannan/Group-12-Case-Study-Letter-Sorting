import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import time
import numpy as np
import threading
import math
import json
import os
import cv2
import requests
from PIL import Image, ImageTk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Import PyBullet and the URDF visualization class
import pybullet as p
import pybullet_data
from urdf_visualization import URDFVisualization  # Import the class we created

class RoboticArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Advanced 3DoF Robotic Arm Controller")
        self.root.geometry("1200x800")
        
        # Set theme and styles
        self.style = ttk.Style()
        self.style.configure("TButton", font=('Arial', 10))
        self.style.configure("TLabel", font=('Arial', 10))
        self.style.configure("Header.TLabel", font=('Arial', 12, 'bold'))
        self.style.configure("Accent.TButton", background="#4CAF50")
        self.style.configure("Warning.TButton", background="#f44336")
        
        # Add position updates flag
        self.position_updates_enabled = False
        
        # Arm dimensions (in cm)
        self.ARM_SEGMENT_1 = 10.0  # Shoulder to elbow
        self.ARM_SEGMENT_2 = 12.0  # Elbow to end-effector
        
        # Current angles
        self.base_angle = 90
        self.shoulder_angle = 90
        self.elbow_angle = 90
        
        # For trajectory planning
        self.waypoints = []
        self.current_waypoint = 0
        self.trajectory_running = False
        self.trajectory_paused = False
        
        # For recording and playback
        self.recording = False
        self.record_positions = []
        self.playback_index = 0
        
        # Create a serial connection (None until connected)
        self.serial_conn = None
        self.serial_lock = threading.Lock()  # Add a lock for thread safety
        
        # For tracking arm status
        self.arm_status = "DISCONNECTED"
        self.is_moving = False
        
        # Webcam
        self.webcam_active = False
        self.webcam_thread = None
        self.last_frame = None
        self.cap = None
        
        # Create main frame with notebook
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create the main control tab
        self.main_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.main_tab, text="Main Control")
        
        # Create the trajectory planning tab
        self.trajectory_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.trajectory_tab, text="Trajectory Planning")
        
        # Create the calibration tab
        self.calibration_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.calibration_tab, text="Calibration")
        
        # Create the settings tab
        self.settings_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.settings_tab, text="Settings")
        
        # Create the URDF visualization tab
        self.urdf_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.urdf_tab, text="URDF Visualization")
        
        # Setup main control tab
        self.setup_main_tab()
        
        # Setup trajectory planning tab
        self.setup_trajectory_tab()
        
        # Setup calibration tab
        self.setup_calibration_tab()
        
        # Setup settings tab
        self.setup_settings_tab()
        
        # Setup URDF visualization tab
        self.setup_urdf_tab()
        
        # Initial port refresh
        self.refresh_ports()
        
        # Initial preview update
        self.update_preview()
        
        # Load settings if available
        self.load_settings()
        
        # Load letter trajectories mapping
        self.load_letter_trajectories()
        
        # Start update thread for reading serial data
        self.update_thread = threading.Thread(target=self.serial_reader, daemon=True)
        self.update_thread.start()
        
        # Setup periodic UI updates
        self.root.after(100, self.update_ui)
        
        # Add window close handler to cleanup PyBullet
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def setup_main_tab(self):
        """Setup the main control tab"""
        # Create left and right frames
        self.left_frame = ttk.Frame(self.main_tab)
        self.left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        self.right_frame = ttk.Frame(self.main_tab)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Create connection frame
        self.connection_frame = ttk.LabelFrame(self.left_frame, text="Connection", padding="10")
        self.connection_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Add connection controls
        ttk.Label(self.connection_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        
        # Port combobox
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(self.connection_frame, textvariable=self.port_var)
        self.port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        # Refresh and connect buttons
        self.refresh_btn = ttk.Button(self.connection_frame, text="Refresh", command=self.refresh_ports)
        self.refresh_btn.grid(row=0, column=2, padx=5, pady=5)
        
        self.connect_btn = ttk.Button(self.connection_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=5, pady=5)
        
        # Status label
        self.status_var = tk.StringVar(value="Not Connected")
        ttk.Label(self.connection_frame, text="Status:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.status_label = ttk.Label(self.connection_frame, textvariable=self.status_var)
        self.status_label.grid(row=1, column=1, columnspan=3, padx=5, pady=5, sticky="w")
        
        # Motion control section
        ttk.Label(self.connection_frame, text="Speed:").grid(row=2, column=0, padx=5, pady=5, sticky="w")
        self.speed_var = tk.IntVar(value=5)
        speed_slider = ttk.Scale(self.connection_frame, from_=1, to=10, variable=self.speed_var, orient=tk.HORIZONTAL)
        speed_slider.grid(row=2, column=1, padx=5, pady=5, sticky="ew")
        ttk.Label(self.connection_frame, textvariable=self.speed_var).grid(row=2, column=2, padx=5, pady=5, sticky="w")
        
        # Emergency stop and home buttons
        button_frame = ttk.Frame(self.connection_frame)
        button_frame.grid(row=3, column=0, columnspan=4, padx=5, pady=5, sticky="ew")
        
        self.stop_btn = ttk.Button(button_frame, text="EMERGENCY STOP", 
                                   command=self.emergency_stop, style="Warning.TButton")
        self.stop_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        self.home_btn = ttk.Button(button_frame, text="HOME", command=self.home_position)
        self.home_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # Sync visualization button
        self.sync_btn = ttk.Button(self.connection_frame, text="Sync Visualization", 
                              command=self.request_current_position)
        self.sync_btn.grid(row=4, column=0, columnspan=4, padx=5, pady=5, sticky="ew")
        
        # Create direct control frame
        self.control_frame = ttk.LabelFrame(self.left_frame, text="Direct Control", padding="10")
        self.control_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Servo sliders
        ttk.Label(self.control_frame, text="Base:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.base_var = tk.IntVar(value=90)
        self.base_slider = ttk.Scale(self.control_frame, from_=0, to=180, variable=self.base_var, 
                                    orient=tk.HORIZONTAL, command=self.update_preview)
        self.base_slider.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        ttk.Label(self.control_frame, textvariable=self.base_var).grid(row=0, column=2, padx=5, pady=5, sticky="w")
        
        ttk.Label(self.control_frame, text="Shoulder:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.shoulder_var = tk.IntVar(value=90)
        self.shoulder_slider = ttk.Scale(self.control_frame, from_=15, to=165, variable=self.shoulder_var, 
                                        orient=tk.HORIZONTAL, command=self.update_preview)
        self.shoulder_slider.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        ttk.Label(self.control_frame, textvariable=self.shoulder_var).grid(row=1, column=2, padx=5, pady=5, sticky="w")
        
        ttk.Label(self.control_frame, text="Elbow:").grid(row=2, column=0, padx=5, pady=5, sticky="w")
        self.elbow_var = tk.IntVar(value=90)
        self.elbow_slider = ttk.Scale(self.control_frame, from_=0, to=180, variable=self.elbow_var, 
                                     orient=tk.HORIZONTAL, command=self.update_preview)
        self.elbow_slider.grid(row=2, column=1, padx=5, pady=5, sticky="ew")
        ttk.Label(self.control_frame, textvariable=self.elbow_var).grid(row=2, column=2, padx=5, pady=5, sticky="w")
        
        # Send button
        self.send_btn = ttk.Button(self.control_frame, text="Send to Arm", command=self.send_servo_positions)
        self.send_btn.grid(row=3, column=0, columnspan=3, padx=5, pady=10)
        
        # Position control frame
        self.position_frame = ttk.LabelFrame(self.left_frame, text="Position Control", padding="10")
        self.position_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # X, Y, Z position controls
        ttk.Label(self.position_frame, text="X (cm):").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.x_var = tk.DoubleVar(value=10.0)
        self.x_entry = ttk.Entry(self.position_frame, textvariable=self.x_var, width=8)
        self.x_entry.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        self.x_slider = ttk.Scale(self.position_frame, from_=-20, to=20, variable=self.x_var, 
                                 orient=tk.HORIZONTAL, command=self.update_position_preview)
        self.x_slider.grid(row=0, column=2, padx=5, pady=5, sticky="ew")
        
        ttk.Label(self.position_frame, text="Y (cm):").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.y_var = tk.DoubleVar(value=0.0)
        self.y_entry = ttk.Entry(self.position_frame, textvariable=self.y_var, width=8)
        self.y_entry.grid(row=1, column=1, padx=5, pady=5, sticky="w")
        self.y_slider = ttk.Scale(self.position_frame, from_=-20, to=20, variable=self.y_var, 
                                 orient=tk.HORIZONTAL, command=self.update_position_preview)
        self.y_slider.grid(row=1, column=2, padx=5, pady=5, sticky="ew")
        
        ttk.Label(self.position_frame, text="Z (cm):").grid(row=2, column=0, padx=5, pady=5, sticky="w")
        self.z_var = tk.DoubleVar(value=5.0)
        self.z_entry = ttk.Entry(self.position_frame, textvariable=self.z_var, width=8)
        self.z_entry.grid(row=2, column=1, padx=5, pady=5, sticky="w")
        self.z_slider = ttk.Scale(self.position_frame, from_=-5, to=20, variable=self.z_var, 
                                 orient=tk.HORIZONTAL, command=self.update_position_preview)
        self.z_slider.grid(row=2, column=2, padx=5, pady=5, sticky="ew")
        
        # Move button
        self.move_btn = ttk.Button(self.position_frame, text="Move to Position", command=self.send_position)
        self.move_btn.grid(row=3, column=0, columnspan=3, padx=5, pady=10)
        
        # Create 3D visualization frame
        self.viz_frame = ttk.LabelFrame(self.right_frame, text="3D Visualization", padding="10")
        self.viz_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Create the figure for the 3D plot
        self.fig = Figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_zlabel('Z (cm)')
        self.ax.set_title('Robotic Arm Position')
        
        # Set fixed axis limits
        self.ax.set_xlim(-25, 25)
        self.ax.set_ylim(-25, 25)
        self.ax.set_zlim(-5, 25)
        
        # Create the canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.viz_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Console output frame
        self.console_frame = ttk.LabelFrame(self.right_frame, text="Console", padding="10")
        self.console_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Console text widget with scrollbar
        console_scroll = ttk.Scrollbar(self.console_frame)
        console_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.console = tk.Text(self.console_frame, height=10, width=40, yscrollcommand=console_scroll.set)
        self.console.pack(fill=tk.BOTH, expand=True)
        console_scroll.config(command=self.console.yview)
        
        # Console clear button
        self.clear_console_btn = ttk.Button(self.console_frame, text="Clear Console", 
                                           command=self.clear_console)
        self.clear_console_btn.pack(fill=tk.X, padx=5, pady=5)
    
    def setup_trajectory_tab(self):
        """Setup the trajectory planning tab"""
        # Create the main frames
        left_frame = ttk.Frame(self.trajectory_tab)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        right_frame = ttk.Frame(self.trajectory_tab)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Waypoints section
        waypoints_frame = ttk.LabelFrame(left_frame, text="Waypoints", padding="10")
        waypoints_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Waypoints list
        ttk.Label(waypoints_frame, text="Defined Waypoints:", style="Header.TLabel").pack(anchor="w", pady=5)
        
        # Waypoints listbox and scrollbar
        list_frame = ttk.Frame(waypoints_frame)
        list_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        waypoint_scroll = ttk.Scrollbar(list_frame)
        waypoint_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.waypoint_listbox = tk.Listbox(list_frame, height=10, yscrollcommand=waypoint_scroll.set)
        self.waypoint_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        waypoint_scroll.config(command=self.waypoint_listbox.yview)
        
        # Waypoint management buttons
        waypoint_buttons_frame = ttk.Frame(waypoints_frame)
        waypoint_buttons_frame.pack(fill=tk.X, pady=5)
        
        self.add_waypoint_btn = ttk.Button(waypoint_buttons_frame, text="Add Current Position", 
                                           command=self.add_waypoint)
        self.add_waypoint_btn.pack(side=tk.LEFT, padx=2, pady=5, fill=tk.X, expand=True)
        
        self.remove_waypoint_btn = ttk.Button(waypoint_buttons_frame, text="Remove Selected", 
                                              command=self.remove_waypoint)
        self.remove_waypoint_btn.pack(side=tk.LEFT, padx=2, pady=5, fill=tk.X, expand=True)
        
        self.clear_waypoints_btn = ttk.Button(waypoint_buttons_frame, text="Clear All", 
                                              command=self.clear_waypoints)
        self.clear_waypoints_btn.pack(side=tk.LEFT, padx=2, pady=5, fill=tk.X, expand=True)
        
        # Waypoint editing
        edit_frame = ttk.Frame(waypoints_frame)
        edit_frame.pack(fill=tk.X, pady=5)
        
        self.move_up_btn = ttk.Button(edit_frame, text="Move Up", command=self.move_waypoint_up)
        self.move_up_btn.pack(side=tk.LEFT, padx=2, pady=5, fill=tk.X, expand=True)
        
        self.move_down_btn = ttk.Button(edit_frame, text="Move Down", command=self.move_waypoint_down)
        self.move_down_btn.pack(side=tk.LEFT, padx=2, pady=5, fill=tk.X, expand=True)
        
        self.edit_waypoint_btn = ttk.Button(edit_frame, text="Edit Selected", command=self.edit_waypoint)
        self.edit_waypoint_btn.pack(side=tk.LEFT, padx=2, pady=5, fill=tk.X, expand=True)
        
        # Waypoint validation
        validate_frame = ttk.Frame(waypoints_frame)
        validate_frame.pack(fill=tk.X, pady=5)
        
        self.validate_trajectory_btn = ttk.Button(validate_frame, text="Validate Trajectory", 
                                                 command=self.validate_trajectory)
        self.validate_trajectory_btn.pack(fill=tk.X, padx=5, pady=5)
        
        # File operations frame
        file_frame = ttk.Frame(waypoints_frame)
        file_frame.pack(fill=tk.X, pady=5)
        
        self.save_trajectory_btn = ttk.Button(file_frame, text="Save Trajectory", 
                                              command=self.save_trajectory)
        self.save_trajectory_btn.pack(side=tk.LEFT, padx=2, pady=5, fill=tk.X, expand=True)
        
        self.load_trajectory_btn = ttk.Button(file_frame, text="Load Trajectory", 
                                              command=self.load_trajectory)
        self.load_trajectory_btn.pack(side=tk.LEFT, padx=2, pady=5, fill=tk.X, expand=True)
        
        # Trajectory execution frame
        execution_frame = ttk.LabelFrame(right_frame, text="Trajectory Execution", padding="10")
        execution_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Execution controls
        ttk.Label(execution_frame, text="Execution Settings:", style="Header.TLabel").pack(anchor="w", pady=5)
        
        # Speed control
        speed_frame = ttk.Frame(execution_frame)
        speed_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(speed_frame, text="Speed:").pack(side=tk.LEFT, padx=5)
        self.trajectory_speed_var = tk.IntVar(value=5)
        trajectory_speed_slider = ttk.Scale(speed_frame, from_=1, to=10, variable=self.trajectory_speed_var, 
                                           orient=tk.HORIZONTAL)
        trajectory_speed_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Label(speed_frame, textvariable=self.trajectory_speed_var).pack(side=tk.LEFT, padx=5)
        
        # Pause between waypoints
        pause_frame = ttk.Frame(execution_frame)
        pause_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(pause_frame, text="Pause between waypoints (sec):").pack(side=tk.LEFT, padx=5)
        self.pause_var = tk.DoubleVar(value=0.5)
        pause_slider = ttk.Scale(pause_frame, from_=0.0, to=5.0, variable=self.pause_var, 
                                orient=tk.HORIZONTAL)
        pause_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Label(pause_frame, textvariable=self.pause_var).pack(side=tk.LEFT, padx=5)
        
        # Loop options
        loop_frame = ttk.Frame(execution_frame)
        loop_frame.pack(fill=tk.X, pady=5)
        
        self.loop_var = tk.BooleanVar(value=False)
        loop_check = ttk.Checkbutton(loop_frame, text="Loop trajectory", variable=self.loop_var)
        loop_check.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(loop_frame, text="Loop count:").pack(side=tk.LEFT, padx=5)
        self.loop_count_var = tk.IntVar(value=1)
        loop_count_spinbox = ttk.Spinbox(loop_frame, from_=1, to=100, textvariable=self.loop_count_var, width=5)
        loop_count_spinbox.pack(side=tk.LEFT, padx=5)
        
        # Execution buttons
        execution_buttons_frame = ttk.Frame(execution_frame)
        execution_buttons_frame.pack(fill=tk.X, pady=10)
        
        self.execute_btn = ttk.Button(execution_buttons_frame, text="Execute Trajectory", 
                                      command=self.execute_trajectory, style="Accent.TButton")
        self.execute_btn.pack(side=tk.LEFT, padx=2, pady=5, fill=tk.X, expand=True)
        
        self.pause_btn = ttk.Button(execution_buttons_frame, text="Pause/Resume", 
                                    command=self.toggle_trajectory_pause)
        self.pause_btn.pack(side=tk.LEFT, padx=2, pady=5, fill=tk.X, expand=True)
        
        self.stop_trajectory_btn = ttk.Button(execution_buttons_frame, text="Stop Trajectory", 
                                             command=self.stop_trajectory, style="Warning.TButton")
        self.stop_trajectory_btn.pack(side=tk.LEFT, padx=2, pady=5, fill=tk.X, expand=True)
        
        # Execution progress
        progress_frame = ttk.Frame(execution_frame)
        progress_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(progress_frame, text="Progress:").pack(side=tk.LEFT, padx=5)
        self.progress_var = tk.DoubleVar(value=0.0)
        self.progress_bar = ttk.Progressbar(progress_frame, variable=self.progress_var, 
                                           orient=tk.HORIZONTAL, length=200, mode='determinate')
        self.progress_bar.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        self.progress_label_var = tk.StringVar(value="0/0")
        ttk.Label(progress_frame, textvariable=self.progress_label_var).pack(side=tk.LEFT, padx=5)
        
        # Recording frame
        recording_frame = ttk.LabelFrame(right_frame, text="Record & Playback", padding="10")
        recording_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Recording controls
        ttk.Label(recording_frame, text="Record arm movements and play them back:", 
                 style="Header.TLabel").pack(anchor="w", pady=5)
        
        recording_buttons_frame = ttk.Frame(recording_frame)
        recording_buttons_frame.pack(fill=tk.X, pady=5)
        
        self.record_btn = ttk.Button(recording_buttons_frame, text="Start Recording", 
                                    command=self.toggle_recording)
        self.record_btn.pack(side=tk.LEFT, padx=2, pady=5, fill=tk.X, expand=True)
        
        self.playback_btn = ttk.Button(recording_buttons_frame, text="Play Recording", 
                              command=self.play_recording)
        self.playback_btn.pack(side=tk.LEFT, padx=2, pady=5, fill=tk.X, expand=True)

        self.save_recording_btn = ttk.Button(recording_buttons_frame, text="Save Recording", 
                                    command=self.save_recording)
        self.save_recording_btn.pack(side=tk.LEFT, padx=2, pady=5, fill=tk.X, expand=True)

        self.load_recording_btn = ttk.Button(recording_buttons_frame, text="Load Recording", 
                                    command=self.load_recording)
        self.load_recording_btn.pack(side=tk.LEFT, padx=2, pady=5, fill=tk.X, expand=True)
        
        # Create Letters Recognition frame
        self.setup_letters_frame(right_frame)
    
    def setup_letters_frame(self, parent_frame):
        """Setup the Letters Recognition frame"""
        # Create the Letters Recognition frame
        letters_frame = ttk.LabelFrame(parent_frame, text="Letters Recognition", padding="10")
        letters_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Add explanation text
        explanation_text = "Capture image of an envelope to identify the recipient's name and execute the corresponding trajectory."
        ttk.Label(letters_frame, text=explanation_text, wraplength=300).pack(anchor="w", pady=5)
        
        # Webcam preview frame
        webcam_frame = ttk.Frame(letters_frame)
        webcam_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Webcam canvas for displaying video feed
        self.webcam_canvas = tk.Canvas(webcam_frame, width=320, height=240, bg="black")
        self.webcam_canvas.pack(side=tk.LEFT, padx=5, pady=5)
        
        # Controls frame
        controls_frame = ttk.Frame(webcam_frame)
        controls_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)
        
        # Webcam toggle button
        self.webcam_btn = ttk.Button(controls_frame, text="Start Webcam", 
                                    command=self.toggle_webcam)
        self.webcam_btn.pack(fill=tk.X, pady=5)
        
        # Capture button
        self.capture_btn = ttk.Button(controls_frame, text="Capture Image", 
                                     command=self.capture_image, state="disabled")
        self.capture_btn.pack(fill=tk.X, pady=5)
        
        # Analyze button
        self.analyze_btn = ttk.Button(controls_frame, text="Identify Recipient", 
                                     command=self.analyze_image, state="disabled")
        self.analyze_btn.pack(fill=tk.X, pady=5)
        
        # Execute corresponding trajectory button
        self.execute_letter_btn = ttk.Button(controls_frame, text="Execute Letter Trajectory", 
                                           command=self.execute_letter_trajectory, state="disabled")
        self.execute_letter_btn.pack(fill=tk.X, pady=5)
        
        # Create a frame for recognition results
        results_frame = ttk.LabelFrame(letters_frame, text="Recognition Results")
        results_frame.pack(fill=tk.X, expand=True, pady=5)
        
        # Recognition result label
        self.recognition_var = tk.StringVar(value="No image analyzed yet")
        ttk.Label(results_frame, text="Identified Name:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Label(results_frame, textvariable=self.recognition_var).grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        # Selected trajectory label
        self.selected_trajectory_var = tk.StringVar(value="None")
        ttk.Label(results_frame, text="Selected Trajectory:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        ttk.Label(results_frame, textvariable=self.selected_trajectory_var).grid(row=1, column=1, padx=5, pady=5, sticky="w")
        
        # Ollama settings frame
        ollama_frame = ttk.LabelFrame(letters_frame, text="Ollama Settings")
        ollama_frame.pack(fill=tk.X, expand=True, pady=5)
        
        # Ollama URL
        ttk.Label(ollama_frame, text="Ollama URL:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.ollama_url_var = tk.StringVar(value="http://localhost:11434/api/generate")
        self.ollama_url_entry = ttk.Entry(ollama_frame, textvariable=self.ollama_url_var, width=30)
        self.ollama_url_entry.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        # Model Name
        ttk.Label(ollama_frame, text="Model Name:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.model_name_var = tk.StringVar(value="llava-phi3")
        self.model_name_entry = ttk.Entry(ollama_frame, textvariable=self.model_name_var, width=30)
        self.model_name_entry.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        
        # Test connection button
        self.test_ollama_btn = ttk.Button(ollama_frame, text="Test Connection", 
                                         command=self.test_ollama_connection)
        self.test_ollama_btn.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        # Current webcam device selection
        ttk.Label(ollama_frame, text="Webcam Device:").grid(row=3, column=0, padx=5, pady=5, sticky="w")
        self.webcam_device_var = tk.IntVar(value=0)
        webcam_device_spinbox = ttk.Spinbox(ollama_frame, from_=0, to=10, textvariable=self.webcam_device_var, width=5)
        webcam_device_spinbox.grid(row=3, column=1, padx=5, pady=5, sticky="w")
        
        # Save Ollama settings button
        self.save_ollama_btn = ttk.Button(ollama_frame, text="Save Settings", 
                                         command=self.save_ollama_settings)
        self.save_ollama_btn.grid(row=4, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        # Trajectory mappings frame
        mapping_frame = ttk.LabelFrame(letters_frame, text="Letter to Trajectory Mappings")
        mapping_frame.pack(fill=tk.X, expand=True, pady=5)
        
        # Mapping controls
        mapping_controls = ttk.Frame(mapping_frame)
        mapping_controls.pack(fill=tk.X, pady=5)
        
        ttk.Label(mapping_controls, text="Letter:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.letter_var = tk.StringVar()
        self.letter_entry = ttk.Entry(mapping_controls, textvariable=self.letter_var, width=5)
        self.letter_entry.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        ttk.Label(mapping_controls, text="Trajectory File:").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        self.traj_file_var = tk.StringVar()
        self.traj_file_entry = ttk.Entry(mapping_controls, textvariable=self.traj_file_var, width=30)
        self.traj_file_entry.grid(row=0, column=3, padx=5, pady=5, sticky="ew")
        
        self.browse_traj_btn = ttk.Button(mapping_controls, text="Browse", 
                                          command=self.browse_trajectory_file)
        self.browse_traj_btn.grid(row=0, column=4, padx=5, pady=5)
        
        self.add_mapping_btn = ttk.Button(mapping_controls, text="Add Mapping", 
                                         command=self.add_letter_mapping)
        self.add_mapping_btn.grid(row=1, column=0, columnspan=5, padx=5, pady=5, sticky="ew")
        
        # Mappings listbox with scrollbar
        mapping_list_frame = ttk.Frame(mapping_frame)
        mapping_list_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        mapping_scroll = ttk.Scrollbar(mapping_list_frame)
        mapping_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.mapping_listbox = tk.Listbox(mapping_list_frame, height=4, yscrollcommand=mapping_scroll.set)
        self.mapping_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        mapping_scroll.config(command=self.mapping_listbox.yview)
        
        # Mapping action buttons
        mapping_actions = ttk.Frame(mapping_frame)
        mapping_actions.pack(fill=tk.X, pady=5)
        
        self.remove_mapping_btn = ttk.Button(mapping_actions, text="Remove Selected", 
                                            command=self.remove_letter_mapping)
        self.remove_mapping_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        
        self.save_mappings_btn = ttk.Button(mapping_actions, text="Save Mappings", 
                                           command=self.save_letter_mappings)
        self.save_mappings_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        
        # Initialize letter trajectories mapping
        self.letter_trajectories = {}
        self.captured_image = None
        self.identified_letter = None

    def setup_calibration_tab(self):
        """Setup the calibration tab"""
        # Create the main frame
        main_frame = ttk.Frame(self.calibration_tab, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Servo calibration frame
        calibration_frame = ttk.LabelFrame(main_frame, text="Servo Calibration", padding="10")
        calibration_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Information label
        info_text = "Adjust the offset values to calibrate each servo. " + \
                   "Positive values rotate clockwise, negative values rotate counter-clockwise."
        ttk.Label(calibration_frame, text=info_text, wraplength=500).pack(pady=10)
        
        # Base servo calibration
        base_frame = ttk.Frame(calibration_frame)
        base_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(base_frame, text="Base Servo Offset:").pack(side=tk.LEFT, padx=5)
        self.base_offset_var = tk.IntVar(value=0)
        base_offset_slider = ttk.Scale(base_frame, from_=-20, to=20, variable=self.base_offset_var, 
                                      orient=tk.HORIZONTAL)
        base_offset_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Label(base_frame, textvariable=self.base_offset_var).pack(side=tk.LEFT, padx=5)
        
        base_test_frame = ttk.Frame(calibration_frame)
        base_test_frame.pack(fill=tk.X, pady=5)
        
        self.base_test_0_btn = ttk.Button(base_test_frame, text="Test 0°", 
                                         command=lambda: self.test_calibration("BASE", 0))
        self.base_test_0_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        
        self.base_test_90_btn = ttk.Button(base_test_frame, text="Test 90°", 
                                          command=lambda: self.test_calibration("BASE", 90))
        self.base_test_90_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        
        self.base_test_180_btn = ttk.Button(base_test_frame, text="Test 180°", 
                                           command=lambda: self.test_calibration("BASE", 180))
        self.base_test_180_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        
        ttk.Separator(calibration_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        # Shoulder servo calibration
        shoulder_frame = ttk.Frame(calibration_frame)
        shoulder_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(shoulder_frame, text="Shoulder Servo Offset:").pack(side=tk.LEFT, padx=5)
        self.shoulder_offset_var = tk.IntVar(value=0)
        shoulder_offset_slider = ttk.Scale(shoulder_frame, from_=-20, to=20, variable=self.shoulder_offset_var, 
                                          orient=tk.HORIZONTAL)
        shoulder_offset_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Label(shoulder_frame, textvariable=self.shoulder_offset_var).pack(side=tk.LEFT, padx=5)
        
        shoulder_test_frame = ttk.Frame(calibration_frame)
        shoulder_test_frame.pack(fill=tk.X, pady=5)
        
        self.shoulder_test_15_btn = ttk.Button(shoulder_test_frame, text="Test 15°", 
                                              command=lambda: self.test_calibration("SHOULDER", 15))
        self.shoulder_test_15_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        
        self.shoulder_test_90_btn = ttk.Button(shoulder_test_frame, text="Test 90°", 
                                              command=lambda: self.test_calibration("SHOULDER", 90))
        self.shoulder_test_90_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        
        self.shoulder_test_165_btn = ttk.Button(shoulder_test_frame, text="Test 165°", 
                                               command=lambda: self.test_calibration("SHOULDER", 165))
        self.shoulder_test_165_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        
        ttk.Separator(calibration_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=10)
        
        # Elbow servo calibration
        elbow_frame = ttk.Frame(calibration_frame)
        elbow_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(elbow_frame, text="Elbow Servo Offset:").pack(side=tk.LEFT, padx=5)
        self.elbow_offset_var = tk.IntVar(value=0)
        elbow_offset_slider = ttk.Scale(elbow_frame, from_=-20, to=20, variable=self.elbow_offset_var, 
                                       orient=tk.HORIZONTAL)
        elbow_offset_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Label(elbow_frame, textvariable=self.elbow_offset_var).pack(side=tk.LEFT, padx=5)
        
        elbow_test_frame = ttk.Frame(calibration_frame)
        elbow_test_frame.pack(fill=tk.X, pady=5)
        
        self.elbow_test_0_btn = ttk.Button(elbow_test_frame, text="Test 0°", 
                                          command=lambda: self.test_calibration("ELBOW", 0))
        self.elbow_test_0_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        
        self.elbow_test_90_btn = ttk.Button(elbow_test_frame, text="Test 90°", 
                                           command=lambda: self.test_calibration("ELBOW", 90))
        self.elbow_test_90_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        
        self.elbow_test_180_btn = ttk.Button(elbow_test_frame, text="Test 180°", 
                                            command=lambda: self.test_calibration("ELBOW", 180))
        self.elbow_test_180_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        
        # Save calibration
        save_frame = ttk.Frame(calibration_frame)
        save_frame.pack(fill=tk.X, pady=15)
        
        self.apply_calibration_btn = ttk.Button(save_frame, text="Apply Calibration", 
                                               command=self.apply_calibration)
        self.apply_calibration_btn.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        self.save_calibration_btn = ttk.Button(save_frame, text="Save to ESP32", 
                                              command=self.save_calibration_to_esp32)
        self.save_calibration_btn.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
    
    def setup_settings_tab(self):
        """Setup the settings tab"""
        # Create the main frame
        main_frame = ttk.Frame(self.settings_tab, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Application settings frame
        settings_frame = ttk.LabelFrame(main_frame, text="Application Settings", padding="10")
        settings_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Arm dimensions settings
        dimensions_frame = ttk.Frame(settings_frame)
        dimensions_frame.pack(fill=tk.X, pady=10)
        
        ttk.Label(dimensions_frame, text="Arm Segment 1 (Shoulder to Elbow) in cm:").grid(row=0, column=0, 
                                                                                         padx=5, pady=5, sticky="w")
        self.arm_segment1_var = tk.DoubleVar(value=self.ARM_SEGMENT_1)
        arm_segment1_entry = ttk.Entry(dimensions_frame, textvariable=self.arm_segment1_var, width=8)
        arm_segment1_entry.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(dimensions_frame, text="Arm Segment 2 (Elbow to End Effector) in cm:").grid(row=1, column=0, 
                                                                                             padx=5, pady=5, sticky="w")
        self.arm_segment2_var = tk.DoubleVar(value=self.ARM_SEGMENT_2)
        arm_segment2_entry = ttk.Entry(dimensions_frame, textvariable=self.arm_segment2_var, width=8)
        arm_segment2_entry.grid(row=1, column=1, padx=5, pady=5)
        
        # Visualization settings
        viz_frame = ttk.LabelFrame(settings_frame, text="Visualization Settings")
        viz_frame.pack(fill=tk.X, pady=10)
        
        # Background color
        bg_frame = ttk.Frame(viz_frame)
        bg_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(bg_frame, text="Background Color:").pack(side=tk.LEFT, padx=5)
        
        self.bg_color_var = tk.StringVar(value="white")
        bg_combo = ttk.Combobox(bg_frame, textvariable=self.bg_color_var, width=15)
        bg_combo['values'] = ('white', 'black', 'lightgray', 'darkgray')
        bg_combo.pack(side=tk.LEFT, padx=5)
        
        self.apply_viz_btn = ttk.Button(bg_frame, text="Apply", command=self.apply_visualization_settings)
        self.apply_viz_btn.pack(side=tk.LEFT, padx=5)
        
        # Communication settings
        comm_frame = ttk.LabelFrame(settings_frame, text="Communication Settings")
        comm_frame.pack(fill=tk.X, pady=10)
        
        # Baud rate
        baud_frame = ttk.Frame(comm_frame)
        baud_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(baud_frame, text="Baud Rate:").pack(side=tk.LEFT, padx=5)
        
        self.baud_var = tk.IntVar(value=115200)
        baud_combo = ttk.Combobox(baud_frame, textvariable=self.baud_var, width=15)
        baud_combo['values'] = (9600, 19200, 38400, 57600, 115200, 230400)
        baud_combo.pack(side=tk.LEFT, padx=5)
        
        # Connection timeout
        timeout_frame = ttk.Frame(comm_frame)
        timeout_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(timeout_frame, text="Connection Timeout (sec):").pack(side=tk.LEFT, padx=5)
        
        self.timeout_var = tk.DoubleVar(value=1.0)
        timeout_spinbox = ttk.Spinbox(timeout_frame, from_=0.1, to=10.0, increment=0.1, 
                                     textvariable=self.timeout_var, width=5)
        timeout_spinbox.pack(side=tk.LEFT, padx=5)
        
        # About section
        about_frame = ttk.LabelFrame(settings_frame, text="About")
        about_frame.pack(fill=tk.X, pady=10)
        
        about_text = "Advanced 3DoF Robotic Arm Controller\n" + \
                    "Version 2.0\n\n" + \
                    "A full-featured control interface for 3-axis robotic arms\n" + \
                    "with smooth motion, trajectory planning, and calibration."
        ttk.Label(about_frame, text=about_text, justify=tk.CENTER).pack(padx=5, pady=10)
        
        # Save settings button
        self.save_settings_btn = ttk.Button(settings_frame, text="Save Settings", 
                                          command=self.save_settings)
        self.save_settings_btn.pack(pady=10)
    
    def setup_urdf_tab(self):
        """Setup the URDF visualization tab"""
        # Create main frame for the URDF tab
        urdf_main_frame = ttk.Frame(self.urdf_tab)
        urdf_main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Create a frame for URDF controls
        urdf_controls_frame = ttk.LabelFrame(urdf_main_frame, text="URDF Model Controls", padding="10")
        urdf_controls_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # URDF file selection frame
        file_frame = ttk.Frame(urdf_controls_frame)
        file_frame.pack(fill=tk.X, pady=5)
        
        self.urdf_path_var = tk.StringVar(value="No URDF file selected")
        ttk.Label(file_frame, text="URDF File:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Label(file_frame, textvariable=self.urdf_path_var, width=50).grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        # Browse button for URDF
        browse_btn = ttk.Button(file_frame, text="Browse...", command=self.browse_urdf)
        browse_btn.grid(row=0, column=2, padx=5, pady=5)
        
        # Joint mapping frame
        mapping_frame = ttk.LabelFrame(urdf_main_frame, text="Joint Mapping", padding="10")
        mapping_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.joint_mapping_frame = ttk.Frame(mapping_frame)
        self.joint_mapping_frame.pack(fill=tk.X, pady=5)
        
        # Add label explaining joint mapping
        ttk.Label(
            self.joint_mapping_frame, 
            text="Load a URDF file to map controller joints to URDF model joints.",
            wraplength=500
        ).pack(pady=10)
        
        # Create visualization frame
        viz_frame = ttk.LabelFrame(urdf_main_frame, text="3D Visualization", padding="10")
        viz_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Initialize the URDF visualization
        self.urdf_viz = URDFVisualization(viz_frame)
        
        # Synchronize button
        sync_btn = ttk.Button(
            urdf_controls_frame, 
            text="Synchronize with Controller", 
            command=self.sync_urdf_with_controller
        )
        sync_btn.pack(fill=tk.X, pady=5)
        
        # Real-time sync checkbox
        self.realtime_sync_var = tk.BooleanVar(value=False)
        realtime_sync_check = ttk.Checkbutton(
            urdf_controls_frame, 
            text="Real-time Synchronization", 
            variable=self.realtime_sync_var
        )
        realtime_sync_check.pack(anchor="w", pady=5)

    # Webcam and computer vision methods
    def toggle_webcam(self):
        """Toggle webcam on/off"""
        if not self.webcam_active:
            # Start webcam
            try:
                device_id = self.webcam_device_var.get()
                self.cap = cv2.VideoCapture(device_id)
                if not self.cap.isOpened():
                    messagebox.showerror("Webcam Error", f"Could not open webcam device {device_id}")
                    return
                
                self.webcam_active = True
                self.webcam_btn.config(text="Stop Webcam")
                self.capture_btn.config(state="normal")
                
                # Start webcam thread
                self.webcam_thread = threading.Thread(target=self.update_webcam, daemon=True)
                self.webcam_thread.start()
                
                self.log(f"Started webcam (device {device_id})")
            except Exception as e:
                self.log(f"Error starting webcam: {str(e)}")
                messagebox.showerror("Webcam Error", f"Failed to start webcam.\n{str(e)}")
        else:
            # Stop webcam
            self.webcam_active = False
            self.webcam_btn.config(text="Start Webcam")
            self.capture_btn.config(state="disabled")
            self.analyze_btn.config(state="disabled")
            
            if self.cap:
                self.cap.release()
                self.cap = None
            
            # Clear the webcam canvas
            self.webcam_canvas.delete("all")
            self.webcam_canvas.create_text(160, 120, text="Webcam Off", fill="white")
            
            self.log("Stopped webcam")
    
    def update_webcam(self):
        """Thread function to update webcam preview"""
        while self.webcam_active and self.cap is not None:
            try:
                ret, frame = self.cap.read()
                if ret:
                    # Resize frame to fit canvas
                    frame = cv2.resize(frame, (320, 240))
                    
                    # Convert OpenCV BGR to RGB for tkinter
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    
                    # Save the last frame
                    self.last_frame = frame.copy()
                    
                    # Convert to PhotoImage
                    img = Image.fromarray(frame_rgb)
                    img_tk = ImageTk.PhotoImage(image=img)
                    
                    # Update canvas
                    self.webcam_canvas.create_image(0, 0, anchor=tk.NW, image=img_tk)
                    self.webcam_canvas.image = img_tk  # Keep a reference
            except Exception as e:
                self.log(f"Webcam error: {str(e)}")
                break
                
            time.sleep(0.03)  # ~30 fps
    
    def capture_image(self):
        """Capture current frame from webcam"""
        if not self.webcam_active or self.last_frame is None:
            self.log("No webcam feed available")
            return
            
        try:
            # Save the captured image
            self.captured_image = self.last_frame.copy()
            
            # Save the image to a temporary file for analysis
            cv2.imwrite("temp_capture.jpg", self.captured_image)
            
            # Enable the analyze button
            self.analyze_btn.config(state="normal")
            
            self.log("Image captured from webcam")
            self.recognition_var.set("Image captured. Click 'Identify Recipient' to analyze.")
        except Exception as e:
            self.log(f"Error capturing image: {str(e)}")
    
    def analyze_image(self):
        """Analyze the captured image with Ollama/LLaVA-Phi3"""
        if self.captured_image is None:
            self.log("No image captured")
            return
            
        try:
            # Convert the image to base64 for Ollama API
            _, img_encoded = cv2.imencode('.jpg', self.captured_image)
            import base64
            img_base64 = "data:image/jpeg;base64," + base64.b64encode(img_encoded.tobytes()).decode('utf-8')
            
            # Prepare the prompt for envelope recipient detection
            prompt = (
                "This is an image of an envelope. Please identify the name of the recipient on this envelope. "
                "Look for the name or initial written on the envelope. "
                "If there are multiple names, identify the main recipient. "
                "If you can identify a single letter or initial, just return that letter. "
                "Return ONLY the name or initial, nothing else."
            )
            
            # Create the request payload
            payload = {
                "model": self.model_name_var.get(),
                "prompt": prompt,
                "stream": False,
                "images": [img_base64]
            }
            
            self.log("Analyzing image with LLaVA-Phi3...")
            self.recognition_var.set("Analyzing image...")
            
            # Show processing cursor
            self.root.config(cursor="watch")
            self.analyze_btn.config(state="disabled")
            
            # Make the API request with timeout
            response = requests.post(
                self.ollama_url_var.get(),
                json=payload,
                headers={"Content-Type": "application/json"},
                timeout=30  # Add timeout to prevent hanging indefinitely
            )
            
            if response.status_code == 200:
                result = response.json()
                response_text = result.get("response", "").strip()
                
                self.log(f"LLaVA-Phi3 identified: '{response_text}'")
                self.recognition_var.set(response_text)
                
                # Check if we have a letter identified
                if response_text:
                    # Get the first letter and make it uppercase for matching
                    self.identified_letter = response_text[0].upper()
                    
                    # Check if we have a trajectory for this letter
                    if self.identified_letter in self.letter_trajectories:
                        trajectory_file = self.letter_trajectories[self.identified_letter]
                        trajectory_filename = os.path.basename(trajectory_file)
                        self.selected_trajectory_var.set(f"{trajectory_filename}")
                        self.execute_letter_btn.config(state="normal")
                        self.log(f"Found trajectory for letter '{self.identified_letter}': {trajectory_filename}")
                    else:
                        self.selected_trajectory_var.set(f"No trajectory for '{self.identified_letter}'")
                        self.execute_letter_btn.config(state="disabled")
                        self.log(f"No trajectory found for letter '{self.identified_letter}'")
                else:
                    self.identified_letter = None
                    self.selected_trajectory_var.set("No name identified")
                    self.execute_letter_btn.config(state="disabled")
            else:
                self.log(f"Error from Ollama API: {response.status_code} - {response.text}")
                self.recognition_var.set(f"Error: {response.status_code}")
                messagebox.showerror("Ollama API Error", f"API returned error {response.status_code}.\n{response.text}")
        except requests.exceptions.Timeout:
            self.log("Ollama API request timed out")
            self.recognition_var.set("Analysis timeout")
            messagebox.showerror("Analysis Error", "Ollama API request timed out.\nThe model may be loading or too busy.")
        except Exception as e:
            self.log(f"Error analyzing image: {str(e)}")
            self.recognition_var.set("Analysis error")
            messagebox.showerror("Analysis Error", f"Failed to analyze image.\n{str(e)}")
        finally:
            # Reset cursor and re-enable button
            self.root.config(cursor="")
            self.analyze_btn.config(state="normal")
    
    def execute_letter_trajectory(self):
        """Execute the trajectory for the identified letter"""
        if not self.identified_letter or self.identified_letter not in self.letter_trajectories:
            self.log("No valid letter trajectory to execute")
            return
            
        trajectory_file = self.letter_trajectories[self.identified_letter]
        
        # Check if file exists
        if not os.path.exists(trajectory_file):
            self.log(f"Trajectory file not found: {trajectory_file}")
            messagebox.showerror("File Error", f"Trajectory file not found:\n{trajectory_file}")
            return
            
        try:
            # Load the trajectory
            with open(trajectory_file, "r") as f:
                trajectory_data = json.load(f)
                
            if "waypoints" in trajectory_data:
                # Clear current waypoints
                self.waypoints = []
                self.waypoint_listbox.delete(0, tk.END)
                
                # Add loaded waypoints
                for x, y, z in trajectory_data["waypoints"]:
                    self.waypoints.append((x, y, z))
                    self.waypoint_listbox.insert(tk.END, f"Point {len(self.waypoints)}: ({x:.2f}, {y:.2f}, {z:.2f})")
                
                self.log(f"Loaded {len(self.waypoints)} waypoints from {trajectory_file}")
                
                # Validate trajectory before execution
                validation_result = self.validate_trajectory_silently()
                if not validation_result:
                    response = messagebox.askyesno(
                        "Validation Warning", 
                        "Some waypoints in this trajectory may be unreachable. Execute anyway?",
                        icon='warning'
                    )
                    if not response:
                        return
                
                # Switch to the trajectory tab
                self.notebook.select(1)  # Index 1 is trajectory planning tab
                
                # Execute the trajectory
                self.execute_trajectory()
            else:
                self.log("Invalid trajectory file format")
                messagebox.showerror("Load Error", "Invalid trajectory file format.")
        except json.JSONDecodeError:
            self.log(f"Invalid JSON format in trajectory file: {trajectory_file}")
            messagebox.showerror("Format Error", "The trajectory file contains invalid JSON data.")
        except Exception as e:
            self.log(f"Error executing letter trajectory: {str(e)}")
            messagebox.showerror("Execution Error", f"Failed to execute trajectory.\n{str(e)}")
            
    def validate_trajectory_silently(self):
        """Validate the trajectory without showing messages, return True if valid"""
        if not self.waypoints:
            return False
            
        all_valid = True
        
        for i, (x, y, z) in enumerate(self.waypoints):
            # Check if position is reachable
            d = math.sqrt(x*x + y*y + z*z)
            if d > (self.ARM_SEGMENT_1 + self.ARM_SEGMENT_2) or d < abs(self.ARM_SEGMENT_1 - self.ARM_SEGMENT_2):
                all_valid = False
                self.log(f"Waypoint {i+1} is out of reach: ({x:.2f}, {y:.2f}, {z:.2f})")
                continue
                
            # Calculate base angle
            theta = math.degrees(math.atan2(y, x))
            if theta < 0 or theta > 180:
                all_valid = False
                continue
                
            # Calculate other angles to check joint limits
            r = math.sqrt(x*x + y*y)
            try:
                # Calculate elbow angle
                elbow_rad = math.acos((self.ARM_SEGMENT_1**2 + self.ARM_SEGMENT_2**2 - d**2) / 
                                     (2 * self.ARM_SEGMENT_1 * self.ARM_SEGMENT_2))
                elbow_deg = 180.0 - math.degrees(elbow_rad)
                
                # Calculate shoulder angle
                alpha = math.atan2(z, r)
                beta = math.acos((self.ARM_SEGMENT_1**2 + d**2 - self.ARM_SEGMENT_2**2) / 
                                (2 * self.ARM_SEGMENT_1 * d))
                shoulder_deg = math.degrees(alpha + beta)
                
                # Check servo limits
                if not (15 <= shoulder_deg <= 165) or not (0 <= elbow_deg <= 180):
                    all_valid = False
                    continue
                    
            except (ValueError, ZeroDivisionError):
                all_valid = False
                continue
        
        return all_valid

    def test_ollama_connection(self):
        """Test the connection to Ollama server"""
        try:
            # Get the base URL from the generate URL
            base_url = self.ollama_url_var.get().rsplit('/', 1)[0]
            models_url = f"{base_url}/api/tags"
            
            self.log(f"Testing connection to Ollama at {models_url}...")
            
            # Check if Ollama is running and the model exists
            # Add a timeout to prevent hanging
            response = requests.get(models_url, timeout=5)
            
            if response.status_code == 200:
                models = response.json().get("models", [])
                model_exists = any(model["name"] == self.model_name_var.get() for model in models)
                
                if model_exists:
                    self.log(f"Successfully connected to Ollama. Model '{self.model_name_var.get()}' is available.")
                    messagebox.showinfo("Connection Test", f"Successfully connected to Ollama.\nModel '{self.model_name_var.get()}' is available.")
                else:
                    self.log(f"Connected to Ollama, but model '{self.model_name_var.get()}' is not available.")
                    
                    # List available models in a more readable format
                    available_models = [model['name'] for model in models[:10]]
                    model_list = "\n- ".join([""] + available_models)
                    if len(models) > 10:
                        model_list += "\n- ..."
                    
                    messagebox.showwarning("Connection Test", 
                                          f"Connected to Ollama, but model '{self.model_name_var.get()}' is not available.\n\n" +
                                          f"Available models:{model_list}")
            else:
                self.log(f"Error connecting to Ollama: {response.status_code} - {response.text}")
                messagebox.showerror("Connection Test", f"Failed to connect to Ollama.\n{response.status_code} - {response.text}")
        except requests.exceptions.Timeout:
            self.log("Connection to Ollama timed out")
            messagebox.showerror("Connection Test", 
                               "Connection to Ollama timed out.\nPlease check if Ollama is running and the URL is correct.")
        except requests.exceptions.ConnectionError:
            self.log("Could not connect to Ollama server")
            messagebox.showerror("Connection Test", 
                               "Could not connect to Ollama server.\nPlease check if Ollama is running and the URL is correct.")
        except Exception as e:
            self.log(f"Error testing Ollama connection: {str(e)}")
            messagebox.showerror("Connection Test", f"Failed to connect to Ollama.\n{str(e)}")
    
    def save_ollama_settings(self):
        """Save Ollama settings to the settings file"""
        try:
            # Create or update settings dictionary
            settings = {}
            if os.path.exists("arm_settings.json"):
                with open("arm_settings.json", "r") as f:
                    settings = json.load(f)
            
            # Update Ollama settings
            settings["ollama_url"] = self.ollama_url_var.get()
            settings["model_name"] = self.model_name_var.get()
            settings["webcam_device"] = self.webcam_device_var.get()
            
            # Save to file
            with open("arm_settings.json", "w") as f:
                json.dump(settings, f, indent=4)
            
            self.log("Ollama settings saved")
            messagebox.showinfo("Settings", "Ollama settings saved successfully.")
        except Exception as e:
            self.log(f"Error saving Ollama settings: {str(e)}")
            messagebox.showerror("Settings Error", f"Failed to save Ollama settings.\n{str(e)}")
    
    def browse_trajectory_file(self):
        """Browse for a trajectory file to map to a letter"""
        try:
            file_path = filedialog.askopenfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Select Trajectory File"
            )
            
            if file_path:
                self.traj_file_var.set(file_path)
        except Exception as e:
            self.log(f"Error browsing trajectory file: {str(e)}")
    
    def add_letter_mapping(self):
        """Add or update a letter to trajectory mapping"""
        letter = self.letter_var.get().strip().upper()
        trajectory_file = self.traj_file_var.get().strip()
        
        if not letter:
            messagebox.showwarning("Mapping Error", "Please enter a letter.")
            return
            
        if not trajectory_file:
            messagebox.showwarning("Mapping Error", "Please select a trajectory file.")
            return
            
        if len(letter) > 1:
            letter = letter[0]  # Take only the first character
            
        # Update the mapping
        self.letter_trajectories[letter] = trajectory_file
        
        # Update the listbox
        self.refresh_mapping_listbox()
        
        self.log(f"Added mapping: Letter '{letter}' → {trajectory_file}")
        
        # Clear the input fields
        self.letter_var.set("")
        self.traj_file_var.set("")
    
    def remove_letter_mapping(self):
        """Remove a letter to trajectory mapping"""
        try:
            selected = self.mapping_listbox.curselection()[0]
            selected_text = self.mapping_listbox.get(selected)
            
            # Extract the letter from the selected text
            letter = selected_text.split("'")[1]
            
            # Remove from the mapping
            if letter in self.letter_trajectories:
                del self.letter_trajectories[letter]
                
                # Update the listbox
                self.refresh_mapping_listbox()
                
                self.log(f"Removed mapping for letter '{letter}'")
            else:
                self.log(f"Letter '{letter}' not found in mappings")
        except IndexError:
            self.log("No mapping selected")
            messagebox.showwarning("Selection Error", "Please select a mapping to remove.")
        except Exception as e:
            self.log(f"Error removing mapping: {str(e)}")
    
    def refresh_mapping_listbox(self):
        """Refresh the letter mappings listbox"""
        self.mapping_listbox.delete(0, tk.END)
        
        # Check if no mappings exist
        if not self.letter_trajectories:
            self.mapping_listbox.insert(tk.END, "No letter mappings defined")
            return
            
        for letter, trajectory_file in sorted(self.letter_trajectories.items()):
            # Handle potentially invalid paths gracefully
            try:
                file_name = os.path.basename(trajectory_file)
                # Check if file exists
                file_exists = os.path.exists(trajectory_file)
                if file_exists:
                    self.mapping_listbox.insert(tk.END, f"Letter '{letter}' → {file_name}")
                else:
                    self.mapping_listbox.insert(tk.END, f"Letter '{letter}' → {file_name} (missing)")
            except:
                self.mapping_listbox.insert(tk.END, f"Letter '{letter}' → {trajectory_file} (invalid path)")
    
    def save_letter_mappings(self):
        """Save letter to trajectory mappings to file"""
        try:
            mappings_file = "letter_trajectories.json"
            
            with open(mappings_file, "w") as f:
                json.dump(self.letter_trajectories, f, indent=4)
                
            self.log(f"Saved {len(self.letter_trajectories)} letter mappings to {mappings_file}")
            messagebox.showinfo("Mappings", f"Saved {len(self.letter_trajectories)} letter mappings.")
        except Exception as e:
            self.log(f"Error saving letter mappings: {str(e)}")
            messagebox.showerror("Save Error", f"Failed to save letter mappings.\n{str(e)}")
    
    def load_letter_trajectories(self):
        """Load letter to trajectory mappings from file"""
        try:
            mappings_file = "letter_trajectories.json"
            
            if os.path.exists(mappings_file):
                with open(mappings_file, "r") as f:
                    self.letter_trajectories = json.load(f)
                    
                self.log(f"Loaded {len(self.letter_trajectories)} letter mappings")
                
                # Update the listbox if it exists
                if hasattr(self, 'mapping_listbox'):
                    self.refresh_mapping_listbox()
        except Exception as e:
            self.log(f"Error loading letter mappings: {str(e)}")
    
    # Connection methods
    def refresh_ports(self):
        """Refresh the list of available serial ports"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)
    
    def toggle_connection(self):
        """Connect to or disconnect from the serial port"""
        if self.serial_conn is None:
            # Try to connect
            try:
                port = self.port_var.get()
                if not port:
                    messagebox.showwarning("Connection Error", "No port selected.")
                    return
                    
                self.serial_conn = serial.Serial(port, self.baud_var.get(), timeout=self.timeout_var.get())
                time.sleep(2)  # Wait for Arduino to reset
                
                # Update UI
                self.status_var.set(f"Connected to {port}")
                self.connect_btn.config(text="Disconnect")
                self.log(f"Connected to {port} at {self.baud_var.get()} baud")
                
                # Removed auto position request
                # self.request_current_position()
            except Exception as e:
                self.log(f"Error connecting: {str(e)}")
                self.status_var.set("Connection failed")
                messagebox.showerror("Connection Error", f"Failed to connect to {port}.\n{str(e)}")
        else:
            # Disconnect
            try:
                with self.serial_lock:  # Use lock to avoid thread conflicts
                    if self.serial_conn and self.serial_conn.is_open:
                        self.serial_conn.close()
            except Exception as e:
                self.log(f"Error disconnecting: {str(e)}")
            finally:
                self.serial_conn = None
                self.status_var.set("Not Connected")
                self.connect_btn.config(text="Connect")
                self.log("Disconnected")
                
                # Reset visualization to match slider values
                self.update_preview()
    
    # User interface updates
    def update_ui(self):
        """Periodically update UI elements"""
        # Change status label color based on arm state
        if self.arm_status == "EMERGENCY_STOP":
            self.status_label.configure(foreground="red")
        elif self.arm_status == "MOVING":
            self.status_label.configure(foreground="blue")
        else:
            self.status_label.configure(foreground="green")
            
        # Update status if disconnected
        if self.serial_conn is None or not hasattr(self.serial_conn, 'is_open') or not self.serial_conn.is_open:
            self.status_label.configure(foreground="gray")
        
        # Sync URDF model if real-time sync is enabled
        if hasattr(self, 'realtime_sync_var') and self.realtime_sync_var.get():
            self.sync_urdf_with_controller()
        
        # Schedule the next update
        self.root.after(100, self.update_ui)
    
    # Control methods
    def send_servo_positions(self):
        """Send the servo positions to the arm"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.log("Not connected to arm")
            messagebox.showwarning("Not Connected", "Please connect to the arm first.")
            return
        
        try:
            base = self.base_var.get()
            shoulder = self.shoulder_var.get()
            elbow = self.elbow_var.get()
            
            with self.serial_lock:  # Use lock for thread safety
                # First send the speed setting
                speed_command = f"SPEED,{self.speed_var.get()}\n"
                self.serial_conn.write(speed_command.encode())
                time.sleep(0.1)  # Small delay to ensure commands are processed in order
                
                # Format the command: S,base,shoulder,elbow
                command = f"S,{base},{shoulder},{elbow}\n"
                self.serial_conn.write(command.encode())
            
            self.log(f"Sent: {command.strip()} at speed {self.speed_var.get()}")
            
            # Update the status
            self.status_var.set("Moving to servo positions...")
            self.arm_status = "MOVING"
            self.is_moving = True
            
            # Record position if recording is active
            if self.recording:
                self.record_positions.append(('S', base, shoulder, elbow))
        except Exception as e:
            self.log(f"Error sending command: {str(e)}")
            messagebox.showerror("Communication Error", f"Failed to send command.\n{str(e)}")
    
    def emergency_stop(self):
        """Send the emergency stop command"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.log("Not connected to arm")
            return
        
        try:
            with self.serial_lock:  # Use lock for thread safety
                # Send emergency stop command
                self.serial_conn.write("STOP\n".encode())
            
            self.log("Emergency stop triggered")
            self.status_var.set("EMERGENCY STOP")
            self.arm_status = "EMERGENCY_STOP"
            self.is_moving = False
            
            # Stop any trajectory execution
            self.trajectory_running = False
            self.recording = False
        except Exception as e:
            self.log(f"Error sending emergency stop: {str(e)}")
    
    def home_position(self):
        """Move the arm to home position"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.log("Not connected to arm")
            messagebox.showwarning("Not Connected", "Please connect to the arm first.")
            return
        
        try:
            with self.serial_lock:  # Use lock for thread safety
                # Send home command
                self.serial_conn.write("HOME\n".encode())
            
            self.log("Moving to home position")
            self.status_var.set("Moving to home position...")
            self.arm_status = "MOVING"
            self.is_moving = True
            
            # Update sliders to home position
            self.base_var.set(90)
            self.shoulder_var.set(90)
            self.elbow_var.set(90)
            
            # Update preview
            self.update_preview()
            
            # Record position if recording is active
            if self.recording:
                self.record_positions.append(('HOME',))
        except Exception as e:
            self.log(f"Error sending home command: {str(e)}")
    
    def send_position(self):
        """Send the XYZ position to the arm using inverse kinematics"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.log("Not connected to arm")
            messagebox.showwarning("Not Connected", "Please connect to the arm first.")
            return
        
        try:
            x = self.x_var.get()
            y = self.y_var.get()
            z = self.z_var.get()
            
            # Check if position is reachable (simple check)
            d = math.sqrt(x*x + y*y + z*z)
            if d > (self.ARM_SEGMENT_1 + self.ARM_SEGMENT_2):
                self.log(f"Position ({x:.2f},{y:.2f},{z:.2f}) is out of reach")
                messagebox.showwarning("Position Error", 
                                      f"Position ({x:.2f},{y:.2f},{z:.2f}) is out of reach.")
                return
                
            with self.serial_lock:  # Use lock for thread safety
                # First send the speed setting
                speed_command = f"SPEED,{self.speed_var.get()}\n"
                self.serial_conn.write(speed_command.encode())
                time.sleep(0.1)  # Small delay to ensure commands are processed in order
                
                # Format the command: P,x,y,z
                command = f"P,{x:.2f},{y:.2f},{z:.2f}\n"
                self.serial_conn.write(command.encode())
            
            self.log(f"Sent: {command.strip()} at speed {self.speed_var.get()}")
            
            # Update status in GUI
            self.status_var.set("Moving to position...")
            self.arm_status = "MOVING"
            self.is_moving = True
            
            # Record position if recording is active
            if self.recording:
                self.record_positions.append(('P', x, y, z))
        except Exception as e:
            self.log(f"Error sending position: {str(e)}")
            messagebox.showerror("Communication Error", f"Failed to send position.\n{str(e)}")
    
    def request_current_position(self):
        """Request the current position from the arm"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        
        try:
            # Enable position updates for a short period
            self.position_updates_enabled = True
            
            # Set a timer to disable updates after 1 second
            self.root.after(1000, lambda: setattr(self, 'position_updates_enabled', False))
            
            with self.serial_lock:  # Use lock for thread safety
                # Send command to request current angles
                self.serial_conn.write("GET_ANGLES\n".encode())
                # Send command to request current position
                self.serial_conn.write("GET_POS\n".encode())
                # Also request arm status
                self.serial_conn.write("STATUS\n".encode())
            
            self.log("Requested current position and status")
        except Exception as e:
            self.log(f"Error requesting position: {str(e)}")
    
    # Utility methods
    def log(self, message):
        """Add a message to the console log with timestamp"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        self.console.insert(tk.END, f"[{timestamp}] {message}\n")
        self.console.see(tk.END)  # Scroll to the end
        
    def clear_console(self):
        """Clear the console log"""
        self.console.delete(1.0, tk.END)
        self.log("Console cleared")
    
    # Visualization methods
    def update_preview(self, *args):
        """Update the 3D visualization based on servo angles"""
        # Get current angles
        base = self.base_var.get()
        shoulder = self.shoulder_var.get()
        elbow = self.elbow_var.get()
        
        # Convert to radians for calculations
        base_rad = math.radians(base)
        shoulder_rad = math.radians(shoulder)
        elbow_rad = math.radians(elbow)
        
        # Calculate joint positions
        # Base position
        base_pos = [0, 0, 0]
        
        # Shoulder position (relative to base)
        shoulder_pos = [0, 0, 0]  # At same position as base
        
        # Elbow position
        elbow_x = self.ARM_SEGMENT_1 * math.cos(shoulder_rad) * math.cos(base_rad)
        elbow_y = self.ARM_SEGMENT_1 * math.cos(shoulder_rad) * math.sin(base_rad)
        elbow_z = self.ARM_SEGMENT_1 * math.sin(shoulder_rad)
        elbow_pos = [elbow_x, elbow_y, elbow_z]
        
        # End effector position
        end_x = elbow_x + self.ARM_SEGMENT_2 * math.cos(shoulder_rad + elbow_rad - math.pi) * math.cos(base_rad)
        end_y = elbow_y + self.ARM_SEGMENT_2 * math.cos(shoulder_rad + elbow_rad - math.pi) * math.sin(base_rad)
        end_z = elbow_z + self.ARM_SEGMENT_2 * math.sin(shoulder_rad + elbow_rad - math.pi)
        end_pos = [end_x, end_y, end_z]
        
        # Update position variables (without triggering callbacks)
        try:
            self.x_var.set(round(end_x, 2))
            self.y_var.set(round(end_y, 2))
            self.z_var.set(round(end_z, 2))
        except Exception as e:
            # This can happen if variables aren't initialized yet
            pass
        
        # Draw the arm
        self.draw_arm([base_pos, shoulder_pos, elbow_pos, end_pos])
        
        # Also update URDF model if real-time sync is enabled
        if hasattr(self, 'realtime_sync_var') and self.realtime_sync_var.get():
            self.sync_urdf_with_controller()
    
    def update_position_preview(self, *args):
        """Update the preview based on XYZ position (using inverse kinematics)"""
        try:
            x = self.x_var.get()
            y = self.y_var.get()
            z = self.z_var.get()
            
            # Calculate base angle (theta) in degrees
            theta = math.degrees(math.atan2(y, x))
            
            # Convert to polar coordinates in the vertical plane
            r = math.sqrt(x*x + y*y)
            d = math.sqrt(r*r + z*z)
            
            # Check if the position is reachable
            if d > (self.ARM_SEGMENT_1 + self.ARM_SEGMENT_2) or d < abs(self.ARM_SEGMENT_1 - self.ARM_SEGMENT_2):
                self.log(f"Position ({x:.2f},{y:.2f},{z:.2f}) is out of reach")
                return False
            
            # Law of cosines to find elbow angle
            try:
                elbow_rad = math.acos((self.ARM_SEGMENT_1**2 + self.ARM_SEGMENT_2**2 - d**2) / 
                                    (2 * self.ARM_SEGMENT_1 * self.ARM_SEGMENT_2))
                elbow_deg = 180.0 - math.degrees(elbow_rad)
            except ValueError:
                self.log(f"Inverse kinematics calculation error. Position may be out of reach.")
                return False
            
            # Find the shoulder angle
            try:
                alpha = math.atan2(z, r)
                beta = math.acos((self.ARM_SEGMENT_1**2 + d**2 - self.ARM_SEGMENT_2**2) / 
                                (2 * self.ARM_SEGMENT_1 * d))
                shoulder_rad = alpha + beta
                shoulder_deg = math.degrees(shoulder_rad)
            except ValueError:
                self.log(f"Inverse kinematics calculation error. Position may be out of reach.")
                return False
            
            # Check if angles are within valid ranges
            if not (0 <= theta <= 180 and 15 <= shoulder_deg <= 165 and 0 <= elbow_deg <= 180):
                self.log(f"Calculated angles out of range: Base={theta:.1f}, Shoulder={shoulder_deg:.1f}, Elbow={elbow_deg:.1f}")
                return False
            
            # Update sliders (without triggering callbacks)
            self.base_var.set(round(theta))
            self.shoulder_var.set(round(shoulder_deg))
            self.elbow_var.set(round(elbow_deg))
            
            # Calculate joint positions for visualization
            base_pos = [0, 0, 0]
            shoulder_pos = [0, 0, 0]
            
            # Elbow position
            elbow_x = self.ARM_SEGMENT_1 * math.cos(shoulder_rad) * math.cos(math.radians(theta))
            elbow_y = self.ARM_SEGMENT_1 * math.cos(shoulder_rad) * math.sin(math.radians(theta))
            elbow_z = self.ARM_SEGMENT_1 * math.sin(shoulder_rad)
            elbow_pos = [elbow_x, elbow_y, elbow_z]
            
            # End effector position should be the target position
            end_pos = [x, y, z]
            
            # Draw the arm
            self.draw_arm([base_pos, shoulder_pos, elbow_pos, end_pos])
            
            return True
        except Exception as e:
            self.log(f"Error calculating inverse kinematics: {str(e)}")
            return False
    
    def draw_arm(self, points):
        """Draw the robotic arm in the 3D plot"""
        # Clear the plot
        self.ax.clear()
        
        # Set axis labels and title
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_zlabel('Z (cm)')
        self.ax.set_title('Robotic Arm Position')
        
        # Set fixed axis limits
        self.ax.set_xlim(-25, 25)
        self.ax.set_ylim(-25, 25)
        self.ax.set_zlim(-5, 25)
        
        # Extract x, y, z coordinates
        x_coords = [p[0] for p in points]
        y_coords = [p[1] for p in points]
        z_coords = [p[2] for p in points]
        
        # Plot the arm segments as a line
        self.ax.plot(x_coords, y_coords, z_coords, 'b-', linewidth=3)
        
        # Plot the joints as points
        self.ax.plot([points[0][0]], [points[0][1]], [points[0][2]], 'ro', markersize=10, label='Base')  # Base
        self.ax.plot([points[2][0]], [points[2][1]], [points[2][2]], 'go', markersize=8, label='Elbow')  # Elbow
        self.ax.plot([points[3][0]], [points[3][1]], [points[3][2]], 'bo', markersize=8, label='End Effector')  # End effector
        
        # Add a grid
        self.ax.grid(True)
        
        # Add legend
        self.ax.legend()
        
        # Set plot background color based on settings
        self.fig.set_facecolor(self.bg_color_var.get())
        self.ax.set_facecolor(self.bg_color_var.get())
        
        # Update the canvas
        self.canvas.draw()
        
    def apply_visualization_settings(self):
        """Apply changes to visualization settings"""
        # Update the plot with new settings
        self.fig.set_facecolor(self.bg_color_var.get())
        self.ax.set_facecolor(self.bg_color_var.get())
        self.canvas.draw()
        self.log(f"Visualization settings updated")
    
    # Serial communication
    def serial_reader(self):
        """Background thread for reading serial data"""
        while True:
            if self.serial_conn is not None and hasattr(self.serial_conn, 'is_open') and self.serial_conn.is_open:
                try:
                    with self.serial_lock:  # Use lock for thread safety
                        if self.serial_conn.in_waiting > 0:
                            data = self.serial_conn.readline().decode('utf-8').strip()
                    
                    if data:
                        # Log the received data
                        self.log(f"Received: {data}")
                        
                        # Process special messages
                        if data.startswith("ANGLES,"):
                            parts = data.split(',')
                            if len(parts) == 4 and self.position_updates_enabled:
                                try:
                                    # Update angle sliders without triggering callbacks
                                    self.root.after(0, lambda: self.base_var.set(int(float(parts[1]))))
                                    self.root.after(0, lambda: self.shoulder_var.set(int(float(parts[2]))))
                                    self.root.after(0, lambda: self.elbow_var.set(int(float(parts[3]))))
                                    # Update preview in the main thread
                                    self.root.after(0, self.update_preview)
                                except ValueError:
                                    self.log("Error parsing angle data")
                        
                        elif data.startswith("POS,"):
                            parts = data.split(',')
                            if len(parts) == 4 and self.position_updates_enabled:
                                try:
                                    # Update position variables in the main thread
                                    x = float(parts[1])
                                    y = float(parts[2])
                                    z = float(parts[3])
                                    
                                    self.root.after(0, lambda: self.x_var.set(x))
                                    self.root.after(0, lambda: self.y_var.set(y))
                                    self.root.after(0, lambda: self.z_var.set(z))
                                    
                                    # Force update of the visualization in the main thread
                                    self.root.after(0, self.update_position_preview)
                                except ValueError:
                                    self.log("Error parsing position data")
                                    
                        elif data == "MOTION_START":
                            # Update status when motion starts
                            self.root.after(0, lambda: self.status_var.set("Motion in progress..."))
                            self.arm_status = "MOVING"
                            self.is_moving = True
                            
                        elif data == "MOTION_COMPLETE":
                            # Update status when motion completes
                            self.root.after(0, lambda: self.status_var.set("Motion complete"))
                            self.arm_status = "IDLE"
                            self.is_moving = False
                            
                            # Removed auto position request
                            # self.root.after(100, self.request_current_position)
                            
                            # If we're executing a trajectory, move to the next point
                            if self.trajectory_running and not self.trajectory_paused:
                                self.root.after(100, self.proceed_to_next_waypoint)
                            
                            # If we're doing a recording playback
                            if self.trajectory_running and self.playback_index < len(self.record_positions):
                                self.root.after(100, self.play_next_recording)
                        
                        elif data == "EMERGENCY_STOP":
                            self.root.after(0, lambda: self.status_var.set("EMERGENCY STOP"))
                            self.arm_status = "EMERGENCY_STOP"
                            self.is_moving = False
                            self.trajectory_running = False
                            
                        elif data == "RESUMED":
                            self.root.after(0, lambda: self.status_var.set("Resumed"))
                            self.arm_status = "IDLE"
                            
                        elif data.startswith("ERROR:"):
                            # Handle error messages
                            error_msg = data[7:]  # Remove "ERROR: " prefix
                            self.log(f"Error from ESP32: {error_msg}")
                            # Show error in status
                            self.root.after(0, lambda: self.status_var.set(f"Error: {error_msg}"))
                            
                        elif data.startswith("STATUS,"):
                            # Process status information
                            parts = data.split(',')
                            if len(parts) >= 3:
                                movement_status = parts[1]
                                safety_status = parts[2]
                                
                                if movement_status == "MOVING":
                                    self.is_moving = True
                                else:
                                    self.is_moving = False
                                    
                                self.arm_status = safety_status
                except Exception as e:
                    self.log(f"Error reading serial: {str(e)}")
            time.sleep(0.05)  # Shorter sleep for more responsive UI
    
    # Calibration methods
    def test_calibration(self, servo, angle):
        """Test a servo at a specific angle with the current calibration"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.log("Not connected to arm")
            messagebox.showwarning("Not Connected", "Please connect to the arm first.")
            return
            
        try:
            with self.serial_lock:  # Use lock for thread safety
                # Send calibration command first
                if servo == "BASE":
                    command = f"CALIBRATE,BASE,{self.base_offset_var.get()}\n"
                elif servo == "SHOULDER":
                    command = f"CALIBRATE,SHOULDER,{self.shoulder_offset_var.get()}\n"
                elif servo == "ELBOW":
                    command = f"CALIBRATE,ELBOW,{self.elbow_offset_var.get()}\n"
                    
                self.serial_conn.write(command.encode())
                time.sleep(0.1)
                
                # Send move command based on servo
                if servo == "BASE":
                    self.serial_conn.write(f"S,{angle},90,90\n".encode())
                    self.log(f"Testing base servo at {angle}° with offset {self.base_offset_var.get()}")
                elif servo == "SHOULDER":
                    self.serial_conn.write(f"S,90,{angle},90\n".encode())
                    self.log(f"Testing shoulder servo at {angle}° with offset {self.shoulder_offset_var.get()}")
                elif servo == "ELBOW":
                    self.serial_conn.write(f"S,90,90,{angle}\n".encode())
                    self.log(f"Testing elbow servo at {angle}° with offset {self.elbow_offset_var.get()}")
        except Exception as e:
            self.log(f"Error testing calibration: {str(e)}")
    
    def apply_calibration(self):
        """Apply calibration settings to the arm"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.log("Not connected to arm")
            messagebox.showwarning("Not Connected", "Please connect to the arm first.")
            return
            
        try:
            with self.serial_lock:  # Use lock for thread safety
                # Send calibration commands
                base_command = f"CALIBRATE,BASE,{self.base_offset_var.get()}\n"
                shoulder_command = f"CALIBRATE,SHOULDER,{self.shoulder_offset_var.get()}\n"
                elbow_command = f"CALIBRATE,ELBOW,{self.elbow_offset_var.get()}\n"
                
                self.serial_conn.write(base_command.encode())
                time.sleep(0.1)
                self.serial_conn.write(shoulder_command.encode())
                time.sleep(0.1)
                self.serial_conn.write(elbow_command.encode())
            
            self.log("Calibration settings applied")
        except Exception as e:
            self.log(f"Error applying calibration: {str(e)}")
    
    def save_calibration_to_esp32(self):
        """Save the current calibration to ESP32 EEPROM"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.log("Not connected to arm")
            messagebox.showwarning("Not Connected", "Please connect to the arm first.")
            return
            
        try:
            # Apply calibration first
            self.apply_calibration()
            time.sleep(0.1)
            
            with self.serial_lock:  # Use lock for thread safety
                # Send save command
                self.serial_conn.write("SAVE_CALIBRATION\n".encode())
            
            self.log("Saving calibration to ESP32 EEPROM")
            messagebox.showinfo("Calibration", "Calibration settings saved to ESP32 EEPROM.")
        except Exception as e:
            self.log(f"Error saving calibration: {str(e)}")
    
    # Settings methods
    def save_settings(self):
        """Save application settings to file"""
        try:
            # Update arm dimensions
            self.ARM_SEGMENT_1 = self.arm_segment1_var.get()
            self.ARM_SEGMENT_2 = self.arm_segment2_var.get()
            
            # Create settings dictionary
            settings = {
                "arm_segment1": self.ARM_SEGMENT_1,
                "arm_segment2": self.ARM_SEGMENT_2,
                "bg_color": self.bg_color_var.get(),
                "baud_rate": self.baud_var.get(),
                "timeout": self.timeout_var.get(),
                "base_offset": self.base_offset_var.get(),
                "shoulder_offset": self.shoulder_offset_var.get(),
                "elbow_offset": self.elbow_offset_var.get(),
                "ollama_url": self.ollama_url_var.get(),
                "model_name": self.model_name_var.get(),
                "webcam_device": self.webcam_device_var.get()
            }
            
            # Save to file
            with open("arm_settings.json", "w") as f:
                json.dump(settings, f, indent=4)
            
            self.log("Settings saved to arm_settings.json")
            messagebox.showinfo("Settings", "Settings saved successfully.")
            
            # Update visualization with new settings
            self.update_preview()
        except Exception as e:
            self.log(f"Error saving settings: {str(e)}")
            messagebox.showerror("Settings Error", f"Failed to save settings.\n{str(e)}")
    
    def load_settings(self):
        """Load application settings from file"""
        try:
            if os.path.exists("arm_settings.json"):
                with open("arm_settings.json", "r") as f:
                    settings = json.load(f)
                
                # Apply loaded settings
                if "arm_segment1" in settings:
                    self.ARM_SEGMENT_1 = settings["arm_segment1"]
                    self.arm_segment1_var.set(self.ARM_SEGMENT_1)
                
                if "arm_segment2" in settings:
                    self.ARM_SEGMENT_2 = settings["arm_segment2"]
                    self.arm_segment2_var.set(self.ARM_SEGMENT_2)
                
                if "bg_color" in settings:
                    self.bg_color_var.set(settings["bg_color"])
                
                if "baud_rate" in settings:
                    self.baud_var.set(settings["baud_rate"])
                
                if "timeout" in settings:
                    self.timeout_var.set(settings["timeout"])
                
                if "base_offset" in settings:
                    self.base_offset_var.set(settings["base_offset"])
                
                if "shoulder_offset" in settings:
                    self.shoulder_offset_var.set(settings["shoulder_offset"])
                
                if "elbow_offset" in settings:
                    self.elbow_offset_var.set(settings["elbow_offset"])
                    
                if "ollama_url" in settings:
                    self.ollama_url_var.set(settings["ollama_url"])
                    
                if "model_name" in settings:
                    self.model_name_var.set(settings["model_name"])
                    
                if "webcam_device" in settings:
                    self.webcam_device_var.set(settings["webcam_device"])
                
                self.log("Settings loaded from arm_settings.json")
                
                # Update visualization with loaded settings
                self.apply_visualization_settings()
        except Exception as e:
            self.log(f"Error loading settings: {str(e)}")
    
    # Trajectory planning methods
    def add_waypoint(self):
        """Add current position as a waypoint"""
        # Get current position
        x = self.x_var.get()
        y = self.y_var.get()
        z = self.z_var.get()
        
        # Add to waypoints list
        self.waypoints.append((x, y, z))
        
        # Update listbox
        self.waypoint_listbox.insert(tk.END, f"Point {len(self.waypoints)}: ({x:.2f}, {y:.2f}, {z:.2f})")
        self.log(f"Added waypoint: ({x:.2f}, {y:.2f}, {z:.2f})")
    
    def remove_waypoint(self):
        """Remove selected waypoint"""
        try:
            selected = self.waypoint_listbox.curselection()[0]
            self.waypoint_listbox.delete(selected)
            self.waypoints.pop(selected)
            
            # Rename all waypoints after deletion
            self.waypoint_listbox.delete(0, tk.END)
            for i, (x, y, z) in enumerate(self.waypoints):
                self.waypoint_listbox.insert(tk.END, f"Point {i+1}: ({x:.2f}, {y:.2f}, {z:.2f})")
                
            self.log(f"Removed waypoint {selected+1}")
        except IndexError:
            self.log("No waypoint selected")
            messagebox.showwarning("Selection Error", "Please select a waypoint to remove.")
    
    def clear_waypoints(self):
        """Clear all waypoints"""
        if messagebox.askyesno("Clear Waypoints", "Are you sure you want to clear all waypoints?"):
            self.waypoints = []
            self.waypoint_listbox.delete(0, tk.END)
            self.log("Cleared all waypoints")
    
    def move_waypoint_up(self):
        """Move selected waypoint up in the list"""
        try:
            selected = self.waypoint_listbox.curselection()[0]
            if selected > 0:
                # Swap waypoints
                self.waypoints[selected], self.waypoints[selected-1] = self.waypoints[selected-1], self.waypoints[selected]
                
                # Update listbox
                self.waypoint_listbox.delete(0, tk.END)
                for i, (x, y, z) in enumerate(self.waypoints):
                    self.waypoint_listbox.insert(tk.END, f"Point {i+1}: ({x:.2f}, {y:.2f}, {z:.2f})")
                
                # Maintain selection
                self.waypoint_listbox.selection_set(selected-1)
                self.log(f"Moved waypoint {selected+1} up")
        except IndexError:
            self.log("No waypoint selected")
            messagebox.showwarning("Selection Error", "Please select a waypoint to move.")
    
    def move_waypoint_down(self):
        """Move selected waypoint down in the list"""
        try:
            selected = self.waypoint_listbox.curselection()[0]
            if selected < len(self.waypoints) - 1:
                # Swap waypoints
                self.waypoints[selected], self.waypoints[selected+1] = self.waypoints[selected+1], self.waypoints[selected]
                
                # Update listbox
                self.waypoint_listbox.delete(0, tk.END)
                for i, (x, y, z) in enumerate(self.waypoints):
                    self.waypoint_listbox.insert(tk.END, f"Point {i+1}: ({x:.2f}, {y:.2f}, {z:.2f})")
                
                # Maintain selection
                self.waypoint_listbox.selection_set(selected+1)
                self.log(f"Moved waypoint {selected+1} down")
        except IndexError:
            self.log("No waypoint selected")
            messagebox.showwarning("Selection Error", "Please select a waypoint to move.")
    
    def edit_waypoint(self):
        """Edit the selected waypoint"""
        try:
            selected = self.waypoint_listbox.curselection()[0]
            
            # Create a dialog for editing
            edit_window = tk.Toplevel(self.root)
            edit_window.title(f"Edit Waypoint {selected+1}")
            edit_window.geometry("300x200")
            edit_window.resizable(False, False)
            edit_window.transient(self.root)
            edit_window.grab_set()
            
            # Get current values
            x, y, z = self.waypoints[selected]
            
            # Create entry fields
            ttk.Label(edit_window, text="X (cm):").grid(row=0, column=0, padx=10, pady=10, sticky="w")
            x_var = tk.DoubleVar(value=x)
            x_entry = ttk.Entry(edit_window, textvariable=x_var, width=10)
            x_entry.grid(row=0, column=1, padx=10, pady=10)
            
            ttk.Label(edit_window, text="Y (cm):").grid(row=1, column=0, padx=10, pady=10, sticky="w")
            y_var = tk.DoubleVar(value=y)
            y_entry = ttk.Entry(edit_window, textvariable=y_var, width=10)
            y_entry.grid(row=1, column=1, padx=10, pady=10)
            
            ttk.Label(edit_window, text="Z (cm):").grid(row=2, column=0, padx=10, pady=10, sticky="w")
            z_var = tk.DoubleVar(value=z)
            z_entry = ttk.Entry(edit_window, textvariable=z_var, width=10)
            z_entry.grid(row=2, column=1, padx=10, pady=10)
            
            # Function to save changes
            def save_changes():
                new_x = x_var.get()
                new_y = y_var.get()
                new_z = z_var.get()
                
                # Update waypoint
                self.waypoints[selected] = (new_x, new_y, new_z)
                
                # Update listbox
                self.waypoint_listbox.delete(selected)
                self.waypoint_listbox.insert(selected, f"Point {selected+1}: ({new_x:.2f}, {new_y:.2f}, {new_z:.2f})")
                self.waypoint_listbox.selection_set(selected)
                
                self.log(f"Updated waypoint {selected+1} to ({new_x:.2f}, {new_y:.2f}, {new_z:.2f})")
                edit_window.destroy()
            
            # Buttons
            button_frame = ttk.Frame(edit_window)
            button_frame.grid(row=3, column=0, columnspan=2, pady=15)
            
            ttk.Button(button_frame, text="Save", command=save_changes).pack(side=tk.LEFT, padx=10)
            ttk.Button(button_frame, text="Cancel", command=edit_window.destroy).pack(side=tk.LEFT, padx=10)
            
        except IndexError:
            self.log("No waypoint selected")
            messagebox.showwarning("Selection Error", "Please select a waypoint to edit.")
    
    def validate_trajectory(self):
        """Validate the trajectory by checking if all waypoints are reachable"""
        if not self.waypoints:
            self.log("No waypoints defined")
            messagebox.showwarning("Validation", "No waypoints defined.")
            return
            
        self.log("Validating trajectory...")
        all_valid = True
        invalid_points = []
        
        for i, (x, y, z) in enumerate(self.waypoints):
            # Check if position is reachable (simple check)
            d = math.sqrt(x*x + y*y + z*z)
            if d > (self.ARM_SEGMENT_1 + self.ARM_SEGMENT_2) or d < abs(self.ARM_SEGMENT_1 - self.ARM_SEGMENT_2):
                all_valid = False
                invalid_points.append(i+1)
                self.log(f"Waypoint {i+1} is out of reach: ({x:.2f}, {y:.2f}, {z:.2f})")
                continue
                
            # Calculate base angle
            theta = math.degrees(math.atan2(y, x))
            if theta < 0 or theta > 180:
                all_valid = False
                invalid_points.append(i+1)
                self.log(f"Waypoint {i+1} has invalid base angle {theta:.1f}°: ({x:.2f}, {y:.2f}, {z:.2f})")
                continue
                
            # Calculate other angles to check joint limits
            r = math.sqrt(x*x + y*y)
            try:
                # Calculate elbow angle
                elbow_rad = math.acos((self.ARM_SEGMENT_1**2 + self.ARM_SEGMENT_2**2 - d**2) / 
                                     (2 * self.ARM_SEGMENT_1 * self.ARM_SEGMENT_2))
                elbow_deg = 180.0 - math.degrees(elbow_rad)
                
                # Calculate shoulder angle
                alpha = math.atan2(z, r)
                beta = math.acos((self.ARM_SEGMENT_1**2 + d**2 - self.ARM_SEGMENT_2**2) / 
                                (2 * self.ARM_SEGMENT_1 * d))
                shoulder_deg = math.degrees(alpha + beta)
                
                # Check servo limits
                if not (15 <= shoulder_deg <= 165):
                    all_valid = False
                    invalid_points.append(i+1)
                    self.log(f"Waypoint {i+1} has invalid shoulder angle {shoulder_deg:.1f}°: ({x:.2f}, {y:.2f}, {z:.2f})")
                    continue
                    
                if not (0 <= elbow_deg <= 180):
                    all_valid = False
                    invalid_points.append(i+1)
                    self.log(f"Waypoint {i+1} has invalid elbow angle {elbow_deg:.1f}°: ({x:.2f}, {y:.2f}, {z:.2f})")
                    continue
                    
            except ValueError:
                all_valid = False
                invalid_points.append(i+1)
                self.log(f"Waypoint {i+1} has invalid geometry: ({x:.2f}, {y:.2f}, {z:.2f})")
        
        if all_valid:
            self.log("Trajectory validation successful - all waypoints are reachable")
            messagebox.showinfo("Validation", "All waypoints are valid and reachable!")
        else:
            self.log(f"Trajectory validation failed - {len(invalid_points)} invalid waypoints")
            messagebox.showwarning("Validation", 
                                  f"Found {len(invalid_points)} invalid waypoints: {', '.join(map(str, invalid_points))}")
    
    def save_trajectory(self):
        """Save trajectory to file"""
        if not self.waypoints:
            self.log("No waypoints defined")
            messagebox.showwarning("Save Error", "No waypoints defined.")
            return
            
        try:
            file_path = filedialog.asksaveasfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Save Trajectory"
            )
            
            if not file_path:
                return  # User cancelled
                
            trajectory_data = {
                "waypoints": self.waypoints,
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
                "count": len(self.waypoints)
            }
            
            with open(file_path, "w") as f:
                json.dump(trajectory_data, f, indent=4)
                
            self.log(f"Trajectory saved to {file_path}")
        except Exception as e:
            self.log(f"Error saving trajectory: {str(e)}")
            messagebox.showerror("Save Error", f"Failed to save trajectory.\n{str(e)}")
    
    def load_trajectory(self):
        """Load trajectory from file"""
        try:
            file_path = filedialog.askopenfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Load Trajectory"
            )
            
            if not file_path:
                return  # User cancelled
                
            with open(file_path, "r") as f:
                trajectory_data = json.load(f)
                
            if "waypoints" in trajectory_data:
                # Clear current waypoints
                self.waypoints = []
                self.waypoint_listbox.delete(0, tk.END)
                
                # Add loaded waypoints
                for x, y, z in trajectory_data["waypoints"]:
                    self.waypoints.append((x, y, z))
                    self.waypoint_listbox.insert(tk.END, f"Point {len(self.waypoints)}: ({x:.2f}, {y:.2f}, {z:.2f})")
                
                self.log(f"Loaded {len(self.waypoints)} waypoints from {file_path}")
            else:
                self.log("Invalid trajectory file format")
                messagebox.showerror("Load Error", "Invalid trajectory file format.")
        except Exception as e:
            self.log(f"Error loading trajectory: {str(e)}")
            messagebox.showerror("Load Error", f"Failed to load trajectory.\n{str(e)}")
    
    def execute_trajectory(self):
        """Execute the stored trajectory"""
        if not self.waypoints:
            self.log("No waypoints defined")
            messagebox.showwarning("Execution Error", "No waypoints defined.")
            return
            
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.log("Not connected to arm")
            messagebox.showwarning("Connection Error", "Please connect to the arm first.")
            return
            
        if self.trajectory_running:
            self.log("Trajectory already running")
            return
            
        try:
            # Start executing the trajectory
            self.trajectory_running = True
            self.trajectory_paused = False
            self.current_waypoint = 0
            
            # Reset progress bar
            self.progress_var.set(0)
            self.progress_label_var.set(f"0/{len(self.waypoints)}")
            
            # First, send speed command
            with self.serial_lock:  # Use lock for thread safety
                speed_command = f"SPEED,{self.trajectory_speed_var.get()}\n"
                self.serial_conn.write(speed_command.encode())
            time.sleep(0.1)
            
            self.log(f"Starting trajectory execution with {len(self.waypoints)} waypoints at speed {self.trajectory_speed_var.get()}")
            
            # Move to the first waypoint
            self.move_to_next_waypoint()
        except Exception as e:
            self.log(f"Error starting trajectory: {str(e)}")
            self.trajectory_running = False
    
    def move_to_next_waypoint(self):
        """Move to the next waypoint in the trajectory"""
        if not self.trajectory_running or self.trajectory_paused:
            return
            
        if self.current_waypoint >= len(self.waypoints):
            # Trajectory complete
            if self.loop_var.get() and self.current_waypoint < self.loop_count_var.get() * len(self.waypoints):
                # Start next loop
                self.current_waypoint = 0
                self.log(f"Starting next loop of trajectory")
                self.move_to_next_waypoint()
            else:
                # All done
                self.trajectory_running = False
                self.log("Trajectory execution complete")
                self.status_var.set("Trajectory complete")
                # Reset progress
                self.progress_var.set(100)
                self.progress_label_var.set(f"{len(self.waypoints)}/{len(self.waypoints)}")
            return
            
        # Get the next waypoint
        x, y, z = self.waypoints[self.current_waypoint]
        
        # Update display
        self.log(f"Moving to waypoint {self.current_waypoint+1}: ({x:.2f}, {y:.2f}, {z:.2f})")
        
        # Update GUI values
        self.x_var.set(x)
        self.y_var.set(y)
        self.z_var.set(z)
        self.update_position_preview()
        
        # Update progress bar
        progress_percent = (self.current_waypoint / len(self.waypoints)) * 100
        self.progress_var.set(progress_percent)
        self.progress_label_var.set(f"{self.current_waypoint+1}/{len(self.waypoints)}")
        
        # Highlight current waypoint in listbox
        self.waypoint_listbox.selection_clear(0, tk.END)
        self.waypoint_listbox.selection_set(self.current_waypoint)
        self.waypoint_listbox.see(self.current_waypoint)
        
        # Send the command
        try:
            with self.serial_lock:  # Use lock for thread safety
                # Send position command
                command = f"P,{x:.2f},{y:.2f},{z:.2f}\n"
                self.serial_conn.write(command.encode())
            
            # The proceed_to_next_waypoint method will be called when the MOTION_COMPLETE message is received
        except Exception as e:
            self.log(f"Error in trajectory execution: {str(e)}")
            self.trajectory_running = False
    
    def proceed_to_next_waypoint(self):
        """Proceed to the next waypoint after current motion completes"""
        if self.trajectory_running and not self.trajectory_paused:
            self.current_waypoint += 1
            
            # Use a delay between waypoints if configured
            if self.pause_var.get() > 0:
                self.log(f"Pausing for {self.pause_var.get():.1f} seconds between waypoints")
                self.root.after(int(self.pause_var.get() * 1000), self.move_to_next_waypoint)
            else:
                # Move to next point immediately
                self.move_to_next_waypoint()
    
    def toggle_trajectory_pause(self):
        """Pause or resume trajectory execution"""
        if not self.trajectory_running:
            return
            
        if self.trajectory_paused:
            # Resume
            self.trajectory_paused = False
            self.log("Trajectory execution resumed")
            self.pause_btn.config(text="Pause")
            
            # Continue with next waypoint
            self.move_to_next_waypoint()
        else:
            # Pause
            self.trajectory_paused = True
            self.log("Trajectory execution paused")
            self.pause_btn.config(text="Resume")
    
    def stop_trajectory(self):
        """Stop trajectory execution"""
        if self.trajectory_running:
            self.trajectory_running = False
            self.trajectory_paused = False
            self.log("Trajectory execution stopped")
            self.pause_btn.config(text="Pause/Resume")
            self.status_var.set("Trajectory stopped")
            
            # Reset progress bar
            self.progress_var.set(0)
            self.progress_label_var.set(f"0/{len(self.waypoints)}")
    
    # Recording and playback methods
    def toggle_recording(self):
        """Start or stop recording arm movements"""
        if not self.recording:
            # Start recording
            self.recording = True
            self.record_positions = []
            self.record_btn.config(text="Stop Recording")
            self.log("Started recording arm movements")
        else:
            # Stop recording
            self.recording = False
            self.record_btn.config(text="Start Recording")
            self.log(f"Stopped recording - captured {len(self.record_positions)} movements")
    
    def play_recording(self):
        """Play back recorded arm movements"""
        if not self.record_positions:
            self.log("No recorded movements to play")
            messagebox.showwarning("Playback Error", "No recorded movements to play.")
            return
            
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.log("Not connected to arm")
            messagebox.showwarning("Connection Error", "Please connect to the arm first.")
            return
            
        if self.trajectory_running:
            self.log("Trajectory already running")
            return
            
        try:
            # Start playback
            self.trajectory_running = True
            self.playback_index = 0
            
            # Reset progress bar
            self.progress_var.set(0)
            self.progress_label_var.set(f"0/{len(self.record_positions)}")
            
            self.log(f"Starting playback of {len(self.record_positions)} recorded movements")
            
            # Start the playback
            self.play_next_recording()
        except Exception as e:
            self.log(f"Error starting playback: {str(e)}")
            self.trajectory_running = False
    
    def play_next_recording(self):
        """Play the next recorded movement"""
        if not self.trajectory_running or self.playback_index >= len(self.record_positions):
            self.trajectory_running = False
            self.log("Playback complete")
            self.progress_var.set(100)
            self.progress_label_var.set(f"{len(self.record_positions)}/{len(self.record_positions)}")
            return
            
        # Get the next movement
        movement = self.record_positions[self.playback_index]
        
        # Update progress bar
        progress_percent = (self.playback_index / len(self.record_positions)) * 100
        self.progress_var.set(progress_percent)
        self.progress_label_var.set(f"{self.playback_index+1}/{len(self.record_positions)}")
        
        try:
            with self.serial_lock:  # Use lock for thread safety
                if movement[0] == 'S':
                    # Servo movement
                    base, shoulder, elbow = movement[1], movement[2], movement[3]
                    
                    # Update GUI
                    self.base_var.set(base)
                    self.shoulder_var.set(shoulder)
                    self.elbow_var.set(elbow)
                    self.update_preview()
                    
                    # Send command
                    command = f"S,{base},{shoulder},{elbow}\n"
                    self.serial_conn.write(command.encode())
                    self.log(f"Playback: Servo movement to ({base}, {shoulder}, {elbow})")
                    
                elif movement[0] == 'P':
                    # Position movement
                    x, y, z = movement[1], movement[2], movement[3]
                    
                    # Update GUI
                    self.x_var.set(x)
                    self.y_var.set(y)
                    self.z_var.set(z)
                    self.update_position_preview()
                    
                    # Send command
                    command = f"P,{x:.2f},{y:.2f},{z:.2f}\n"
                    self.serial_conn.write(command.encode())
                    self.log(f"Playback: Position movement to ({x:.2f}, {y:.2f}, {z:.2f})")
                    
                elif movement[0] == 'HOME':
                    # Home position
                    self.serial_conn.write("HOME\n".encode())
                    self.log("Playback: Moving to home position")
                    
                    # Update GUI
                    self.base_var.set(90)
                    self.shoulder_var.set(90)
                    self.elbow_var.set(90)
                    self.update_preview()
            
            # The next movement will be triggered by MOTION_COMPLETE
            self.playback_index += 1
        except Exception as e:
            self.log(f"Error in playback: {str(e)}")
            self.trajectory_running = False
    
    def save_recording(self):
        """Save recorded movements to file"""
        if not self.record_positions:
            self.log("No recorded movements to save")
            messagebox.showwarning("Save Error", "No recorded movements to save.")
            return
        
        try:
            file_path = filedialog.asksaveasfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Save Recording"
            )
            
            if not file_path:
                return  # User cancelled
            
            # Convert record positions to a serializable format
            recording_data = {
                "movements": self.record_positions,
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
                "count": len(self.record_positions)
            }
            
            with open(file_path, "w") as f:
                json.dump(recording_data, f, indent=4)
                
            self.log(f"Recording saved to {file_path}")
            messagebox.showinfo("Save Recording", f"Recording saved with {len(self.record_positions)} movements.")
        except Exception as e:
            self.log(f"Error saving recording: {str(e)}")
            messagebox.showerror("Save Error", f"Failed to save recording.\n{str(e)}")
    
    def load_recording(self):
        """Load recorded movements from file"""
        try:
            file_path = filedialog.askopenfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Load Recording"
            )
            
            if not file_path:
                return  # User cancelled
            
            with open(file_path, "r") as f:
                recording_data = json.load(f)
            
            if "movements" in recording_data:
                self.record_positions = recording_data["movements"]
                self.log(f"Loaded recording with {len(self.record_positions)} movements from {file_path}")
                messagebox.showinfo("Load Recording", f"Recording loaded with {len(self.record_positions)} movements.")
            else:
                self.log("Invalid recording file format")
                messagebox.showerror("Load Error", "Invalid recording file format.")
        except Exception as e:
            self.log(f"Error loading recording: {str(e)}")
            messagebox.showerror("Load Error", f"Failed to load recording.\n{str(e)}")

    def browse_urdf(self):
        """Open file dialog to select a URDF model."""
        file_path = filedialog.askopenfilename(
            title="Select URDF File",
            filetypes=[("URDF Files", "*.urdf"), ("All Files", "*.*")]
        )
        
        if file_path:
            # Update path display
            self.urdf_path_var.set(os.path.basename(file_path))
            
            # Load the URDF model
            success = self.urdf_viz.load_urdf(file_path)
            
            if success:
                # If successful, set up joint mapping UI
                self.setup_joint_mapping()
                
                # Log in main console
                self.log(f"Loaded URDF model: {os.path.basename(file_path)}")
            else:
                self.log(f"Failed to load URDF model: {os.path.basename(file_path)}")
    
    def setup_joint_mapping(self):
        """Set up UI for mapping controller joints to URDF model joints."""
        # Clear existing widgets
        for widget in self.joint_mapping_frame.winfo_children():
            widget.destroy()
        
        # Get URDF joints
        urdf_joints = self.urdf_viz.get_joint_info()
        
        if not urdf_joints:
            ttk.Label(
                self.joint_mapping_frame, 
                text="No movable joints found in the URDF model.",
                wraplength=500
            ).pack(pady=10)
            return
        
        # Create joint mapping UI
        ttk.Label(self.joint_mapping_frame, text="Controller Joint").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Label(self.joint_mapping_frame, text="URDF Joint").grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        # Create mapping dictionary if it doesn't exist
        if not hasattr(self, 'joint_mapping'):
            self.joint_mapping = {
                'base': None,
                'shoulder': None,
                'elbow': None
            }
        
        # Create dropdown for base joint
        ttk.Label(self.joint_mapping_frame, text="Base:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.base_joint_var = tk.StringVar()
        base_combo = ttk.Combobox(
            self.joint_mapping_frame, 
            textvariable=self.base_joint_var,
            values=[joint['joint_name'] for joint in urdf_joints] + ["None"]
        )
        base_combo.grid(row=1, column=1, padx=5, pady=5, sticky="w")
        base_combo.bind("<<ComboboxSelected>>", lambda e: self.update_joint_mapping('base', self.base_joint_var.get()))
        
        # Create dropdown for shoulder joint
        ttk.Label(self.joint_mapping_frame, text="Shoulder:").grid(row=2, column=0, padx=5, pady=5, sticky="w")
        self.shoulder_joint_var = tk.StringVar()
        shoulder_combo = ttk.Combobox(
            self.joint_mapping_frame, 
            textvariable=self.shoulder_joint_var,
            values=[joint['joint_name'] for joint in urdf_joints] + ["None"]
        )
        shoulder_combo.grid(row=2, column=1, padx=5, pady=5, sticky="w")
        shoulder_combo.bind("<<ComboboxSelected>>", lambda e: self.update_joint_mapping('shoulder', self.shoulder_joint_var.get()))
        
        # Create dropdown for elbow joint
        ttk.Label(self.joint_mapping_frame, text="Elbow:").grid(row=3, column=0, padx=5, pady=5, sticky="w")
        self.elbow_joint_var = tk.StringVar()
        elbow_combo = ttk.Combobox(
            self.joint_mapping_frame, 
            textvariable=self.elbow_joint_var,
            values=[joint['joint_name'] for joint in urdf_joints] + ["None"]
        )
        elbow_combo.grid(row=3, column=1, padx=5, pady=5, sticky="w")
        elbow_combo.bind("<<ComboboxSelected>>", lambda e: self.update_joint_mapping('elbow', self.elbow_joint_var.get()))
        
        # Try to automatically map joints by name
        self.auto_map_joints(urdf_joints)
    
    def auto_map_joints(self, urdf_joints):
        """Try to automatically map joints based on names."""
        for joint in urdf_joints:
            name = joint['joint_name'].lower()
            
            if 'base' in name or 'rotate' in name or 'rotation' in name:
                self.base_joint_var.set(joint['joint_name'])
                self.update_joint_mapping('base', joint['joint_name'])
                
            elif 'shoulder' in name:
                self.shoulder_joint_var.set(joint['joint_name'])
                self.update_joint_mapping('shoulder', joint['joint_name'])
                
            elif 'elbow' in name:
                self.elbow_joint_var.set(joint['joint_name'])
                self.update_joint_mapping('elbow', joint['joint_name'])
    
    def update_joint_mapping(self, controller_joint, urdf_joint):
        """Update the mapping between controller joints and URDF joints."""
        if urdf_joint == "None":
            self.joint_mapping[controller_joint] = None
            self.log(f"Unmapped controller {controller_joint} joint")
            return
        
        # Get the joint index
        urdf_joints = self.urdf_viz.get_joint_info()
        for joint in urdf_joints:
            if joint['joint_name'] == urdf_joint:
                self.joint_mapping[controller_joint] = joint['joint_index']
                self.log(f"Mapped controller {controller_joint} joint to URDF joint {urdf_joint}")
                break
    
    def sync_urdf_with_controller(self):
        """Synchronize the URDF model with the current controller joint angles."""
        if not hasattr(self, 'joint_mapping'):
            self.log("No joint mapping defined.")
            return
        
        # Get current controller joint angles
        base = self.base_var.get() if hasattr(self, 'base_var') else 90
        shoulder = self.shoulder_var.get() if hasattr(self, 'shoulder_var') else 90
        elbow = self.elbow_var.get() if hasattr(self, 'elbow_var') else 90
        
        # Convert to radians for PyBullet
        base_rad = math.radians(base)
        shoulder_rad = math.radians(shoulder)
        elbow_rad = math.radians(elbow)
        
        # Create joint positions dictionary
        joint_positions = {}
        
        if self.joint_mapping['base'] is not None:
            joint_positions[self.joint_mapping['base']] = base_rad
        
        if self.joint_mapping['shoulder'] is not None:
            joint_positions[self.joint_mapping['shoulder']] = shoulder_rad
        
        if self.joint_mapping['elbow'] is not None:
            joint_positions[self.joint_mapping['elbow']] = elbow_rad
        
        # Update the URDF model
        self.urdf_viz.update_joint_positions(joint_positions)
        self.log("Synchronized URDF model with controller positions")
    
    def on_closing(self):
        """Handle window closing event."""
        # Clean up webcam if active
        if self.webcam_active:
            self.webcam_active = False
            if self.cap is not None:
                self.cap.release()
                self.cap = None
        
        # Clean up URDF visualization
        if hasattr(self, 'urdf_viz'):
            self.urdf_viz.cleanup()
        
        # Close serial connection if open
        if self.serial_conn is not None and hasattr(self.serial_conn, 'is_open') and self.serial_conn.is_open:
            try:
                with self.serial_lock:
                    self.serial_conn.close()
            except:
                pass
        
        # Delete temporary files
        if os.path.exists("temp_capture.jpg"):
            try:
                os.remove("temp_capture.jpg")
            except:
                pass
        
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = RoboticArmGUI(root)
    root.mainloop()