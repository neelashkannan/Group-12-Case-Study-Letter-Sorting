# Robotic Arm Envelope Sorter

An automated mail sorting system that uses computer vision and a 5DoF robotic arm to read recipient names on envelopes and sort them into designated bins.

---

## 📋 Overview

This project combines robotics, computer vision, and AI to create a smart mail sorting workflow. It captures envelope images with a webcam, uses the LLaVA-Phi3 vision model (via Ollama) to identify recipient names, and commands a 5DoF robotic arm (controlled by an ESP32) to place each envelope into the appropriate bin based on the first letter of the recipient’s name.



---

## ✨ Key Features

- **Intelligent Envelope Recognition:** Uses LLaVA-Phi3 via Ollama to read recipient names.
- **3‑Axis Cartesian & Joint Control:** Direct servo-angle or XYZ coordinate commands.
- **Trajectory Planning:** Create, save, load, validate, and execute complex waypoints per letter.
- **Real‑time 3D Visualization:** Preview arm movements in PyBullet before execution.
- **Automatic Sorting Logic:** Maps first letter of names to pre‑defined trajectories.
- **Calibration Tools:** Servo tuning for precise movement.
- **Recording & Playback:** Record sequences and replay for testing.

---

## 🛠️ Hardware Components

- 5DoF Robotic Arm with standard hobby servos
- Seeeduino Xiao ESP32 microcontroller
- PCA9685 PWM servo driver board
- USB webcam (compatible with `usb_cam` driver)
- Computer with USB ports
- Sorting bins or designated physical trays

---

## 💻 Software Requirements

- **Operating System:** Linux (Ubuntu 22.04 LTS recommended)
- **ROS 2:** Humble Hawksbill
- **Python:** 3.7+

**Python Packages:**

```bash
pip install opencv-python numpy matplotlib pillow requests pyserial pybullet
```

**Additional Tools:**

- Arduino IDE (for ESP32 firmware)
- Ollama (with LLaVA-Phi3 model)
- Colcon build tools (`colcon-common-extensions`)

---

## 📦 Installation & Setup

### 1. ROS 2 Workspace Setup

1. Create and initialize your workspace:

   ```bash
   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
   git clone <your-repo-url>  # clone this package here
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

2. Ensure dependencies are installed:

   ```bash
   sudo apt update
   sudo apt install ros-humble-ros-base ros-humble-robot-state-publisher ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-usb-cam
   ```

### 2. Ollama & LLaVA-Phi3

1. Install Ollama:
   ```bash
   curl https://ollama.ai/install.sh | sh
   ```
2. Pull and run the vision model:
   ```bash
   ollama pull llava-phi3
   ollama serve
   ```

### 3. ESP32 Firmware Configuration

1. Open the Arduino IDE, set board to **Seeeduino Xiao ESP32**, and set baud rate to **115200**.
2. Install these libraries via Library Manager:
   - `Adafruit PWM Servo Driver`
   - `Wire` (built‑in)
3. Replace your `processCommand` in `main.ino` with the updated parser below to support Python app commands:
   ```cpp
   void processCommand(String command) {
     command.trim();
     if (command.startsWith("S,")) {
       // Format: "S,base,shoulder,elbow"
       int a[3];
       int idx = 0;
       char* tok = strtok((char*)command.c_str(), ",");
       while(tok && idx < 3) { tok = strtok(NULL, ","); a[idx++] = atoi(tok); }
       moveSmoothly(a);
       Serial.println("MOTION_COMPLETE");
     }
     else if (command.startsWith("P,")) {
       // Format: "P,x,y,z"
       int x, y, z;
       sscanf(command.c_str(), "P,%d,%d,%d", &x, &y, &z);
       inverseKinematicsMove(x, y, z);
       Serial.println("MOTION_COMPLETE");
     }
     else if (command == "HOME") { /* ... */ }
     else if (command == "STOP") { Serial.println("EMERGENCY_STOP"); }
     else if (command == "GET_ANGLES") { /* ... */ }
     else if (command == "GET_POS") { /* ... */ }
     else if (command == "STATUS") { /* ... */ }
     else if (command.startsWith("SPEED,")) { /* ... */ }
     // Support original SETPOS/SMOOTH as fallback
     else if (command.startsWith("SETPOS")) { /* original code */ }
     else if (command.startsWith("SMOOTH")) { /* original code */ }
   }
   void setup() { Serial.begin(115200); /* ... */ }
   ```
4. Upload firmware to the ESP32.

### A) Controller2 Vision Application (Standalone)

This Python application runs completely outside of your ROS2 workspace. It captures webcam images, invokes LLaVA‑Phi3 via Ollama, and sends serial commands to the ESP32.

1. Connect your ESP32 (115200 baud) via USB.
2. Run the controller script from its folder (no ROS setup required):
   ```bash
   cd path/to/controller2_vision
   python3 controller2_vision.py
   ```

---

### B) ROS 2 + MoveIt2 Trajectory Planning Simulation

This is an entirely separate ROS2 workflow for planning and simulation using MoveIt2.

1. Source ROS 2 and your ROS workspace:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```
2. Launch the MoveIt2 demo with your robot configuration:
   ```bash
   ros2 launch your_moveit_package moveit_rms_setup.launch.py
   ```

**What this brings up:**

- `robot_state_publisher` with your URDF/SRDF
- `ros2_control_node` loading your controllers YAML
- MoveIt2 planning pipeline and RViz2 with the MoveIt plugin

**Optional arguments** (e.g. `robot_description`, `use_sim_time`):

```bash
ros2 launch your_moveit_package moveit_rms_setup.launch.py \
  robot_description:=/path/to/your/urdf/my_robot.urdf \
  use_sim_time:=true
```

---

1. **Trajectory Planning**

   - Open the "Trajectory Planning" tab in the GUI.
   - Define waypoints and save JSON files for each letter (e.g., `letter_A_trajectory.json`).
   - Map letters to trajectory files and click **Save Mappings**.

2. **Sorting Envelopes**

   1. Click **Start Webcam**.
   2. Place an envelope in view and click **Capture Image**.
   3. Click **Identify Recipient** to read the name.
   4. Verify the mapped letter and click **Execute Letter Trajectory**.

3. **Manual Control & Calibration**

   - Use the **Main Control** tab for direct servo sliders or XYZ entry.
   - Preview in the 3D viewer before sending commands.

---

## ⚠️ Troubleshooting

- **Controller errors**: If a controller fails to spawn, check your `your_controllers.yaml` under `controller_manager:` and correct the names.
- **No camera feed**: Verify `/dev/video0` permissions and that `usb_cam` node is running.
- **Vision model issues**: Ensure `ollama serve` is running and lists `llava-phi3` (`ollama list`).
- **Servo jitter/misalignment**: Re-calibrate `SERVOMIN`/`SERVOMAX` in the PCA9685 code.
- **ESP32 no response**: Confirm baud rate is 115200 in both firmware and Python script.

---



