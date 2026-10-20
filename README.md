# TeleTwin: Computer Vision Hand Tracker & Dobot Magician Teleoperation System

A real-time teleoperation and digital twin bridge for the Dobot Magician robotic arm, powered by computer vision hand gesture tracking (MediaPipe Hands) and an asynchronous UDP client-server architecture.

---

## Overview

**TeleTwin** bridges human hand kinematics directly to industrial robotic arm control. By leveraging monocular camera feeds, the system detects hand landmarks in 3D space, projects a calibrated physical robot workspace polygon onto camera coordinates, determines bounding containment via ray casting, and transmits coordinate (X, Y, Z, R) and pneumatic gripper commands over high-speed UDP sockets to a dedicated hardware controller.

---

## Architecture & System Workflow

```
┌─────────────────────────────────┐
│     Human Operator Hand Feed    │
└────────────────┬────────────────┘
                 │ (OpenCV / MediaPipe Hands)
                 ▼
┌─────────────────────────────────┐
│    3D Pose & Gesture Tracking   │
│  - Hand Landmarks Extraction    │
│  - Workspace Ray-Casting Bounds │
│  - Coordinate Frame Projection  │
└────────────────┬────────────────┘
                 │ (UDP Socket: 127.0.0.1:5005)
                 ▼
┌─────────────────────────────────┐
│    Dobot Gripper / Arm Server   │
│  - dobot_gripper_server.py      │
│  - ctypes C++ DLL Interface     │
│  - DobotDll.dll API execution   │
└────────────────┬────────────────┘
                 │ (USB / Serial Command Pipeline)
                 ▼
┌─────────────────────────────────┐
│   Dobot Magician Robotic Arm    │
│  - 4-DOF End Effector Motion    │
│  - Pneumatic Gripper / Suction  │
└─────────────────────────────────┘
```

- **Dual-Camera Setup:** Supports dual video streams—one camera dedicated to hand tracking and an optional external camera for visual feedback.
- **Decoupled Architecture:** Separates the Python 64-bit computer vision pipeline from 32-bit DLL driver requirements via a local UDP loopback protocol (`127.0.0.1:5005`).
- **Web Monitoring Interface:** Includes `web_main.py` with an embedded multi-threaded HTTP server for browser-based monitoring and remote teleoperation.

---

## Hardware & Software Requirements

### Hardware
- **Robotic Arm:** Dobot Magician (with USB cable and power supply).
- **End-Effector:** Dobot Pneumatic Gripper or Suction Cup kit.
- **Cameras:**
  - Tracking Camera (built-in webcam or USB camera).
  - Optional external camera for live feedback.

### Software
- **Operating System:** Windows 10 / 11 (required for Dobot Windows DLLs).
- **Python:** Python 3.8 - 3.10 (32-bit recommended if directly importing Dobot DLLs; 64-bit supported when using the UDP client-server decoupling).
- **Libraries:**
  - `opencv-python`
  - `mediapipe`
  - `numpy`
- **Drivers:** Silicon Labs CP210x USB to UART Bridge Virtual COM Port driver.

---

## Project Structure

```
dobottwin/
├── config/
│   └── settings.py              # Calibration and workspace offsets
├── vision/
│   └── hand_tracker.py          # MediaPipe hand tracking implementation
├── dobot_gripper_server.py      # UDP listener executing Dobot DLL commands
├── gripper_control.py           # Standalone gripper testing script
├── main.py                      # Primary CV hand tracking & UDP client
├── move_points.py               # Pre-programmed waypoint verification script
├── web_main.py                  # Web teleoperation and telemetry server
├── DobotDll.dll                 # Dobot Magician C++ API dynamic library
├── DobotDllType.py              # Python ctypes wrapper for DobotDll
├── msvcp120.dll / msvcr120.dll  # Visual C++ redistributable dependencies
├── Qt5Core.dll / Qt5Network.dll # Qt runtime dependencies for Dobot communication
├── DobotDemoV2.0/               # Reference vendor SDK documentation and demos
└── .gitignore                   # Git ignore configuration
```

---

## Setup & Running

### Step 1: Clone the Repository
```bash
git clone https://github.com/davidcreate777/dobot-magician-teletwin.git
cd dobot-magician-teletwin
```

### Step 2: Connect the Robotic Arm
1. Connect the Dobot Magician to power and plug the USB cable into your computer.
2. Verify the assigned COM port in Windows Device Manager.

### Step 2: Start the Dobot Hardware Server
Run the hardware control server to bind the robot to the UDP socket:
```bash
python dobot_gripper_server.py
```

### Step 3: Launch Hand Tracking Teleoperation
In a separate terminal, launch the vision tracker:
```bash
python main.py
```

- **Controls:**
  - Press `c`: Calibrate the table plane distance to your hand's current depth.
  - Open hand / Pinch: Controls gripper opening and closing.
  - Move hand inside the polygon: Controls robotic arm 3D end-effector position.
  - Press `q` or `ESC`: Safely terminate tracking and close sockets.

### Step 4 (Alternative): Web Teleoperation Dashboard
To run the web-enabled interface:
```bash
python web_main.py
```
Open `http://localhost:8000` in your web browser.

---

## Limitations

- **Lighting & Occlusion:** MediaPipe tracking requires clear lighting and unoccluded fingers.
- **Workspace Envelope:** The physical robot coordinates are restricted to the 5-point calibrated safety polygon to prevent mechanical collisions.
