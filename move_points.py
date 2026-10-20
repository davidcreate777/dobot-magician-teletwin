# move_points.py
import sys
import os
import time
import math

# For Python 3.8+ DLL loading compatibility on Windows
if os.name == 'nt' and hasattr(os, 'add_dll_directory'):
    try:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        os.add_dll_directory(current_dir)
        os.add_dll_directory(os.getcwd())
    except Exception:
        pass

import DobotDllType as dType

# Define Cartesian limits
X_MIN, X_MAX = 153.98, 358.58
Y_MIN, Y_MAX = -272.45, 275.21
Z_MIN, Z_MAX = 5.57, 113.04

# Point 5 is set to the current location of the arm.
POINTS = [
    {"id": 1, "x": 153.98, "y": -272.45, "z": 5.57},
    {"id": 2, "x": 295.44, "y": -203.22, "z": -7.97},
    {"id": 3, "x": 358.58, "y": -36.32,  "z": 8.01},
    {"id": 4, "x": 337.29, "y": 143.40,  "z": 8.05},
    {"id": 5, "x": 158.42, "y": 275.21,  "z": -13.16}
]

def check_limits(x, y, z):
    violations = []
    if not (X_MIN <= x <= X_MAX):
        violations.append(f"X: {x:.2f} is outside range [{X_MIN}, {X_MAX}]")
    if not (Y_MIN <= y <= Y_MAX):
        violations.append(f"Y: {y:.2f} is outside range [{Y_MIN}, {Y_MAX}]")
    if not (Z_MIN <= z <= Z_MAX):
        violations.append(f"Z: {z:.2f} is outside range [{Z_MIN}, {Z_MAX}]")
    return violations

def main():
    print("====================================================")
    print("      Dobot Magician Point-to-Point Control          ")
    print("====================================================\n")
    
    print("Loading Dobot API DLL...")
    try:
        api = dType.load()
        print("API DLL loaded successfully.")
    except Exception as e:
        print(f"Error loading Dobot DLL: {e}")
        print("Make sure DobotDll.dll and its dependencies are in this directory.")
        return

    # Auto-detect ports
    print("Searching for connected Dobots...")
    ports = dType.SearchDobot(api)
    port_name = "COM3"
    if ports:
        port_name = ports[0]
        print(f"Auto-detected port: {port_name}")
    else:
        print(f"No Dobots auto-detected. Defaulting to fallback port: {port_name}")

    print(f"Connecting to Dobot on port {port_name}...")
    connect_result = dType.ConnectDobot(api, port_name, 115200)
    state = connect_result[0]
    
    if state != dType.DobotConnect.DobotConnect_NoError:
        print("Could not connect to Dobot. Exiting.")
        return
        
    print("Connected successfully. Initializing parameters...")
    
    # Query current position of the arm
    pose = dType.GetPose(api)
    start_x, start_y, start_z = pose[0], pose[1], pose[2]
    print(f"\n[INFO] Queried current arm position: X:{start_x:.2f}, Y:{start_y:.2f}, Z:{start_z:.2f}")
    
    # Dynamically set Point 5 to the current starting position
    POINTS[4] = {"id": 5, "x": start_x, "y": start_y, "z": start_z}
    print(f"[INFO] Point 5 updated to current starting position: X:{start_x:.2f}, Y:{start_y:.2f}, Z:{start_z:.2f}\n")
    
    # Clean queued command
    dType.SetQueuedCmdClear(api)
    
    # Set speed and acceleration params for PTP motion
    dType.SetPTPCommonParams(api, 100, 100, isQueued=1)
    dType.SetPTPJointParams(api, 150, 150, 150, 150, 150, 150, 150, 150, isQueued=1)
    
    # Start Execution
    dType.SetQueuedCmdStartExec(api)
    
    try:
        for pt in POINTS:
            pt_id = pt["id"]
            x, y, z = pt["x"], pt["y"], pt["z"]
            
            # Calculate radial distance from base origin (0,0) in X-Y plane
            radius = math.sqrt(x**2 + y**2)
            print(f"\nEvaluating Point {pt_id}: X:{x:.2f}, Y:{y:.2f}, Z:{z:.2f} | Arc Radius: {radius:.2f} mm")
            
            # Check limits
            violations = check_limits(x, y, z)
            
            if violations:
                print(f"Point {pt_id} is OUT OF LIMITS: {', '.join(violations)}")
                
                # Check if this is Z-axis violation specifically and try to correct it
                if not (Z_MIN <= z <= Z_MAX):
                    print(f"  [DECISION] Analyzing Z coordinate ({z:.2f})...")
                    abs_z = abs(z)
                    if Z_MIN <= abs_z <= Z_MAX:
                        corrected_z = abs_z
                        print(f"  [CORRECTION] Detected sign typo! Corrected Z from {z:.2f} to {corrected_z:.2f} (absolute value).")
                        z = corrected_z
                        violations = check_limits(x, y, z)
                    else:
                        clamped_z = min(max(z, Z_MIN), Z_MAX)
                        print(f"  [CORRECTION] Clamped Z from {z:.2f} to {clamped_z:.2f} (boundary limit).")
                        z = clamped_z
                        violations = check_limits(x, y, z)
            
            # Recheck violations after correction
            if violations:
                print("----------------------------------------------------")
                print(f"!!! ALERT !!! Point {pt_id} is STILL OUT OF LIMITS after correction:")
                for v in violations:
                    print(f"  [VIOLATION] {v}")
                print(f"-> SKIPPING movement to Point {pt_id} to protect hardware.")
                print("----------------------------------------------------")
                continue
                
            # Point is valid, move there
            print(f"-> Point {pt_id} is within limits. Moving Dobot to (X:{x:.2f}, Y:{y:.2f}, Z:{z:.2f})...")
            
            # Queue the PTP move command (PTPMOVJXYZMode = Joint Move to Cartesian Coordinates)
            queued_idx = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, x, y, z, 0.0, isQueued=1)[0]
            
            # Wait for execution to reach this command index
            while queued_idx > dType.GetQueuedCmdCurrentIndex(api)[0]:
                dType.dSleep(100)
                
            print(f"SUCCESS: Reached Point {pt_id}.")
            time.sleep(1.0)  # Pause for 1 second at the point
            
    finally:
        print("\nStopping queued commands and disconnecting...")
        dType.SetQueuedCmdStopExec(api)
        dType.DisconnectDobot(api)
        print("Disconnected.")

if __name__ == "__main__":
    main()
