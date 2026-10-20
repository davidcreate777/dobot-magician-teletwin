# dobot_gripper_server.py
import sys
import os
import time
import socket

# For Python 3.8+ DLL loading compatibility on Windows
if os.name == 'nt' and hasattr(os, 'add_dll_directory'):
    try:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        os.add_dll_directory(current_dir)
        os.add_dll_directory(os.getcwd())
    except Exception:
        pass

import DobotDllType as dType

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

def main():
    print("====================================================")
    print("     Dobot Magician Gripper UDP Socket Server        ")
    print("====================================================\n")
    
    print("Loading Dobot API DLL...")
    try:
        api = dType.load()
        print("API DLL loaded successfully.")
    except Exception as e:
        print(f"Error loading Dobot DLL: {e}")
        print("Ensure DobotDll.dll and its dependencies are present.")
        return

    # Port selection
    print("Searching for connected Dobots...")
    ports = dType.SearchDobot(api)
    port_name = "COM3"  # default fallback
    if ports:
        port_name = ports[0]
        print(f"Auto-detected Dobot port: {port_name}")
    else:
        print(f"No Dobots auto-detected. Defaulting to fallback port: {port_name}")

    print(f"Connecting to Dobot on port {port_name}...")
    connect_result = dType.ConnectDobot(api, port_name, 115200)
    state = connect_result[0]
    
    CON_STR = {
        dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
        dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
        dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
    }
    
    print(f"Connection Status: {CON_STR.get(state, f'Unknown Error ({state})')}")
    
    if state != dType.DobotConnect.DobotConnect_NoError:
        print("Could not connect to Dobot. Exiting.")
        return

    print("Successfully connected to Dobot Magician.")
    
    # Clear command queue
    dType.SetQueuedCmdClear(api)
    # Start command execution (required for command processing)
    dType.SetQueuedCmdStartExec(api)
    # Put gripper in passive off state initially
    dType.SetEndEffectorGripper(api, 0, 0, isQueued=0)

    # Initialize PTP parameters for smooth real-time response
    dType.SetPTPCommonParams(api, 100, 100, isQueued=0)
    dType.SetPTPJointParams(api, 150, 150, 150, 150, 150, 150, 150, 150, isQueued=0)

    # Query current position to establish tracking height (Z) and orientation (R)
    print("Querying current Dobot pose...")
    pose = dType.GetPose(api)
    target_z = pose[2]
    target_r = pose[3]
    print(f"Tracking plane established: Z={target_z:.2f} mm, R={target_r:.2f} deg")
    last_queued_idx = dType.GetQueuedCmdCurrentIndex(api)[0]

    # Initialize UDP socket server
    print(f"Binding UDP Server to {UDP_IP}:{UDP_PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(1.0)  # Timeout to allow checking exit signals
    
    print("UDP Server is ready! Listening for commands...")
    print("Commands: 'OPEN', 'CLOSE', 'OFF', 'MOVE:x,y'")
    print("Press Ctrl+C to exit.")

    try:
        while True:
            try:
                # Nested try-finally to handle socket non-blocking state restore
                try:
                    # Blocking read for the first packet
                    data, addr = sock.recvfrom(1024)
                    
                    # Drain any additional pending packets in the socket buffer
                    # to only keep the latest one and avoid command queueing lag
                    sock.setblocking(False)
                    while True:
                        try:
                            next_data, next_addr = sock.recvfrom(1024)
                            data = next_data
                            addr = next_addr
                        except Exception:
                            break
                finally:
                    sock.setblocking(True)
                    
                command = data.decode("utf-8").strip().upper()
                    
                if command == "OPEN":
                    print(f"[Command] Received: '{command}' from {addr}")
                    dType.SetEndEffectorGripper(api, 1, 0, isQueued=0)
                    print("  -> Executed: OPEN Gripper")
                elif command == "CLOSE":
                    print(f"[Command] Received: '{command}' from {addr}")
                    dType.SetEndEffectorGripper(api, 1, 1, isQueued=0)
                    print("  -> Executed: CLOSE Gripper")
                elif command == "OFF":
                    print(f"[Command] Received: '{command}' from {addr}")
                    dType.SetEndEffectorGripper(api, 0, 0, isQueued=0)
                    print("  -> Executed: DISABLE Gripper")
                elif command.startswith("MOVE:"):
                    try:
                        coords_str = command[5:]
                        parts = coords_str.split(",")
                        if len(parts) == 3:
                            tx = float(parts[0])
                            ty = float(parts[1])
                            tdz = float(parts[2])
                            
                            # Calculate target Z height relative to start_z
                            tz = target_z + tdz
                            
                            # Clamp Z to physical safe limits [-35.0, 130.0]
                            Z_MIN, Z_MAX = -35.0, 130.0
                            tz_clamped = max(Z_MIN, min(tz, Z_MAX))
                            
                            # Check current hardware command queue length
                            current_idx = dType.GetQueuedCmdCurrentIndex(api)[0]
                            queue_len = last_queued_idx - current_idx
                            
                            if queue_len <= 1:
                                # Safety check XY coordinates against Cartesian limits
                                X_MIN, X_MAX = 153.98, 358.58
                                Y_MIN, Y_MAX = -272.45, 275.21
                                if X_MIN <= tx <= X_MAX and Y_MIN <= ty <= Y_MAX:
                                    res = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, tx, ty, tz_clamped, target_r, isQueued=1)
                                    if res:
                                        last_queued_idx = res[0]
                                else:
                                    print(f"  [Safety Check Failed] Coords out of reach: X={tx:.2f}, Y={ty:.2f}")
                        else:
                            print(f"  [Parse Error] Incorrect coordinates format in: {command}")
                    except ValueError as e:
                        print(f"  [Parse Error] Failed to parse coordinates from: {command} - {e}")
            except socket.timeout:
                continue
            except KeyboardInterrupt:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print("\nReleasing gripper control and disconnecting...")
        # Release gripper control (passive state) on exit
        dType.SetEndEffectorGripper(api, 0, 0, isQueued=0)
        dType.DisconnectDobot(api)
        sock.close()
        print("Server shutdown complete.")

if __name__ == "__main__":
    main()
