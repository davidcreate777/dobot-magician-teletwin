# gripper_control.py
import sys
import os
import time

# For Python 3.8+ DLL loading compatibility on Windows
if os.name == 'nt' and hasattr(os, 'add_dll_directory'):
    try:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        os.add_dll_directory(current_dir)
        os.add_dll_directory(os.getcwd())
    except Exception as e:
        pass

import DobotDllType as dType

def main():
    print("====================================================")
    print("      Dobot Magician Gripper Terminal Control        ")
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
    if not ports:
        print("No Dobots detected via SearchDobot.")
        port_name = input("Please enter COM port name manually (e.g. COM3) or press Enter to exit: ").strip()
        if not port_name:
            print("Exiting.")
            return
    else:
        print("Detected ports:")
        for idx, port in enumerate(ports):
            print(f"  [{idx + 1}] {port}")
        
        choice = input(f"Select port (1-{len(ports)}) [Default 1]: ").strip()
        if not choice:
            port_name = ports[0]
        else:
            try:
                port_idx = int(choice) - 1
                if 0 <= port_idx < len(ports):
                    port_name = ports[port_idx]
                else:
                    print("Invalid choice, defaulting to first port.")
                    port_name = ports[0]
            except ValueError:
                print("Invalid entry, defaulting to first port.")
                port_name = ports[0]

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

    print("Successfully connected to Dobot.")
    print(f"Firmware: {connect_result[1]}, version: {connect_result[2]}")
    
    # Initialize basic settings
    # Clear command queue just in case
    dType.SetQueuedCmdClear(api)
    
    try:
        while True:
            print("\n----------------------------------------------------")
            print("                    CONTROL MENU                    ")
            print("----------------------------------------------------")
            print("  [1] GRIP  - Close Gripper (Squeeze)")
            print("  [2] OPEN  - Open Gripper (Release grab)")
            print("  [3] OFF   - Disable Gripper Control (Passive state)")
            print("  [4] EXIT  - Disconnect & Exit")
            print("----------------------------------------------------")
            
            cmd = input("Select action (1-4): ").strip()
            
            if cmd == "1":
                print("Action: GRIP (Close Gripper)")
                # enableCtrl=1 (enable control), on=1 (grip), isQueued=0 (execute immediately)
                dType.SetEndEffectorGripper(api, 1, 1, isQueued=0)
            elif cmd == "2":
                print("Action: OPEN (Open Gripper)")
                # enableCtrl=1 (enable control), on=0 (release grab), isQueued=0 (execute immediately)
                dType.SetEndEffectorGripper(api, 1, 0, isQueued=0)
            elif cmd == "3":
                print("Action: OFF (Disable Gripper)")
                # enableCtrl=0 (disable control), on=0 (passive state), isQueued=0 (execute immediately)
                dType.SetEndEffectorGripper(api, 0, 0, isQueued=0)
            elif cmd == "4" or cmd.lower() == 'exit':
                print("Exiting and disconnecting...")
                break
            else:
                print("Invalid choice, please select 1, 2, 3, or 4.")
                
            time.sleep(0.5)
            
    finally:
        print("Releasing gripper control and disconnecting...")
        # Release gripper control (passive state) on exit
        dType.SetEndEffectorGripper(api, 0, 0, isQueued=0)
        dType.DisconnectDobot(api)
        print("Disconnected.")

if __name__ == "__main__":
    main()
