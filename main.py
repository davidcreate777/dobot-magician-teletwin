# main.py
import cv2
import numpy as np
import time
import socket
import sys
from vision.hand_tracker import HandTracker

# Camera Indices Configuration
TRACKING_CAM_INDEX = 1  # Built-in laptop camera for hand tracking
EXTERNAL_CAM_INDEX = 0  # External USB camera for live feedback

# Calibration offsets to align the physical Dobot coordinates with the camera feed
BASE_OFFSET_X = 260.0  # mm (forward distance from Dobot base to camera center)
BASE_OFFSET_Y = -50.0  # mm (left/right offset, negative shifts camera to the left)

# The 5 physical points defining the workspace polygon
WORKSPACE_POLYGON = [
    [153.98, -272.45],  # Point 1 (Base Right)
    [295.44, -203.22],  # Point 2 (Mid Right)
    [358.58, -36.32],   # Point 3 (Far Center)
    [337.29, 143.40],   # Point 4 (Mid Left)
    [158.42, 275.21]    # Point 5 (Base Left)
]

def dobot_to_screen(dobot_x, dobot_y, w, h, focal_length, z_table):
    cx = w / 2.0
    cy = h / 2.0
    
    # Map Dobot coordinates to camera coordinates (x_cam, y_cam, z_cam)
    # y_cam is positive downwards (towards the base), so larger dobot_x (further away) 
    # means smaller/negative y_cam.
    x_cam = dobot_y - BASE_OFFSET_Y
    y_cam = BASE_OFFSET_X - dobot_x
    z_cam = z_table
    
    # Project to 2D screen coordinates
    u = int(cx + (x_cam * focal_length) / z_cam)
    v = int(cy + (y_cam * focal_length) / z_cam)
    return (u, v)

def is_inside_polygon(x, y, polygon):
    """Ray-casting algorithm to check if point (x, y) is inside the polygon."""
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def main():
    print("====================================================")
    print("     TeleTwin CV Hand Tracker & UDP Socket client   ")
    print("====================================================\n")

    # Initialize UDP socket client for sending gripper commands to the 32-bit server
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_server_addr = ("127.0.0.1", 5005)
    last_hand_open_state = None  # Track state changes to prevent spamming
    last_move_time = 0.0
    last_sent_x = 0.0
    last_sent_y = 0.0
    last_sent_z = 0.0

    # Default distance from camera to table surface (updated dynamically on calibration)
    z_table = 450.0

    # Initialize main tracking camera (laptop camera)
    print(f"Connecting to main tracking camera (Index {TRACKING_CAM_INDEX} via DirectShow)...")
    cap = cv2.VideoCapture(TRACKING_CAM_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"Error: Could not open tracking camera index {TRACKING_CAM_INDEX}. Exiting.")
        sys.exit(1)

    # Set resolution for main tracking camera
    w_cam, h_cam = 640, 480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w_cam)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h_cam)

    # Initialize external camera
    print(f"Connecting to external camera (Index {EXTERNAL_CAM_INDEX} via DirectShow)...")
    cap_external = cv2.VideoCapture(EXTERNAL_CAM_INDEX, cv2.CAP_DSHOW)
    external_active = True
    if not cap_external.isOpened():
        print(f"Warning: Could not open external camera index {EXTERNAL_CAM_INDEX}. Proceeding with tracking only.")
        external_active = False

    # Initialize camera window and make it full screen
    cv2.namedWindow("TeleTwin Live Hand Tracker", cv2.WINDOW_NORMAL)
    cv2.setWindowProperty("TeleTwin Live Hand Tracker", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    # Initialize hand tracker
    tracker = HandTracker(min_detection_confidence=0.7, min_tracking_confidence=0.5)

    print("Live feed started!")
    print("Controls:")
    print("  Press 'c' to Calibrate workspace plane to your hand's current depth")
    print("  Press 'ESC' or 'q' to quit.")

    while True:
        success, frame = cap.read()
        if not success or frame is None:
            print("Failed to read frame from camera. Exiting...")
            break

        # Mirror frame horizontally for standard mirror view
        frame = cv2.flip(frame, 1)

        # Process hand tracking frame
        hand_data = tracker.process_frame(frame)
        is_inside = False
        dobot_x, dobot_y = 0.0, 0.0

        # Calculate focal length dynamically based on frame width/height
        h, w = frame.shape[:2]
        cx_img = w / 2.0
        cy_img = h / 2.0
        diagonal_pixels = np.sqrt(w**2 + h**2)
        focal_length = diagonal_pixels / (2 * np.tan(np.radians(70.0 / 2.0)))

        # Pre-calculate workspace polygon points flat on the table using current z_table
        arc_pts = []
        for pt in WORKSPACE_POLYGON:
            arc_pts.append(dobot_to_screen(pt[0], pt[1], w, h, focal_length, z_table))
        arc_pts = np.array(arc_pts, dtype=np.int32)

        # Process keystrokes first to allow calibration to be handled in the current frame
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break

        if hand_data is not None:
            # Estimate physical depth (Z in mm) based on palm pixel distance
            pt_wrist = hand_data.landmarks[0][:2] * [w, h]
            pt_middle_mcp = hand_data.landmarks[9][:2] * [w, h]
            palm_pixel_dist = np.linalg.norm(pt_wrist - pt_middle_mcp)
            
            if palm_pixel_dist <= 0:
                hand_z = z_table
            else:
                hand_z = (focal_length * 80.0) / palm_pixel_dist
                
            # Estimate physical horizontal position (X in mm)
            wrist_2d_norm = hand_data.wrist_pos[:2]
            u_px = wrist_2d_norm[0] * w
            v_px = wrist_2d_norm[1] * h
            hand_x = ((u_px - cx_img) * hand_z) / focal_length
            hand_y = ((v_px - cy_img) * hand_z) / focal_length
            
            # Map to Dobot coordinates
            # Dobot Y (left/right) maps to Camera X (hand_x)
            # Dobot X (forward distance) maps to Camera Y (hand_y)
            dobot_y = hand_x + BASE_OFFSET_Y
            dobot_x = BASE_OFFSET_X - hand_y

            # Handle dynamic plane calibration on 'c' key press
            if key == ord('c'):
                z_table = hand_z
                print(f"[CALIBRATION] Workspace height plane locked to hand depth: {z_table:.1f} mm")

            # Boundary check using ray-casting point-in-polygon check
            is_inside = is_inside_polygon(dobot_x, dobot_y, WORKSPACE_POLYGON)

            # Draw skeleton connections
            landmarks_px = []
            for lm in hand_data.landmarks:
                cx_lm, cy_lm = int(lm[0] * w), int(lm[1] * h)
                landmarks_px.append((cx_lm, cy_lm))
                cv2.circle(frame, (cx_lm, cy_lm), 4, (0, 165, 255), -1, cv2.LINE_AA) # Amber joints

            connections = [
                (0, 1), (1, 2), (2, 3), (3, 4),      # Thumb
                (0, 5), (5, 6), (6, 7), (7, 8),      # Index
                (0, 9), (9, 10), (10, 11), (11, 12), # Middle
                (0, 13), (13, 14), (14, 15), (15, 16),# Ring
                (0, 17), (17, 18), (18, 19), (19, 20),# Pinky
                (5, 9), (9, 13), (13, 17)            # Palm base
            ]
            for start_idx, end_idx in connections:
                cv2.line(frame, landmarks_px[start_idx], landmarks_px[end_idx], (255, 128, 0), 2, cv2.LINE_AA) # Orange connections

            # Send hand state change to UDP server
            is_open = hand_data.is_open
            if last_hand_open_state is None or last_hand_open_state != is_open:
                last_hand_open_state = is_open
                cmd_msg = b"OPEN" if is_open else b"CLOSE"
                try:
                    udp_sock.sendto(cmd_msg, udp_server_addr)
                    print(f"[Sent Command] {cmd_msg.decode('utf-8')}")
                except Exception as e:
                    print(f"Error sending command: {e}")

            # Send coordinate updates for arm movement tracking
            if is_inside:
                current_time = time.time()
                if current_time - last_move_time >= 0.15:
                    dist_moved = np.hypot(dobot_x - last_sent_x, dobot_y - last_sent_y)
                    delta_z = hand_z - z_table
                    z_diff = abs(delta_z - last_sent_z)
                    
                    if dist_moved >= 10.0 or z_diff >= 5.0:
                        cmd_msg = f"MOVE:{dobot_x:.2f},{dobot_y:.2f},{delta_z:.2f}".encode('utf-8')
                        try:
                            udp_sock.sendto(cmd_msg, udp_server_addr)
                            last_move_time = current_time
                            last_sent_x = dobot_x
                            last_sent_y = dobot_y
                            last_sent_z = delta_z
                        except Exception as e:
                            print(f"Error sending move command: {e}")

        # --- Draw 2D Workspace Flat-on-Table Overlay ---
        overlay = frame.copy()
        
        # Color configuration based on containment
        if is_inside:
            hud_color = (255, 200, 100)  # Pale Blue-Green
            edge_color = (255, 170, 70)
            status_str = "INSIDE"
            status_color = (0, 255, 0)
        else:
            hud_color = (0, 0, 255)      # Soft Red warning
            edge_color = (0, 0, 220)
            status_str = "OUTSIDE" if hand_data is not None else "NO HAND"
            status_color = (0, 0, 255) if hand_data is not None else (150, 150, 150)

        # Draw filled workspace segment (the full 5-sided polygon)
        cv2.fillPoly(overlay, [arc_pts], hud_color)
        cv2.addWeighted(overlay, 0.15, frame, 0.85, 0, frame)

        # Draw outlines
        cv2.polylines(frame, [arc_pts], True, edge_color, 2, cv2.LINE_AA)
        
        # Draw radial guide lines from origin to the base corners (Point 1 and Point 5)
        origin_u, origin_v = dobot_to_screen(0, 0, w, h, focal_length, z_table)
        cv2.line(frame, (origin_u, origin_v), tuple(arc_pts[0]), edge_color, 1, cv2.LINE_AA)
        cv2.line(frame, (origin_u, origin_v), tuple(arc_pts[-1]), edge_color, 1, cv2.LINE_AA)
        
        # Draw robot origin center dot
        cv2.circle(frame, (origin_u, origin_v), 5, edge_color, -1, cv2.LINE_AA)

        # Plot hand coordinate marker on HUD
        if hand_data is not None:
            # Map wrist screen coordinates
            wrist_u, wrist_v = landmarks_px[0]
            
            # Draw hand status dot
            dot_color = (0, 255, 0) if is_inside else (0, 0, 255)
            cv2.circle(frame, (wrist_u, wrist_v), 7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.circle(frame, (wrist_u, wrist_v), 5, dot_color, -1, cv2.LINE_AA)
            
            # Show live hand coordinates
            cv2.putText(frame, f"Hand Pos: X:{dobot_x:.1f} Y:{dobot_y:.1f}", (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Show status and info
        cv2.putText(frame, f"Workspace: ", (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, status_str, (155, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2, cv2.LINE_AA)
        cv2.putText(frame, f"Calibrated Depth: {z_table:.1f} mm (Press 'c' to Calibrate)", (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, "Base", (origin_u - 15, origin_v + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, edge_color, 1, cv2.LINE_AA)

        # Show camera feed
        cv2.imshow("TeleTwin Live Hand Tracker", frame)

        # Read and show external camera if active
        if external_active:
            ret_eff, frame_eff = cap_external.read()
            if ret_eff and frame_eff is not None:
                # Resize to a compact preview window (320x240)
                frame_eff_resized = cv2.resize(frame_eff, (320, 240))
                cv2.imshow("External Camera Feedback", frame_eff_resized)

    # Clean up
    print("Releasing cameras and resources...")
    cap.release()
    if external_active:
        cap_external.release()
    tracker.close()
    cv2.destroyAllWindows()
    print("Shutdown complete.")

if __name__ == "__main__":
    main()
