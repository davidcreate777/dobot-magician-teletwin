# web_main.py
import cv2
import numpy as np
import time
import socket
import sys
import threading
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from vision.hand_tracker import HandTracker

# Camera Indices Configuration (swapped as requested)
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

# Thread-safe shared state variables
state_lock = threading.Lock()
latest_tracking_frame = None
latest_external_frame = None
hand_x = 0.0
hand_y = 0.0
hand_z = 0.0
dobot_x = 0.0
dobot_y = 0.0
delta_z = 0.0
z_table = 450.0
is_inside = False
is_open = True
trigger_calibration = False
running = True
last_hand_open_state = None

def dobot_to_screen(dobot_x_val, dobot_y_val, w, h, focal_length, current_z_table):
    cx = w / 2.0
    cy = h / 2.0
    
    x_cam = dobot_y_val - BASE_OFFSET_Y
    y_cam = BASE_OFFSET_X - dobot_x_val
    z_cam = current_z_table
    
    u = int(cx + (x_cam * focal_length) / z_cam)
    v = int(cy + (y_cam * focal_length) / z_cam)
    return (u, v)

def is_inside_polygon(x, y, polygon):
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

# HTML/CSS Content Constants
HTML_CONTENT = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>TeleTwin Dashboard</title>
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;600;700&family=Outfit:wght@400;600;700&display=swap" rel="stylesheet">
    <link rel="stylesheet" href="/style.css">
</head>
<body>
    <div class="container">
        <header class="header">
            <h1>TeleTwin <span>Dashboard</span></h1>
            <p>Real-time Hand Tracking & Arm Teleoperation Control</p>
        </header>
        
        <div class="dashboard-grid">
            <!-- Main Hand Tracking Feed -->
            <div class="card feed-card tracking-card">
                <div class="card-header">
                    <h2>Tracking camera (Laptop)</h2>
                    <span class="badge tracking">LIVE</span>
                </div>
                <div class="video-container">
                    <img src="/video_feed" alt="Tracking Feed" id="feed-tracking">
                </div>
            </div>

            <!-- External Camera Feed -->
            <div class="card feed-card external-card">
                <div class="card-header">
                    <h2>External Feed</h2>
                    <span class="badge external">ACTIVE</span>
                </div>
                <div class="video-container">
                    <img src="/external_feed" alt="External Feed" id="feed-external">
                </div>
            </div>

            <!-- Controls and Status widgets -->
            <div class="card controls-card">
                <div class="card-header">
                    <h2>System Control</h2>
                </div>
                
                <div class="control-panel">
                    <button id="btn-calib" onclick="triggerCalibrate()">Calibrate Workspace</button>
                    <p class="help-text">Place hand flat on table in tracking view and click Calibrate.</p>
                </div>

                <div class="separator"></div>

                <div class="telemetry-section">
                    <h3>Telemetry</h3>
                    
                    <div class="telemetry-grid">
                        <div class="tele-item">
                            <span class="label">Dobot X</span>
                            <span class="value" id="coord-x">0.0</span>
                            <span class="unit">mm</span>
                        </div>
                        <div class="tele-item">
                            <span class="label">Dobot Y</span>
                            <span class="value" id="coord-y">0.0</span>
                            <span class="unit">mm</span>
                        </div>
                        <div class="tele-item">
                            <span class="label">Z Offset</span>
                            <span class="value" id="coord-z">0.0</span>
                            <span class="unit">mm</span>
                        </div>
                    </div>
                </div>

                <div class="separator"></div>

                <div class="status-section">
                    <div class="status-row">
                        <span>Workspace Status</span>
                        <span id="status-badge" class="status-badge outside">NO HAND</span>
                    </div>
                    <div class="status-row">
                        <span>Gripper State</span>
                        <span id="gripper-badge" class="status-badge inside">OPEN</span>
                    </div>
                    <div class="status-row">
                        <span>Calibrated Z-Base</span>
                        <span><strong id="calib-depth">450.0</strong> mm</span>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script>
        async function updateStatus() {
            try {
                const response = await fetch('/api/status');
                if (!response.ok) return;
                const data = await response.json();
                
                // Update text coordinates
                document.getElementById('coord-x').innerText = data.dobot_x.toFixed(1);
                document.getElementById('coord-y').innerText = data.dobot_y.toFixed(1);
                document.getElementById('coord-z').innerText = data.delta_z.toFixed(1);
                
                // Update containment state
                const statusBadge = document.getElementById('status-badge');
                statusBadge.innerText = data.status_str;
                if (data.status_str === "INSIDE") {
                    statusBadge.className = "status-badge inside";
                } else if (data.status_str === "OUTSIDE") {
                    statusBadge.className = "status-badge outside";
                } else {
                    statusBadge.className = "status-badge offline";
                }
                
                // Update gripper state
                const gripperBadge = document.getElementById('gripper-badge');
                gripperBadge.innerText = data.is_open ? "OPEN" : "CLOSED";
                gripperBadge.className = data.is_open ? "status-badge inside" : "status-badge outside";
                
                // Update calibration baseline
                document.getElementById('calib-depth').innerText = data.z_table.toFixed(1);
            } catch (e) {
                console.error("Telemetry update failed:", e);
            }
        }

        async function triggerCalibrate() {
            try {
                const btn = document.getElementById('btn-calib');
                btn.classList.add('active-btn');
                btn.innerText = "Calibrating...";
                
                const response = await fetch('/api/calibrate', { method: 'POST' });
                if (response.ok) {
                    btn.innerText = "Height Plane Locked!";
                } else {
                    btn.innerText = "Calibration Failed";
                }
                
                setTimeout(() => {
                    btn.classList.remove('active-btn');
                    btn.innerText = "Calibrate Workspace";
                }, 2000);
            } catch (e) {
                console.error("Calibration call failed:", e);
            }
        }

        // Telemetry update rate: 5 Hz (every 200ms)
        setInterval(updateStatus, 200);
    </script>
</body>
</html>
"""

CSS_CONTENT = """* {
    margin: 0;
    padding: 0;
    box-sizing: border-box;
}

:root {
    --bg-color: #090b11;
    --card-bg: rgba(18, 24, 38, 0.6);
    --card-border: rgba(255, 255, 255, 0.08);
    --text-main: #f8fafc;
    --text-sub: #94a3b8;
    --accent-color: #3b82f6;
    --accent-hover: #2563eb;
    --accent-glow: rgba(59, 130, 246, 0.35);
    --success-color: #10b981;
    --success-glow: rgba(16, 185, 129, 0.25);
    --danger-color: #ef4444;
    --danger-glow: rgba(239, 68, 68, 0.25);
    --offline-color: #64748b;
}

body {
    font-family: 'Inter', sans-serif;
    background-color: var(--bg-color);
    color: var(--text-main);
    min-height: 100vh;
    padding: 2rem 1.5rem;
}

.container {
    max-width: 1400px;
    margin: 0 auto;
}

.header {
    margin-bottom: 2rem;
    text-align: left;
}

.header h1 {
    font-family: 'Outfit', sans-serif;
    font-size: 2.2rem;
    font-weight: 700;
    letter-spacing: -0.5px;
}

.header h1 span {
    color: var(--accent-color);
    text-shadow: 0 0 15px var(--accent-glow);
}

.header p {
    color: var(--text-sub);
    font-size: 0.95rem;
    margin-top: 0.25rem;
}

/* Responsive Dashboard Layout */
.dashboard-grid {
    display: grid;
    gap: 1.5rem;
    grid-template-columns: repeat(3, 1fr);
}

/* Cards Design System with Glassmorphism */
.card {
    background-color: var(--card-bg);
    border: 1px solid var(--card-border);
    border-radius: 16px;
    padding: 1.25rem;
    backdrop-filter: blur(16px);
    -webkit-backdrop-filter: blur(16px);
    box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.25);
    display: flex;
    flex-direction: column;
}

.card-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 1rem;
}

.card-header h2 {
    font-family: 'Outfit', sans-serif;
    font-size: 1.15rem;
    font-weight: 600;
    color: var(--text-main);
}

.badge {
    font-size: 0.7rem;
    font-weight: 700;
    padding: 0.25rem 0.6rem;
    border-radius: 20px;
    letter-spacing: 0.5px;
}

.badge.tracking {
    background-color: var(--success-color);
    color: #fff;
    box-shadow: 0 0 10px var(--success-glow);
}

.badge.external {
    background-color: var(--accent-color);
    color: #fff;
    box-shadow: 0 0 10px var(--accent-glow);
}

.video-container {
    position: relative;
    width: 100%;
    aspect-ratio: 4/3;
    background-color: #040507;
    border-radius: 12px;
    overflow: hidden;
    display: flex;
    align-items: center;
    justify-content: center;
}

.video-container img {
    width: 100%;
    height: 100%;
    object-fit: cover;
}

/* Control widget items */
.control-panel {
    display: flex;
    flex-direction: column;
    gap: 0.75rem;
}

#btn-calib {
    width: 100%;
    background-color: var(--accent-color);
    color: #fff;
    border: none;
    padding: 0.9rem;
    border-radius: 10px;
    font-weight: 600;
    font-size: 0.95rem;
    cursor: pointer;
    box-shadow: 0 4px 15px var(--accent-glow);
    transition: all 0.25s ease;
}

#btn-calib:hover {
    background-color: var(--accent-hover);
    transform: translateY(-2px);
    box-shadow: 0 6px 20px var(--accent-glow);
}

.active-btn {
    background-color: var(--success-color) !important;
    box-shadow: 0 6px 20px var(--success-glow) !important;
}

.help-text {
    font-size: 0.8rem;
    color: var(--text-sub);
    line-height: 1.3;
}

.separator {
    height: 1px;
    background-color: var(--card-border);
    margin: 1.25rem 0;
}

.telemetry-section h3 {
    font-size: 0.85rem;
    text-transform: uppercase;
    letter-spacing: 0.5px;
    color: var(--text-sub);
    margin-bottom: 0.75rem;
}

.telemetry-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 0.75rem;
}

.tele-item {
    background-color: rgba(255, 255, 255, 0.03);
    border: 1px solid var(--card-border);
    border-radius: 10px;
    padding: 0.75rem;
    text-align: center;
}

.tele-item .label {
    display: block;
    font-size: 0.75rem;
    color: var(--text-sub);
    margin-bottom: 0.25rem;
}

.tele-item .value {
    font-size: 1.15rem;
    font-weight: 700;
    font-family: 'Outfit', sans-serif;
    color: #fff;
}

.tele-item .unit {
    font-size: 0.7rem;
    color: var(--text-sub);
    margin-left: 2px;
}

/* Status section */
.status-section {
    display: flex;
    flex-direction: column;
    gap: 0.75rem;
}

.status-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    font-size: 0.9rem;
    color: var(--text-sub);
}

.status-row span:last-child {
    color: var(--text-main);
}

.status-badge {
    font-size: 0.75rem;
    font-weight: 700;
    padding: 0.25rem 0.65rem;
    border-radius: 6px;
    letter-spacing: 0.5px;
}

.status-badge.inside {
    background-color: rgba(16, 185, 129, 0.15);
    color: var(--success-color);
    border: 1px solid rgba(16, 185, 129, 0.3);
}

.status-badge.outside {
    background-color: rgba(239, 68, 68, 0.15);
    color: var(--danger-color);
    border: 1px solid rgba(239, 68, 68, 0.3);
}

.status-badge.offline {
    background-color: rgba(100, 116, 139, 0.15);
    color: var(--offline-color);
    border: 1px solid rgba(100, 116, 139, 0.3);
}

/* Responsive Media Queries */

/* Tablet layout */
@media (max-width: 1024px) {
    .dashboard-grid {
        grid-template-columns: repeat(2, 1fr);
    }
    
    .controls-card {
        grid-column: span 2;
    }
    
    .telemetry-grid {
        grid-template-columns: repeat(3, 1fr);
    }
}

/* Mobile layout */
@media (max-width: 640px) {
    body {
        padding: 1rem 0.75rem;
    }
    
    .header h1 {
        font-size: 1.8rem;
    }
    
    .dashboard-grid {
        grid-template-columns: 1fr;
        gap: 1rem;
    }
    
    .controls-card {
        grid-column: span 1;
    }
    
    .telemetry-grid {
        grid-template-columns: 1fr;
    }
}
"""

class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    """Multi-threaded HTTPServer to handle video streaming requests concurrently."""
    daemon_threads = True

class WebServerHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # Silence default access logging to console to keep outputs clean
        pass
        
    def do_GET(self):
        global latest_tracking_frame, latest_external_frame, hand_x, hand_y, hand_z
        global dobot_x, dobot_y, delta_z, z_table, is_inside, is_open
        
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(HTML_CONTENT)))
            self.end_headers()
            self.wfile.write(HTML_CONTENT.encode('utf-8'))
            
        elif self.path == '/style.css':
            self.send_response(200)
            self.send_header('Content-Type', 'text/css')
            self.send_header('Content-Length', str(len(CSS_CONTENT)))
            self.end_headers()
            self.wfile.write(CSS_CONTENT.encode('utf-8'))
            
        elif self.path == '/api/status':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            
            with state_lock:
                status_str = "INSIDE" if is_inside else ("OUTSIDE" if latest_tracking_frame is not None else "NO HAND")
                status_payload = {
                    "dobot_x": dobot_x,
                    "dobot_y": dobot_y,
                    "delta_z": delta_z,
                    "z_table": z_table,
                    "is_inside": is_inside,
                    "is_open": is_open,
                    "status_str": status_str
                }
            payload_bytes = json.dumps(status_payload).encode('utf-8')
            self.send_header('Content-Length', str(len(payload_bytes)))
            self.end_headers()
            self.wfile.write(payload_bytes)
            
        elif self.path == '/video_feed':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            
            while running:
                with state_lock:
                    if latest_tracking_frame is None:
                        img_bytes = None
                    else:
                        ret, jpeg = cv2.imencode('.jpg', latest_tracking_frame)
                        img_bytes = jpeg.tobytes() if ret else None
                
                if img_bytes is not None:
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(img_bytes)}\r\n\r\n'.encode('ascii'))
                        self.wfile.write(img_bytes)
                        self.wfile.write(b'\r\n')
                    except Exception:
                        break
                time.sleep(0.04)  # stream at ~25 FPS to preserve cpu
                
        elif self.path == '/external_feed':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            
            while running:
                with state_lock:
                    if latest_external_frame is None:
                        img_bytes = None
                    else:
                        ret, jpeg = cv2.imencode('.jpg', latest_external_frame)
                        img_bytes = jpeg.tobytes() if ret else None
                
                if img_bytes is not None:
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(img_bytes)}\r\n\r\n'.encode('ascii'))
                        self.wfile.write(img_bytes)
                        self.wfile.write(b'\r\n')
                    except Exception:
                        break
                time.sleep(0.04)  # stream at ~25 FPS
        else:
            self.send_error(404, "File Not Found")
            
    def do_POST(self):
        global trigger_calibration
        if self.path == '/api/calibrate':
            with state_lock:
                trigger_calibration = True
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', '15')
            self.end_headers()
            self.wfile.write(b'{"status":"ok"}')
        else:
            self.send_error(404, "Endpoint Not Found")

def start_web_server():
    server_address = ('0.0.0.0', 8080)
    httpd = ThreadingHTTPServer(server_address, WebServerHandler)
    print("====================================================")
    print("  Web Server Dashboard Started on http://localhost:8080")
    print("  (Also accessible from local devices at http://<your-pc-ip>:8080)")
    print("====================================================\n")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()

def main():
    global latest_tracking_frame, latest_external_frame, hand_x, hand_y, hand_z
    global dobot_x, dobot_y, delta_z, z_table, is_inside, is_open
    global last_hand_open_state, trigger_calibration, running

    print("====================================================")
    print("     TeleTwin CV Hand Tracker & Web Server Dashboard  ")
    print("====================================================\n")

    # Start the local Web Server in a daemon background thread
    web_thread = threading.Thread(target=start_web_server, daemon=True)
    web_thread.start()

    # Initialize UDP socket client for sending gripper commands to the 32-bit server
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_server_addr = ("127.0.0.1", 5005)
    
    last_move_time = 0.0
    last_sent_x = 0.0
    last_sent_y = 0.0
    last_sent_z = 0.0

    # Initialize main tracking camera (laptop camera - Index 1)
    print(f"Connecting to main tracking camera (Index {TRACKING_CAM_INDEX} via DirectShow)...")
    cap = cv2.VideoCapture(TRACKING_CAM_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"Error: Could not open tracking camera index {TRACKING_CAM_INDEX}. Exiting.")
        sys.exit(1)

    # Set resolution for main tracking camera
    w_cam, h_cam = 640, 480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w_cam)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h_cam)

    # Initialize external camera (Index 0)
    print(f"Connecting to external camera (Index {EXTERNAL_CAM_INDEX} via DirectShow)...")
    cap_external = cv2.VideoCapture(EXTERNAL_CAM_INDEX, cv2.CAP_DSHOW)
    external_active = True
    if not cap_external.isOpened():
        print(f"Warning: Could not open external camera index {EXTERNAL_CAM_INDEX}. Proceeding with tracking only.")
        external_active = False

    # Initialize hand tracker
    tracker = HandTracker(min_detection_confidence=0.7, min_tracking_confidence=0.5)

    print("Main tracking loop running in background... Open dashboard in browser to control!")

    try:
        while True:
            # Capture tracking camera frame
            success, frame = cap.read()
            if not success or frame is None:
                print("Failed to read frame from tracking camera.")
                break

            # Mirror frame horizontally for standard mirror view
            frame = cv2.flip(frame, 1)

            # Process hand tracking frame
            hand_data = tracker.process_frame(frame)
            local_is_inside = False
            local_dobot_x, local_dobot_y = 0.0, 0.0
            local_delta_z = 0.0
            local_hand_x, local_hand_y, local_hand_z = 0.0, 0.0, 0.0

            # Calculate focal length dynamically based on frame width/height
            h, w = frame.shape[:2]
            cx_img = w / 2.0
            cy_img = h / 2.0
            diagonal_pixels = np.sqrt(w**2 + h**2)
            focal_length = diagonal_pixels / (2 * np.tan(np.radians(70.0 / 2.0)))

            # Pre-calculate workspace polygon points flat on the table
            arc_pts = []
            for pt in WORKSPACE_POLYGON:
                arc_pts.append(dobot_to_screen(pt[0], pt[1], w, h, focal_length, z_table))
            arc_pts = np.array(arc_pts, dtype=np.int32)

            if hand_data is not None:
                # Estimate depth (Z) based on palm pixel distance
                pt_wrist = hand_data.landmarks[0][:2] * [w, h]
                pt_middle_mcp = hand_data.landmarks[9][:2] * [w, h]
                palm_pixel_dist = np.linalg.norm(pt_wrist - pt_middle_mcp)
                
                if palm_pixel_dist <= 0:
                    local_hand_z = z_table
                else:
                    local_hand_z = (focal_length * 80.0) / palm_pixel_dist
                    
                # Estimate horizontal coordinates (X)
                wrist_2d_norm = hand_data.wrist_pos[:2]
                u_px = wrist_2d_norm[0] * w
                v_px = wrist_2d_norm[1] * h
                local_hand_x = ((u_px - cx_img) * local_hand_z) / focal_length
                local_hand_y = ((v_px - cy_img) * local_hand_z) / focal_length
                
                # Map to Dobot coordinates
                local_dobot_y = local_hand_x + BASE_OFFSET_Y
                local_dobot_x = BASE_OFFSET_X - local_hand_y
                local_delta_z = local_hand_z - z_table

                # Check if calibration was triggered via web interface
                with state_lock:
                    if trigger_calibration:
                        z_table = local_hand_z
                        trigger_calibration = False
                        print(f"[CALIBRATION] Web UI calibration: Height locked at {z_table:.1f} mm")

                # Boundary containment check
                local_is_inside = is_inside_polygon(local_dobot_x, local_dobot_y, WORKSPACE_POLYGON)

                # Draw hand landmarks
                landmarks_px = []
                for lm in hand_data.landmarks:
                    cx_lm, cy_lm = int(lm[0] * w), int(lm[1] * h)
                    landmarks_px.append((cx_lm, cy_lm))
                    cv2.circle(frame, (cx_lm, cy_lm), 4, (0, 165, 255), -1, cv2.LINE_AA) # Amber joints

                # Skeleton connections
                connections = [
                    (0, 1), (1, 2), (2, 3), (3, 4),      # Thumb
                    (0, 5), (5, 6), (6, 7), (7, 8),      # Index
                    (0, 9), (9, 10), (10, 11), (11, 12), # Middle
                    (0, 13), (13, 14), (14, 15), (15, 16),# Ring
                    (0, 17), (17, 18), (18, 19), (19, 20),# Pinky
                    (5, 9), (9, 13), (13, 17)            # Palm base
                ]
                for start_idx, end_idx in connections:
                    cv2.line(frame, landmarks_px[start_idx], landmarks_px[end_idx], (255, 128, 0), 2, cv2.LINE_AA)

                # Send hand open/closed command to UDP server on state changes
                local_is_open = hand_data.is_open
                if last_hand_open_state is None or last_hand_open_state != local_is_open:
                    last_hand_open_state = local_is_open
                    cmd_msg = b"OPEN" if local_is_open else b"CLOSE"
                    try:
                        udp_sock.sendto(cmd_msg, udp_server_addr)
                    except Exception as e:
                        print(f"Error sending gripper command: {e}")

                # Send coordinate move commands under thresholds (150ms / 10mm XY / 5mm Z)
                if local_is_inside:
                    current_time = time.time()
                    if current_time - last_move_time >= 0.15:
                        dist_moved = np.hypot(local_dobot_x - last_sent_x, local_dobot_y - last_sent_y)
                        z_diff = abs(local_delta_z - last_sent_z)
                        
                        if dist_moved >= 10.0 or z_diff >= 5.0:
                            cmd_msg = f"MOVE:{local_dobot_x:.2f},{local_dobot_y:.2f},{local_delta_z:.2f}".encode('utf-8')
                            try:
                                udp_sock.sendto(cmd_msg, udp_server_addr)
                                last_move_time = current_time
                                last_sent_x = local_dobot_x
                                last_sent_y = local_dobot_y
                                last_sent_z = local_delta_z
                            except Exception as e:
                                print(f"Error sending coordinate command: {e}")

                # Draw hand wrist status dot
                wrist_u, wrist_v = landmarks_px[0]
                dot_color = (0, 255, 0) if local_is_inside else (0, 0, 255)
                cv2.circle(frame, (wrist_u, wrist_v), 7, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.circle(frame, (wrist_u, wrist_v), 5, dot_color, -1, cv2.LINE_AA)

            # Draw Workspace Overlay HUD on Frame
            overlay = frame.copy()
            hud_color = (255, 200, 100) if local_is_inside else (0, 0, 255)
            edge_color = (255, 170, 70) if local_is_inside else (0, 0, 220)
            
            cv2.fillPoly(overlay, [arc_pts], hud_color)
            cv2.addWeighted(overlay, 0.15, frame, 0.85, 0, frame)
            cv2.polylines(frame, [arc_pts], True, edge_color, 2, cv2.LINE_AA)
            
            origin_u, origin_v = dobot_to_screen(0, 0, w, h, focal_length, z_table)
            cv2.line(frame, (origin_u, origin_v), tuple(arc_pts[0]), edge_color, 1, cv2.LINE_AA)
            cv2.line(frame, (origin_u, origin_v), tuple(arc_pts[-1]), edge_color, 1, cv2.LINE_AA)
            cv2.circle(frame, (origin_u, origin_v), 5, edge_color, -1, cv2.LINE_AA)
            cv2.putText(frame, "Base", (origin_u - 15, origin_v + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, edge_color, 1, cv2.LINE_AA)

            # Capture external camera frame if active
            frame_ext_resized = None
            if external_active:
                ret_eff, frame_eff = cap_external.read()
                if ret_eff and frame_eff is not None:
                    # Keep aspect ratio but downscale for web streaming efficiency
                    frame_ext_resized = cv2.resize(frame_eff, (320, 240))

            # Safely write variables to the shared web state
            with state_lock:
                latest_tracking_frame = frame.copy()
                if frame_ext_resized is not None:
                    latest_external_frame = frame_ext_resized.copy()
                hand_x = local_hand_x
                hand_y = local_hand_y
                hand_z = local_hand_z
                dobot_x = local_dobot_x
                dobot_y = local_dobot_y
                delta_z = local_delta_z
                is_inside = local_is_inside
                is_open = last_hand_open_state if last_hand_open_state is not None else True

            # Yield thread execution briefly to preserve CPU cycles
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        print("\nReleasing resources and shutting down...")
        running = False
        cap.release()
        if external_active:
            cap_external.release()
        tracker.close()
        cv2.destroyAllWindows()
        print("Shutdown complete.")

if __name__ == "__main__":
    main()
