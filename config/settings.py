# config/settings.py
import numpy as np

# Camera settings
CAMERA_INDEX = 1
# We default to 640x480 for laptop webcams to ensure compatibility and low latency.
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# Camera Field of View (FOV) in degrees.
# - Laptop webcam: ~70.0 degrees
CAMERA_FOV = 70.0

# Reference physical size of the hand feature (wrist to middle MCP joint in mm)
# Used to scale the pixel distance to calculate Z coordinate in mm.
HAND_PHYSICAL_SIZE = 80.0

# MediaPipe Hand Tracker settings
MIN_DETECTION_CONFIDENCE = 0.7
MIN_TRACKING_CONFIDENCE = 0.5

# Virtual Workspace Dimensions (3D Polygon Prism in millimeters)
# - X axis (horizontal): mapped to Dobot Y (left/right)
# - Y axis (vertical): height direction (positive down, negative up)
# - Z axis (depth): mapped to Dobot X (distance from base/camera)
WORKSPACE_HEIGHT = 100.0   # 10 cm height depth (mm)
DEFAULT_Z_BASE = 450.0     # Default table surface depth from camera (mm)

# 2D Polygon Vertices in (X_cam, Z_cam) plane (derived from physical Dobot trajectory points)
WORKSPACE_POLYGON = [
    [-272.45, 153.98],
    [-203.22, 295.44],
    [-36.32, 358.58],
    [143.40, 337.29],
    [275.21, 158.42]
]

# Translucent face opacities for visual shading
WORKSPACE_ALPHA = 0.10             # Fully shaded default opacity
WORKSPACE_WARNING_ALPHA = 0.20     # Red warning shading opacity

# Display configuration flags
SHOW_HAND_LANDMARKS = True
SHOW_WORKSPACE = True
SHOW_DIAGNOSTICS = True

# Colors (BGR Format for OpenCV)
COLOR_TEXT = (255, 255, 255)
COLOR_TEXT_BG = (30, 30, 30)
COLOR_VIRTUAL_TCP = (0, 0, 255)        # Bright Red for wrist TCP dot
COLOR_WORKSPACE_INSIDE = (255, 200, 100)  # Pale blue overlay
COLOR_WORKSPACE_OUTSIDE = (255, 200, 100) # Pale blue overlay
COLOR_HAND_LANDMARKS = (255, 128, 0)   # Orange skeleton lines
COLOR_HAND_JOINTS = (0, 165, 255)      # Amber joints
COLOR_GRID_LINES = (255, 170, 70)      # Pale blue lines for workspace reference
COLOR_WAYPOINT = (0, 165, 255)         # Orange (BGR) for verified waypoints
COLOR_ROBOT = (0, 255, 0)              # Green (BGR) for simulated robot position

# Waypoint generation & Robot simulation settings
SMOOTHING_ALPHA = 0.3
WAYPOINT_THRESHOLD = 30.0              # Distance threshold in mm
SIMULATED_ROBOT_SPEED = 120.0          # Speed in mm/s
