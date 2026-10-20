# vision/hand_tracker.py
import cv2
import mediapipe as mp
import numpy as np
from dataclasses import dataclass
from typing import Optional

@dataclass
class HandData:
    """
    Structured object containing hand tracking results.
    Treat this as sensor readings.
    """
    wrist_pos: np.ndarray        # [x, y, z] in MediaPipe normalized coordinate space
    palm_center: np.ndarray      # [x, y, z] computed center of palm
    palm_normal: np.ndarray      # [nx, ny, nz] normalized normal vector of the palm
    landmarks: np.ndarray        # [21, 3] raw landmarks in MediaPipe space
    is_open: bool                # True if hand is open, False if closed
    hand_type: str               # "Left" or "Right"

class HandTracker:
    """
    Responsible only for MediaPipe hand tracking execution and parameter extraction.
    No drawing or workspace logic.
    """
    def __init__(self, min_detection_confidence=0.7, min_tracking_confidence=0.5):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=1,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )

    def process_frame(self, frame: np.ndarray) -> Optional[HandData]:
        """
        Processes a BGR image frame to track the hand.
        Returns HandData if a hand is detected, otherwise None.
        """
        # Convert image to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        if not results.multi_hand_landmarks:
            return None

        # Process the primary hand detected
        landmarks_proto = results.multi_hand_landmarks[0]
        hand_class = results.multi_handedness[0].classification[0]
        hand_type = hand_class.label  # "Left" or "Right"

        # Convert landmarks to numpy array [21, 3]
        landmarks = np.array([[lm.x, lm.y, lm.z] for lm in landmarks_proto.landmark])

        # Extract specific landmarks
        wrist = landmarks[0]
        index_mcp = landmarks[5]
        middle_mcp = landmarks[9]
        pinky_mcp = landmarks[17]

        # Calculate Palm Center as average of wrist, index MCP, and pinky MCP
        palm_center = (wrist + index_mcp + pinky_mcp) / 3.0

        # Calculate Palm Normal (Orientation)
        v_dir = middle_mcp - wrist
        v_side = pinky_mcp - index_mcp
        
        # Cross product to get the palm normal vector
        if hand_type == "Right":
            palm_normal = np.cross(v_side, v_dir)
        else:
            palm_normal = np.cross(v_dir, v_side)
            
        norm = np.linalg.norm(palm_normal)
        if norm > 0:
            palm_normal = palm_normal / norm

        # Hand Open/Closed State Heuristic
        extended_fingers = 0
        fingers = [
            (8, 6),   # Index
            (12, 10), # Middle
            (16, 14), # Ring
            (20, 18)  # Pinky
        ]
        
        for tip_idx, pip_idx in fingers:
            tip_pos = landmarks[tip_idx]
            pip_pos = landmarks[pip_idx]
            
            dist_to_tip = np.linalg.norm(tip_pos - wrist)
            dist_to_pip = np.linalg.norm(pip_pos - wrist)
            
            if dist_to_tip > dist_to_pip:
                extended_fingers += 1

        is_open = extended_fingers >= 3

        return HandData(
            wrist_pos=wrist,
            palm_center=palm_center,
            palm_normal=palm_normal,
            landmarks=landmarks,
            is_open=is_open,
            hand_type=hand_type
        )

    def close(self):
        """Clean up MediaPipe resources."""
        self.hands.close()
