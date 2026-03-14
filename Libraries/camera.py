#!/usr/bin/env python3
import cv2
import numpy as np
import time
from typing import Optional, Tuple

class DroneDetector:
    """Detects drone using combined background subtraction and LED detection"""
    
    def __init__(self, history=1000, min_area=1000, max_area=200000):
        """
        Initialize drone detector
        
        Args:
            history: Number of frames for background model
            min_area: Minimum contour area to consider
            max_area: Maximum contour area to consider
        """
        self.backSub = cv2.createBackgroundSubtractorMOG2(history=history)
        self.min_area = min_area
        self.max_area = max_area
        
        # Morphological kernel
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        
        # LED detection parameters (green LED)
        self.led_lower = np.array([40, 100, 200])
        self.led_upper = np.array([80, 255, 255])
    
    def detect_led_mask(self, frame):
        """
        Detect LED using HSV color filtering
        
        Args:
            frame: BGR image
        
        Returns:
            Binary mask of LED regions
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        led_mask = cv2.inRange(hsv, self.led_lower, self.led_upper)
        return led_mask
    
    def detect_motion_mask(self, frame):
        """
        Detect motion using background subtraction
        
        Args:
            frame: BGR image
        
        Returns:
            Binary mask of moving regions
        """
        # Apply background subtraction
        blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)
        fg_mask = self.backSub.apply(blurred_frame)
        
        # Additional filtering
        retval, mask_thresh = cv2.threshold(fg_mask, 180, 255, cv2.THRESH_BINARY)
        
        # Morphological operations
        mask_eroded = cv2.morphologyEx(mask_thresh, cv2.MORPH_OPEN, self.kernel)
        mask_dilated = cv2.dilate(mask_eroded, self.kernel, iterations=1)
        
        return mask_dilated
    
    def detect(self, frame) -> Optional[Tuple[float, float, float, float]]:
        """
        Detect drone in frame using combined motion and LED detection
        
        Args:
            frame: BGR image from camera
        
        Returns:
            (x, y, w, h) bounding box center and size, or None if not detected
        """
        # Get both masks
        motion_mask = self.detect_motion_mask(frame)
        led_mask = self.detect_led_mask(frame)
        
        # Find contours in motion mask
        contours, _ = cv2.findContours(
            motion_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Filter by area
        large_contours = [
            cnt for cnt in contours 
            if self.min_area < cv2.contourArea(cnt) < self.max_area
        ]
        
        if not large_contours:
            return None
        
        # Check each contour for LED presence
        for cnt in large_contours:
            x, y, w, h = cv2.boundingRect(cnt)
            
            # Check if LED is present in this region
            roi = led_mask[y:y+h, x:x+w]
            if cv2.countNonZero(roi) > 0:
                # Found drone with LED!
                center_x = x + w / 2.0
                center_y = y + h / 2.0
                return (center_x, center_y, float(w), float(h))
        
        # No contour with LED found
        return None


class DualCameraTracker:
    """Dual camera tracking system for 3D position with improved detection"""
    
    def __init__(self, camera1_id=0, camera2_id=1, cage_size=1.5):
        """
        Initialize dual camera tracker
        
        Args:
            camera1_id: Device ID for front camera
            camera2_id: Device ID for side camera
            cage_size: Size of cage in meters (assumes cubic)
        """
        # Open cameras
        self.cap1 = cv2.VideoCapture(camera1_id)
        self.cap2 = cv2.VideoCapture(camera2_id)
        
        if not self.cap1.isOpened():
            raise RuntimeError(f"Could not open camera {camera1_id}")
        if not self.cap2.isOpened():
            raise RuntimeError(f"Could not open camera {camera2_id}")
        
        # Reduce buffer for lower latency
        self.cap1.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap2.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Initialize detectors (separate background models for each camera)
        self.detector1 = DroneDetector()
        self.detector2 = DroneDetector()
        
        self.cage_size = cage_size
        
        # Get image dimensions
        ret, frame = self.cap1.read()
        if ret:
            self.img_height, self.img_width = frame.shape[:2]
        else:
            raise RuntimeError("Could not read from camera 1")
        
        # Current state
        self.position_3d = None
        self.last_detection1 = None
        self.last_detection2 = None
        self.last_frame1 = None
        self.last_frame2 = None
        
        print(f"Dual camera tracker initialized")
        print(f"  Camera 1 (Front): Device {camera1_id}")
        print(f"  Camera 2 (Side): Device {camera2_id}")
        print(f"  Image size: {self.img_width}x{self.img_height}")
        print(f"  Cage size: {cage_size}m")
        print(f"  Detection: Motion + LED (green)")
    
    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Capture frames from both cameras
        
        Returns:
            (frame1, frame2) or (None, None) if capture fails
        """
        ret1, frame1 = self.cap1.read()
        ret2, frame2 = self.cap2.read()
        
        if ret1 and ret2:
            self.last_frame1 = frame1
            self.last_frame2 = frame2
            return frame1, frame2
        return None, None
    
    def compute_3d_position(
        self, 
        detection1: Optional[Tuple[float, float, float, float]], 
        detection2: Optional[Tuple[float, float, float, float]]
    ) -> Optional[Tuple[float, float, float]]:
        """
        Compute 3D position from two detections using simplified triangulation
        
        Args:
            detection1: (x, y, w, h) from front camera
            detection2: (x, y, w, h) from side camera
        
        Returns:
            (X, Y, Z) in meters relative to cage center, or None
        """
        if detection1 is None or detection2 is None:
            return None
        
        x1, y1, w1, h1 = detection1
        x2, y2, w2, h2 = detection2
        
        # Convert pixel coordinates to normalized coordinates [-0.5, 0.5]
        u1 = (x1 / self.img_width) - 0.5
        v1 = 0.5 - (y1 / self.img_height)  # Flip Y axis (image origin is top-left)
        
        u2 = (x2 / self.img_width) - 0.5
        v2 = 0.5 - (y2 / self.img_height)
        
        # Map to world coordinates
        # Front camera (Camera 1) gives X and Y
        X = u1 * self.cage_size
        Y1 = v1 * self.cage_size
        
        # Side camera (Camera 2) gives Z and Y
        Z = u2 * self.cage_size
        Y2 = v2 * self.cage_size
        
        # Average Y coordinates for robustness
        Y = (Y1 + Y2) / 2.0
        
        return (X, Y, Z)
    
    def track_once(self) -> Optional[Tuple[float, float, float]]:
        """
        Single tracking iteration
        
        Returns:
            (X, Y, Z) position or None
        """
        # Get frames
        frame1, frame2 = self.get_frames()
        
        if frame1 is None or frame2 is None:
            return None
        
        # Detect in both frames
        det1 = self.detector1.detect(frame1)
        det2 = self.detector2.detect(frame2)
        
        # Store detections for visualization
        self.last_detection1 = det1
        self.last_detection2 = det2
        
        # Compute 3D position
        position = self.compute_3d_position(det1, det2)
        self.position_3d = position
        
        return position
    
    def get_position(self) -> Optional[Tuple[float, float, float]]:
        """Get current 3D position"""
        return self.position_3d
    
    def get_last_detections(self) -> Tuple[Optional[Tuple], Optional[Tuple]]:
        """Get last detections from both cameras"""
        return self.last_detection1, self.last_detection2
    
    def get_last_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get last captured frames"""
        return self.last_frame1, self.last_frame2
    
    def release(self):
        """Release camera resources"""
        self.cap1.release()
        self.cap2.release()
        print("Cameras released")


# Legacy single camera detection function for backward compatibility
cap = None
backSub = None

def init_single_camera(camera_id=0):
    """Initialize single camera for legacy code"""
    global cap, backSub
    cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)
    backSub = cv2.createBackgroundSubtractorMOG2(history=1000)
    
    if not cap.isOpened():
        print("Error: Could not open video device.")
        exit(1)
    
    ret, frame = cap.read()
    return ret

def detect_led(frame):
    """Legacy LED detection function"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([40, 100, 200])  
    upper = np.array([80, 255, 255])
    led_mask = cv2.inRange(hsv, lower, upper)
    return led_mask

def detect_mask(frame):
    """Legacy motion mask detection function"""
    global backSub
    blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)
    fg_mask = backSub.apply(blurred_frame)
    retval, mask_thresh = cv2.threshold(fg_mask, 180, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask_eroded = cv2.morphologyEx(mask_thresh, cv2.MORPH_OPEN, kernel)
    mask_dialated = cv2.dilate(mask_eroded, kernel, iterations=1)
    return mask_dialated

def detect_drone():
    """Legacy drone detection function"""
    global cap
    ret, frame = cap.read()
    if not ret:
        return

    motion_mask = detect_mask(frame)
    led_mask = detect_led(frame)

    contours, _ = cv2.findContours(motion_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    min_contour_area = 1000
    max_contour_area = 200000
    large_contours = [cnt for cnt in contours if min_contour_area < cv2.contourArea(cnt) < max_contour_area]

    frame_out = frame.copy()
    for cnt in large_contours:
        x, y, w, h = cv2.boundingRect(cnt)

        roi = led_mask[y:y+h, x:x+w]
        if cv2.countNonZero(roi) > 0:
            cv2.rectangle(frame_out, (x, y), (x+w, y+h), (0, 0, 200), 3)

    cv2.imshow('Frame_final', frame_out)
    cv2.waitKey(1)


# Simple test when run directly
if __name__ == "__main__":
    print("Testing dual camera tracker with LED detection...")
    print("Press 'q' to quit")
    
    try:
        tracker = DualCameraTracker(camera1_id=0, camera2_id=1, cage_size=1.5)
        
        while True:
            position = tracker.track_once()
            
            if position:
                X, Y, Z = position
                print(f"\rPosition: X={X:+.3f}m, Y={Y:+.3f}m, Z={Z:+.3f}m", end='')
            else:
                print("\rDrone not detected in both cameras", end='')
            
            # Small delay
            time.sleep(0.033)  # ~30 FPS
            
            # Check for quit (this won't work without cv2.imshow, but kept for structure)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        tracker.release()
