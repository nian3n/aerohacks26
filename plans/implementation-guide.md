# Implementation Guide
## Step-by-Step Instructions for 3D Drone Tracking

---

## Quick Start Implementation Path

### Phase 1: Basic Dual Camera Setup (Simplified Approach)

Start with a simplified implementation that doesn't require full camera calibration. This gets you working quickly and can be refined later.

#### Step 1: Test Dual Camera Capture

```python
# test_dual_cameras.py
import cv2
import numpy as np

# Open both cameras
cap1 = cv2.VideoCapture(0)  # Front camera
cap2 = cv2.VideoCapture(1)  # Side camera

while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()
    
    if ret1 and ret2:
        # Display both views
        combined = np.hstack([frame1, frame2])
        cv2.imshow('Front (Left) | Side (Right)', combined)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()
```

#### Step 2: Extend Existing Detection to Both Cameras

```python
# Libraries/camera.py - Enhanced version
import cv2
import numpy as np
import threading
import queue
from dataclasses import dataclass
from typing import Optional, Tuple
import time

@dataclass
class Detection:
    """Container for detection results"""
    timestamp: float
    camera_id: int
    x: float
    y: float
    radius: float
    confidence: float

class DroneDetector:
    """Detects drone using color-based segmentation"""
    
    def __init__(self, hsv_lower=(40, 50, 50), hsv_upper=(90, 255, 255)):
        self.hsv_lower = np.array(hsv_lower)
        self.hsv_upper = np.array(hsv_upper)
        self.min_area = 100
    
    def detect(self, frame):
        """
        Detect drone in frame
        
        Returns:
            (x, y, radius) or None
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        
        # Noise reduction
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        largest = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(largest) < self.min_area:
            return None
        
        (x, y), radius = cv2.minEnclosingCircle(largest)
        
        return (float(x), float(y), float(radius))

class DualCameraTracker:
    """Simplified dual camera tracking system"""
    
    def __init__(self, camera1_id=0, camera2_id=1, cage_size=1.5):
        self.cap1 = cv2.VideoCapture(camera1_id)
        self.cap2 = cv2.VideoCapture(camera2_id)
        
        # Reduce buffer for lower latency
        self.cap1.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap2.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        self.detector = DroneDetector()
        self.cage_size = cage_size
        
        # Current state
        self.position_3d = None
        self.running = False
        
        # Get image dimensions
        ret, frame = self.cap1.read()
        if ret:
            self.img_height, self.img_width = frame.shape[:2]
    
    def get_frames(self):
        """Capture frames from both cameras"""
        ret1, frame1 = self.cap1.read()
        ret2, frame2 = self.cap2.read()
        
        if ret1 and ret2:
            return frame1, frame2
        return None, None
    
    def compute_3d_position(self, detection1, detection2):
        """
        Compute 3D position from two detections
        
        Args:
            detection1: (x, y, radius) from front camera
            detection2: (x, y, radius) from side camera
        
        Returns:
            (X, Y, Z) in meters relative to cage center
        """
        if detection1 is None or detection2 is None:
            return None
        
        x1, y1, r1 = detection1
        x2, y2, r2 = detection2
        
        # Convert pixel coordinates to normalized coordinates [-0.5, 0.5]
        u1 = (x1 / self.img_width) - 0.5
        v1 = 0.5 - (y1 / self.img_height)  # Flip Y axis
        
        u2 = (x2 / self.img_width) - 0.5
        v2 = 0.5 - (y2 / self.img_height)
        
        # Map to world coordinates
        # Front camera gives X and Y
        X = u1 * self.cage_size
        Y1 = v1 * self.cage_size
        
        # Side camera gives Z and Y
        Z = u2 * self.cage_size
        Y2 = v2 * self.cage_size
        
        # Average Y for robustness
        Y = (Y1 + Y2) / 2.0
        
        return (X, Y, Z)
    
    def track_once(self):
        """Single tracking iteration"""
        frame1, frame2 = self.get_frames()
        
        if frame1 is None or frame2 is None:
            return None
        
        # Detect in both frames
        det1 = self.detector.detect(frame1)
        det2 = self.detector.detect(frame2)
        
        # Compute 3D position
        position = self.compute_3d_position(det1, det2)
        self.position_3d = position
        
        return position
    
    def get_position(self):
        """Get current 3D position"""
        return self.position_3d
    
    def release(self):
        """Release camera resources"""
        self.cap1.release()
        self.cap2.release()

# Simple test
if __name__ == "__main__":
    tracker = DualCameraTracker(camera1_id=0, camera2_id=1, cage_size=1.5)
    
    try:
        while True:
            position = tracker.track_once()
            
            if position:
                X, Y, Z = position
                print(f"Position: X={X:.3f}m, Y={Y:.3f}m, Z={Z:.3f}m")
            else:
                print("Drone not detected")
            
            # Small delay
            time.sleep(0.033)  # ~30 FPS
            
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        tracker.release()
```

#### Step 3: Add Kalman Filtering

```python
# Libraries/position.py
import numpy as np

class KalmanFilter3D:
    """Simple 3D Kalman filter for position smoothing"""
    
    def __init__(self, dt=0.033):
        """
        Initialize Kalman filter
        
        Args:
            dt: Time step in seconds (default 30 FPS)
        """
        self.dt = dt
        self.initialized = False
        
        # State: [x, y, z, vx, vy, vz]
        self.state = np.zeros(6)
        
        # Covariance matrix
        self.P = np.eye(6) * 1.0
        
        # State transition (constant velocity model)
        self.F = np.array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1]
        ])
        
        # Measurement matrix (measure position only)
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
        
        # Process noise
        self.Q = np.eye(6) * 0.01
        self.Q[3:, 3:] *= 2  # Higher noise for velocity
        
        # Measurement noise
        self.R = np.eye(3) * 0.05
    
    def update(self, measurement):
        """
        Update filter with new measurement
        
        Args:
            measurement: (x, y, z) or None
        
        Returns:
            (position, velocity) tuple
        """
        if measurement is None:
            # No measurement, just predict
            return self.predict()
        
        z = np.array(measurement)
        
        if not self.initialized:
            # Initialize with first measurement
            self.state[:3] = z
            self.state[3:] = 0
            self.initialized = True
            return self.state[:3], self.state[3:]
        
        # Predict step
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        # Update step
        y = z - self.H @ self.state  # Innovation
        S = self.H @ self.P @ self.H.T + self.R  # Innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain
        
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
        
        return self.state[:3], self.state[3:]
    
    def predict(self):
        """Predict next state without measurement"""
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.state[:3], self.state[3:]
    
    def get_position(self):
        """Get current position estimate"""
        return self.state[:3]
    
    def get_velocity(self):
        """Get current velocity estimate"""
        return self.state[3:]
    
    def reset(self):
        """Reset filter"""
        self.initialized = False
        self.state = np.zeros(6)
        self.P = np.eye(6) * 1.0
```

#### Step 4: Integrate with Position Controller

```python
# Libraries/position.py (continued)

class PositionTracker:
    """Complete position tracking with filtering"""
    
    def __init__(self, camera1_id=0, camera2_id=1, cage_size=1.5):
        from Libraries.camera import DualCameraTracker
        
        self.tracker = DualCameraTracker(camera1_id, camera2_id, cage_size)
        self.kalman = KalmanFilter3D(dt=0.033)
        
        self.position = None
        self.velocity = None
        self.target = np.array([0.0, 0.0, 0.0])  # Center of cage
    
    def update(self):
        """Update position estimate"""
        # Get raw 3D position
        raw_position = self.tracker.track_once()
        
        # Filter with Kalman
        self.position, self.velocity = self.kalman.update(raw_position)
        
        return self.position, self.velocity
    
    def get_position(self):
        """Get current filtered position"""
        return self.position
    
    def get_velocity(self):
        """Get current velocity estimate"""
        return self.velocity
    
    def get_error(self):
        """Get position error from target"""
        if self.position is None:
            return None
        return self.target - self.position
    
    def set_target(self, x, y, z):
        """Set target position"""
        self.target = np.array([x, y, z])
    
    def release(self):
        """Release resources"""
        self.tracker.release()
```

---

## Phase 2: Camera Calibration (For Better Accuracy)

Once the basic system is working, add proper camera calibration for improved accuracy.

### Calibration Procedure

#### Step 1: Capture Calibration Images

```python
# calibration/capture_images.py
import cv2
import os

def capture_calibration_images(camera_id, output_dir, num_images=20):
    """
    Capture checkerboard images for calibration
    
    Args:
        camera_id: Camera index
        output_dir: Directory to save images
        num_images: Number of images to capture
    """
    os.makedirs(output_dir, exist_ok=True)
    
    cap = cv2.VideoCapture(camera_id)
    count = 0
    
    print(f"Capturing calibration images for camera {camera_id}")
    print("Press SPACE to capture, ESC to exit")
    
    while count < num_images:
        ret, frame = cap.read()
        if not ret:
            continue
        
        # Display
        cv2.putText(frame, f"Captured: {count}/{num_images}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Calibration', frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord(' '):  # Space to capture
            filename = os.path.join(output_dir, f"calib_{count:02d}.jpg")
            cv2.imwrite(filename, frame)
            print(f"Saved {filename}")
            count += 1
        elif key == 27:  # ESC to exit
            break
    
    cap.release()
    cv2.destroyAllWindows()
    print(f"Captured {count} images")

# Run for both cameras
if __name__ == "__main__":
    capture_calibration_images(0, "calibration/camera1", num_images=20)
    capture_calibration_images(1, "calibration/camera2", num_images=20)
```

#### Step 2: Perform Calibration

```python
# calibration/calibrate.py
import cv2
import numpy as np
import glob
import pickle

def calibrate_camera(image_dir, checkerboard_size=(9, 6), square_size=0.025):
    """
    Calibrate camera from checkerboard images
    
    Args:
        image_dir: Directory with calibration images
        checkerboard_size: Inner corners (width, height)
        square_size: Size of checkerboard square in meters
    
    Returns:
        camera_matrix, dist_coeffs, rvecs, tvecs
    """
    # Prepare object points
    objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard_size[0], 
                            0:checkerboard_size[1]].T.reshape(-1, 2)
    objp *= square_size
    
    objpoints = []  # 3D points
    imgpoints = []  # 2D points
    
    images = glob.glob(f"{image_dir}/*.jpg")
    
    print(f"Processing {len(images)} images...")
    
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Find checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)
        
        if ret:
            objpoints.append(objp)
            
            # Refine corner positions
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            
            print(f"✓ {fname}")
        else:
            print(f"✗ {fname} - corners not found")
    
    if len(objpoints) == 0:
        print("ERROR: No valid calibration images found!")
        return None
    
    print(f"\nCalibrating with {len(objpoints)} images...")
    
    # Calibrate
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    
    print(f"Calibration RMS error: {ret:.4f}")
    print(f"\nCamera Matrix:\n{camera_matrix}")
    print(f"\nDistortion Coefficients:\n{dist_coeffs}")
    
    return camera_matrix, dist_coeffs, rvecs, tvecs

def save_calibration(filename, camera_matrix, dist_coeffs):
    """Save calibration data"""
    data = {
        'camera_matrix': camera_matrix,
        'dist_coeffs': dist_coeffs
    }
    with open(filename, 'wb') as f:
        pickle.dump(data, f)
    print(f"Saved calibration to {filename}")

def load_calibration(filename):
    """Load calibration data"""
    with open(filename, 'rb') as f:
        data = pickle.load(f)
    return data['camera_matrix'], data['dist_coeffs']

# Run calibration
if __name__ == "__main__":
    # Calibrate camera 1
    K1, dist1, _, _ = calibrate_camera("calibration/camera1")
    if K1 is not None:
        save_calibration("calibration/camera1_calib.pkl", K1, dist1)
    
    # Calibrate camera 2
    K2, dist2, _, _ = calibrate_camera("calibration/camera2")
    if K2 is not None:
        save_calibration("calibration/camera2_calib.pkl", K2, dist2)
```

---

## Phase 3: Threading and Optimization

### Multi-threaded Architecture

```python
# Libraries/camera.py (threaded version)
import threading
import queue
import time

class ThreadedDualCameraTracker:
    """Multi-threaded version for better performance"""
    
    def __init__(self, camera1_id=0, camera2_id=1, cage_size=1.5):
        self.cap1 = cv2.VideoCapture(camera1_id)
        self.cap2 = cv2.VideoCapture(camera2_id)
        
        self.cap1.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap2.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        self.detector = DroneDetector()
        self.cage_size = cage_size
        
        # Queues for inter-thread communication
        self.frame_queue1 = queue.Queue(maxsize=2)
        self.frame_queue2 = queue.Queue(maxsize=2)
        self.detection_queue = queue.Queue(maxsize=2)
        
        # State
        self.position_3d = None
        self.running = False
        self.threads = []
        
        # Get image dimensions
        ret, frame = self.cap1.read()
        if ret:
            self.img_height, self.img_width = frame.shape[:2]
    
    def start(self):
        """Start all threads"""
        self.running = True
        
        # Camera capture threads
        t1 = threading.Thread(target=self._capture_loop, args=(self.cap1, self.frame_queue1))
        t2 = threading.Thread(target=self._capture_loop, args=(self.cap2, self.frame_queue2))
        
        # Detection thread
        t3 = threading.Thread(target=self._detection_loop)
        
        # Position computation thread
        t4 = threading.Thread(target=self._position_loop)
        
        for t in [t1, t2, t3, t4]:
            t.daemon = True
            t.start()
            self.threads.append(t)
    
    def _capture_loop(self, cap, frame_queue):
        """Capture frames continuously"""
        while self.running:
            ret, frame = cap.read()
            if ret:
                try:
                    frame_queue.put_nowait((time.time(), frame))
                except queue.Full:
                    try:
                        frame_queue.get_nowait()
                        frame_queue.put_nowait((time.time(), frame))
                    except:
                        pass
    
    def _detection_loop(self):
        """Detect drone in frames"""
        while self.running:
            try:
                ts1, frame1 = self.frame_queue1.get(timeout=0.1)
                ts2, frame2 = self.frame_queue2.get(timeout=0.1)
                
                # Detect in both frames
                det1 = self.detector.detect(frame1)
                det2 = self.detector.detect(frame2)
                
                # Put detections in queue
                try:
                    self.detection_queue.put_nowait((det1, det2))
                except queue.Full:
                    try:
                        self.detection_queue.get_nowait()
                        self.detection_queue.put_nowait((det1, det2))
                    except:
                        pass
            except queue.Empty:
                continue
    
    def _position_loop(self):
        """Compute 3D position"""
        while self.running:
            try:
                det1, det2 = self.detection_queue.get(timeout=0.1)
                position = self.compute_3d_position(det1, det2)
                self.position_3d = position
            except queue.Empty:
                continue
    
    def compute_3d_position(self, detection1, detection2):
        """Compute 3D position from detections"""
        if detection1 is None or detection2 is None:
            return None
        
        x1, y1, r1 = detection1
        x2, y2, r2 = detection2
        
        u1 = (x1 / self.img_width) - 0.5
        v1 = 0.5 - (y1 / self.img_height)
        
        u2 = (x2 / self.img_width) - 0.5
        v2 = 0.5 - (y2 / self.img_height)
        
        X = u1 * self.cage_size
        Y1 = v1 * self.cage_size
        Z = u2 * self.cage_size
        Y2 = v2 * self.cage_size
        
        Y = (Y1 + Y2) / 2.0
        
        return (X, Y, Z)
    
    def get_position(self):
        """Get current 3D position"""
        return self.position_3d
    
    def stop(self):
        """Stop all threads"""
        self.running = False
        for t in self.threads:
            t.join(timeout=1.0)
        self.cap1.release()
        self.cap2.release()
```

---

## Testing and Validation

### Validation Tests

```python
# tests/test_tracking.py
import numpy as np
from Libraries.position import PositionTracker
import time

def test_static_position():
    """Test tracking of stationary drone"""
    tracker = PositionTracker(camera1_id=0, camera2_id=1, cage_size=1.5)
    
    positions = []
    
    print("Testing static position tracking for 5 seconds...")
    start_time = time.time()
    
    while time.time() - start_time < 5.0:
        pos, vel = tracker.update()
        if pos is not None:
            positions.append(pos)
        time.sleep(0.033)
    
    if len(positions) > 0:
        positions = np.array(positions)
        mean_pos = np.mean(positions, axis=0)
        std_pos = np.std(positions, axis=0)
        
        print(f"Mean position: {mean_pos}")
        print(f"Std deviation: {std_pos}")
        print(f"Max deviation: {np.max(std_pos):.4f}m")
    
    tracker.release()

def test_detection_rate():
    """Test detection success rate"""
    tracker = PositionTracker(camera1_id=0, camera2_id=1, cage_size=1.5)
    
    total = 0
    detected = 0
    
    print("Testing detection rate for 10 seconds...")
    start_time = time.time()
    
    while time.time() - start_time < 10.0:
        pos, vel = tracker.update()
        total += 1
        if pos is not None:
            detected += 1
        time.sleep(0.033)
    
    rate = (detected / total) * 100 if total > 0 else 0
    print(f"Detection rate: {rate:.1f}% ({detected}/{total})")
    
    tracker.release()

if __name__ == "__main__":
    test_static_position()
    test_detection_rate()
```

---

## Troubleshooting Guide

### Common Issues and Solutions

#### 1. Cameras Not Detected
- Check camera IDs with `ls /dev/video*`
- Try different IDs: 0, 1, 2, etc.
- Ensure cameras have permissions

#### 2. Poor Detection
- Adjust HSV color range for your drone
- Improve lighting conditions
- Reduce background clutter
- Increase `min_area` threshold

#### 3. Noisy Position Estimates
- Increase Kalman filter measurement noise (R matrix)
- Decrease process noise (Q matrix)
- Add more morphological operations to mask

#### 4. Low Frame Rate
- Reduce image resolution
- Use threaded implementation
- Optimize detection algorithm
- Check CPU usage

#### 5. Synchronization Issues
- Increase `max_time_diff` tolerance
- Use hardware-synchronized cameras if available
- Reduce camera buffer size

---

## Next Steps

1. **Start with Phase 1** - Get basic tracking working
2. **Test and tune** - Adjust HSV ranges, Kalman parameters
3. **Add calibration** - Implement Phase 2 for better accuracy
4. **Optimize** - Add threading (Phase 3) if needed
5. **Integrate with PID** - Connect to your drone controller
6. **Add visualization** - Create debugging displays

The system is designed to be implemented incrementally, starting simple and adding complexity as needed.