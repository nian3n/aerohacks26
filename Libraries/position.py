#!/usr/bin/env python3
import numpy as np
from typing import Optional, Tuple
from Libraries.camera import DualCameraTracker


class KalmanFilter3D:
    """
    Kalman filter for 3D drone position and velocity estimation
    
    State vector: [x, y, z, vx, vy, vz]
    Measurement: [x, y, z]
    """
    
    def __init__(self, dt=0.033, process_noise=0.01, measurement_noise=0.05):
        """
        Initialize Kalman filter
        
        Args:
            dt: Time step in seconds (default ~30 FPS)
            process_noise: Process noise covariance
            measurement_noise: Measurement noise covariance
        """
        self.dt = dt
        self.initialized = False
        
        # State vector: [x, y, z, vx, vy, vz]
        self.state = np.zeros(6)
        
        # Covariance matrix
        self.P = np.eye(6) * 1.0
        
        # State transition matrix (constant velocity model)
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
        
        # Process noise covariance
        self.Q = np.eye(6) * process_noise
        self.Q[3:, 3:] *= 2  # Higher noise for velocity
        
        # Measurement noise covariance
        self.R = np.eye(3) * measurement_noise
    
    def predict(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Predict next state
        
        Returns:
            (position, velocity) predictions
        """
        # State prediction
        self.state = self.F @ self.state
        
        # Covariance prediction
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        return self.state[:3], self.state[3:]
    
    def update(self, measurement: Optional[Tuple[float, float, float]]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update filter with new measurement
        
        Args:
            measurement: (x, y, z) position measurement or None
        
        Returns:
            (position, velocity) estimates
        """
        if measurement is None:
            # No measurement, just predict
            return self.predict()
        
        z = np.array(measurement)
        
        if not self.initialized:
            # Initialize state with first measurement
            self.state[:3] = z
            self.state[3:] = 0  # Zero velocity
            self.initialized = True
            return self.state[:3], self.state[3:]
        
        # Predict step
        self.predict()
        
        # Update step
        y = z - self.H @ self.state  # Innovation
        S = self.H @ self.P @ self.H.T + self.R  # Innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain
        
        # State update
        self.state = self.state + K @ y
        
        # Covariance update
        self.P = (np.eye(6) - K @ self.H) @ self.P
        
        return self.state[:3], self.state[3:]
    
    def get_position(self) -> np.ndarray:
        """Get current position estimate"""
        return self.state[:3]
    
    def get_velocity(self) -> np.ndarray:
        """Get current velocity estimate"""
        return self.state[3:]
    
    def reset(self):
        """Reset filter to initial state"""
        self.initialized = False
        self.state = np.zeros(6)
        self.P = np.eye(6) * 1.0


class PositionTracker:
    """Complete position tracking system with filtering"""
    
    def __init__(self, camera1_id=0, camera2_id=1, cage_size=1.5, 
                 enable_kalman=True, dt=0.033):
        """
        Initialize position tracker
        
        Args:
            camera1_id: Device ID for front camera
            camera2_id: Device ID for side camera
            cage_size: Size of cage in meters
            enable_kalman: Whether to use Kalman filtering
            dt: Time step for Kalman filter
        """
        # Initialize camera tracker
        self.tracker = DualCameraTracker(camera1_id, camera2_id, cage_size)
        
        # Initialize Kalman filter
        self.enable_kalman = enable_kalman
        if enable_kalman:
            self.kalman = KalmanFilter3D(dt=dt)
        
        # State
        self.raw_position = None
        self.filtered_position = None
        self.velocity = None
        self.target = np.array([0.0, 0.0, 0.0])  # Center of cage
        
        print(f"Position tracker initialized")
        print(f"  Kalman filter: {'Enabled' if enable_kalman else 'Disabled'}")
        print(f"  Target position: {self.target}")
    
    def update(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Update position estimate
        
        Returns:
            (position, velocity) tuple
            - position: np.ndarray [x, y, z] or None
            - velocity: np.ndarray [vx, vy, vz] or None
        """
        # Get raw 3D position from cameras
        raw_pos = self.tracker.track_once()
        self.raw_position = raw_pos
        
        if self.enable_kalman:
            # Filter with Kalman
            pos, vel = self.kalman.update(raw_pos)
            self.filtered_position = pos
            self.velocity = vel
        else:
            # No filtering
            if raw_pos is not None:
                self.filtered_position = np.array(raw_pos)
                self.velocity = np.zeros(3)
            else:
                self.filtered_position = None
                self.velocity = None
        
        return self.filtered_position, self.velocity
    
    def get_position(self) -> Optional[np.ndarray]:
        """Get current filtered position"""
        return self.filtered_position
    
    def get_raw_position(self) -> Optional[Tuple[float, float, float]]:
        """Get current raw (unfiltered) position"""
        return self.raw_position
    
    def get_velocity(self) -> Optional[np.ndarray]:
        """Get current velocity estimate"""
        return self.velocity
    
    def get_error(self) -> Optional[np.ndarray]:
        """
        Get position error from target
        
        Returns:
            Error vector [ex, ey, ez] or None
        """
        if self.filtered_position is None:
            return None
        return self.target - self.filtered_position
    
    def get_distance_from_target(self) -> Optional[float]:
        """
        Get Euclidean distance from target
        
        Returns:
            Distance in meters or None
        """
        error = self.get_error()
        if error is None:
            return None
        return np.linalg.norm(error)
    
    def set_target(self, x: float, y: float, z: float):
        """
        Set target position
        
        Args:
            x, y, z: Target coordinates in meters
        """
        self.target = np.array([x, y, z])
        print(f"Target position set to: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
    
    def reset_kalman(self):
        """Reset Kalman filter"""
        if self.enable_kalman:
            self.kalman.reset()
            print("Kalman filter reset")
    
    def release(self):
        """Release resources"""
        self.tracker.release()


# Simple test
if __name__ == "__main__":
    import time
    
    print("Testing position tracker...")
    print("Press Ctrl+C to quit")
    
    try:
        # Initialize tracker
        tracker = PositionTracker(
            camera1_id=0, 
            camera2_id=1, 
            cage_size=1.5,
            enable_kalman=True
        )
        
        # Main loop
        frame_count = 0
        start_time = time.time()
        
        while True:
            # Update position
            position, velocity = tracker.update()
            
            # Display results
            if position is not None:
                X, Y, Z = position
                error = tracker.get_error()
                distance = tracker.get_distance_from_target()
                
                print(f"\rPos: X={X:+.3f} Y={Y:+.3f} Z={Z:+.3f} | "
                      f"Err: {distance:.3f}m | "
                      f"Frame: {frame_count}", end='')
            else:
                print(f"\rNo detection | Frame: {frame_count}", end='')
            
            frame_count += 1
            
            # Calculate FPS every 30 frames
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                print(f" | FPS: {fps:.1f}", end='')
            
            time.sleep(0.001)  # Small delay
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tracker.release()
        print("Position tracker stopped")
