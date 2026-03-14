# Visual Debugging and Monitoring Tools
## Real-time Visualization for 3D Drone Tracking

---

## Overview

Visual debugging tools are essential for tuning and validating your 3D tracking system. This document provides visualization utilities for monitoring detection, position tracking, and system performance.

---

## 1. Real-time 3D Position Visualizer

### 3D Cage Visualization with Matplotlib

```python
# visualization/position_viewer.py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import threading
import queue

class Position3DVisualizer:
    """Real-time 3D position visualization"""
    
    def __init__(self, cage_size=1.5, history_length=100):
        self.cage_size = cage_size
        self.history_length = history_length
        
        # Position history
        self.position_history = []
        self.position_queue = queue.Queue()
        
        # Setup plot
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self._setup_plot()
        
    def _setup_plot(self):
        """Setup 3D plot with cage boundaries"""
        s = self.cage_size / 2
        
        # Draw cage wireframe
        # Bottom square
        self.ax.plot([-s, s], [-s, -s], [-s, -s], 'k-', alpha=0.3)
        self.ax.plot([s, s], [-s, s], [-s, -s], 'k-', alpha=0.3)
        self.ax.plot([s, -s], [s, s], [-s, -s], 'k-', alpha=0.3)
        self.ax.plot([-s, -s], [s, -s], [-s, -s], 'k-', alpha=0.3)
        
        # Top square
        self.ax.plot([-s, s], [-s, -s], [s, s], 'k-', alpha=0.3)
        self.ax.plot([s, s], [-s, s], [s, s], 'k-', alpha=0.3)
        self.ax.plot([s, -s], [s, s], [s, s], 'k-', alpha=0.3)
        self.ax.plot([-s, -s], [s, -s], [s, s], 'k-', alpha=0.3)
        
        # Vertical edges
        self.ax.plot([-s, -s], [-s, -s], [-s, s], 'k-', alpha=0.3)
        self.ax.plot([s, s], [-s, -s], [-s, s], 'k-', alpha=0.3)
        self.ax.plot([s, s], [s, s], [-s, s], 'k-', alpha=0.3)
        self.ax.plot([-s, -s], [s, s], [-s, s], 'k-', alpha=0.3)
        
        # Target point at center
        self.ax.scatter([0], [0], [0], c='green', marker='x', s=100, label='Target')
        
        # Initialize trajectory line
        self.trajectory_line, = self.ax.plot([], [], [], 'b-', alpha=0.5, linewidth=1)
        
        # Initialize current position marker
        self.position_marker = self.ax.scatter([], [], [], c='red', marker='o', s=100, label='Drone')
        
        # Labels and limits
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_xlim(-s, s)
        self.ax.set_ylim(-s, s)
        self.ax.set_zlim(-s, s)
        self.ax.legend()
        self.ax.set_title('Drone Position in Cage')
        
    def update(self, position):
        """Update with new position"""
        if position is not None:
            self.position_queue.put(position)
    
    def _animation_update(self, frame):
        """Animation update function"""
        # Get all available positions
        while not self.position_queue.empty():
            try:
                pos = self.position_queue.get_nowait()
                self.position_history.append(pos)
                
                # Limit history length
                if len(self.position_history) > self.history_length:
                    self.position_history.pop(0)
            except queue.Empty:
                break
        
        if len(self.position_history) > 0:
            # Update trajectory
            history = np.array(self.position_history)
            self.trajectory_line.set_data(history[:, 0], history[:, 1])
            self.trajectory_line.set_3d_properties(history[:, 2])
            
            # Update current position
            current = self.position_history[-1]
            self.position_marker._offsets3d = ([current[0]], [current[1]], [current[2]])
            
            # Update title with current position
            self.ax.set_title(f'Drone Position: X={current[0]:.3f}, Y={current[1]:.3f}, Z={current[2]:.3f}')
        
        return self.trajectory_line, self.position_marker
    
    def start(self):
        """Start animation"""
        self.anim = FuncAnimation(self.fig, self._animation_update, 
                                  interval=33, blit=False)
        plt.show()
```

---

## 2. Dual Camera View with Detection Overlay

```python
# visualization/camera_viewer.py
import cv2
import numpy as np

class DualCameraViewer:
    """Display both camera views with detection overlays"""
    
    def __init__(self, window_name="Dual Camera View"):
        self.window_name = window_name
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
    
    def draw_detection(self, frame, detection, camera_name):
        """
        Draw detection overlay on frame
        
        Args:
            frame: Image frame
            detection: (x, y, radius) or None
            camera_name: Name to display
        
        Returns:
            Annotated frame
        """
        annotated = frame.copy()
        h, w = frame.shape[:2]
        
        # Draw camera name
        cv2.putText(annotated, camera_name, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Draw crosshair at center
        cv2.line(annotated, (w//2 - 20, h//2), (w//2 + 20, h//2), (0, 255, 0), 1)
        cv2.line(annotated, (w//2, h//2 - 20), (w//2, h//2 + 20), (0, 255, 0), 1)
        
        if detection is not None:
            x, y, radius = detection
            x, y = int(x), int(y)
            radius = int(radius)
            
            # Draw detection circle
            cv2.circle(annotated, (x, y), radius, (0, 255, 0), 2)
            
            # Draw center point
            cv2.circle(annotated, (x, y), 3, (0, 0, 255), -1)
            
            # Draw crosshair
            cv2.line(annotated, (x - 10, y), (x + 10, y), (0, 0, 255), 1)
            cv2.line(annotated, (x, y - 10), (x, y + 10), (0, 0, 255), 1)
            
            # Display coordinates
            coord_text = f"({x}, {y})"
            cv2.putText(annotated, coord_text, (x + 10, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        else:
            # No detection
            cv2.putText(annotated, "NO DETECTION", (10, h - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        return annotated
    
    def show(self, frame1, frame2, detection1, detection2, position_3d=None):
        """
        Display both camera views side by side
        
        Args:
            frame1, frame2: Camera frames
            detection1, detection2: Detection results
            position_3d: Optional 3D position (X, Y, Z)
        """
        # Annotate frames
        annotated1 = self.draw_detection(frame1, detection1, "Camera 1 (Front)")
        annotated2 = self.draw_detection(frame2, detection2, "Camera 2 (Side)")
        
        # Combine side by side
        combined = np.hstack([annotated1, annotated2])
        
        # Add 3D position info if available
        if position_3d is not None:
            X, Y, Z = position_3d
            pos_text = f"3D Position: X={X:.3f}m, Y={Y:.3f}m, Z={Z:.3f}m"
            cv2.putText(combined, pos_text, (10, combined.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        cv2.imshow(self.window_name, combined)
    
    def close(self):
        """Close window"""
        cv2.destroyWindow(self.window_name)
```

---

## 3. Performance Monitor

```python
# visualization/performance_monitor.py
import time
import numpy as np
from collections import deque

class PerformanceMonitor:
    """Monitor system performance metrics"""
    
    def __init__(self, window_size=100):
        self.window_size = window_size
        
        # Metrics
        self.frame_times = deque(maxlen=window_size)
        self.detection_times = deque(maxlen=window_size)
        self.detection_success = deque(maxlen=window_size)
        
        self.last_time = time.time()
    
    def update(self, detection_success, detection_time=None):
        """
        Update metrics
        
        Args:
            detection_success: True if drone was detected
            detection_time: Time taken for detection (optional)
        """
        current_time = time.time()
        frame_time = current_time - self.last_time
        self.last_time = current_time
        
        self.frame_times.append(frame_time)
        self.detection_success.append(1 if detection_success else 0)
        
        if detection_time is not None:
            self.detection_times.append(detection_time)
    
    def get_fps(self):
        """Get current FPS"""
        if len(self.frame_times) == 0:
            return 0.0
        return 1.0 / np.mean(self.frame_times)
    
    def get_detection_rate(self):
        """Get detection success rate (%)"""
        if len(self.detection_success) == 0:
            return 0.0
        return np.mean(self.detection_success) * 100
    
    def get_avg_detection_time(self):
        """Get average detection time (ms)"""
        if len(self.detection_times) == 0:
            return 0.0
        return np.mean(self.detection_times) * 1000
    
    def get_stats(self):
        """Get all statistics"""
        return {
            'fps': self.get_fps(),
            'detection_rate': self.get_detection_rate(),
            'avg_detection_time_ms': self.get_avg_detection_time(),
            'frame_time_ms': np.mean(self.frame_times) * 1000 if self.frame_times else 0
        }
    
    def print_stats(self):
        """Print statistics to console"""
        stats = self.get_stats()
        print(f"\rFPS: {stats['fps']:.1f} | "
              f"Detection: {stats['detection_rate']:.1f}% | "
              f"Frame Time: {stats['frame_time_ms']:.1f}ms", end='')
```

---

## 4. Complete Debugging System

```python
# visualization/debug_system.py
import cv2
import numpy as np
import threading
from Libraries.camera import DualCameraTracker
from Libraries.position import KalmanFilter3D
from visualization.camera_viewer import DualCameraViewer
from visualization.performance_monitor import PerformanceMonitor

class DebugTrackingSystem:
    """Complete tracking system with visualization"""
    
    def __init__(self, camera1_id=0, camera2_id=1, cage_size=1.5, 
                 enable_3d_plot=False):
        # Core components
        self.tracker = DualCameraTracker(camera1_id, camera2_id, cage_size)
        self.kalman = KalmanFilter3D(dt=0.033)
        
        # Visualization
        self.camera_viewer = DualCameraViewer()
        self.performance = PerformanceMonitor()
        
        # 3D visualization (optional, runs in separate process)
        self.enable_3d_plot = enable_3d_plot
        if enable_3d_plot:
            from visualization.position_viewer import Position3DVisualizer
            self.position_viz = Position3DVisualizer(cage_size)
            self.viz_thread = threading.Thread(target=self.position_viz.start)
            self.viz_thread.daemon = True
            self.viz_thread.start()
        
        self.running = False
    
    def run(self):
        """Run tracking with visualization"""
        self.running = True
        
        print("Starting debug tracking system...")
        print("Press 'q' to quit, 's' to save screenshot")
        
        try:
            while self.running:
                # Get frames
                frame1, frame2 = self.tracker.get_frames()
                
                if frame1 is None or frame2 is None:
                    continue
                
                # Detect
                det1 = self.tracker.detector.detect(frame1)
                det2 = self.tracker.detector.detect(frame2)
                
                # Compute 3D position
                raw_pos = self.tracker.compute_3d_position(det1, det2)
                
                # Filter
                filtered_pos, velocity = self.kalman.update(raw_pos)
                
                # Update performance metrics
                self.performance.update(raw_pos is not None)
                
                # Display camera views
                self.camera_viewer.show(frame1, frame2, det1, det2, filtered_pos)
                
                # Update 3D plot
                if self.enable_3d_plot and filtered_pos is not None:
                    self.position_viz.update(filtered_pos)
                
                # Print stats
                self.performance.print_stats()
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    # Save screenshot
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    filename = f"screenshot_{timestamp}.jpg"
                    combined = np.hstack([frame1, frame2])
                    cv2.imwrite(filename, combined)
                    print(f"\nSaved {filename}")
        
        except KeyboardInterrupt:
            print("\nStopping...")
        
        finally:
            self.stop()
    
    def stop(self):
        """Stop system and cleanup"""
        self.running = False
        self.tracker.release()
        self.camera_viewer.close()
        print("\nSystem stopped")

# Run debug system
if __name__ == "__main__":
    debug_system = DebugTrackingSystem(
        camera1_id=0, 
        camera2_id=1, 
        cage_size=1.5,
        enable_3d_plot=True  # Set to False if matplotlib causes issues
    )
    debug_system.run()
```

---

## 5. Data Logging for Analysis

```python
# visualization/data_logger.py
import csv
import time
from datetime import datetime

class TrackingDataLogger:
    """Log tracking data for offline analysis"""
    
    def __init__(self, filename=None):
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"tracking_log_{timestamp}.csv"
        
        self.filename = filename
        self.file = open(filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        
        # Write header
        self.writer.writerow([
            'timestamp', 'time_sec',
            'raw_x', 'raw_y', 'raw_z',
            'filtered_x', 'filtered_y', 'filtered_z',
            'vel_x', 'vel_y', 'vel_z',
            'detected'
        ])
        
        self.start_time = time.time()
        print(f"Logging to {filename}")
    
    def log(self, raw_position, filtered_position, velocity):
        """
        Log tracking data
        
        Args:
            raw_position: Raw 3D position or None
            filtered_position: Filtered position
            velocity: Velocity estimate
        """
        timestamp = datetime.now().isoformat()
        time_sec = time.time() - self.start_time
        
        if raw_position is not None:
            raw_x, raw_y, raw_z = raw_position
            detected = 1
        else:
            raw_x = raw_y = raw_z = None
            detected = 0
        
        if filtered_position is not None:
            filt_x, filt_y, filt_z = filtered_position
        else:
            filt_x = filt_y = filt_z = None
        
        if velocity is not None:
            vel_x, vel_y, vel_z = velocity
        else:
            vel_x = vel_y = vel_z = None
        
        self.writer.writerow([
            timestamp, time_sec,
            raw_x, raw_y, raw_z,
            filt_x, filt_y, filt_z,
            vel_x, vel_y, vel_z,
            detected
        ])
    
    def close(self):
        """Close log file"""
        self.file.close()
        print(f"Log saved to {self.filename}")
```

---

## 6. Analysis Tools

```python
# visualization/analyze_log.py
import pandas as pd
import matplotlib.pyplot as plt
import numpy as npclaude sonnet

def analyze_tracking_log(filename):
    """Analyze tracking log file"""
    # Load data
    df = pd.read_csv(filename)
    
    # Create figure with subplots
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    
    # 1. Position over time
    ax = axes[0, 0]
    ax.plot(df['time_sec'], df['filtered_x'], label='X', alpha=0.7)
    ax.plot(df['time_sec'], df['filtered_y'], label='Y', alpha=0.7)
    ax.plot(df['time_sec'], df['filtered_z'], label='Z', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (m)')
    ax.set_title('Position vs Time')
    ax.legend()
    ax.grid(True)
    
    # 2. Velocity over time
    ax = axes[0, 1]
    ax.plot(df['time_sec'], df['vel_x'], label='Vx', alpha=0.7)
    ax.plot(df['time_sec'], df['vel_y'], label='Vy', alpha=0.7)
    ax.plot(df['time_sec'], df['vel_z'], label='Vz', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Velocity vs Time')
    ax.legend()
    ax.grid(True)
    
    # 3. Distance from center
    df['distance'] = np.sqrt(df['filtered_x']**2 + df['filtered_y']**2 + df['filtered_z']**2)
    ax = axes[1, 0]
    ax.plot(df['time_sec'], df['distance'])
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance from Center (m)')
    ax.set_title('Distance from Target')
    ax.grid(True)
    
    # 4. Detection rate over time
    window = 30  # 1 second window at 30 FPS
    df['detection_rate'] = df['detected'].rolling(window=window).mean() * 100
    ax = axes[1, 1]
    ax.plot(df['time_sec'], df['detection_rate'])
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Detection Rate (%)')
    ax.set_title(f'Detection Rate (rolling {window} frames)')
    ax.grid(True)
    
    # 5. Position distribution
    ax = axes[2, 0]
    ax.hist2d(df['filtered_x'].dropna(), df['filtered_y'].dropna(), bins=30, cmap='hot')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('XY Position Distribution')
    ax.axhline(0, color='white', linestyle='--', alpha=0.5)
    ax.axvline(0, color='white', linestyle='--', alpha=0.5)
    
    # 6. Statistics
    ax = axes[2, 1]
    ax.axis('off')
    
    stats_text = f"""
    Tracking Statistics
    ==================
    Duration: {df['time_sec'].max():.1f} seconds
    Total Frames: {len(df)}
    
    Detection Rate: {df['detected'].mean() * 100:.1f}%
    
    Position (mean ± std):
    X: {df['filtered_x'].mean():.3f} ± {df['filtered_x'].std():.3f} m
    Y: {df['filtered_y'].mean():.3f} ± {df['filtered_y'].std():.3f} m
    Z: {df['filtered_z'].mean():.3f} ± {df['filtered_z'].std():.3f} m
    
    Distance from Center:
    Mean: {df['distance'].mean():.3f} m
    Max: {df['distance'].max():.3f} m
    Std: {df['distance'].std():.3f} m
    """
    
    ax.text(0.1, 0.5, stats_text, fontsize=10, family='monospace',
            verticalalignment='center')
    
    plt.tight_layout()
    plt.savefig(filename.replace('.csv', '_analysis.png'), dpi=150)
    print(f"Analysis saved to {filename.replace('.csv', '_analysis.png')}")
    plt.show()

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        analyze_tracking_log(sys.argv[1])
    else:
        print("Usage: python analyze_log.py <log_file.csv>")
```

---

## Usage Examples

### Basic Visualization

```python
from visualization.debug_system import DebugTrackingSystem

# Run with camera view only
debug = DebugTrackingSystem(camera1_id=0, camera2_id=1, cage_size=1.5)
debug.run()
```

### With 3D Visualization

```python
# Run with 3D plot (requires matplotlib)
debug = DebugTrackingSystem(
    camera1_id=0, 
    camera2_id=1, 
    cage_size=1.5,
    enable_3d_plot=True
)
debug.run()
```

### With Data Logging

```python
from visualization.data_logger import TrackingDataLogger
from Libraries.position import PositionTracker

tracker = PositionTracker(camera1_id=0, camera2_id=1, cage_size=1.5)
logger = TrackingDataLogger()

try:
    while True:
        pos, vel = tracker.update()
        raw_pos = tracker.tracker.position_3d
        logger.log(raw_pos, pos, vel)
        
except KeyboardInterrupt:
    logger.close()
    tracker.release()
```

### Analyze Logged Data

```bash
python visualization/analyze_log.py tracking_log_20260314_120000.csv
```

---

## Tips for Effective Debugging

1. **Start Simple**: Begin with camera viewer only, add 3D plot later
2. **Monitor Performance**: Watch FPS and detection rate
3. **Log Data**: Record sessions for offline analysis
4. **Tune Visually**: Adjust HSV ranges while watching detection overlay
5. **Check Synchronization**: Verify both cameras detect simultaneously
6. **Validate Kalman**: Compare raw vs filtered positions
7. **Test Static First**: Validate with stationary drone before moving tests