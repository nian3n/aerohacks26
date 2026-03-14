# 3D Drone Tracking System - Quick Start Guide

## Overview

This system uses 2 cameras positioned at 90-degree angles (front and side) to track a drone's 3D position within a 1.5m x 1.5m cage using background subtraction and triangulation.

## What's Implemented (Phase 1)

✅ **Dual Camera Capture** - Synchronized frame acquisition from 2 cameras
✅ **Hybrid Detection** - Combined background subtraction + LED detection (green) for robust tracking
✅ **3D Triangulation** - Simplified geometric reconstruction from 2D detections
✅ **Kalman Filtering** - Position smoothing and velocity estimation
✅ **Position Output** - Error from target for PID controller integration
✅ **Visualization Tool** - Real-time display with detection overlays

## File Structure

```
aerohacks26/
├── Libraries/
│   ├── camera.py          # Dual camera tracker with background subtraction
│   ├── position.py        # Kalman filter and position tracking
│   ├── controller.py      # (existing)
│   └── drone_rc.py        # (existing)
├── test_dual_camera.py    # Test/visualization script
├── plans/                 # Architecture documentation
│   ├── 3d-localization-architecture.md
│   ├── implementation-guide.md
│   ├── visual-debugging-tools.md
│   └── summary.md
└── README_3D_TRACKING.md  # This file
```

## Quick Start

### 1. Test Your Cameras

First, verify both cameras are accessible:

```bash
# On Linux, check available cameras
ls /dev/video*

# Should show something like:
# /dev/video0  /dev/video1
```

### 2. Run the Test Script

```bash
python test_dual_camera.py
```

This will:
- Open both cameras (IDs 0 and 1 by default)
- Display both camera views side-by-side
- Show detection overlays (bounding boxes)
- Display 3D position in real-time
- Show FPS and detection rate

### 3. Controls

While the test script is running:
- **'q'** - Quit
- **'r'** - Reset Kalman filter
- **'s'** - Save screenshot
- **'k'** - Toggle Kalman filter on/off

### 4. Expected Output

You should see:
```
Dual Camera 3D Tracking Test
============================================================

Controls:
  'q' - Quit
  'r' - Reset Kalman filter
  's' - Save screenshot
  'k' - Toggle Kalman filter on/off

Starting in 2 seconds...
Dual camera tracker initialized
  Camera 1 (Front): Device 0
  Camera 2 (Side): Device 1
  Image size: 640x480
  Cage size: 1.5m
Position tracker initialized
  Kalman filter: Enabled
  Target position: [0. 0. 0.]
```

The window will show:
- Left side: Front camera view
- Right side: Side camera view
- Green boxes: Detected drone
- Yellow text: 3D position (X, Y, Z in meters)
- Distance from cage center

## Using in Your Code

### Basic Usage

```python
from Libraries.position import PositionTracker

# Initialize
tracker = PositionTracker(
    camera1_id=0,      # Front camera
    camera2_id=1,      # Side camera
    cage_size=1.5,     # Cage size in meters
    enable_kalman=True # Use Kalman filtering
)

# Main control loop
while True:
    # Update position
    position, velocity = tracker.update()
    
    if position is not None:
        X, Y, Z = position  # Position in meters
        error = tracker.get_error()  # Error from center
        distance = tracker.get_distance_from_target()
        
        # Send to PID controller
        # control_drone(error)
```

### Integration with PID Controller

```python
from Libraries.position import PositionTracker
from Libraries import drone_rc

# Initialize
tracker = PositionTracker(camera1_id=0, camera2_id=1, cage_size=1.5)

# Control loop
while True:
    position, velocity = tracker.update()
    
    if position is not None:
        # Get error from target (center of cage)
        error = tracker.get_error()  # [ex, ey, ez]
        
        # Extract components
        error_x = error[0]  # Left-Right error
        error_y = error[1]  # Up-Down error
        error_z = error[2]  # Front-Back error
        
        # Send to drone controller
        # Example: Use error_x for roll, error_y for throttle, error_z for pitch
        # drone_rc.set_target_roll(error_x * gain)
        # drone_rc.set_target_pitch(error_z * gain)
        # etc.
```

## Configuration

### Camera IDs

If your cameras are on different device IDs, change them when initializing:

```python
tracker = PositionTracker(
    camera1_id=1,  # Change if needed
    camera2_id=2,  # Change if needed
    cage_size=1.5
)
```

### Detection Parameters

Adjust detection sensitivity in [`Libraries/camera.py`](Libraries/camera.py):

```python
# In DroneDetector.__init__()
self.min_area = 1000      # Minimum contour area (decrease for smaller drone)
self.max_area = 200000    # Maximum contour area (increase for larger drone)

# LED detection HSV range (for green LED)
self.led_lower = np.array([40, 100, 200])
self.led_upper = np.array([80, 255, 255])
```

**Note**: The detector now uses **hybrid detection** - it looks for moving objects (background subtraction) that also contain a green LED. This significantly improves accuracy by reducing false positives.

### Kalman Filter Tuning

Adjust filter parameters in [`Libraries/position.py`](Libraries/position.py):

```python
# In KalmanFilter3D.__init__()
process_noise = 0.01      # Lower = trust motion model more
measurement_noise = 0.05  # Lower = trust measurements more
```

### Cage Size

If your cage is different from 1.5m:

```python
tracker = PositionTracker(
    camera1_id=0,
    camera2_id=1,
    cage_size=2.0  # Change to your cage size
)
```

## Coordinate System

```
        Y (Up)
        |
        |
        |_________ X (Right, from front camera)
       /
      /
     Z (Depth, away from front camera)

Origin: Center of cage
```

- **X-axis**: Left (-) to Right (+) from front camera view
- **Y-axis**: Down (-) to Up (+)
- **Z-axis**: Front (-) to Back (+) from front camera view

## Troubleshooting

### Problem: "Could not open camera"

**Solution**: Check camera IDs
```bash
ls /dev/video*
# Try different IDs: 0, 1, 2, etc.
```

### Problem: "No detection"

**Solutions**:
1. Check lighting - needs consistent, bright lighting
2. Adjust detection parameters (min_area, max_area)
3. Let background model stabilize (wait 5-10 seconds)
4. Ensure drone is moving (background subtraction detects motion)

### Problem: Low FPS

**Solutions**:
1. Reduce camera resolution (add to camera.py):
   ```python
   self.cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
   self.cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
   ```
2. Disable Kalman filter temporarily:
   ```python
   tracker = PositionTracker(..., enable_kalman=False)
   ```
3. Close visualization window (it's CPU intensive)

### Problem: Noisy position estimates

**Solutions**:
1. Increase Kalman measurement noise (trust measurements less)
2. Improve lighting conditions
3. Ensure cameras are stable (not vibrating)
4. Check that both cameras detect simultaneously

### Problem: Position seems wrong

**Solutions**:
1. Verify camera placement (front and side at 90 degrees)
2. Check cage_size parameter matches actual cage
3. Ensure cameras are centered on cage
4. Verify coordinate system matches your setup

## Testing Checklist

Before integrating with PID controller:

- [ ] Both cameras capture frames successfully
- [ ] Drone detected in both camera views (green boxes)
- [ ] 3D position displayed and updates smoothly
- [ ] Position is approximately correct (compare to actual position)
- [ ] FPS is acceptable (>20 FPS recommended)
- [ ] Detection rate is high (>80% recommended)
- [ ] Kalman filter smooths noisy measurements
- [ ] Error from center calculated correctly

## Next Steps

### Phase 2: Camera Calibration (Optional)

If you need better accuracy, implement camera calibration:
- See [`plans/implementation-guide.md`](plans/implementation-guide.md) Phase 2
- Requires checkerboard pattern and calibration images
- Improves accuracy from ±5cm to ±1-2cm

### Phase 3: Threading (Optional)

If you need better performance:
- See [`plans/implementation-guide.md`](plans/implementation-guide.md) Phase 3
- Separate threads for capture, detection, and position computation
- Can improve FPS by 2-3x

### Integration with PID

Connect position output to your existing PID controller:
1. Get position error: `error = tracker.get_error()`
2. Map error to control signals (roll, pitch, throttle, yaw)
3. Send to drone via [`drone_rc.py`](Libraries/drone_rc.py)

## Performance Expectations

With this Phase 1 implementation:

- **Frame Rate**: 20-30 FPS (depends on CPU)
- **Detection Rate**: 80-95% (with good lighting)
- **Position Accuracy**: ±2-5cm (simplified triangulation)
- **Latency**: <50ms end-to-end

## Support

For detailed architecture and implementation details, see:
- [`plans/3d-localization-architecture.md`](plans/3d-localization-architecture.md) - Complete technical design
- [`plans/implementation-guide.md`](plans/implementation-guide.md) - Step-by-step guide
- [`plans/visual-debugging-tools.md`](plans/visual-debugging-tools.md) - Advanced debugging
- [`plans/summary.md`](plans/summary.md) - Executive overview

## Key Classes

### `DroneDetector` (camera.py)
- Detects drone using background subtraction
- Returns bounding box (x, y, w, h)

### `DualCameraTracker` (camera.py)
- Manages both cameras
- Runs detection on both views
- Computes 3D position via triangulation

### `KalmanFilter3D` (position.py)
- Filters noisy position measurements
- Estimates velocity
- Handles missing detections

### `PositionTracker` (position.py)
- High-level interface
- Integrates camera tracker and Kalman filter
- Provides position error for control

## Example Output

```
Position: X=+0.123m, Y=-0.045m, Z=+0.234m
Distance from center: 0.267m
FPS: 28.3 | Detection Rate: 92.5%
```

This means:
- Drone is 12.3cm to the right
- Drone is 4.5cm below center
- Drone is 23.4cm behind center
- Total distance from target: 26.7cm
- System running at 28.3 FPS
- Detecting drone in 92.5% of frames
