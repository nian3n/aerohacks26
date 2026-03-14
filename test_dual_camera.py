#!/usr/bin/env python3
"""
Simple test script for dual camera 3D tracking
Displays both camera views with detection overlays and 3D position
"""

import cv2
import numpy as np
import time
from Libraries.position import PositionTracker


def draw_detection_overlay(frame, detection, camera_name):
    """
    Draw detection overlay on frame
    
    Args:
        frame: Image frame
        detection: (x, y, w, h) bounding box or None
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
        x, y, box_w, box_h = detection
        x, y = int(x), int(y)
        box_w, box_h = int(box_w), int(box_h)
        
        # Draw bounding box
        cv2.rectangle(annotated, 
                     (x - box_w//2, y - box_h//2), 
                     (x + box_w//2, y + box_h//2), 
                     (0, 255, 0), 2)
        
        # Draw center point
        cv2.circle(annotated, (x, y), 5, (0, 0, 255), -1)
        
        # Draw crosshair
        cv2.line(annotated, (x - 15, y), (x + 15, y), (0, 0, 255), 2)
        cv2.line(annotated, (x, y - 15), (x, y + 15), (0, 0, 255), 2)
        
        # Display coordinates
        coord_text = f"({x}, {y})"
        cv2.putText(annotated, coord_text, (x + 10, y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Display detection status
        cv2.putText(annotated, "DETECTED", (10, h - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        # No detection
        cv2.putText(annotated, "NO DETECTION", (10, h - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    return annotated


def main():
    """Main test function"""
    print("=" * 60)
    print("Dual Camera 3D Tracking Test")
    print("=" * 60)
    print("\nControls:")
    print("  'q' - Quit")
    print("  'r' - Reset Kalman filter")
    print("  's' - Save screenshot")
    print("  'k' - Toggle Kalman filter on/off")
    print("\nStarting in 2 seconds...")
    time.sleep(2)
    
    try:
        # Initialize position tracker
        tracker = PositionTracker(
            camera1_id=1,  # Front camera
            camera2_id=3,  # Side camera
            cage_size=1.5,
            enable_kalman=True
        )
        
        # Performance tracking
        frame_count = 0
        detection_count = 0
        start_time = time.time()
        
        # Main loop
        while True:
            # Update position
            position, velocity = tracker.update()
            
            # Get frames and detections for visualization
            frame1, frame2 = tracker.tracker.get_frames()
            det1, det2 = tracker.tracker.get_last_detections()
            
            if frame1 is None or frame2 is None:
                print("Error: Could not read from cameras")
                break
            
            # Draw detection overlays
            annotated1 = draw_detection_overlay(frame1, det1, "Camera 1 (Front)")
            annotated2 = draw_detection_overlay(frame2, det2, "Camera 2 (Side)")
            
            # Combine side by side
            combined = np.hstack([annotated1, annotated2])
            
            # Add 3D position info
            if position is not None:
                X, Y, Z = position
                error = tracker.get_error()
                distance = tracker.get_distance_from_target()
                
                # Position text
                pos_text = f"3D Position: X={X:+.3f}m  Y={Y:+.3f}m  Z={Z:+.3f}m"
                cv2.putText(combined, pos_text, (10, combined.shape[0] - 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                # Error text
                err_text = f"Distance from center: {distance:.3f}m"
                cv2.putText(combined, err_text, (10, combined.shape[0] - 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                detection_count += 1
            else:
                # No 3D position
                no_pos_text = "3D Position: NOT AVAILABLE (need both cameras)"
                cv2.putText(combined, no_pos_text, (10, combined.shape[0] - 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Calculate and display FPS
            frame_count += 1
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0
            detection_rate = (detection_count / frame_count * 100) if frame_count > 0 else 0
            
            fps_text = f"FPS: {fps:.1f} | Detection Rate: {detection_rate:.1f}%"
            cv2.putText(combined, fps_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Kalman status
            kalman_text = f"Kalman: {'ON' if tracker.enable_kalman else 'OFF'}"
            cv2.putText(combined, kalman_text, (combined.shape[1] - 150, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Display
            cv2.imshow('Dual Camera 3D Tracking', combined)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("\nQuitting...")
                break
            elif key == ord('r'):
                tracker.reset_kalman()
                print("\nKalman filter reset")
            elif key == ord('s'):
                # Save screenshot
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f"screenshot_{timestamp}.jpg"
                cv2.imwrite(filename, combined)
                print(f"\nScreenshot saved: {filename}")
            elif key == ord('k'):
                # Toggle Kalman filter
                tracker.enable_kalman = not tracker.enable_kalman
                if tracker.enable_kalman:
                    tracker.kalman = KalmanFilter3D()
                print(f"\nKalman filter: {'ON' if tracker.enable_kalman else 'OFF'}")
        
        # Print final statistics
        print("\n" + "=" * 60)
        print("Session Statistics:")
        print("=" * 60)
        print(f"Total frames: {frame_count}")
        print(f"Detections: {detection_count}")
        print(f"Detection rate: {detection_rate:.1f}%")
        print(f"Average FPS: {fps:.1f}")
        print(f"Duration: {elapsed:.1f} seconds")
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        tracker.release()
        cv2.destroyAllWindows()
        print("\nTest completed")


if __name__ == "__main__":
    main()
