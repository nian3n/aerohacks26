#!/usr/bin/env python3
import cv2
import numpy as np

#open default 
cap = cv2.VideoCapture(0)
backSub = cv2.createBackgroundSubtractorMOG2()

if not cap.isOpened():
    print("Error: Could not open video device.")
    exit(1)

#find the drone using background substraction
def detect_drone():
    ret, frame = cap.read()
    if ret:
        # Apply background subtraction
        blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)
        fg_mask = backSub.apply(blurred_frame)
        #additional filtering
        retval, mask_thresh = cv2.threshold( fg_mask, 180, 255, cv2.THRESH_BINARY)
        # set the kernal
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        # Apply erosion
        mask_eroded = cv2.morphologyEx(mask_thresh, cv2.MORPH_OPEN, kernel)
        contours, hierarchy = cv2.findContours(mask_eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print(contours)
        frame_ct = cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
        min_contour_area = 250  # Define your minimum area threshold
        large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]
        frame_out = frame.copy()
        for cnt in large_contours:
            x, y, w, h = cv2.boundingRect(cnt)
            frame_out = cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 200), 3)
        # Display the resulting frame
        cv2.imshow('Frame_final', frame_out)
        cv2.waitKey(1)
            
#find the box's egdes using corner detection
def detect_box():
    ret, frame = cap.read()
    if not ret:
        return

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Adaptive threshold finds edges regardless of lighting/contrast
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    adaptive = cv2.adaptiveThreshold(
        blurred, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        11, 2  # block size, C constant — tune these
    )

    # Also run Canny and combine both — catches more edges
    canny = cv2.Canny(blurred, 30, 100)
    combined = cv2.bitwise_or(adaptive, canny)

    # Close gaps between edges
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        if cv2.contourArea(cnt) < 1000:  # skip tiny noise
            continue

        epsilon = 0.03 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        if len(approx) == 4 and cv2.isContourConvex(approx):
            # Check angles are close to 90 degrees
            pts = approx.reshape(4, 2).astype(np.float32)
            angles = []
            for i in range(4):
                p0 = pts[(i - 1) % 4]
                p1 = pts[i]
                p2 = pts[(i + 1) % 4]
                v1 = p0 - p1
                v2 = p2 - p1
                cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
                angles.append(np.degrees(np.arccos(np.clip(cos_angle, -1, 1))))

            # All 4 angles should be roughly 90° for a rectangle
            if all(60 < a < 120 for a in angles):
                cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
                x, y, _, _ = cv2.boundingRect(approx)
                cv2.putText(frame, f"{int(cv2.contourArea(cnt))}px",
                            (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    cv2.imshow('Frame', frame)
    cv2.waitKey(1)
if __name__ == "__main__":
    while True:
        detect_box()
