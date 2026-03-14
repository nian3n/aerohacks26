#!/usr/bin/env python3
import cv2
import numpy as np

#open default 
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
backSub = cv2.createBackgroundSubtractorMOG2(history=1000)

if not cap.isOpened():
    print("Error: Could not open video device.")
    exit(1)

ret, frame = cap.read()

def detect_led(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([40, 100, 200])  
    upper = np.array([80, 255, 255])

    led_mask = cv2.inRange(hsv, lower, upper)
    
    return led_mask

def detect_mask(frame):
    # Apply background subtraction
    blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)
    fg_mask = backSub.apply(blurred_frame)
    #additional filtering
    retval, mask_thresh = cv2.threshold( fg_mask, 180, 255, cv2.THRESH_BINARY)
    # set the kernal
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    # Dialate & Erode
    mask_eroded = cv2.morphologyEx(mask_thresh, cv2.MORPH_OPEN, kernel)
    mask_dialated = cv2.dilate(mask_eroded, kernel, iterations=1)
    return mask_dialated
           
#find the drone using background substraction
def detect_drone():
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
            
if __name__ == "__main__":
    while True:
        detect_drone()
