#!/usr/bin/env python3
import cv2
import numpy as np

#open default 
cap = cv2.VideoCapture(0)
backSub = cv2.createBackgroundSubtractorMOG2(history=1000)

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
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        # Dialate & Erode
        mask_eroded = cv2.morphologyEx(mask_thresh, cv2.MORPH_OPEN, kernel)
        mask_dialated = cv2.dilate(mask_eroded, kernel, iterations=1)

        contours, hierarchy = cv2.findContours(mask_dialated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print(contours)
        frame_ct = cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
        min_contour_area = 15000  # Define your minimum area threshold
        max_contour_area = 200000  
        large_contours = [cnt for cnt in contours if (cv2.contourArea(cnt) > min_contour_area and cv2.contourArea(cnt) < max_contour_area)]
        frame_out = frame.copy()
        for cnt in large_contours:
            x, y, w, h = cv2.boundingRect(cnt)
            frame_out = cv2.rectangle(frame_out, (x, y), (x+w, y+h), (0, 0, 200), 3)
        # Display the resulting frame
        cv2.imshow('Frame_final', frame_out)
        cv2.waitKey(1)
            
if __name__ == "__main__":
    while True:
        detect_drone()
