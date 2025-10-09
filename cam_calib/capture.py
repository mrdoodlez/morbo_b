#!/usr/bin/env python

import cv2

cap = cv2.VideoCapture('/dev/video0')
i = 0
while True:
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    k = cv2.waitKey(1)
    if k == ord(' '):  # press Space to capture
        cv2.imwrite(f'calib/img_{i:02d}.jpg', frame)
        print(f"Saved img_{i:02d}.jpg")
        i += 1
    elif k == 27:  # Esc to exit
        break
