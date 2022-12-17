import os
import sys
import time
import cv2
import numpy as np
import handdetection

start_time = time.time()
fps_calc_window = 1
num_frames = 0
fps = 0

# For selecting the camera input source
# (0 is the built-in camera, 1 is the first external camera, etc.)
camera_input = 0 if len(sys.argv) < 2 else int(sys.argv[1])
cap = cv2.VideoCapture(camera_input)

handdetection.init()

while cap.isOpened():
    success, image = cap.read()

    if not success:
        print("Ignoring empty camera frame.")
        # If loading a video, use 'break' instead of 'continue'.
        continue

    processed = handdetection.process(image)

    num_frames += 1
    if (time.time() - start_time) > fps_calc_window:
        fps = num_frames / fps_calc_window
        num_frames = 0
        start_time = time.time()        

    print(f"FPS: {fps}")

    # Flip the image horizontally for a selfie-view display.
    flipped = cv2.flip(processed, 1)
    cv2.imshow('MediaPipe Hands', flipped)
    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()