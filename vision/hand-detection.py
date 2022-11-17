import os
import sys
import time
import cv2
import numpy as np
import mediapipe as mp
from google.protobuf.json_format import MessageToDict
from scipy.linalg import lstsq

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

start_time = time.time()
fps_calc_window = 1
num_frames = 0
fps = 0

# For selecting the camera input source
# (0 is the built-in camera, 1 is the first external camera, etc.)
camera_input = 0 if len(sys.argv) < 2 else int(sys.argv[1])
cap = cv2.VideoCapture(camera_input)

with mp_hands.Hands(
    model_complexity=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:

  while cap.isOpened():
    success, image = cap.read()

    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue

    image_height, image_width, _ = image.shape

    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(image)

    num_frames += 1
    if (time.time() - start_time) > fps_calc_window:
        fps = num_frames / fps_calc_window
        num_frames = 0
        start_time = time.time()        

    # Draw the hand annotations on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if results.multi_hand_landmarks:
        os.system('clear')
        x_coords = np.array([])
        y_coords = np.array([])
        z_coords = np.array([])
        for (hand_landmarks, handedness) in zip(results.multi_hand_landmarks, results.multi_handedness):
            handedness_dict = MessageToDict(handedness)
            print(f"{handedness_dict['classification'][0]['label']} Hand Keypoints Detected:")
            for keypoint in mp_hands.HandLandmark:
                print(f"{keypoint.name}: ({hand_landmarks.landmark[keypoint.value].x * image_width}, {hand_landmarks.landmark[keypoint.value].y * image_height}, {hand_landmarks.landmark[keypoint.value].z})")
            print("\n\n")

            # get coordinates
            x_coords = np.append(x_coords, np.array([hand_landmarks.landmark[i].x for i in range(21)]))
            y_coords = np.append(y_coords, np.array([hand_landmarks.landmark[i].y for i in range(21)]))
            z_coords = np.append(z_coords, np.array([hand_landmarks.landmark[i].z for i in range(21)]))
            print("\nx_coords", x_coords)
            print("\ny_coords", y_coords)
            print("\nz_coords", z_coords)

            # set up linear system
            ones = np.repeat(1, len(x_coords))
            A = np.concatenate((x_coords[:,np.newaxis], y_coords[:,np.newaxis], ones[:,np.newaxis]),axis=1)
            b = z_coords
            plane_coeffs, residual, rnk, s = lstsq(A, b)

            # get best fit plane
            X, Y = np.linspace(np.min(x_coords), np.max(x_coords), 10), np.linspace(np.min(y_coords), np.max(y_coords), 10)
            best_fit_plane = plane_coeffs[0]*X + plane_coeffs[1]*Y + plane_coeffs[2]
            print("\n best fit plane", best_fit_plane)

            thumb_tip = np.array([hand_landmarks.landmark[4].x, hand_landmarks.landmark[4].y])
            index_tip = np.array([hand_landmarks.landmark[8].x, hand_landmarks.landmark[8].y])
            middle_tip = np.array([hand_landmarks.landmark[12].x, hand_landmarks.landmark[12].y])
            ring_tip = np.array([hand_landmarks.landmark[16].x, hand_landmarks.landmark[16].y])
            pinky_tip = np.array([hand_landmarks.landmark[20].x, hand_landmarks.landmark[20].y])

            index_distance = np.linalg.norm(index_tip - thumb_tip)
            middle_distance = np.linalg.norm(middle_tip - thumb_tip)
            ring_distance = np.linalg.norm(ring_tip - thumb_tip)
            pinky_distance = np.linalg.norm(pinky_tip - thumb_tip)

            print(f"Index Distance: {index_distance}")
            print(f"Middle Distance: {middle_distance}")
            print(f"Ring Distance: {ring_distance}")
            print(f"Pinky Distance: {pinky_distance}")

            touch_gesture_threshold = 0.05
            
            if index_distance < touch_gesture_threshold:
              print(f"\nINDEX GESTURE\n")
            elif middle_distance < touch_gesture_threshold:
              print(f"\nMIDDLE GESTURE\n")
            elif ring_distance < touch_gesture_threshold:
              print(f"\nRING GESTURE\n")
            elif pinky_distance < touch_gesture_threshold:
              print(f"\nPINKY GESTURE\n")

            mp_drawing.draw_landmarks(
                image,
                hand_landmarks,
                mp_hands.HAND_CONNECTIONS,
                mp_drawing_styles.get_default_hand_landmarks_style(),
                mp_drawing_styles.get_default_hand_connections_style())

        print(f"FPS: {fps}")

    # Flip the image horizontally for a selfie-view display.
    flipped = cv2.flip(image, 1)
    cv2.imshow('MediaPipe Hands', flipped)
    if cv2.waitKey(5) & 0xFF == 27:
      break

cap.release()