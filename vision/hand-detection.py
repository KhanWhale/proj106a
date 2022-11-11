import os
import sys
import time
import cv2
import mediapipe as mp
from google.protobuf.json_format import MessageToDict

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

        for (hand_landmarks, handedness) in zip(results.multi_hand_landmarks, results.multi_handedness):
            handedness_dict = MessageToDict(handedness)
            print(f"{handedness_dict['classification'][0]['label']} Hand Keypoints Detected:")
            for keypoint in mp_hands.HandLandmark:
                print(f"{keypoint.name}: ({hand_landmarks.landmark[keypoint.value].x * image_width}, {hand_landmarks.landmark[keypoint.value].y * image_height}, {hand_landmarks.landmark[keypoint.value].z})")
            print("\n\n")

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