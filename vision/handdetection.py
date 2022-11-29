import os
import cv2
import numpy as np
import mediapipe as mp
from google.protobuf.json_format import MessageToDict

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

hands = None

def process(image):
  image_height, image_width, _ = image.shape

  # To improve performance, optionally mark the image as not writeable to
  # pass by reference.
  image.flags.writeable = False
  image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

  results = hands.process(image)

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
              
  return image
      

def init():
  global hands
  try:
    hands = mp_hands.Hands(
      model_complexity=1,
      min_detection_confidence=0.5,
      min_tracking_confidence=0.5)
  except Exception as e:
    print("Error initializing MediaPipe")