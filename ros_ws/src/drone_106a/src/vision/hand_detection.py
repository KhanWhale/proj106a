import os
import cv2
from enum import Enum
import numpy as np
import mediapipe as mp
from google.protobuf.json_format import MessageToDict

class Gesture(Enum):
	INDEX = 1
	MIDDLE = 2
	RING = 3
	PINKY = 4

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

	landmarks, gestures = {}, {}

	if results.multi_hand_landmarks and len(results.multi_hand_landmarks) == 2:
		for (hand_landmarks, handedness) in zip(results.multi_hand_landmarks, results.multi_handedness):
			handedness_dict = MessageToDict(handedness)
			which_hand = handedness_dict['classification'][0]['label']
			# print(f"{which_hand} Keypoints Detected")
			landmarks[which_hand] = hand_landmarks
			thumb_tip = np.array([hand_landmarks.landmark[4].x, hand_landmarks.landmark[4].y])
			index_tip = np.array([hand_landmarks.landmark[8].x, hand_landmarks.landmark[8].y])
			middle_tip = np.array([hand_landmarks.landmark[12].x, hand_landmarks.landmark[12].y])
			ring_tip = np.array([hand_landmarks.landmark[16].x, hand_landmarks.landmark[16].y])
			pinky_tip = np.array([hand_landmarks.landmark[20].x, hand_landmarks.landmark[20].y])

			index_distance = np.linalg.norm(index_tip - thumb_tip)
			middle_distance = np.linalg.norm(middle_tip - thumb_tip)
			ring_distance = np.linalg.norm(ring_tip - thumb_tip)
			pinky_distance = np.linalg.norm(pinky_tip - thumb_tip)

			touch_gesture_threshold = 0.05
			if index_distance < touch_gesture_threshold:
				gestures[which_hand] = Gesture.INDEX
				print(f"gesture hand is {which_hand}")
			elif middle_distance < touch_gesture_threshold:
				gestures[which_hand] = Gesture.MIDDLE
				print(f"gesture hand is {which_hand}")
			elif ring_distance < touch_gesture_threshold:
				gestures[which_hand] = Gesture.RING
				print(f"gesture hand is {which_hand}")
			elif pinky_distance < touch_gesture_threshold:
				gestures[which_hand] = Gesture.PINKY
				print(f"gesture hand is {which_hand}")

			mp_drawing.draw_landmarks(
					image,
					hand_landmarks,
					mp_hands.HAND_CONNECTIONS,
					mp_drawing_styles.get_default_hand_landmarks_style(),
					mp_drawing_styles.get_default_hand_connections_style())
	else:
		landmarks, gestures = None, None
						
	return image, landmarks, gestures
			

def init():
	global hands
	try:
		hands = mp_hands.Hands(
			max_num_hands=2,
			model_complexity=1,
			min_detection_confidence=0.5,
			min_tracking_confidence=0.5)
	except Exception as e:
		print("Error initializing MediaPipe")
