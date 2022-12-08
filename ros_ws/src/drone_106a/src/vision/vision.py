#!/usr/bin/env python3

# Send req to vision-manager-startup with startupCheck.srv 
# Publish handState on topic handState

import os
import sys
import time
import cv2
import numpy as np
import mediapipe as mp
import hand_detection
import pyrealsense2 as rs
from scipy.linalg import lstsq
import matplotlib.pyplot as plt
import hand_orientation
from graph import *
import rospy
from drone_106a.msg import handState
from drone_106a.srv import startupCheck

pipeline = rs.pipeline()
config = rs.config()
roll_max = 0
roll_min = 0
pitch_max = 0
pitch_min = 0
yaw_max = 0
yaw_min = 0
yaw_mean = []
h_max = 0
h_min = 0
h_mean = []
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
	if s.get_info(rs.camera_info.name) == 'RGB Camera':
		found_rgb = True
		break
if not found_rgb:
	print("The demo requires Depth camera with Color sensor")
	exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
	config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
	config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

align_to = rs.stream.color
align = rs.align(align_to)

hand_detection.init()

# FPS calculation initialization
start_time = time.time()
fps_calc_window = 1
num_frames = 0
fps = 0
base_frame = False

rospy.init_node('vision_node', anonymous=True)
pub = rospy.Publisher('handState', handState, queue_size=50)
handstate_msg = handState()

def visionLoop(breakOnHand):

	global roll_max, roll_min, pitch_max, pitch_min, yaw_max, yaw_min, h_max, h_min, yaw_mean, h_mean

	rospy.loginfo(f"Break ON Hand : {breakOnHand}")
	global base_frame, num_frames, start_time
	j = 0
	try:
		while True:
			x_coords = np.array([])
			y_coords = np.array([])
			z_coords = np.array([])
			if not base_frame:
				print("\n\n\n *************** Smart Jedi X-Wing Starfighter *************** \n\nPlease place your hand in the frame to initialize the base frame\n\n")
				#cv2.waitKey(10)
				time.sleep(3)
				base_frame=True
			frames = pipeline.wait_for_frames()
			aligned_frames = align.process(frames)

			aligned_depth_frame = aligned_frames.get_depth_frame()
			color_frame = aligned_frames.get_color_frame()

			# Validate that both frames are valid
			if not aligned_depth_frame or not color_frame:
				continue

			depth_image = np.asanyarray(aligned_depth_frame.get_data())
			color_image = np.asanyarray(color_frame.get_data())
			image_height, image_width, _ = color_image.shape

			processed_img, hand_landmarks, gesture = hand_detection.process(color_image)
			if hand_landmarks and len(hand_landmarks.keys()) == 2: #make sure we only process when both visible
				j += 1

				#LEFT HAND
				landmarks = hand_landmarks['Left'].landmark
				depths = []
				for i in range(21): # keypoint in mp.solutions.hands.HandLandmark
					pixel_x, pixel_y = landmarks[i].x * image_width, landmarks[i].y * image_height
					rounded_x, rounded_y = round(pixel_x), round(pixel_y)
					depth = 0
					if rounded_x >= 0 and rounded_x < 480 and rounded_y >= 0 and rounded_y < 480:
						depth = depth_image[rounded_y][rounded_x]
					x_coords = np.append(x_coords, landmarks[i].x)
					y_coords = np.append(y_coords, landmarks[i].y)
					z_coords = np.append(z_coords, landmarks[i].z)
					depths = np.append(depths, depth)
				
				depths[depths == 0] = np.mean(depths)
				depths[depths == None] = np.mean(depths)
				#z_coords = z_coords

				# angle_1, angle_2, angle_3 = hand_orientation.get_angles(x_coords, y_coords, z_coords)
				_, roll, _ = hand_orientation.get_angles(x_coords, y_coords, z_coords)
				mean_depth_left = np.mean(depths) / 10

				#RIGHT HAND
				landmarks = hand_landmarks['Right'].landmark
				depths = []
				for i in range(21): # keypoint in mp.solutions.hands.HandLandmark
					pixel_x, pixel_y = landmarks[i].x * image_width, landmarks[i].y * image_height
					rounded_x, rounded_y = round(pixel_x), round(pixel_y)
					depth = 0
					if rounded_x >= 0 and rounded_x < 480 and rounded_y >= 0 and rounded_y < 480:
						depth = depth_image[rounded_y][rounded_x]
					x_coords = np.append(x_coords, landmarks[i].x)
					y_coords = np.append(y_coords, landmarks[i].y)
					z_coords = np.append(z_coords, landmarks[i].z)
					depths = np.append(depths, depth)
				
				depths[depths == 0] = np.mean(depths)
				depths[depths == None] = np.mean(depths)
				#z_coords = z_coords

				# angle_1, angle_2, angle_3 = hand_orientation.get_angles(x_coords, y_coords, z_coords)
				_, pitch, _ = hand_orientation.get_angles(x_coords, y_coords, z_coords)
				mean_depth_right = np.mean(depths) / 10


				if breakOnHand:

					roll_max = max(roll, roll_max)
					roll_min = min(roll, roll_min)

					pitch_min = min(pitch, pitch_min)
					pitch_max = max(pitch, pitch_max)

					h_max = max(mean_depth_left, h_max)
					h_min = min(mean_depth_left, h_min)
					h_mean.append(mean_depth_left)

					yaw_max = max(mean_depth_right, yaw_max)
					yaw_min = min(mean_depth_right, yaw_min)
					yaw_mean.append(mean_depth_right)

					print("\n Angle 1 (Yaw):", mean_depth_right)
					print("\n Angle 2 (Pitch):", pitch)
					print("\n Angle 3 (Roll):", roll)
					print("\n Mean Depth:", mean_depth_left)

					
					print("\n\n")
					flipped = cv2.flip(processed_img, 1)
					cv2.imshow('MediaPipe Hands', flipped)
					if cv2.waitKey(5) & 0xFF == 27:
						break

					if j < 50: 
						continue
					else:
						yaw_mean = np.mean(yaw_mean)
						h_mean = np.mean(h_mean)
						print(f"rolls : {roll_min} {roll_max}")
						print(f"pitches : {pitch_min} {pitch_max}")
						print(f"yaws : {yaw_min} {yaw_max}")
						print(f"hs : {h_min} {h_max}")

						return


				#YAW AND HEIGHT BOTH STRICTLY POSITIVE, MAP TO WITHIN RANGE
				#clip the heights 
				yaw = int(yaw_min if mean_depth_right < yaw_min else min(mean_depth_right, yaw_max))
				roll = roll_min if roll < roll_min else min(roll, roll_max)
				pitch = pitch_min if pitch < pitch_min else min(pitch, pitch_max)
				height = (h_min if mean_depth_left < h_min else min(mean_depth_left, h_max)) - h_mean #height should be a delta

					
				print("\n Angle 1 (Yaw):", yaw)
				print("\n Angle 2 (Pitch):", pitch)
				print("\n Angle 3 (Roll):", roll)
				print("\n Mean Depth:", height)
				print("\n\n")

				handstate_msg.roll = roll
				handstate_msg.pitch = pitch
				handstate_msg.yaw = yaw
				handstate_msg.height = height
				print(gesture)
				handstate_msg.gestureLeft = gesture['Left'].value if 'Left' in gesture and gesture['Left'].value else 0 
				handstate_msg.gestureRight = gesture['Right'].value if 'Right' in gesture and gesture['Right'].value else 0 
				pub.publish(handstate_msg)
				rospy.loginfo(str(handstate_msg))

			if gesture:
				print(gesture)
			
			num_frames += 1
			if (time.time() - start_time) > fps_calc_window:
				fps = num_frames / fps_calc_window
				num_frames = 0
				start_time = time.time()        

			#print(f"FPS: {fps}")
					#os.system('clear')
			# Flip the image horizontally for a selfie-view display.
			flipped = cv2.flip(processed_img, 1)
			cv2.imshow('MediaPipe Hands', flipped)
			if cv2.waitKey(5) & 0xFF == 27:
				break

	finally:
		if not breakOnHand:
			pipeline.stop()


visionLoop(True)

try:
	rospy.wait_for_service('vision_manager_startup')
	srvPrxy = rospy.ServiceProxy('vision_manager_startup', startupCheck)
	ack = srvPrxy(True)
	if not ack:
		pipeline.stop()
		exit(1)
except rospy.ServiceException as e:
	print(f"Drone startup service failed: {e}")
	pipeline.stop()
	exit(2)

visionLoop(False)
if __name__ == "__main__":
	print("vision")

	