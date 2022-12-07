#!/usr/bin/env python3

# Send req to manager_startup with startupCheck.srv
# Handles e-stop via keyboard, drone errors, and regular commands
# Set timer v. last command and send "battery?" command if timer goes off
# Receives commands on topic droneCommand
# Send error responses on topic droneResp => messagetype String

import threading
import socket
import sys
import time
import rospy
from drone_106a.srv import startupCheck
from std_msgs.msg import String, Float64, Time, Int32

# GLOBALS
HOST_ADDR = ("", 9000)
TELLO_ADDR = ("192.168.10.1", 8889)
TELLO_SOCKET = None
CONTROLLER_READY = False
COMMAND_COMPLETE = False
LISTENER_SOCKET = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
LISTENER_SOCKET.bind(('', 9001))

def recv():  # receive data
	respPublisher = rospy.Publisher("droneResp", String, queue_size=10)
	count = 0
	while True:
		try:
			data, server = LISTENER_SOCKET.recvfrom(1024)
			data = data.decode(encoding="utf-8")
			if data:  # Received either OK or numeric value, i.e. command complete
				COMMAND_COMPLETE = True
				respPublisher.publish(data)
				print(data)
		except Exception:
			print("\nExit . . .\n")
			break


def issue_command(command):
	command = command.encode(encoding="utf-8")
	bytes_sent = TELLO_SOCKET.sendto(command, TELLO_ADDR)
	print(f"Sent {bytes_sent} to Tello")
	return

def controller_command(command):
	CONTROLLER_READY = True
	command = command.encode(encoding="utf-8")
	bytes_sent = TELLO_SOCKET.sendto(command, TELLO_ADDR)
	print(f"Sent {bytes_sent} to Tello")
	return

def cleanup():
	# issue_command('land')
	issue_command('command')
	TELLO_SOCKET.close()
	return

if __name__ == "__main__":
	print("driver")
	rospy.init_node('driver_node', anonymous = True)

	#OPEN & BIND SOCKET
	TELLO_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	TELLO_SOCKET.bind(HOST_ADDR)

	#Setup a thread to receive the OK?
	recvThread = threading.Thread(target=recv)
	recvThread.start()


	# Put the tello in commander mode
	# issue_command('command')

	curr_time = time.time()
	while time.time() - curr_time < 8:
		if COMMAND_COMPLETE:
			print("HELLO")
			break
	try:
		rospy.wait_for_service('driver_manager_startup')
		srvPrxy = rospy.ServiceProxy('driver_manager_startup', startupCheck)
		ack = srvPrxy(True)
		if not ack:
			cleanup()
			exit(1)
	except rospy.ServiceException as e:
		print(f"Drone startup service failed: {e}")
		cleanup()
		exit(2)
	rospy.loginfo('yo')
	#set up the subscriber with the relevant socket
	# command_sub = rospy.Subscriber(
	# 	"droneCommand", String, callback=controller_command
	# )
	tofPublisher = rospy.Publisher("droneTof", String, queue_size=10, latch=True)
	tofPublisher.publish(str(rospy.Time.now()))
	rospy.loginfo(f"{time.time()}")
	# issue_command('takeoff')
	try: 
		rospy.spin()
	except (rospy.exceptions.ROSException, KeyboardInterrupt) as e:
		print('cleaned up')
		cleanup()
