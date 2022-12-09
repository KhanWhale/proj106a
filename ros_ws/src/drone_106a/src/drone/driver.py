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
tello_socket = None
controller_ready = False
command_complete = False
# LISTENER_SOCKET = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# LISTENER_SOCKET.bind(('', 9001))

def recv():  # receive data
	global command_complete
	respPublisher = rospy.Publisher("droneResp", String, queue_size=10)
	count = 0
	while True:
		try:
			# data, server = LISTENER_SOCKET.recvfrom(1024)
			data, server = tello_socket.recvfrom(1024)
			data = data.decode(encoding="utf-8")
			if data:  # TODO: TEST THAT => Received either OK or numeric value, i.e. command complete
				command_complete = True
				respPublisher.publish(data)
				print(data)
		except Exception:
			print("\nExit . . .\n")
			break


def issue_command(command):
	command = command.encode(encoding="utf-8")
	bytes_sent = tello_socket.sendto(command, TELLO_ADDR)
	print(f"Sent {bytes_sent}: {command} to Tello")
	return

def controller_command(command):
	global controller_ready
	controller_ready = True
	command = str(command.data)
	print(command[3:])
	command = command.encode(encoding="utf-8")
	bytes_sent = tello_socket.sendto(command, TELLO_ADDR)
	# print(f"Sent {bytes_sent}: {command} to Tello")
	return

def cleanup():
	issue_command('rc 0 0 0 0')
	issue_command('land')
	issue_command('command')
	tello_socket.close()
	return

def startup():
	global tello_socket

	#OPEN & BIND SOCKET
	tello_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	tello_socket.bind(HOST_ADDR)


if __name__ == "__main__":
	print("driver")
	rospy.init_node('driver_node', anonymous = True)

	startup()

	#Setup a thread to receive the OK?
	recvThread = threading.Thread(target=recv)
	recvThread.start()

	curr_time = time.time()
	issue_command('command')
	while time.time() - curr_time < 8:
		# Put the tello in commander mode
		issue_command('battery?')
		time.sleep(0.5)
		print(f'startup command status {command_complete}')
		if command_complete:
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
	# set up the subscriber with the relevant socket
	command_sub = rospy.Subscriber(
		"droneCommand", String, callback=controller_command
	)
	
	issue_command('rc 0 0 0 0')
	issue_command('takeoff')
	issue_command('speed 20')
	tofPublisher = rospy.Publisher("droneTof", String, queue_size=9, latch=True)
	tofPublisher.publish(str(time.time()))
	rospy.loginfo(f"{time.time()}")
	try: 
		rospy.spin()
	except (rospy.exceptions.ROSException, KeyboardInterrupt) as e:
		print('cleaned up')
		cleanup()
