#!/usr/bin/env python3

# Send req to driver-manager-startup with startupCheck.srv
# Handles e-stop via keyboard, drone errors, and regular commands
# Set timer v. last command and send "battery?" command if timer goes off
# Receives commands on topic droneCommand
# Send error responses on topic droneResp => messagetype String

import threading
import socket
import sys
import time
import rospy
from std_msgs.msg import String

# GLOBALS
HOST_ADDR = ("", 9000)
TELLO_ADDR = ("192.168.10.1", 8889)
TELLO_SOCKET = None
CONTROLLER_READY = False


def recv():  # receive data
    respPublisher = rospy.Publisher("droneResp", String, queue_size=10)
    count = 0
    while True:
        try:
            data, server = TELLO_SOCKET.recvfrom(1518)
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
    issue_command('land')
    issue_command('command')
    TELLO_SOCKET.close()
    return

if __name__ == "__main__":
    print("driver")
    rospy.init_node('driver_node')

    #OPEN & BIND SOCKET
    TELLO_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    TELLO_SOCKET.bind(HOST_ADDR)

    #Setup a thread to receive the OK?
    recvThread = threading.Thread(target=recv)
    recvThread.start()

    # Put the tello in commander mode
    issue_command('command', TELLO_SOCKET)

    #set up the subscriber with the relevant socket
    command_sub = rospy.Subscriber(
        "droneCommand", String, callback=controller_command
    )

    while not CONTROLLER_READY: #haven't received first command
            issue_command('battery?')
            time.sleep(7)

    try: 
        rospy.spin()
    except (rospy.exceptions.ROSException, KeyboardInterrupt) as e:
        cleanup()
