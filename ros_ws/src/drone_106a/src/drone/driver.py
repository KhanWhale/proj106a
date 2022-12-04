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
from drone_106a.msg import droneCommand

# GLOBALS
HOST_ADDR = ("", 9000)
TELLO_ADDR = ("192.168.10.1", 8889)
COMMAND_COMPLETE = False
RC_COMMAND = False


def recv(sock):  # receive data
    respPublisher = rospy.Publisher("droneResp", String)
    count = 0
    while True:
        try:
            data, server = sock.recvfrom(1518)
            data = data.decode(encoding="utf-8")
            if data:  # Received either OK or numeric value, i.e. command complete
                COMMAND_COMPLETE = True
                respPublisher.publish(data)
                print(data)
        except Exception:
            print("\nExit . . .\n")
            break


def issue_command(command, callback_args):
    sock = callback_args[0]
    command = command.encode(encoding="utf-8")
    bytes_sent = sock.sendto(command, TELLO_ADDR)
    print(f"Sent {bytes_sent} to Tello")
    return


if __name__ == "__main__":
    print("driver")

    # STEP 1: BIND SOCKET
    tello_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tello_socket.bind(HOST_ADDR)

    # STEP 2: Setup a thread to receive the OK?
    recvThread = threading.Thread(target=recv)
    recvThread.start()

    # STEP 3: have a command issuing loop
    # 3a - enter command mode
    tello_socket.sendto("command".encode(encoding="utf-8"))  # enter command mode
    command_sub = rospy.Subscriber(
        "droneCommand", droneCommand, callback=issue_command, callback_args=[tello_socket]
    )
    tello_socket.sendto("command".encode(encoding="utf-8"))  # exit command mode
    tello_socket.close()
