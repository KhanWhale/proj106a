#!/usr/bin/env python3

# Send req to controller-manager-startup with startupCheck.srv
# Listen on topic handState for changes in input
# multithreaded
# Send final command on topic droneCommand

import socket
from threading import Thread, Lock


localIP     = "0.0.0.0"
statePort   = 8890
bufferSize  = 1024

# Create a datagram socket
UDPServerSocket = None 

# global state lock
lock = Lock()
# global state
currInput = None

# controller prev state
prevInput = None

def getSingleDatagram():
    
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]
    address = bytesAddressPair[1]
    print(message)
    print(address)
    return message

def droneBind():
    
    global UDPServerSocket

    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    
    # Bind to address and ip
    UDPServerSocket.bind((localIP, statePort))
    print("UDP server up and listening")

    # Need timeout
    getSingleDatagram()


def startup():
    
    # set up droneCommand topic

    # connect to drone
    droneBind()

    # Call startup service

# UDP state receive loop
def mainLoop():

    global prevInput

    while True:
        stateString = getSingleDatagram()
        stateDict = {}

        # parse string and populate stateDict

        myCurrInput = None

        if lock.acquire(blocking = False):
            myCurrInput = currInput
            lock.release()
        else:
            myCurrInput = prevInput

        commandString = computeControl(myCurrInput, stateDict)

        # publish on topic droneCommand 
        
        prevInput = myCurrInput


# main control fn
def computeControl(input, state):

    # MATH + CMD construction

    # return CMD

    pass

# CV input updater fn
def receiveCV():

    global currInput

    while True:

        # listen to topic handState

        recvInput = []

        lock.acquire()
        currInput = recvInput
        lock.release()


if __name__ == "__main__":
    print("controller")

    startup()    

    # create thread to receive and update CV input
    recvCVThread = Thread(target=receiveCV)
    recvCVThread.start()

    # create thread to loop on UDPServerSocket
    loopThread = Thread(target=mainLoop)
    loopThread.start() 

