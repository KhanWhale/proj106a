#!/usr/bin/env python3

# Send req to manager_startup with startupCheck.srv
# Listen on topic handState for changes in input
# multithreaded
# Send final command on topic droneCommand

import rospy
import socket
from std_msgs.msg import String, Time, Float64, Int32
from threading import Thread, Lock
from drone_106a.msg import handState
from drone_106a.srv import startupCheck
from controllerClass import controllerClass
import numpy as np

localIP     = "0.0.0.0"
statePort   = 8890
bufferSize  = 1024

# Create a datagram socket
UDPServerSocket = None 
destinationSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)


# global state lock
lock = Lock()
# global state
currInput = None

# controller prev state
DEFAULTINPUT = {'roll': 0, 'pitch': 0, 'yaw': 0, 'h': 30}
prevInput = DEFAULTINPUT
commandPublisher = None

controllerObj = None

def getSingleDatagram():
    
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]
    destinationSocket.sendto(message, ('', 9001))
    message = message.decode()[: -2]
    address = str(bytesAddressPair[1])
    print(message)
    rospy.loginfo('pls' + message)
    # print(address)
    return message

def droneBind():
    
    global UDPServerSocket

    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    
    # Bind to address and ip
    UDPServerSocket.bind((localIP, statePort))
    UDPServerSocket.sendto('command'.encode('utf-8'), ('192.168.10.1', 8889))

    rospy.loginfo("UDP server up and listening")

    # TODO: Need timeout
    getSingleDatagram()


def startup():

    global commandPublisher, controllerObj

    # set up droneCommand topic
    commandPublisher = rospy.Publisher('droneCommand', String, queue_size=10)

    # connect to drone
    droneBind()

    # ROLL, PITCH, YAW, HEIGHT
    Kp = np.array([1.0, 1.0, 1.0, 1.0])
    Kd = np.array([1.0, 1.0, 1.0, 1.0])
    Ki = np.array([1.0, 1.0, 1.0, 1.0])
    Kth = np.array([10, 10, 10, 10])
    heightScale = 1.0

    # set up controller object
    controllerObj = controllerClass(Kp, Kd, Ki, Kth, heightScale)

    # Call startup service
    rospy.wait_for_service('controller_manager_startup')
    rospy.loginfo("UR MOM")
    try:
        srvPrxy = rospy.ServiceProxy('controller_manager_startup', startupCheck)
        rospy.loginfo("UR MOM 1.5")
        ack = srvPrxy(True)
        rospy.loginfo("UR MOM 2")
        if not ack:
            print("Controller received ack = FALSE")
    except rospy.ServiceException as e:
        print(f"Controller startup service failed: {e}")
    
    start = rospy.wait_for_message('droneTof', String)
    rospy.loginfo(start)
    rospy.loginfo('food')
    controllerObj.set_start(start)

def parser(msg):
    dict = {}
    line = msg.split(";")
    for pair in line:
        if (len(pair) == 2):
            splits = pair.split(":")
            dict[splits[0]] = float(splits[1])
    return dict

# UDP state receive loop
def mainLoop():

    global prevInput

    while True:
        stateString = getSingleDatagram()
        rospy.loginfo("YEET" + stateString)
        # parse string and populate stateDict
        stateDict = parser(stateString)
        commandPublisher.publish('rc 0 0 0 0')

        # myCurrInput = None

        # if lock.acquire(blocking = False):
        #     if currInput is not None and currInput != {}:
        #         myCurrInput = currInput
        #     else:
        #         myCurrInput = prevInput
        #     lock.release()
        # else:
        #     myCurrInput = prevInput

        # rospy.loginfo(myCurrInput)
        # commandString = computeControl(myCurrInput, stateDict)

        # # publish on topic droneCommand
        # # commandPublisher.publish(commandString) 
        # commandPublisher.publish('rc 0 0 0 0')
        
        # prevInput = myCurrInput


# TODO main control fn
# both are dicts!
def computeControl(input, state):

    # MATH + CMD construction

    # return CMD

    return controllerObj.control_wrapper(input, state, log=True)

def reCVcallback(hs):

    global currInput
    
    recvInput = {'roll': hs.roll, 'pitch': hs.pitch, 'yaw': hs.yaw, 'h': hs.height}

    rospy.loginfo(recvInput)

    lock.acquire()
    currInput = recvInput
    lock.release()

# CV input updater fn
def receiveCV():
    # listen to topic handState
    rospy.Subscriber("handState", handState, reCVcallback)

if __name__ == "__main__":
    
    rospy.init_node('controller', anonymous=True)

    print("controller")

    # return # TODO: REMOVE WHEN READY

    startup()    

    # create thread to receive and update CV input
    recvCVThread = Thread(target=receiveCV)
    recvCVThread.start()


    # create thread to loop on UDPServerSocket
    loopThread = Thread(target=mainLoop)
    loopThread.start() 

    try:
        rospy.spin()
    except (rospy.exceptions.ROSException, KeyboardInterrupt) as e:
        UDPServerSocket.close()
        exit(0)
    


