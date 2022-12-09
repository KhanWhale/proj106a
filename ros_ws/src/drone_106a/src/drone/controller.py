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
import time
import numpy as np
from queue import Queue

localIP     = "0.0.0.0"
statePort   = 8890
bufferSize  = 1024
landrecvd = False
# Create a datagram socket
UDPServerSocket = None 
# destinationSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)


# global state lock
# lock = Lock()
# global state
# currInput = None

# controller prev state
# DEFAULTINPUT = {'roll': 0, 'pitch': 0, 'yaw': 0, 'h': 30}
prevInput = None # DEFAULTINPUT
commandPublisher = None
droneRespSubscriber = None
state = {}
state_vec = np.zeros((10, 1))
xyz = None

controllerObj = None

q = Queue(10)

discrete_cmd = None
discrete_cmd_options = ["takeoff", "land", "flip l", "flip r", "flip f", "flip b", "ESTOP"]

def droneRespCallback(droneResp):
    global discrete_cmd
    if discrete_cmd != "ESTOP":
        if droneResp.data == "ok" or droneResp.data == 'error':
            discrete_cmd = None

def getSingleDatagram():

    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]
    # destinationSocket.sendto(message, ('', 9001))
    message = message.decode()[: -2]
    address = str(bytesAddressPair[1])
    print(message)
    rospy.loginfo(message)
    # print(address)
    return message

def droneBind():
    
    global UDPServerSocket

    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    
    # Bind to address and ip
    UDPServerSocket.bind((localIP, statePort))
    # UDPServerSocket.sendto('command'.encode('utf-8'), ('192.168.10.1', 8889))

    rospy.loginfo("UDP server up and listening")

    # TODO: Need timeout
    getSingleDatagram()


def startup():

    global commandPublisher, controllerObj, droneRespSubscriber

    # set up droneCommand topic
    commandPublisher = rospy.Publisher('droneCommand', String, queue_size=10)

    droneRespSubscriber = rospy.Subscriber("droneResp", String, callback=droneRespCallback)

    # connect to drone
    droneBind()

    # ROLL, PITCH, YAW, HEIGHT
    Kp = np.array([1.0, 1.0, 1.0, 1.0])
    Kd = np.array([1.0, 1.0, 1.0, 1.0])
    Ki = np.array([1.0, 1.0, 1.0, 1.0])
    Kth = np.array([20, 20, 20, 20])
    heightScale = 1.0

    # set up controller object
    controllerObj = controllerClass(Kp, Kd, Ki, Kth, heightScale)

    # Call startup service
    rospy.wait_for_service('controller_manager_startup')
    try:
        srvPrxy = rospy.ServiceProxy('controller_manager_startup', startupCheck)
        ack = srvPrxy(True)
        rospy.loginfo(f"received {ack} from manager")
        if not ack:
            print("Controller received ack = FALSE")
    except rospy.ServiceException as e:
        print(f"Controller startup service failed: {e}")
    
    start_time = rospy.wait_for_message('droneTof', String)
    rospy.loginfo(f'recevied start_time = {float(start_time.data)} ')
    controllerObj.set_start(start_time.data)

def parser(msg):
    global state
    if 'z' not in state:
        state['z'] = 0
        state['x'] = 0
        state['y'] = 0
    else:
        state['z'] = state['z'] + state['vgz']*0.1 
        state['y'] = state['y'] + state['vgy']*0.1 + (state['agy']/2)*(0.1**2)
        state['x'] = state['x'] + state['vgx']*0.1 + (state['agx']/2)*(0.1**2)
    line = msg.split(";")
    for pair in line:
        splits = pair.split(":")
        if (len(splits) == 2):
            state[splits[0]] = float(splits[1])

    #have the dictionary of values
    return 

def stateLoop():
    rospy.loginfo("entered stateLoop")
    while True:
        stateString = getSingleDatagram()
        parser(stateString)

def mainCallback(hs):
    global landrecvd
    recvInput = {'roll': hs.roll, 'pitch': hs.pitch, 'yaw': hs.yaw, 'h': hs.height, 'gestureLeft': hs.gestureLeft, 'gestureRight' : hs.gestureRight}

    rospy.loginfo(f'recvInput = {recvInput}')

    if recvInput and state:

        commandString = computeControl(recvInput, state.copy())
        
    else:

        rospy.loginfo(f"state = {state}")

        commandString = "rc 0 0 0 0"

    rospy.loginfo(f"commandString = {commandString}")

    # publish on topic droneCommand
    #if commandString == "land":
    #    commandPublisher.publish('rc 0 0 0 0') 
    global discrete_cmd
    if not discrete_cmd:
        if commandString in discrete_cmd_options:
            discrete_cmd = commandString
            commandPublisher.publish('rc 0 0 0 0')
            commandPublisher.publish(discrete_cmd)
        else:
            commandPublisher.publish(commandString) 
            #commandPublisher.publish('rc 0 0 0 0')
    elif discrete_cmd == "ESTOP":
        commandPublisher.publish('rc 0 0 0 0')


# # UDP state receive loop
# def mainLoop():

#     rospy.loginfo("entered mainLoop")

#     global prevInput

#     while True:
#         stateString = getSingleDatagram()
#         # parse string and populate stateDict
#         stateDict = parser(stateString)
#         # commandPublisher.publish('rc 0 0 0 0')

#         myCurrInput = None
#         global q
#         if q.empty():
#             myCurrInput = prevInput
#         else:
#             myCurrInput = q.get()

#         rospy.loginfo(f'myCurrInput = {myCurrInput}')

#         if myCurrInput:

#             commandString = computeControl(myCurrInput, stateDict)

#             prevInput = myCurrInput
        
#         else:

#             commandString = "rc 0 0 0 0"

#         rospy.loginfo(f"commandString = {commandString}")

#         # publish on topic droneCommand
#         commandPublisher.publish(commandString) 
#         # commandPublisher.publish('rc 0 0 0 0')


# TODO main control fn
# both are dicts!
def computeControl(input, state):

    # MATH + CMD construction

    # return CMD

    return controllerObj.control_wrapper(input, state, log=False)

# def reCVcallback(hs):

#     global currInput
    
#     recvInput = {'roll': hs.roll, 'pitch': hs.pitch, 'yaw': hs.yaw, 'h': hs.height, 'gesture': hs.gesture}

#     # rospy.loginfo(f"CV gave {str(recvInput)}")

#     rospy.loginfo("putting into queue")

#     q.put(recvInput)

# CV input updater fn
# def receiveCV():
#     rospy.loginfo("Entered recv handState loop")
#     # listen to topic handState
#     rospy.Subscriber("handState", handState, reCVcallback)

#     try:
#         rospy.spin()
#     except (rospy.exceptions.ROSException, KeyboardInterrupt) as e:
#         UDPServerSocket.close()
#         exit(0)

if __name__ == "__main__":
    
    rospy.init_node('controller', anonymous=True)

    print("controller")

    # return # TODO: REMOVE WHEN READY

    startup()    

    # # create thread to receive and update CV input
    # recvCVThread = Thread(target=receiveCV)
    # recvCVThread.start()


    # # create thread to loop on UDPServerSocket
    # loopThread = Thread(target=mainLoop)
    # loopThread.start() 

    # create thread to loop on UDPServerSocket
    loopThread = Thread(target=stateLoop)
    loopThread.start() 

    rospy.loginfo("Entered recv handState loop")    

    # listen to topic handState
    rospy.Subscriber("handState", handState, mainCallback)

    try:
        rospy.spin()
    except (rospy.exceptions.ROSException, KeyboardInterrupt) as e:
        UDPServerSocket.close()
        exit(0)

