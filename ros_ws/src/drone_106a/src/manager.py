#!/usr/bin/env python3

# Listen to following, once true received from all, send true response to all
# startupCheck.srv
# driver-manager-startup
# controller-manager-startup
# vision-manager-startup

import rospy
from drone_106a.srv import startupCheck

driver_ready = None
controller_ready = None
vision_ready = None

def waiting():
    global driver_ready
    global controller_ready
    global vision_ready
    return (driver_ready == None) or (controller_ready == None) or (vision_ready == None)

def success():
    global driver_ready
    global controller_ready
    global vision_ready
    return driver_ready and True and controller_ready and vision_ready

def driver_manager_startup(req):
    global driver_ready
    driver_ready = req.ready
    print("driver ready")
    while True:
        if not waiting():
            break
    return success()

def controller_manager_startup(req):
    global controller_ready
    controller_ready = req.ready
    print("controller ready")
    while True:
        if not waiting():
            break
    return success()

def vision_manager_startup(req):
    global vision_ready
    vision_ready = req.ready
    print("vision ready")
    while True:
        if not waiting():
            break
    return success()

if __name__ == "__main__":
    rospy.init_node('manager_server')
    rospy.loginfo("HI")
    s1 = rospy.Service('driver_manager_startup', startupCheck, driver_manager_startup)
    s2 = rospy.Service('controller_manager_startup', startupCheck, controller_manager_startup)
    s3 = rospy.Service('vision_manager_startup', startupCheck, vision_manager_startup)
    print("manager_startup service ready")
    rospy.spin()