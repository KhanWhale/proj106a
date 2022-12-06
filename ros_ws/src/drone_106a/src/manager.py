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
    return (driver_ready == None) or (controller_ready == None) or (vision_ready == None)

def success():
    return driver_ready and controller_ready and vision_ready

def driver_manager_startup(req):
    global driver_ready
    driver_ready = req.ready
    while True:
        if not waiting():
            break
    return startupCheck(success())


if __name__ == "__main__":
    rospy.init_node('manager_server')
    s = rospy.Service('driver-manager-startup', startupCheck, driver_manager_startup)
    print("driver-manager-startup service ready")
    rospy.spin()


# Example usage (need to modify) for testing
# rospy.wait_for_service('add_two_ints')
# try:
#     add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
#     resp1 = add_two_ints(x, y)
#     return resp1.sum
# except rospy.ServiceException as e:
#     print("Service call failed: %s"%e)