#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty
import sys

# Calls the clear_costmaps ROS service to clear the costmaps when given a clearing command line argument different from 0.

clearing = sys.argv[1]

rospy.init_node('clear_costmap')
rospy.wait_for_service('/evader/move_base/clear_costmaps')


while True:
    if clearing != "0":
        srv = rospy.ServiceProxy('/evader/move_base/clear_costmaps', Empty)
        service_example = srv()

    rospy.sleep(4)
    