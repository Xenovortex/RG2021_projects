#! /usr/bin/env python3

import rospy
 
from std_srvs.srv import Empty
 
import sys
 
rospy.init_node('clear_costmap')
 
rospy.wait_for_service('/evader/move_base/clear_costmaps')

while True:
    srv = rospy.ServiceProxy('/evader/move_base/clear_costmaps', Empty)
    
    service_example = srv()
    
    rospy.loginfo("clear costmaps: {}".format(service_example))

    rospy.sleep(4)