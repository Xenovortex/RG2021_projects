#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import rogata_library as rgt
import numpy as np





def odom_callback( odom, agent):
    pos = np.array([odom.pose.pose.position.x,
                   -odom.pose.pose.position.y])*100+np.array([500,500])
    #TODO convert odom coordinates into map coordinates
    print("updatet pos")
    rogata.set_pos(agent,pos)





if __name__ == '__main__':
    rospy.init_node("agent_tracker")
    try:
        rogata = rgt.rogata_helper()
        rospy.Subscriber("cat/odom"  , Odometry, odom_callback,"cat_obj")
        rospy.Subscriber("mouse/odom", Odometry, odom_callback, "mouse_obj")

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
