#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import rogata_library as rgt
import numpy as np

# Calls the rogata_helper to get the positions of the goal and entry and publishes them on the specified topics.

if __name__ == '__main__':
    rospy.init_node("information_service")
    rogata = rgt.rogata_helper()

    rate       = rospy.Rate(10)  # 10hz
    pub_goal   = rospy.Publisher("information/goal", String, queue_size=10)
    pub_entry  = rospy.Publisher("information/entry", String, queue_size=10)

    try:
        while not rospy.is_shutdown():
            goal = rogata.get_pos("goal_obj")
            entry = rogata.get_pos("entry_obj")
            
            goal = (goal - np.array([500,500])) / 100
            goal[1] = goal[1] * (-1)
            
            entry = (entry - np.array([500,500])) / 100
            entry[1] = entry[1] * (-1)

            pub_goal.publish("{} {}".format(goal[0], goal[1]))
            pub_entry.publish("{} {}".format(entry[0], entry[1]))
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
