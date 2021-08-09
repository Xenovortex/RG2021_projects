#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
import rogata_library as rgt
import numpy as np

if __name__ == '__main__':
    rospy.init_node("Info")
    rogata = rgt.rogata_helper()

    rate       = rospy.Rate(10)  # 10hz
    pub_goal   = rospy.Publisher("information/goal", Int32, queue_size=10)
    pub_entry  = rospy.Publisher("information/entry", Int32, queue_size=10)


    try:
        while not rospy.is_shutdown():
            goal = rogata.get_pos("goal_obj")
            entry = rogata.get_pos("entry_obj")
            
            goal = (goal - np.array([500,500])) / 100
            goal[1] = goal[1] * (-1)
            
            entry = (entry - np.array([500,500])) / 100
            entry[1] = entry[1] * (-1)

            rospy.loginfo("entry: {}".format(entry))
            rospy.loginfo("goal: {}".format(goal))

            pub_goal.publish(goal)
            pub_entry.publish(entry)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
