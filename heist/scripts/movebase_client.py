#! /usr/bin/env python3

from os import wait
import rospy
import numpy as np
import rogata_library as rgt
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(x, y, z, w):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w


    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client')
        rogata = rgt.rogata_helper()
        goal = np.zeros(2)
        entry = np.zeros(2)

        if not rospy.is_shutdown():
            goal = rogata.get_pos("goal_obj")
            entry = rogata.get_pos("entry_obj")
            
            goal = (goal - np.array([500,500])) / 100
            goal[1] = goal[1] * (-1)
            
            entry = (entry - np.array([500,500])) / 100
            entry[1] = entry[1] * (-1)

            rospy.loginfo("entry: {}".format(entry))
            rospy.loginfo("goal: {}".format(goal))

        rospy.loginfo("movebase_client: sending goal 1")
        result = movebase_client(goal[0], goal[1], 0.0, 0.0)
        if result:
            rospy.loginfo("Goal execution 1 done!")
        result2= movebase_client(entry[0], entry[1], 0.0, 0.0)
        if result2:
            rospy.loginfo("Goal execution 2 done!")

    except rospy.ROSInterruptException:
        pass
        

