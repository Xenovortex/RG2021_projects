#! /usr/bin/env python3

from os import wait
import rospy
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from rospy.numpy_msg import numpy_msg



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

class Goal_memory:

    def __init__(self):
        self.goal_out = np.zeros(2)
        self.entry_out = np.zeros(2)
        self.goal_ready = False
        self.entry_ready = False
        
        rospy.Subscriber("/information/goal", String, self.goal_callback)
        rospy.Subscriber("/information/entry", String, self.entry_callback)


    def goal_callback(self, goal):
        self.goal_out = np.fromstring(goal.data, dtype=float, sep=' ')
        rospy.loginfo("mb_c: goal: {}, {}".format(self.goal_out, self.goal_out.dtype))
        self.goal_ready = True

    def entry_callback(self, entry):
        self.entry_out = np.fromstring(entry.data, dtype=float, sep=' ')
        rospy.loginfo("mb_c: entry: {}, {}".format(self.entry_out, self.entry_out.dtype))
        self.entry_ready = True


if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client')

        rospy.loginfo("Before check_msg")
        check_msg = Goal_memory()
        rospy.loginfo("After check_msg")
        rate = rospy.Rate(10)
        
        while not (check_msg.goal_ready and check_msg.entry_ready):
            rate.sleep()
            
        rospy.loginfo("mb_c2: goal: {}, {}".format(check_msg.goal_out, check_msg.goal_out.dtype))
        rospy.loginfo("mb_c2: entry: {}, {}".format(check_msg.entry_out, check_msg.entry_out.dtype))

        rospy.loginfo("movebase_client: sending goal 1")
        result = movebase_client(check_msg.goal_out[0], check_msg.goal_out[1], 0.0, 1.0)
        if result:
            rospy.loginfo("Goal execution 1 done!")
        result2= movebase_client(check_msg.entry_out[0], check_msg.entry_out[1], 0.0, 1.0)
        if result2:
            rospy.loginfo("Goal execution 2 done!")

    except rospy.ROSInterruptException:
        pass
        

