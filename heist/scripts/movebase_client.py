#! /usr/bin/env python3

import rospy
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String


def movebase_client(x, y, z, w):
    """Client to interface with the move_base node of the navigation stack. Sends goal after getting position from the topics of the information_service node.

    Args:
        x (float): x coordinate of the goal
        y (float): y coordinate of the goal
        z (float): quaternions orientation at the goal
        w (float): quaternions orientation at the goal

    Returns:
        (int): result of action client
    """
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
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
    """ Subscribes to the information_service node and receives the positions of the goal and the entry."""

    def __init__(self):
        self.goal_out = np.zeros(2)
        self.entry_out = np.zeros(2)
        self.goal_ready = False
        self.entry_ready = False
        
        rospy.Subscriber("/information/goal", String, self.goal_callback)
        rospy.Subscriber("/information/entry", String, self.entry_callback)

    def goal_callback(self, goal):
        self.goal_out = np.fromstring(goal.data, dtype=float, sep=' ')
        self.goal_ready = True

    def entry_callback(self, entry):
        self.entry_out = np.fromstring(entry.data, dtype=float, sep=' ')
        self.entry_ready = True


if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client')

        check_msg = Goal_memory()
        rate = rospy.Rate(10)
        
        while not (check_msg.goal_ready and check_msg.entry_ready):
            rate.sleep()

        result = movebase_client(check_msg.goal_out[0], check_msg.goal_out[1], 0.0, 1.0)
        if result:
            rospy.loginfo("Goal execution 1 done!")
        result2= movebase_client(check_msg.entry_out[0], check_msg.entry_out[1], 0.0, 1.0)
        if result2:
            rospy.loginfo("Goal execution 2 done!")

    except rospy.ROSInterruptException:
        pass
        

