#!/usr/bin/env python3
import rospy
import math
import sys
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Predict_guard_movement:

    def __init__(self):
        self.perception_topic = sys.argv[1]
        self.published_topic = sys.argv[2]
        self.current_state_guard = np.zeros(5)
        self.last_state_guard = np.zeros(5)
        self.future_state_guard = np.zeros(5)

        rate = rospy.Rate(0.5)  # 0.5hz
        rospy.Subscriber(self.perception_topic, Odometry, self.perception_callback)
        
        self.pub = rospy.Publisher(self.published_topic, Odometry, queue_size=10)
        

        while not rospy.is_shutdown():
            rate.sleep()

    
    def perception_callback(self, odom):
        orientation = euler_from_quaternion([odom.pose.pose.orientation.x,
                                            odom.pose.pose.orientation.y,
                                            odom.pose.pose.orientation.z,
                                            odom.pose.pose.orientation.w])[2]
        
        x_pos = odom.pose.pose.position.x
        y_pos = odom.pose.pose.position.y

        linear_v = odom.twist.twist.linear.x
        angular_w = odom.twist.twist.angular.z

        self.last_state_guard[0] = self.current_state_guard[0]
        self.last_state_guard[1] = self.current_state_guard[1]
        self.last_state_guard[2] = self.current_state_guard[2]
        self.last_state_guard[3] = self.current_state_guard[3]
        self.last_state_guard[4] = self.current_state_guard[4]

        self.current_state_guard[0] = x_pos
        self.current_state_guard[1] = y_pos
        self.current_state_guard[2] = orientation
        self.current_state_guard[3] = linear_v
        self.current_state_guard[4] = angular_w

        self.predict_future()

        self.pub.publish(odom)

        rospy.loginfo("guard_prediction_perception: {}".format(self.current_state_guard))

        
    def predict_future(self):
        # TODO: prediction
        self.future_state_guard[0] = self.current_state_guard[0]
        self.future_state_guard[1] = self.current_state_guard[1]
        self.future_state_guard[2] = self.current_state_guard[2]
        self.future_state_guard[3] = self.current_state_guard[3]
        self.future_state_guard[4] = self.current_state_guard[4]

        

if __name__ == '__main__':
    rospy.init_node("Predict_guard_movement")
    try:
        node = Predict_guard_movement()
    except rospy.ROSInterruptException:
        pass