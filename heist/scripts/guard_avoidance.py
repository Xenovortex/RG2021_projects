#!/usr/bin/env python3
import rospy
import math
import sys
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Guard_avoidance:

    def __init__(self):
        self.evader_odom_topic = sys.argv[1]
        self.predict_guard_movement_topic = sys.argv[2]
        self.perception_topic = sys.argv[3]
        self.published_topic = sys.argv[4]
        self.state_evader = np.zeros(5)
        self.future_state_evader = np.zeros(5) # TODO: get that from pathfinding somehow
        self.state_guard = np.zeros(5)
        self.future_state_guard = np.zeros(5)

        rate = rospy.Rate(0.5)  # 0.5hz
        rospy.Subscriber(self.evader_odom_topic, Odometry, self.odom_callback)  
        rospy.Subscriber(self.predict_guard_movement_topic, Odometry, self.prediction_callback)        
        rospy.Subscriber(self.perception_topic, Odometry, self.perception_callback)
        
        self.pub = rospy.Publisher(self.published_topic, Odometry, queue_size=10)

        while not rospy.is_shutdown():
            rate.sleep()
            
    def odom_callback(self, odom):
        orientation = euler_from_quaternion([odom.pose.pose.orientation.x,
                                            odom.pose.pose.orientation.y,
                                            odom.pose.pose.orientation.z,
                                            odom.pose.pose.orientation.w])[2]
        
        x_pos = odom.pose.pose.position.x
        y_pos = odom.pose.pose.position.y

        linear_v = odom.twist.twist.linear.x
        angular_w = odom.twist.twist.angular.z

        self.state_evader[0] = x_pos
        self.state_evader[1] = y_pos
        self.state_evader[2] = orientation
        self.state_evader[3] = linear_v
        self.state_evader[4] = angular_w
        rospy.loginfo("guard_avoidance_odom: {}".format(self.state_evader))


    def prediction_callback(self, odom):
        orientation = euler_from_quaternion([odom.pose.pose.orientation.x,
                                            odom.pose.pose.orientation.y,
                                            odom.pose.pose.orientation.z,
                                            odom.pose.pose.orientation.w])[2]
        
        x_pos = odom.pose.pose.position.x
        y_pos = odom.pose.pose.position.y

        linear_v = odom.twist.twist.linear.x
        angular_w = odom.twist.twist.angular.z

        self.future_state_guard[0] = x_pos
        self.future_state_guard[1] = y_pos
        self.future_state_guard[2] = orientation
        self.future_state_guard[3] = linear_v
        self.future_state_guard[4] = angular_w

        rospy.loginfo("guard_avoidance_prediction: {}".format(self.state_evader))

        # DUMMY DATA
        self.pub.publish(odom)
    
    def perception_callback(self, odom):
        orientation = euler_from_quaternion([odom.pose.pose.orientation.x,
                                            odom.pose.pose.orientation.y,
                                            odom.pose.pose.orientation.z,
                                            odom.pose.pose.orientation.w])[2]
        
        x_pos = odom.pose.pose.position.x
        y_pos = odom.pose.pose.position.y

        linear_v = odom.twist.twist.linear.x
        angular_w = odom.twist.twist.angular.z

        self.state_guard[0] = x_pos
        self.state_guard[1] = y_pos
        self.state_guard[2] = orientation
        self.state_guard[3] = linear_v
        self.state_guard[4] = angular_w

        rospy.loginfo("guard_avoidance_perception: {}".format(self.state_guard))


    def visibility(guard,thief,wall_objects,max_seeing_distance):
        distance   = np.linalg.norm(thief-guard)
        direction  = (thief-guard)/distance
        direction  = np.arctan2(direction[1],direction[0])

        min_intersect = guard + max_seeing_distance * np.array([np.cos(direction),np.sin(direction)])

        for walls in wall_objects:

            intersection = rogata.intersect(walls,guard,direction,max_seeing_distance)
            if np.linalg.norm(intersection-guard) <= np.linalg.norm(min_intersect-guard):
                min_intersect = intersection

        if np.linalg.norm(min_intersect-guard) >= distance:
            return 1
        else:
            return 0


    def generate_map(self):
        pass

        

if __name__ == '__main__':
    rospy.init_node("Guard_avoidance")
    try:
        node = Guard_avoidance()
    except rospy.ROSInterruptException:
        pass
