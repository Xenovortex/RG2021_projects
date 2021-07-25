#!/usr/bin/env python3
import os
import rospy
import math
import sys
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose2D
import rogata_library as rgt
from rogata_engine.srv import *
import rospkg


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
        #rospy.loginfo("guard_avoidance_odom: {}".format(self.state_evader))


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

        #rospy.loginfo("guard_avoidance_prediction: {}".format(self.state_evader))

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

        #rospy.loginfo("guard_avoidance_perception: {}".format(self.state_guard))
        rospy.loginfo("walls: {}".format(["walls_obj"]))
        self.laser_scanner(["walls_obj"], self.state_guard, np.arange(-10, 10, 0.1))


    def laser_scanner(self, object_list,test_point,angles):
        # rospack    = rospkg.RosPack()
        # catch_path = rospack.get_path('heist')

        # filepath   = os.path.join(catch_path, 'maps/left_wall.npy')
        # left_wall  = np.load(filepath)
        # filepath   = os.path.join(catch_path, 'maps/right_wall.npy')
        # right_wall = np.load(filepath)
        # filepath   = os.path.join(catch_path, 'maps/outer_wall.npy')
        # outer_wall = np.load(filepath)
        # walls_obj  = rgt.game_object('walls_obj', [right_wall,left_wall,outer_wall], np.array([1]))
        intersect = rospy.ServiceProxy('intersect_line',RequestInter)
        # rogata = rgt.scene([walls_obj])
        scan = np.zeros((len(angles),2))
        for i in angles:
            end_point = np.array([100000,100000]) 
            for k in range(len(object_list)):
                rospy.loginfo("walls: {}".format(len(object_list)))
                line      = Pose2D(test_point[0],test_point[1],i)
                #name      = String()
                #name.data = object_list[k]
                req       = RequestInterRequest(str(object_list[k]),line,1000)
                rospy.loginfo("req: {}".format(req))
                response  = intersect(req)
                rospy.loginfo("response: {}".format(response))
                new_point = np.array([response.x,response.y])

                if np.linalg.norm(new_point-test_point) <= np.linalg.norm(end_point-test_point):
                    end_point = new_point

            scan[i,:] = end_point

        rospy.loginfo("laser_scan: {}".format(scan))


    def generate_map(self):
        pass

        

if __name__ == '__main__':
    rospy.init_node("Guard_avoidance")
    try:
        node = Guard_avoidance()
    except rospy.ROSInterruptException:
        pass
