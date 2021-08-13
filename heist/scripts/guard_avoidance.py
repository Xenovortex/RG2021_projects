#!/usr/bin/env python3
import os
from os.path import join, abspath, dirname 

#from rogata_library.rogata_library import rogata_helper
import rospy
import math
import sys
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import rogata_library as rgt
from rogata_engine.srv import *
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point32
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

class Guard_avoidance:

    def __init__(self, rogata):
        self.evader_odom_topic = sys.argv[1]
        self.predict_guard_movement_topic = sys.argv[2]
        self.perception_topic = sys.argv[3]
        self.published_topic = sys.argv[4]
        self.state_evader = np.zeros(5)
        self.future_state_evader = np.zeros(5) # TODO: get that from pathfinding somehow
        self.state_guard = np.zeros(5)
        self.future_state_guard = np.zeros(5)
        self.rogata = rogata
        self.save = False
        self.num_angles = 300
        self.num_interpolate = 20
        self.pointcloud = np.zeros((self.num_angles * self.num_interpolate, 2))

        rate = rospy.Rate(0.5)  # 0.5hz
        rospy.Subscriber(self.evader_odom_topic, Odometry, self.odom_callback)  
        rospy.Subscriber(self.predict_guard_movement_topic, Odometry, self.prediction_callback)        
        rospy.Subscriber(self.perception_topic, Odometry, self.perception_callback)
        
        self.pub = rospy.Publisher(self.published_topic, PointCloud2, queue_size=10)

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
        #self.visibility(self.state_evader, self.state_guard, ["walls_obj"], 1000)
        self.laser_scanner(["walls_obj"], self.state_guard[0:2], np.linspace(-3.14, 3.14, self.num_angles)) # 0.1
        # move to laser scan later
        
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_link' #base_laser_link
        pointcloud = pcl2.create_cloud_xyz32(header, self.pointcloud)

        self.pub.publish(pointcloud)
    
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
        #rospy.loginfo("walls: {}".format(["walls_obj"]))


    def interpolate(self, laser_scan):
        x_new = np.linspace(self.state_guard[0], laser_scan[0], self.num_interpolate)
        y_new = np.linspace(self.state_guard[1], laser_scan[1], self.num_interpolate)
        return x_new, y_new
        


    def laser_scanner(self, object_list,test_point,angles):
        test_point = np.array([test_point[0],-test_point[1]])*100+np.array([500,500])

        scan = np.zeros((len(angles),2))
        for i, angle in enumerate(angles):
            end_point = np.array([100000,100000]) 
            for obj in object_list:
                response  = self.rogata.intersect(obj, test_point, angle, 1000)
                #rospy.loginfo("response: {}".format(response))
                new_point = np.array([response[0],response[1]])

                if np.linalg.norm(new_point-test_point) <= np.linalg.norm(end_point-test_point):
                    #rospy.loginfo("hi {}".format(new_point))
                    end_point = new_point
                    

            scan[i,:] = end_point
    
            
        sim_scan = (scan - np.tile(np.array([500,500]), (len(angles), 1))) / 100
        sim_scan[:, 1] = -sim_scan[:, 1]
        # pixel: x: -1000, 1000 y: -1000, 1000
        # gazebo: x:-5, 5

        pointcloud = np.apply_along_axis(self.interpolate, 1, sim_scan)
        self.pointcloud = np.swapaxes(pointcloud, 1, 2).reshape(-1, 2)
        #self.pointcloud = sim_scan
        #self.pointcloud = np.concatenate((self.pointcloud, np.zeros((self.pointcloud.shape[0], 1))), axis=1)
        self.pointcloud = np.concatenate((self.pointcloud, np.full((self.pointcloud.shape[0], 1), 0.1)), axis=1)

        #rospy.loginfo("sim_scan: {}".format(sim_scan))
        #rospy.loginfo("angles: {}".format(angles))
        #rospy.loginfo("laser_scan: {}".format(scan))
        # save data to plot
        if self.save == False:
            path = join(dirname(abspath(__file__)), "laser_scan.npy")
            rospy.loginfo("{}".format(path))
            np.save(path, self.pointcloud)
            self.save = True


    def generate_map(self):
        pass

if __name__ == '__main__':
    rospy.init_node("Guard_avoidance")
    
    try:
        rospy.loginfo("Before helper")
        rogata = rgt.rogata_helper()
        rospy.loginfo("After helper")
        node = Guard_avoidance(rogata)
    except rospy.ROSInterruptException:
        pass
