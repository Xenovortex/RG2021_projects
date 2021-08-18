#!/usr/bin/env python3
from os.path import join, abspath, dirname 
import rospy
import sys
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import rogata_library as rgt
from rogata_engine.srv import *
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

class Guard_avoidance:
    """Compute guard viewing field based on received positions from preception and predict_guard_movement node and publish guard viewing field"""

    def __init__(self, rogata):
        self.evader_odom_topic = sys.argv[1]
        self.predict_guard_movement_topic = sys.argv[2]
        self.perception_topic = sys.argv[3]
        self.published_topic = sys.argv[4]    
        self.state_evader = np.zeros(5)
        self.state_guard = np.zeros(5)
        self.future_state_guard = np.zeros(5)
        self.rogata = rogata
        self.save = False
        self.num_angles = 600
        self.num_interpolate = 30
        self.pointcloud = np.zeros((self.num_angles * self.num_interpolate, 2))

        rate = rospy.Rate(10)  # 0.5hz
        rospy.Subscriber(self.evader_odom_topic, Odometry, self.odom_callback)  
        rospy.Subscriber(self.predict_guard_movement_topic, Odometry, self.prediction_callback)        
        rospy.Subscriber(self.perception_topic, Odometry, self.perception_callback)
        
        self.pub = rospy.Publisher(self.published_topic, PointCloud2, queue_size=10)

        while not rospy.is_shutdown():
            rate.sleep()
 
    # Odom callback for evader's odometry data.       
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

    # Callback for guard's predicted position from the guard_prediction node.
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

    # Calback for the evader's perception of the guard, which we get from the perception node. Data comes as noisy 'footsteps'.
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

        self.laser_scanner(["walls_obj"], self.state_guard[0:2], np.linspace(-3.14, 3.14, self.num_angles)) # 0.1

    # Generate points between guard and intersect point by linear interpolation
    def interpolate(self, laser_scan):
        x_new = np.linspace(self.state_guard[0], laser_scan[0], self.num_interpolate)
        y_new = np.linspace(self.state_guard[1], laser_scan[1], self.num_interpolate)
        return x_new, y_new    

    # Performs raycasting and generate pointcloud of guard viewing field
    def laser_scanner(self, object_list,test_point,angles):
        test_point = np.array([test_point[0],-test_point[1]])*100+np.array([500,500])
        scan = np.zeros((len(angles),2))
        
        for i, angle in enumerate(angles):
            end_point = np.array([100000,100000]) 
            for obj in object_list:
                response  = self.rogata.intersect(obj, test_point, angle, 1000)
                new_point = np.array([response[0],response[1]])

                if np.linalg.norm(new_point-test_point) <= np.linalg.norm(end_point-test_point):
                    end_point = new_point
                    

            scan[i,:] = end_point
            
        sim_scan = (scan - np.tile(np.array([500,500]), (len(angles), 1))) / 100
        sim_scan[:, 1] = -sim_scan[:, 1]

        pointcloud = np.apply_along_axis(self.interpolate, 1, sim_scan)
        self.pointcloud = np.swapaxes(pointcloud, 1, 2).reshape(-1, 2)
        self.pointcloud = np.concatenate((self.pointcloud, np.full((self.pointcloud.shape[0], 1), 0.1)), axis=1)

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_link'
        pointcloud = pcl2.create_cloud_xyz32(header, self.pointcloud)

        self.pub.publish(pointcloud)

        # Save data to plot
        if self.save == False:
            cloud_path = join(dirname(abspath(__file__)), "pointcloud.npy")
            np.save(cloud_path, self.pointcloud)

            intersect_path = join(dirname(abspath(__file__)), "laser_scan.npy")
            np.save(intersect_path, sim_scan)
            
            scan_pixel_path = join(dirname(abspath(__file__)), "laser_scan_pixel.npy")
            np.save(scan_pixel_path, scan)
            
            self.save = True

            
if __name__ == '__main__':
    rospy.init_node("Guard_avoidance")
    
    try:
        rogata = rgt.rogata_helper()
        node = Guard_avoidance(rogata)
    except rospy.ROSInterruptException:
        pass
