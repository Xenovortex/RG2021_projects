#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
import rospkg
import rogata_library as rgt
import numpy as np
import os


class Refree:

    def __init__(self):
        rospack = rospkg.RosPack()
        catch_path = rospack.get_path('catch')
        filepath = os.path.join(catch_path, 'scripts/squares.npy')
        squares = np.load(filepath)

        self.walls = rgt.game_object('walls', [squares], np.array([1]))

        example_scene = rgt.scene([self.walls])

        self.game_state = Int32()
        self.game_state = 0

        self.x_cat = rospy.get_param('cat_start_pos_x')
        self.y_cat = rospy.get_param('cat_start_pos_y')

        self.x_mouse = rospy.get_param('mouse_start_pos_x')
        self.y_mouse = rospy.get_param('mouse_start_pos_y')

        self.x_cheese = rospy.get_param('cheese_pos_x')
        self.y_cheese = rospy.get_param('cheese_pos_y')

        rate = rospy.Rate(10)  # 10hz
        self.pub = rospy.Publisher("game_state", Int32, queue_size=10)
        rospy.Subscriber("cat/odom", Odometry, self.cat_odom_callback)
        rospy.Subscriber("mouse/odom", Odometry, self.mouse_odom_callback)

        while not rospy.is_shutdown():
            self.pub.publish(self.game_state)
            rate.sleep()

    def cat_odom_callback(self, odom):
        self.x_cat = odom.pose.pose.position.x
        self.y_cat = odom.pose.pose.position.y

        # check if cat is near mouse
        dist = math.sqrt((self.x_mouse - self.x_cat) ** 2 + (self.y_mouse - self.y_cat) ** 2)
        if dist < 0.2 and self.game_state == 0:
            self.game_state = 1
        if self.game_state == 1:
            rospy.loginfo("mouse was caught. cat wins")
            print("game over. cat wins")

    def mouse_odom_callback(self, odom):
        self.x_mouse = odom.pose.pose.position.x
        self.y_mouse = odom.pose.pose.position.y

        # check if mouse is near cheese
        dist = math.sqrt((self.x_mouse - self.x_cheese) ** 2 + (self.y_mouse - self.y_cheese) ** 2)
        if dist < 0.2 and self.game_state == 0:
            self.game_state = 2
        if self.game_state == 2:
            rospy.loginfo("cheese was caught. mouse wins")
            print("game over. mouse wins")


if __name__ == '__main__':
    rospy.init_node("Refree")
    try:
        node = Refree()

    except rospy.ROSInterruptException:
        pass
