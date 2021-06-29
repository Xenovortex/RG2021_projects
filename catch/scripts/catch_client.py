#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry

rogata = rgt.rogata_helper()

class Catch:

    def __init__(self):
        self.game_state = Int32()
        self.game_state = 0


        rate = rospy.Rate(10)  # 10hz
        self.pub = rospy.Publisher("game_state", Int32, queue_size=10)

        #TODO use sim position not odom for easier testing
        rospy.Subscriber("cat/odom", Odometry, self.odom_callback,"cat")
        rospy.Subscriber("mouse/odom", Odometry, self.odom_callback, "mouse")

        while not rospy.is_shutdown():
            self.pub.publish(self.game_state)
            rate.sleep()

    def odom_callback(self, odom, agent):
        pos = np.array([odom.pose.pose.position.x,
                        odom.pose.pose.position.y])
        #TODO convert odom coordinates into map coordinates
        rogata.set_pos(agent,pos)




if __name__ == '__main__':
    rospy.init_node("Refree")
    try:
        node = Catch()
    except rospy.ROSInterruptException:
        pass
