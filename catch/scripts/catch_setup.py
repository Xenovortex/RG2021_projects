#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import rospkg
import rogata_library as rgt
import numpy as np
import os


rospack    = rospkg.RosPack()
catch_path = rospack.get_path('catch')


filepath   = os.path.join(catch_path, 'scripts/squares.npy')
squares    = np.load(filepath)
wall_obj   = rgt.game_object('walls', [squares], np.array([1]))

name      = "cat"
cat_id    = 1
hit_box   = {"type":"rectangle","height":2,"width":2}
cat_obj   = rgt.dynamic_object(name,hit_box,cat_id)

name      = "mouse"
mouse_id  = 1
hit_box   = {"type":"rectangle","height":2,"width":2}
mouse_obj = rgt.dynamic_object(name,hit_box,mouse_id)





def odom_callback(self, odom, agent):
    pos = np.array([odom.pose.pose.position.x,
                    odom.pose.pose.position.y])
    #TODO convert odom coordinates into map coordinates
    rogata.set_pos(agent,pos)





if __name__ == '__main__':
    rospy.init_node("rogata_engine")
    try:
        example_scene = rgt.scene([wall_obj,cat_obj,mouse_obj])
        rogata = rgt.rogata_helper()
        rospy.Subscriber("cat/odom"  , Odometry, odom_callback,"cat")
        rospy.Subscriber("mouse/odom", Odometry, odom_callback, "mouse")
    except rospy.ROSInterruptException:
        pass
