#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import rospkg
import rogata_library as rgt
import numpy as np
import os


rospack    = rospkg.RosPack()
catch_path = rospack.get_path('heist')


filepath   = os.path.join(catch_path, 'maps/walls.npy')
walls      = np.load(filepath)
walls_obj  = rgt.game_object('walls_obj', [walls], np.array([1]))

name      = "guard_obj"
cat_id    = 1
hit_box   = {"type":"rectangle","height":2,"width":2}
cat_obj   = rgt.dynamic_object(name,hit_box,cat_id)

name      = "evader_obj"
mouse_id  = 1
hit_box   = {"type":"rectangle","height":2,"width":2}
mouse_obj = rgt.dynamic_object(name,hit_box,mouse_id)


if __name__ == '__main__':
    rospy.init_node("rogata_engine")
    try:
        example_scene = rgt.scene([walls_obj,cat_obj,mouse_obj])
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
