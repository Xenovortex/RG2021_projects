#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import rogata_library as rgt
import numpy as np





if __name__ == '__main__':
    rospy.init_node("Refree")
    rogata = rgt.rogata_helper()

    game_state = Int32()
    game_state = 0
    rate       = rospy.Rate(10)  # 10hz
    pub        = rospy.Publisher("game_state", Int32, queue_size=10)

    try:
        while not rospy.is_shutdown():
            cat_pos   = rogata.get_pos("cat_obj")
            mouse_pos = rogata.get_pos("mouse_obj")

            if np.linalg.norm(mouse_pos-cat_pos) <= 3:
                game_state = -1
                print("The Cat Wins!")
            #TODO add cheese winning condition


            pub.publish(game_state)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
