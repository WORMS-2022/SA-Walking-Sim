#!/usr/bin/python3.8

import rospy
from worms_gazebo.worms_hexapod import WORMS_Hexapod


if __name__ == '__main__':
    rospy.init_node('walker_demo')

    rospy.loginfo('Instantiating robot Client')
    robot = WORMS_Hexapod() # instantiate new robot just to publish commands
    rospy.sleep(1)

    rospy.loginfo('Walk in x-dir Starting')

    robot.set_walk_velocity(1, 0, 0)
    rospy.sleep(15)
    robot.set_walk_velocity(1, 0, 0)
    rospy.sleep(15)
    robot.set_walk_velocity(1, 0, 0)
    rospy.sleep(15)
    robot.set_walk_velocity(0, 0, 0)

    rospy.loginfo('Walk in x-dir Finished')
