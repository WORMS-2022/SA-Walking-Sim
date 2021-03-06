#!/usr/bin/python3.8

import rospy
from phantomx_gazebo.phantomx import PhantomX


if __name__ == '__main__':
    rospy.init_node('walker_demo')

    rospy.loginfo('Instantiating robot Client')
    robot = PhantomX()
    rospy.sleep(1)

    rospy.loginfo('Walk in x-dir Starting')

    robot.set_walk_velocity(1, 0, 0)
    rospy.sleep(15)
    robot.set_walk_velocity(0, 0, 0)

    rospy.loginfo('Walk in x-dir Finished')
