#!/usr/bin/python3.8

import rospy
from worms_gazebo.worms_hexapod import WORMS_Hexapod
from worms_gazebo.walker import WJFunc, WFunc, Walker


if __name__ == '__main__':
    rospy.init_node('set_angle_test')

    rospy.loginfo('Instantiating robot Client')
    robot = WORMS_Hexapod() # instantiate new robot just to publish commands

    rospy.loginfo('Setting angles')
    func = WFunc()
    set_pos = func.get(0, 40, [0, 0, 0])

    robot.set_angles(set_pos)
