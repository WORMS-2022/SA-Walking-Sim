#!/usr/bin/python3.8

import rospy
from std_srvs.srv import Empty

rospy.wait_for_service('/gazebo/reset_world')
reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
reset_world()
