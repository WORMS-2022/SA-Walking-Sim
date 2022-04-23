#!/usr/bin/python3.8

import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

pub_cmd_vel = rospy.Publisher('/phantomx/cmd_vel', Twist, queue_size=1)

def stop_robot():
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.angular.z = 0
        pub_cmd_vel.publish(msg)

if __name__ == "__main__":
    rospy.init_node('reset_sim')
    rospy.wait_for_service('/gazebo/reset_world')
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    reset_world()
    stop_robot()
