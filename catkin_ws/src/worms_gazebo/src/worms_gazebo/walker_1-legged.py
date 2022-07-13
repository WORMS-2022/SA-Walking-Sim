#!/usr/bin/python3.8

from threading import Thread
import rospy
import math
from worms_gazebo.worms_hexapod import WORMS_Hexapod
from geometry_msgs.msg import Twist
import numpy as np


jleg = ['j_coxa', 'j_thigh', 'j_shin']
suffix = ['f', 'm', 'r']
sides = ['l', 'r']
joints = []
for j in jleg:
    for s in suffix:
        for side in sides:
            z = j + "_" + side + s
            joints.append(z)

# large list for all properties of joints
hexapod_joints = joints


class WJFunc:
    """Walk Joint Function"""
    def __init__(self):
        self.offset = 0
        self.scale = 1
        self.in_offset = 0
        self.in_scale = 1

    def get(self, x, desired_angle=np.pi/4):
        """x between 0 and 1"""
        f = math.sin(self.in_offset + self.in_scale * x)
        return self.offset + self.scale * f

    def clone(self):
        z = WJFunc()
        z.offset = self.offset
        z.scale = self.scale
        z.in_offset = self.in_offset
        z.in_scale = self.in_scale
        return z

    def mirror(self):
        z = self.clone()
        z.offset *= -1
        z.scale *= -1
        return z

    def __str__(self):
        return 'y = {} + {} * sin({} + {} * x)'.format(
            self.offset, self.scale, self.in_offset, self.in_scale)


class WFunc:
    """Walk Function"""
    def __init__(self, n_phases=6, **kwargs):
        self.parameters = {}

        self.parameters['swing_scale'] = 20.0
        self.parameters['vx_scale'] = 0.5
        self.parameters['vy_scale'] = 0.5
        self.parameters['vt_scale'] = 0.4

        for k, v in kwargs.items():
            self.parameters[k] = v

        self.n_phases = n_phases

        self.joints = hexapod_joints
        self.generate()

    def generate(self):
        f1 = WJFunc()
        f1.in_scale = math.pi
        f1.scale = self.parameters['swing_scale']

        f2 = f1.clone()
        f2.scale = 0

        f3 = f1.clone()
        f3.scale = 1
        f3.in_scale = -(np.pi + np.pi/4)

        f4 = f2.clone()

        zero = WJFunc()
        zero.scale = 0

	# list of dictionaries. For each phase, provide a phase joint function
        self.pfn_list = [{joint:f2 for joint in hexapod_joints} for j in range(self.n_phases)] # f2 are basically 0's

	# defines the desired motion for different parts of the leg
        self.set_func('j_thigh', f1)
        self.set_func('j_shin', f3)
        self.set_func('j_coxa', zero)

        # self.show()

    # TODO: Refactor for easier change of leg sequencing
    def set_func(self, joint, fp):
        for i, leg in enumerate(['lr', 'lm', 'lf', 'rr', 'rm', 'rf']):
            self.pfn_list[i][joint + '_' + leg] = fp # gives desired func

    def get(self, state, x, velocity): # state is interchangeable with phase
        """x between 0 and 1"""
        """gets joint angles, based on velocity set earlier, determines the current angle"""
        angles = {}
        for j in hexapod_joints:
                angles[j] = self.pfn_list[state][j].get(x) # angles for thigh and shin


        rospy.loginfo(angles)
        self.apply_velocity(angles, velocity, state, x) # angles for coxa
        return angles


    def apply_velocity(self, angles, velocity, state, x):
        # applies precomputed velocity to the coxa joints to allow robot to move forward
        # VX - L forward-moving limbs are -d, R forward-moving limbs are +d
        v = velocity[0] * self.parameters['vx_scale']
        d = (x * 2 - 1) * v
        for i, leg in enumerate(['lr', 'lm', 'lf', 'rr', 'rm', 'rf']):
            if (i == state and 'l' not in leg) or (i != state and 'l' in leg):
                angles['j_coxa_' + leg]+=d
            else:
                angles['j_coxa_' + leg]-=d


class Walker:
    """Class for making a WORMS Hexapod walk"""
    def __init__(self, darwin, n_phases=6):
        self.darwin = darwin
        self.running = False

        self.velocity = [0, 0, 0]
        self.walking = False
        self.func = WFunc(n_phases=n_phases)

        # self.ready_pos=get_walk_angles(10)
        self.ready_pos = self.func.get(True, 0, [0, 0, 0])

        self._th_walk = None

        self._sub_cmd_vel = rospy.Subscriber(
            darwin.ns + "cmd_vel", Twist, self._cb_cmd_vel, queue_size=1)

        self.n_phases = n_phases


    def _cb_cmd_vel(self, msg): # callback_command_velocity, send velocities to subscribers and tell them to start up
        """Catches cmd_vel and update walker speed"""
        print('cmdvel', msg)
        vx = msg.linear.x
        vy = msg.linear.y
        vt = msg.angular.z
        self.start()
        self.set_velocity(vx, vy, vt)

    def init_walk(self):
        """If not there yet, go to initial walk position"""
        rospy.loginfo('Going to walk position')
        if self.get_dist_to_ready() > 0.02:
            self.darwin.set_angles_slow(self.ready_pos) # sets all angles to "zero", i.e. spawn angle

    def start(self): # starts the process
        if not self.running:
            self.running = True
            self.init_walk()
            self._th_walk = Thread(target=self._do_walk) # starts a background process
            self._th_walk.start()
            self.walking = True

    def stop(self):
        if self.running:
            self.walking = False
            rospy.loginfo('Waiting for stopped')
            while not rospy.is_shutdown() and self._th_walk is not None:
                rospy.sleep(0.1)
            rospy.loginfo('Stopped')
            self.running = False

    def set_velocity(self, x, y, t):
        self.velocity = [x, y, t]

    def _do_walk(self):
        """Main walking loop

        Smoothly update velocity vectors and apply corresponding angles.
        """
        r = rospy.Rate(100) # unit is hertz
        rospy.loginfo('Started walking thread')
        func = self.func

        # Global walk loop
        n = 50 # num of steps within a phase, somewhat like fps
        gait_state = 0 # 0~5, the 6 phases
        i = 0 # a counter for steps
        self.current_velocity = [0, 0, 0]
        while (not rospy.is_shutdown() and
               (self.walking or i < n or self.is_walking())):
            if not self.walking:
                self.velocity = [0, 0, 0]
            if not self.is_walking() and i == 0: # gets the robot to be stable when starting the simulation
                # Do not move if nothing to do and already at 0
                angles = func.get(0, 0, self.current_velocity)
                self.update_velocity(self.velocity, n)
                self.darwin.set_angles(angles)
                r.sleep()
                continue

            x = float(i) / n # a step in the sin curve
            angles = func.get(gait_state, x, self.current_velocity) # gets the angle
            self.update_velocity(self.velocity, n)
            self.darwin.set_angles(angles)
            i += 1
            if i > n:
                i = 0
                gait_state = (gait_state + 1) % self.n_phases # transition to next state (AKA phase)
            r.sleep()
        rospy.loginfo('Finished walking thread')

        self._th_walk = None

    def is_walking(self):
        e = 0.02
        for v in self.current_velocity:
            if abs(v) > e:
                return True
        return False

    def is_standing(self):
        for v in self.current_velocity:
            if not (v == 0.0):
                return False
        return True

    def rescale(self, angles, coef):
        z = {}
        for j, v in angles.items():
            offset = self.ready_pos[j]
            v -= offset
            v *= coef
            v += offset
            z[j] = v
        return z

    def update_velocity(self, target, n):
        a = 3 / float(n)
        b = 1 - a
        t_and_v = zip(target, self.current_velocity)
        self.current_velocity = [a * t + b * v for (t, v) in t_and_v]

    def get_dist_to_ready(self):
        angles = self.darwin.get_angles()
        return get_distance(self.ready_pos, angles)


def interpolate(anglesa, anglesb, coefa):
    z = {}
    joints = anglesa.keys()
    for j in joints:
        z[j] = anglesa[j] * coefa + anglesb[j] * (1 - coefa)
    return z


def get_distance(anglesa, anglesb):
    d = 0
    joints = anglesa.keys()
    if len(joints) == 0:
        return 0
    for j in joints:
        d += abs(anglesb[j] - anglesa[j])
    d /= len(joints)
    return d


if __name__ == '__main__':
    rospy.init_node('walker')
    rospy.sleep(1)

    rospy.loginfo('Instantiating Robot Client')
    robot = WORMS_Hexapod()
    rospy.loginfo('Instantiating Robot Walker')
    walker = Walker(robot)

    rospy.sleep(0.5)

    robot.set_walk_velocity(0, 0, 0)

    rospy.loginfo('Walker Ready')
    while not rospy.is_shutdown():
        rospy.sleep(1)
