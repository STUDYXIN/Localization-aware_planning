#!/usr/bin/env python3

import numpy as np
import sys
import rospy
import tf
import math
import sys
import select
import os
if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios
    settings = termios.tcgetattr(sys.stdin)

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray
from quadrotor_msgs.msg import PositionCommand


class Controller():

    def __init__(self):
        self.N = 10
        self.rate = rospy.Rate(50)
        self.curr_state = np.zeros(4)
        self.automony_mode = False

        init_x = rospy.get_param('/keyboard_controller/init_x', 0.01)
        init_y = rospy.get_param('/keyboard_controller/init_y', 0.01)
        init_z = rospy.get_param('/keyboard_controller/init_z', 1.0)
        self.init_yaw = rospy.get_param('/keyboard_controller/init_yaw', 1.57)

        self.init_pos = np.array([init_x, init_y, init_z])

        self.pos_cmd_pub = rospy.Publisher('/position_cmd', PositionCommand, queue_size=50)

        self.cmd = PositionCommand()
        self.cmd.header.stamp = rospy.Time.now()
        self.cmd.header.frame_id = "world"
        self.cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        self.cmd.trajectory_id = 0
        self.cmd.position.x = 0.0
        self.cmd.position.y = 0.0
        self.cmd.position.z = 0.0
        self.cmd.velocity.x = 0.0
        self.cmd.velocity.y = 0.0
        self.cmd.velocity.z = 0.0
        self.cmd.acceleration.x = 0.0
        self.cmd.acceleration.y = 0.0
        self.cmd.acceleration.z = 0.0
        self.cmd.yaw = 0.0
        self.cmd.yaw_dot = 0.0

        self.auto_takeoff()

        self.control_loop()

    def quart_to_rpy(self, x, y, z, w):
        r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        p = math.asin(2 * (w * y - z * x))
        y = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        return r, p, y

    def get_current_state(self, event):
        try:
            (trans, rot) = self.listener.lookupTransform(self.world_frame_id, self.bask_link_frame_id, rospy.Time(0))

            self.curr_state[0] = trans[0]
            self.curr_state[1] = trans[1]
            self.curr_state[2] = trans[2]
            roll, pitch, self.curr_state[3] = self.quart_to_rpy(rot[0], rot[1], rot[2], rot[3])  # r,p,y
            curr_state_send = Float32MultiArray()
            curr_state_send.data = [self.curr_state[0], self.curr_state[1], self.curr_state[2], (self.curr_state[3] + np.pi) % (2 * np.pi) - np.pi, roll, pitch]
            self.curr_state_pub.publish(curr_state_send)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def pub_control_cmd(self):
        # control_cmd = PositionCommand()
        # control_cmd.header.stamp = rospy.Time.now()
        # control_cmd.header.frame_id = 'world'
        # control_cmd.position.x = self.curr_state[0]
        # control_cmd.position.y = self.curr_state[1]
        # control_cmd.position.z = self.curr_state[2]

        rospy.loginfo("x: %.1f, y: %.1f, z: %.1f, yaw: %.1f" % (self.cmd.position.x, self.cmd.position.y, self.cmd.position.z, self.cmd.yaw))

        self.pos_cmd_pub.publish(self.cmd)

    def getKey(self):
        if os.name == 'nt':
            return msvcrt.getch()

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def auto_takeoff(self):
        rospy.loginfo("[Traj server] Initializing")
        rospy.sleep(1.0)

        for i in range(100):
            self.cmd.position.z += self.init_pos[2] / 100.0
            self.cmd.yaw = self.init_yaw
            self.pos_cmd_pub.publish(self.cmd)
            rospy.sleep(0.01)

        dist = math.sqrt(self.init_pos[0] * self.init_pos[0] + self.init_pos[1] * self.init_pos[1])
        n = int(dist / 0.01)
        init_pos_xy = np.array([self.init_pos[0], self.init_pos[1]])
        init_unit = init_pos_xy / np.linalg.norm(init_pos_xy) * 0.01
        for i in range(n):
            self.cmd.position.x += init_unit[0]
            self.cmd.position.y += init_unit[1]
            self.cmd.yaw = self.init_yaw
            self.pos_cmd_pub.publish(self.cmd)
            rospy.sleep(0.01)

        rospy.sleep(3.5)
        rospy.loginfo("[Traj server] Initialization finished")

    def control_loop(self):
        while not rospy.is_shutdown():
            key = self.getKey()
            if key == '\x03':
                break

            pos_delta = np.array([0.0, 0.0, 0.0])
            yaw_delta = 0.0

            if key == 'w':
                pos_delta = np.array([0, 0.2, 0])
            elif key == 's':
                pos_delta = np.array([0, -0.2, 0])
            elif key == 'a':
                pos_delta = np.array([-0.2, 0, 0])
            elif key == 'd':
                pos_delta = np.array([0.2, 0, 0])
            elif key == 'y':
                pos_delta = np.array([0, 0, 0.2])
            elif key == 'n':
                pos_delta = np.array([0, 0, -0.2])
            elif key == 'j':
                print('j!!!')
                yaw_delta = 0.1
            elif key == 'l':
                print('l!!!')
                yaw_delta = -0.1

            self.cmd.position.x += pos_delta[0]
            self.cmd.position.y += pos_delta[1]
            self.cmd.position.z += pos_delta[2]

            self.cmd.yaw = np.clip(self.cmd.yaw + yaw_delta, -math.pi, math.pi)
            print('yaw: ', self.cmd.yaw)
            print('pi: ', math.pi)

            self.pub_control_cmd()
            self.rate.sleep()

        self.pub_control_cmd()


if __name__ == '__main__':
    rospy.init_node('control')
    controller = Controller()
