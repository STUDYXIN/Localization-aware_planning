#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry


def odom_callback(odom_msg):
    time = odom_msg.header.stamp
    # print("time",time,type(time))
    pose = odom_msg.pose.pose.position
    x, y, z = pose.x, pose.y, pose.z
    # print("pose",pose,type(pose))
    quaternion = odom_msg.pose.pose.orientation
    q1, q2, q3, q4 = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    # print("quaternion",quaternion,type(quaternion))
    br.sendTransform((x, y, z), (q1, q2, q3, q4), time, drone_visual_frame, scan_frame)


rospy.init_node("drone_visualize")
scan_frame = rospy.get_param("~scan_frame", "tag_0")
odom_name = rospy.get_param("~odom_name", "/mine/odom")
# ght = rospy.param("~scan_height",15)
drone_visual_frame = "drone_visual_frame"

odom_sub = rospy.Subscriber(odom_name, Odometry, odom_callback, queue_size=1)
br = tf.TransformBroadcaster()

rospy.spin()
