#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class PathPublisher:
    def __init__(self):
        # 订阅 /airsim_node/drone_1/odom_local_enu 话题
        self.odom_sub = rospy.Subscriber('/airsim_node/drone_1/odom_local_enu', Odometry, self.odom_callback)
        # 发布路径
        self.path_pub = rospy.Publisher('/drone_path', Path, queue_size=10)
        self.path = Path()
        self.path.header.frame_id = 'world'  # 设置帧ID为世界坐标系

    def odom_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        self.path.poses.append(pose_stamped)
        self.path.header.stamp = rospy.Time.now()

        self.path_pub.publish(self.path)

if __name__ == '__main__':
    rospy.init_node('path_publisher', anonymous=True)
    pp = PathPublisher()
    rospy.spin()
