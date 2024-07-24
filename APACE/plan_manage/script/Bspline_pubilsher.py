#!/usr/bin/env python

import rospy
from trajectory.msg import Bspline  # 替换为你实际使用的消息类型
from geometry_msgs.msg import Point
import math

def publish_circle_bspline():
    rospy.init_node('circle_bspline_publisher', anonymous=True)
    bspline_pub = rospy.Publisher('/planning/bspline', Bspline, queue_size=10)
    rate = rospy.Rate(10)  # 发布频率为 10 Hz

    bspline_msg = Bspline()
    bspline_msg.order = 3  # 三次B样条
    bspline_msg.traj_id = 1  # 轨迹ID，可以自定义
    bspline_msg.start_time = rospy.Time.now()  # 当前时间作为起始时间

    # 设置 B样条的结点向量（假设均匀分布）
    interval = 1.0  # 间隔
    num_points = 20  # 点的数量
    bspline_msg.knots = [i * interval for i in range(num_points)]

    # 计算圆形的位置点和偏航角
    radius = 5.0  # 圆的半径
    angular_step = 2.0 * math.pi / num_points
    for i in range(num_points):
        point = Point()
        point.x = radius * math.cos(i * angular_step)
        point.y = radius * math.sin(i * angular_step)
        point.z = 0.0  # 如果需要，可以调整z坐标

        bspline_msg.pos_pts.append(point)
        bspline_msg.yaw_pts.append(i * angular_step)  # 假设偏航角与位置对应

    while not rospy.is_shutdown():
        bspline_pub.publish(bspline_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_circle_bspline()
    except rospy.ROSInterruptException:
        pass
