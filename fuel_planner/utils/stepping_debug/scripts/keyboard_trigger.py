#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def keyboard_input_publisher():
    # 初始化ROS节点
    rospy.init_node('keyboard_input_node', anonymous=True)

    # 创建一个发布者，话题名为 /keyboard_input，消息类型为 std_msgs/String
    pub = rospy.Publisher('/keyboard_input', String, queue_size=10)

    # 设置发布频率
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # 等待用户输入
        user_input = input("Please enter a character (or string): ")

        # 创建消息并赋值
        msg = String()
        msg.data = user_input

        # 发布消息
        rospy.loginfo(f"Sending keyboard input: {user_input}")
        pub.publish(msg)

        # 按照设定频率循环
        rate.sleep()

if __name__ == '__main__':
    try:
        keyboard_input_publisher()
    except rospy.ROSInterruptException:
        pass
