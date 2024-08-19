#!/usr/bin/env python3

import airsim
import rospy

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Enable collision detection
#client.enableApiControl(False)
# client.simEnableCollisions(True)

# # Main loop
# while True:
#     # Check for collisions
#     collision_info = client.simGetCollisionInfo()
#     if collision_info.has_collided:
#         print("Collision detected!")
#         print("Object 1: ", collision_info.object_name_1)
#         print("Object 2: ", collision_info.object_name_2)
#         print("Position: ", collision_info.position)
#         print("Normal: ", collision_info.normal)
#         print("Penetration depth: ", collision_info.penetration_depth)
#         print("Impact point: ", collision_info.impact_point)
#         print("Impact normal: ", collision_info.impact_normal)
#         print("Time stamp: ", collision_info.time_stamp)

#     # Add any other logic or functionality here

#     # Sleep for a short duration
#     airsim.time.sleep(0.1)

class CollsionChecker:
    def __init__(self):
        # # 订阅 /airsim_node/drone_1/odom_local_enu 话题
        # self.odom_sub = rospy.Subscriber('/airsim_node/drone_1/odom_local_enu', Odometry, self.odom_callback)
        # # 发布路径
        # self.path_pub = rospy.Publisher('/drone_path', Path, queue_size=10)
        # self.path = Path()
        # self.path.header.frame_id = 'world'  # 设置帧ID为世界坐标系

        self.collsion_cnt=0

        while not rospy.is_shutdown():
            # Check for collisions
            collision_info = client.simGetCollisionInfo()
            if collision_info.has_collided:
                self.collsion_cnt+=1
                #跳过刚开始在地板上的碰撞
                if self.collsion_cnt>1:
                    print("Collision detected!")
                    print("Position: ", collision_info.position)
                    print("Normal: ", collision_info.normal)
                    print("Penetration depth: ", collision_info.penetration_depth)
                    print("Impact point: ", collision_info.impact_point)
                    print("Time stamp: ", collision_info.time_stamp)

                    # rospy.signal_shutdown("Collision detected!")

                

                # client.enableApiControl(True)

                # # Land the drone
                # client.landAsync().join()
                # client.armDisarm(False)  # 上锁
                # print("Drone landed.")

                # while not rospy.is_shutdown():
                #     pass


if __name__ == '__main__':
    rospy.init_node('collision_checker', anonymous=True)
    node = CollsionChecker()
    rospy.spin()