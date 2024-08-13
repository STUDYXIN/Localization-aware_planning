#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <signal.h>
#include <sensor_msgs/Imu.h>
#include <odom_ekf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Path.h>

void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_pub");
    ros::NodeHandle nh("~");
    // check stop signal
    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();
    // initialize the filter
    kalmanxd filter(Eigen::VectorXd::Zero(6));

    // initialize the subscribers
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/airsim_node/drone_1/pose_gt",
        100,
        boost::bind(&Pose_Data_t::feed, &filter.pose_data, _1),
        ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(
        "/airsim_node/drone_1/imu/imu",
        100,
        boost::bind(&Imu_Data_t::feed, &filter.imu_data, _1),
        ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());

    // transformBroadcaster
    tf2_ros::TransformBroadcaster tf_broadcaster;

    filter.odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("drone_path", 10);
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world"; // 设置轨迹的参考坐标系

    ros::Rate r(100.0);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();

        nav_msgs::Odometry odom_msg;
        ros::Duration dt = filter.pose_data.rcv_stamp - filter.pose_data.last_rcv_stamp;

        // time stamp
        ros::Time current_time = ros::Time::now();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "world";
        odom_msg.child_frame_id = "base_link";

        // position in ENU
        odom_msg.pose.pose.position.x = filter.pose_data.p(0);
        odom_msg.pose.pose.position.y = filter.pose_data.p(1);
        odom_msg.pose.pose.position.z = filter.pose_data.p(2);

        // velocity in ENU
        odom_msg.twist.twist.linear.x = (filter.pose_data.p(0) - filter.pose_data.p_last(0)) / dt.toSec();
        odom_msg.twist.twist.linear.y = (filter.pose_data.p(1) - filter.pose_data.p_last(1)) / dt.toSec();
        odom_msg.twist.twist.linear.z = (filter.pose_data.p(2) - filter.pose_data.p_last(2)) / dt.toSec();

        // q in ENU
        odom_msg.pose.pose.orientation.x = filter.pose_data.q.x();
        odom_msg.pose.pose.orientation.y = filter.pose_data.q.y();
        odom_msg.pose.pose.orientation.z = filter.pose_data.q.z();
        odom_msg.pose.pose.orientation.w = filter.pose_data.q.w();

        if (filter.pose_data.q.norm() > 0)
        {
            filter.odom_pub.publish(odom_msg);
        }
        else
        {
            ROS_WARN("[odom_pub] Invalid quaternion detected. Skipping Odometry broadcast!");
        }
        // 构造一个TransformStamped消息
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";    // 源坐标系
        transformStamped.child_frame_id = "base_link"; // 目标坐标系

        // 设置变换的平移和旋转
        transformStamped.transform.translation.x = filter.pose_data.p(0); // 以米为单位的平移
        transformStamped.transform.translation.y = filter.pose_data.p(1);
        transformStamped.transform.translation.z = filter.pose_data.p(2);
        transformStamped.transform.rotation.x = filter.pose_data.q.x();
        transformStamped.transform.rotation.y = filter.pose_data.q.y();
        transformStamped.transform.rotation.z = filter.pose_data.q.z();
        transformStamped.transform.rotation.w = filter.pose_data.q.w();

        // 发布坐标变换
        if (filter.pose_data.q.norm() > 0)
        {
            tf_broadcaster.sendTransform(transformStamped);
        }
        else
        {
            ROS_WARN("[odom_pub] Invalid quaternion detected. Skipping TF broadcast!");
        }

        // 发布轨迹
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = current_time;
        pose_stamped.header.frame_id = "world"; // 设置轨迹点的参考坐标系

        // 设置轨迹点的位置
        pose_stamped.pose.position.x = filter.pose_data.p(0);
        pose_stamped.pose.position.y = filter.pose_data.p(1);
        pose_stamped.pose.position.z = filter.pose_data.p(2);

        // // 设置轨迹点的姿态
        // tf2::Quaternion quaternion;
        // quaternion.setRPY(0, 0, 0); // 如果需要设置姿态，可以使用tf2库来创建四元数
        // pose_stamped.pose.orientation = tf2::toMsg(quaternion);

        // 添加轨迹点到轨迹消息中
        path_msg.poses.push_back(pose_stamped);

        // 发布轨迹消息
        path_pub.publish(path_msg);
    }

    // initialize the publishers
    return 0;
}