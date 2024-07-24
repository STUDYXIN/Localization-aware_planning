#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

ros::Publisher odometry_pub;
nav_msgs::Odometry odometry_msg;

void positionCommandCallback(const quadrotor_msgs::PositionCommand::ConstPtr &position_command_msg)
{
    // Create an Odometry message and fill in the required fields
    
    odometry_msg.header = position_command_msg->header;
    odometry_msg.pose.pose.position = position_command_msg->position;
    odometry_msg.twist.twist.linear = position_command_msg->velocity;

    // Publish the Odometry message
    // odometry_pub.publish(odometry_msg);
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "pose_stamped_publisher");

    ros::NodeHandle nh;
    odometry_pub = nh.advertise<nav_msgs::Odometry>("odometry_topic", 10);
    // Create a publisher for the PoseStamped messages
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_stamped_topic", 10);

    ros::Subscriber position_command_sub = nh.subscribe("/desired_traj", 10, positionCommandCallback);
    // Create a PoseStamped message
    geometry_msgs::PoseStamped pose_msg;

    // Set the frame ID and timestamp for the message
    pose_msg.header.frame_id = "base_link";

    // Set the initial pose values
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    pose_msg.pose.position.z = 0.0;

    pose_msg.pose.orientation.x = 0.0;
    pose_msg.pose.orientation.y = 0.0;
    pose_msg.pose.orientation.z = 0.0;
    pose_msg.pose.orientation.w = 1.0;

    // Set the publishing rate (10 Hz in this example)
    ros::Rate rate(200);

    while (ros::ok())
    {
        // Update the timestamp
        pose_msg.header.stamp = ros::Time::now();

        // nav_msgs::Odometry odometry_msg;
        odometry_pub.publish(odometry_msg);
        // odometry_msg.header = position_command_msg->header;
        // odometry_msg.pose.pose.position = position_command_msg->position;
        // odometry_msg.twist.twist.linear = position_command_msg->velocity;

        // Publish the message
        pose_pub.publish(pose_msg);

        // Sleep to maintain the specified publishing rate
        rate.sleep();
    }
    
    return 0;
}
