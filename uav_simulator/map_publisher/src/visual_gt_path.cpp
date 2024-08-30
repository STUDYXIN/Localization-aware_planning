#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

class PathPublisher {
public:
  PathPublisher() {
    // Subscribe to '/airsim_node/drone_1/odom_local_enu' topic
    odom_sub = nh.subscribe("/airsim_node/drone_1/odom_local_enu", 10, &PathPublisher::odomCallback, this);
    // Publish path on '/drone_path' topic
    path_pub = nh.advertise<nav_msgs::Path>("/drone_path", 10);
    path.header.frame_id = "world";  // Set frame ID to 'world' coordinate system
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.pose = msg->pose.pose;

    path.poses.push_back(pose_stamped);
    path.header.stamp = ros::Time::now();

    path_pub.publish(path);
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber odom_sub;
  ros::Publisher path_pub;
  nav_msgs::Path path;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_publisher");
  PathPublisher pp;
  ros::spin();

  return 0;
}
