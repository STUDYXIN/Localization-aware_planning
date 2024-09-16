#ifndef FOV_MARKER_H
#define FOV_MARKER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <vector>

class FOVMarker {
public:
  FOVMarker();
  void init(ros::NodeHandle& nh);
  void publishFOV(const nav_msgs::Odometry& msg);

private:
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
};

#endif  // FOV_MARKER_H
