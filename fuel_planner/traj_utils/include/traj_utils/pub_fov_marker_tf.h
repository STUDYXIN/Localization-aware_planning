#ifndef FOV_MARKER_H
#define FOV_MARKER_H

#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <visualization_msgs/Marker.h>

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
