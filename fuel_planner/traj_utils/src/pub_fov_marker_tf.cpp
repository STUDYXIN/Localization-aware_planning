#include "traj_utils/pub_fov_marker_tf.h"

FOVMarker::FOVMarker() {
}

void FOVMarker::init(ros::NodeHandle& nh) {
  // 初始化发布者
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("/fov_marker", 10);
}

void FOVMarker::publishFOV(const nav_msgs::Odometry& msg) {
  // 获取位置
  Eigen::Vector3d pos_ros(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
  tf::Quaternion q_cam;
  tf::quaternionMsgToTF(msg.pose.pose.orientation, q_cam);
  tf::Matrix3x3 tf_mat_cam(q_cam);
  Eigen::Matrix3d R_cam;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      R_cam(i, j) = tf_mat_cam[i][j];
    }
  }
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      tf::Transform(q_cam, tf::Vector3(pos_ros.x(), pos_ros.y(), pos_ros.z())), ros::Time::now(), "world", "camera"));

  // 创建视场（FOV）的 Marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "fov_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.05;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  double fov_depth = 0.8;                          // 视场的深度
  double fov_horizontal = 79.1396 * M_PI / 180.0;  // 水平FOV
  double fov_vertical = 63.5803 * M_PI / 180.0;    // 垂直FOV

  double half_fov_width = tan(fov_horizontal / 2.0) * fov_depth;
  double half_fov_height = tan(fov_vertical / 2.0) * fov_depth;

  std::vector<Eigen::Vector3d> vertices_camera_frame = {
    { fov_depth, -half_fov_width, -half_fov_height },
    { fov_depth, half_fov_width, -half_fov_height },
    { fov_depth, half_fov_width, half_fov_height },
    { fov_depth, -half_fov_width, half_fov_height },
  };
  std::vector<Eigen::Vector3d> world_vertices;
  for (const auto& vertex : vertices_camera_frame) {
    Eigen::Vector3d world_vertex = R_cam * vertex + pos_ros;
    world_vertices.push_back(world_vertex);
  }
  geometry_msgs::Point zero;
  zero.x = pos_ros.x();
  zero.y = pos_ros.y();
  zero.z = pos_ros.z();
  for (size_t i = 0; i < world_vertices.size(); ++i) {
    geometry_msgs::Point pt1;
    pt1.x = world_vertices[i].x();
    pt1.y = world_vertices[i].y();
    pt1.z = world_vertices[i].z();
    marker.points.push_back(pt1);

    geometry_msgs::Point pt2;
    pt2.x = world_vertices[(i + 1) % world_vertices.size()].x();
    pt2.y = world_vertices[(i + 1) % world_vertices.size()].y();
    pt2.z = world_vertices[(i + 1) % world_vertices.size()].z();
    marker.points.push_back(pt2);
  }

  for (size_t i = 0; i < world_vertices.size(); ++i) {
    marker.points.push_back(zero);
    geometry_msgs::Point pt;
    pt.x = world_vertices[i].x();
    pt.y = world_vertices[i].y();
    pt.z = world_vertices[i].z();
    marker.points.push_back(pt);
  }

  // 发布 Marker
  marker_pub_.publish(marker);
}
