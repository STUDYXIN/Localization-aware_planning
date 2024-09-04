#include <plan_manage/yaw_initial_planner.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <iostream>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace Eigen;

namespace fast_planner {

/* ----------------------- class YawInitialPlanner ---------------------- */

YawInitialPlanner::YawInitialPlanner(ros::NodeHandle& nh) {
  yaw_path_pub_ = nh.advertise<visualization_msgs::Marker>("yaw_initial_planner/yaw_path", 20);

  nh.param("yaw_initial/yaw_diff", yaw_diff_, -1.0);
  nh.param("yaw_initial/half_vert_num", half_vert_num_, -1);
  nh.param("yaw_initial/max_yaw_rate", max_yaw_rate_, -1.0);
  nh.param("yaw_initial/w", w_, -1.0);
}

// void YawInitialPlanner::setMap(const shared_ptr<MapServer>& map) {
//   map_server_ = map;
// }

void YawInitialPlanner::searchPathOfYaw(
    const vector<Vector3d>& pos, const vector<Vector3d>& acc, const double& dt, vector<double>& path) {
  // Step1: 依据pos，acc构建yaw图
  Graph yaw_graph;
  yaw_graph.setParams(w_, max_yaw_rate_, dt);
  int gid = 0;
  vector<YawVertex::Ptr> layer, last_layer;
  Eigen::Vector3d zero(0, 0, 0);

  // Start and end yaw are not specified, add additional start and end layers
  int num_layers = pos.size() + 2;

  for (int i = 0; i < num_layers; ++i) {
    // add one layer of vertice representing discretized yaws at one waypoint
    if (i == 0 || i == num_layers - 1) {  // start or end layers
      YawVertex::Ptr vert(new YawVertex(0, 0, gid++, zero, zero, true));
      yaw_graph.addVertex(vert);
      layer.push_back(vert);
    }

    else {
      // 中间层的yaw角度是采样得到的
      // intermediate layers
      // TODO: it works but looks strange....
      int vert_num = 2 * half_vert_num_ + 1;
      vector<double> yaw_samples;
      for (int j = 0; j < vert_num; j++) yaw_samples.push_back((j - half_vert_num_) * yaw_diff_);
      yaw_samples.push_back(M_PI);

      for (const auto& yaw : yaw_samples) {
        YawVertex::Ptr vert(new YawVertex(yaw, 0, gid++, pos[i - 1], acc[i - 1], false));
        yaw_graph.addVertex(vert);
        layer.push_back(vert);
      }
    }

    // add edges between vertices in adjacent layers
    for (const auto& v1 : last_layer) {
      for (const auto& v2 : layer) yaw_graph.addEdge(v1->id_, v2->id_, calcCoVisibility(v1, v2));
    }
    last_layer.clear();
    last_layer.swap(layer);
  }

  // Step2: 使用dijkstra算法对yaw图进行搜索
  vector<YawVertex::Ptr> vert_path;
  yaw_graph.dijkstraSearch(0, gid - 1, vert_path);

  // Step3: 将存储在vert_path中的yaw角数值提取出来
  for (const auto& vert : vert_path) path.push_back(vert->yaw_);

  // publishYawPath(pos, path);
}

double YawInitialPlanner::calcCoVisibility(YawVertex::Ptr v1, YawVertex::Ptr v2) {
  if (v1->virtual_ || v2->virtual_) return 0;

  // 这里筛选特征点是先拿到两个pos的中点，用中点做KD树radius search，radius设为相机视野的最大距离
  // （论文里说相邻的yaw角要满足共视关系，其实代码里反而是筛选了满足共视关系的特征点，而且这样就说共视了有点简陋）
  // （这里getFeatures拿到的feature全是读文件读来的）

  // ROS_INFO("calcCoVisibility");

  vector<Eigen::Vector3d> features;
  Eigen::Vector3d mid_knot = (v1->pos_ + v2->pos_) / 2.0;

  feature_map_->getFeatures(mid_knot, features);

  // std::cout << "features size: " << features.size() << std::endl;

  double cost = calcCoVisibility(v1->pos_, v2->pos_, v1->acc_, v2->acc_, v1->yaw_, v2->yaw_, features);

  // std::cout << "cost: " << cost << std::endl;

  return cost;
}

double YawInitialPlanner::calcCoVisibility(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, const Eigen::Vector3d& acc1,
    const Eigen::Vector3d& acc2, const double& yaw_rad1, const double& yaw_rad2, const vector<Eigen::Vector3d>& features) {

  double covisibility = 0.0;
  for (const auto& f : features) {
    double v1 = calcVisibility(pos1, acc1, yaw_rad1, f);
    double v2 = calcVisibility(pos2, acc2, yaw_rad2, f);
    covisibility += v1 * v2;
  }
  return covisibility;
}

double YawInitialPlanner::calcVisibility(
    const Eigen::Vector3d& pos, const Eigen::Vector3d& acc, const double& yaw_rad, const Eigen::Vector3d& f) {
  // z_b: thrust direction，方向垂直机身向上，对应论文里的n1
  const Eigen::Vector3d z_b = acc + Eigen::Vector3d(0, 0, 9.81);
  const Eigen::Vector3d z_b_u = z_b.normalized();

  // c: the intermediate frame after yaw rotation，对应论文里的n3
  const Eigen::Vector3d x_c(std::cos(yaw_rad), std::sin(yaw_rad), 0.0);
  const Eigen::Vector3d y_b = z_b_u.cross(x_c);  // y_b = y_c
  const Eigen::Vector3d y_b_u = y_b.normalized();

  // x_b: camera optical direction,朝向机头，对应论文里的n2
  const Eigen::Vector3d x_b = y_b_u.cross(z_b_u);

  // 这里的v_f对应论文里的bearing measurement，由于这里就做了归一化，所以后面跟论文公式有出入
  const Eigen::Vector3d v_f = (f - pos).normalized();

  // 论文公式(7)
  const double sin_theta_z_b = z_b.cross(v_f).norm();
  const double sin_theta_y_b = y_b_u.cross(v_f).norm();
  const double cos_theta_x_b = x_b.dot(v_f);

  const double k = 20;
  const double sin_alpha = std::sin(M_PI / 3.0);  // 这tm论文里写的alpha1，alpha3到这里直接暴力写死pi/3了？
  const double visib_z_b = 1 / (1 + std::exp(-k * (sin_theta_z_b - sin_alpha)));  // 论文公式(4)
  const double visib_y_b = 1 / (1 + std::exp(-k * (sin_theta_y_b - sin_alpha)));  // 论文公式(5)
  const double visib_x_b = 1 / (1 + std::exp(-k * cos_theta_x_b));                // 论文公式(6)
  double visib = visib_z_b * visib_y_b * visib_x_b;                               // 论文公式(3)

  return visib;
}

// void YawInitialPlanner::publishYawPath(const vector<Position>& pos, const vector<double>& yaw) {
//   vector<Eigen::Vector3d> pts2;

//   for (int i = 0; i < yaw.size(); ++i) {
//     Eigen::Vector3d dir(cos(yaw[i]), sin(yaw[i]), 0);
//     Eigen::Vector3d pdir = pos[i] + 1.0 * dir;
//     pts2.push_back(pdir);
//   }

//   visualization_msgs::Marker mk;
//   mk.header.frame_id = "world";
//   mk.header.stamp = ros::Time::now();
//   mk.type = visualization_msgs::Marker::LINE_LIST;
//   mk.action = visualization_msgs::Marker::DELETE;
//   yaw_path_pub_.publish(mk);

//   mk.action = visualization_msgs::Marker::ADD;
//   mk.pose.orientation.x = 0.0;
//   mk.pose.orientation.y = 0.0;
//   mk.pose.orientation.z = 0.0;
//   mk.pose.orientation.w = 1.0;

//   mk.color.r = 0.5;
//   mk.color.g = 0.0;
//   mk.color.b = 1.0;
//   mk.color.a = 1.0;
//   mk.scale.x = 0.05;

//   geometry_msgs::Point pt;
//   Vector3d ori = Vector3d::Zero();
//   for (int i = 0; i < int(pos.size()); ++i) {
//     pt.x = pos[i](0);
//     pt.y = pos[i](1);
//     pt.z = pos[i](2);
//     mk.points.push_back(pt);

//     pt.x = pts2[i](0);
//     pt.y = pts2[i](1);
//     pt.z = pts2[i](2);
//     mk.points.push_back(pt);
//     if ((pts2[i] - pos[i]).norm() > 1.5) {
//       ROS_ERROR("yaw[i]: %f", yaw[i]);
//       getchar();
//     }
//   }
//   yaw_path_pub_.publish(mk);
// }
}  // namespace fast_planner