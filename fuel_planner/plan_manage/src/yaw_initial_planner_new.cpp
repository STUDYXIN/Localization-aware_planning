#include <plan_manage/yaw_initial_planner_new.h>
#include <plan_env/utils.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <iostream>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace Eigen;

namespace fast_planner {

/* ----------------------- class YawInitialPlanner ---------------------- */

YawInitialPlanner::YawInitialPlanner(ros::NodeHandle& nh) {
  yaw_path_pub_ = nh.advertise<visualization_msgs::Marker>("yaw_initial_planner/yaw_path", 20);

  // nh.param("yaw_initial/yaw_diff", yaw_diff_, -1.0);
  nh.param("yaw_initial/half_vert_num", half_vert_num_, -1);
  yaw_diff_ = M_PI / (half_vert_num_ + 1);
  nh.param("yaw_initial/max_yaw_rate", max_yaw_rate_, -1.0);
  nh.param("yaw_initial/w", w_, -1.0);
  nh.param("yaw_initial/min_info_gain", min_info_gain_, -1.0);
}

bool YawInitialPlanner::searchPathOfYaw(const double start_yaw, const double end_yaw, const vector<Vector3d>& pos,
    const vector<Vector3d>& acc, const double& dt, vector<double>& path) {

  // Step1: 依据pos，acc构建yaw图
  Graph yaw_graph;
  yaw_graph.setParams(w_, max_yaw_rate_, dt);
  int gid = 0;
  vector<YawVertex::Ptr> layer, last_layer;
  Eigen::Vector3d zero(0, 0, 0);

  // Start and end yaw are not specified, add additional start and end layers
  int num_layers = pos.size();

  int min_feature_num = Utils::getGlobalParam().min_feature_num_plan_;
  int min_covisible_feature_num = Utils::getGlobalParam().min_covisible_feature_num_plan_;

  for (int i = 0; i < num_layers; ++i) {
    if (i == 0) {
      double gain = calcInformationGain(pos.front(), start_yaw);

      Quaterniond ori = Utils::calcOrientation(start_yaw, acc[i]);

      vector<pair<int, Vector3d>> res;
      auto f_num = feature_map_->get_NumCloud_using_Odom(pos[i], ori, res);

      // if (f_num <= min_feature_num) {
      //   ROS_ERROR("[yaw initial planner]: Start point is not localizable!!!");
      //   return false;
      // }

      YawVertex::Ptr vert(new YawVertex(start_yaw, gain, gid++, pos.front(), acc.front(), true, true));
      vert->setFeatures(res);
      yaw_graph.addVertex(vert);
      layer.push_back(vert);
    }

    else if (i == num_layers - 1) {
      double gain = calcInformationGain(pos.back(), end_yaw);

      Quaterniond ori = Utils::calcOrientation(end_yaw, acc[i]);

      vector<pair<int, Vector3d>> res;
      auto f_num = feature_map_->get_NumCloud_using_Odom(pos[i], ori, res);

      if (f_num <= min_feature_num) {
        ROS_ERROR("[yaw initial planner]: End point is not localizable!!!");
        return false;
      }

      YawVertex::Ptr vert(new YawVertex(end_yaw, gain, gid++, pos.back(), acc.back(), true, true));
      vert->setFeatures(res);
      yaw_graph.addVertex(vert);
      layer.push_back(vert);
    }

    else {
      int vert_num = 2 * half_vert_num_ + 1;
      vector<double> yaw_samples;
      for (int j = 0; j < vert_num; j++) yaw_samples.push_back((j - half_vert_num_) * yaw_diff_);
      yaw_samples.push_back(M_PI);

      for (const auto& yaw : yaw_samples) {
        Quaterniond ori = Utils::calcOrientation(yaw, acc[i]);

        vector<pair<int, Vector3d>> res;
        auto f_num = feature_map_->get_NumCloud_using_Odom(pos[i], ori, res);

        //     int get_NumCloud_using_Odom(
        // const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient, vector<pair<int, Eigen::Vector3d>>& res);

        if (f_num <= min_feature_num) continue;

        double gain = calcInformationGain(pos[i], yaw);

        YawVertex::Ptr vert(new YawVertex(yaw, gain, gid++, pos[i], acc[i], false, false));
        vert->setFeatures(res);
        yaw_graph.addVertex(vert);
        layer.push_back(vert);
      }
    }

    if (i != 0) {
      int add_edge_num = 0;
      for (const auto& v1 : last_layer) {
        for (const auto& v2 : layer) {
          if (getCoVisibileNum(v1, v2) > min_covisible_feature_num) {
            yaw_graph.addEdge(v1->id_, v2->id_, v2->info_gain_);
            add_edge_num++;
          }
        }
      }

      // cout << "add edge num: " << add_edge_num << endl;
      if (add_edge_num == 0) return false;  // 中间有断裂的层，search失败
    }

    last_layer.clear();
    last_layer.swap(layer);
  }

  // Step2: 使用dijkstra算法对yaw图进行搜索
  vector<YawVertex::Ptr> vert_path;
  if (!yaw_graph.dijkstraSearch(0, gid - 1, vert_path)) {
    return false;
  }

  // Step3: 将存储在vert_path中的yaw角数值提取出来
  double total_info_gain = 0;
  for (const auto& vert : vert_path) {
    total_info_gain += vert->info_gain_;
    path.push_back(vert->yaw_);
  }

  // if (total_info_gain < min_info_gain_) {
  //   ROS_ERROR("[yaw initial planner]: Total information gain is too less!!!");
  //   return false;
  // }

  publishYawPath(pos, path);

  return true;
}

int YawInitialPlanner::countVisibleCells(const Vector3d& pos, const double& yaw) {
  return frontier_finder_->countVisibleCells(pos, yaw, target_frontier_);
}

double YawInitialPlanner::calcInformationGain(const Vector3d& pos, const double& yaw) {
  return static_cast<double>(countVisibleCells(pos, yaw));
}

int YawInitialPlanner::getCoVisibileNum(const YawVertex::Ptr& v1, const YawVertex::Ptr& v2) {
  int commonFeatureCount = 0;
  for (const auto& id1 : v1->features_id_) {
    for (const auto& id2 : v2->features_id_) {
      if (id1 == id2) {
        commonFeatureCount++;
      }
    }
  }

  // cout << "commonFeatureCount: " << commonFeatureCount << endl;
  return commonFeatureCount;
}

void YawInitialPlanner::publishYawPath(const vector<Vector3d>& pos, const vector<double>& yaw) {
  vector<Vector3d> pts2;

  for (size_t i = 0; i < yaw.size(); ++i) {
    Vector3d dir(cos(yaw[i]), sin(yaw[i]), 0);
    Vector3d pdir;
    if (i == 0 || i == yaw.size() - 1) {
      pdir = pos[i] + 1.5 * dir;
    } else {
      pdir = pos[i] + 0.9 * dir;
    }
    pts2.push_back(pdir);
  }

  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  yaw_path_pub_.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = 0.5;
  mk.color.g = 0.0;
  mk.color.b = 1.0;
  mk.color.a = 1.0;
  mk.scale.x = 0.05;

  geometry_msgs::Point pt;
  int cnt = 0;
  for (int i = 0; i < int(pos.size()); ++i) {
    pt.x = pos[i](0);
    pt.y = pos[i](1);
    pt.z = pos[i](2);
    mk.points.push_back(pt);

    pt.x = pts2[i](0);
    pt.y = pts2[i](1);
    pt.z = pts2[i](2);
    mk.points.push_back(pt);
    // if ((pts2[i] - pos[i]).norm() > 1.5) {
    //   ROS_ERROR("yaw[i]: %f", yaw[i]);
    //   getchar();
    // }
    cnt++;
  }
  ROS_INFO("[publishYawPath]: %d", cnt);
  yaw_path_pub_.publish(mk);
}
}  // namespace fast_planner