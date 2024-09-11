#ifndef _YAW_INITIAL_PLANNER_NEW_H_
#define _YAW_INITIAL_PLANNER_NEW_H_

#include "active_perception/frontier_finder.h"

#include "plan_manage/yaw_graph_utils.h"

using std::vector;

namespace fast_planner {

class YawInitialPlanner {
public:
  using Ptr = shared_ptr<YawInitialPlanner>;

  YawInitialPlanner(ros::NodeHandle& nh);

  void setTargetFrontier(const std::vector<Eigen::Vector3d>& target_frontier) {
    target_frontier_ = target_frontier;
  }

  void setFrontierFinder(shared_ptr<FrontierFinder> frontier_finder) {
    frontier_finder_ = frontier_finder;
  }

  void setFeatureMap(shared_ptr<FeatureMap>& feature_map) {
    feature_map_ = feature_map;
  }

  bool searchPathOfYaw(const double start_yaw, const double end_yaw, const vector<Vector3d>& pos, const vector<Vector3d>& acc,
      const double& dt, vector<double>& path);

  int countVisibleCells(const Vector3d& pos, const double& yaw);
  double calcInformationGain(const Vector3d& pos, const double& yaw);
  int getCoVisibileNum(const YawVertex::Ptr& v1, const YawVertex::Ptr& v2);

  void publishYawPath(const vector<Eigen::Vector3d>& pos, const vector<double>& yaw);

private:
  shared_ptr<FeatureMap> feature_map_ = nullptr;
  shared_ptr<FrontierFinder> frontier_finder_ = nullptr;

  std::vector<Eigen::Vector3d> target_frontier_;

  // Visualization
  ros::Publisher yaw_path_pub_;
  // params
  double yaw_diff_;
  int half_vert_num_;
  double max_yaw_rate_, w_;

  double min_info_gain_;
};

}  // namespace fast_planner

#endif