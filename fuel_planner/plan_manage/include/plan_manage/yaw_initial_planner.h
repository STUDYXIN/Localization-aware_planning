#ifndef _YAW_GRAPH_UTILS_H_
#define _YAW_GRAPH_UTILS_H_

#include "active_perception/frontier_finder.h"

#include "plan_env/feature_map.h"
#include "plan_env/utils.hpp"

#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <ros/ros.h>
#include <unordered_map>
#include <vector>

using std::list;
using std::pair;
using std::queue;
using std::set;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

namespace fast_planner {

// vertex type for yaw planning
class YawVertex {
public:
  using Ptr = shared_ptr<YawVertex>;

  YawVertex(const double y, const size_t layer) {
    yaw_ = y;
    Utils::roundPi(yaw_);
    layer_ = layer;
  }

  double calEdgeDiff(const double yaw) const {
    double diff = fabs(yaw - yaw_);
    return std::min(diff, 2 * M_PI - diff);
  }

  double calEdgeDiff(const YawVertex::Ptr& v) const {
    return calEdgeDiff(v->yaw_);
  }

  vector<YawVertex::Ptr> edges_;     // 记录了以该节点为父节点的边
  YawVertex::Ptr parent_ = nullptr;  // 父节点

  int candiate_parent_num_ = 0;  // 候选的父节点数量
  int graph_id_;                 // 在图搜索中的id
  size_t layer_;                 // 在图中所属的层数
  double g_value_;               // A*算法的g_score
  double h_value_;               // A*算法的h_score
  double f_value_;               // A*算法的f_score
  double yaw_;                   // 航向角
  int yaw_id_;                   // 航向角的id
  vector<int> features_id_;      // 自身能看到的特征点的id
  set<int> frontiers_id_;        // 自身能看到的frontier的id
  set<int> frontiers_id_path_;   // 在路径上到该节点总共能看到的frontier的ids
};

class YawNodeComparator {
public:
  bool operator()(const YawVertex::Ptr& v1, const YawVertex::Ptr& v2) {
    return v1->f_value_ > v2->f_value_;
  }
};

class YawInitialPlanner {
public:
  using Ptr = shared_ptr<YawInitialPlanner>;

  YawInitialPlanner(ros::NodeHandle& nh);

  void setTargetFrontier(const vector<Vector3d>& target_frontier) {
    target_frontier_ = target_frontier;
  }

  void setFrontierFinder(const shared_ptr<FrontierFinder>& frontier_finder) {
    frontier_finder_ = frontier_finder;
  }

  void setFeatureMap(const shared_ptr<FeatureMap>& feature_map) {
    feature_map_ = feature_map;
  }

  void setPos(const vector<Vector3d>& pos) {
    pos_ = pos;
  }

  void setAcc(const vector<Vector3d>& acc) {
    acc_ = acc;
  }

  void setVisbleFrontiers(const YawVertex::Ptr& v);
  void setVisbleFeatures(const YawVertex::Ptr& v);
  int getCoVisibileNum(const YawVertex::Ptr& v1, const YawVertex::Ptr& v2);

  void yaw2id(const YawVertex::Ptr& node);
  void id2yaw(const YawVertex::Ptr& node);
  void addVertex(YawVertex::Ptr& vertex);
  void estimateHeuristic(const YawVertex::Ptr& v);
  double calEdgeCost(const YawVertex::Ptr& from, const YawVertex::Ptr& to);

  bool checkFeasibility(const YawVertex::Ptr& v1, const YawVertex::Ptr& v2);

  void reset();
  bool search(const double start_yaw, const double end_yaw, const double& dt, vector<double>& path);

  void publishYawPath();
  void extractObservedFeatures(vector<vector<Vector3d>>& observed_features);

private:
  struct Param {
    int piece_num_;               // 把360度等分为piece_num_份
    vector<double> yaw_samples_;  // yaw采样
    double basic_cost_;           // 基础代价
    double ld_frontier_;
    double max_yaw_rate_;            // 最大角速度
    int max_diff_yaw_id_;            // 两层之间最大的yaw_id差值
    double dt_;                      // 时间间隔
    int min_feature_num_;            // 最小可视特征点数量
    int min_covisible_feature_num_;  // 最小共视特征点数量
  };

  Param param_;

  // 记得重置
  int graph_id_ = 0;
  vector<YawVertex::Ptr> vertice_;
  std::priority_queue<YawVertex::Ptr, std::vector<YawVertex::Ptr>, YawNodeComparator> open_set_;
  vector<YawVertex::Ptr> vert_path_;

  YawVertex::Ptr start_vert_ = nullptr;
  YawVertex::Ptr end_vert_ = nullptr;

  // 规划过程要用到的外部辅助类
  shared_ptr<FeatureMap> feature_map_ = nullptr;
  shared_ptr<FrontierFinder> frontier_finder_ = nullptr;
  vector<Vector3d> target_frontier_;
  vector<Vector3d> pos_;
  vector<Vector3d> acc_;

  // 规划过程调用的函数
  bool astarSearch(const int start, const int goal);

  // Visualization
  ros::Publisher yaw_path_pub_;
};

}  // namespace fast_planner

#endif