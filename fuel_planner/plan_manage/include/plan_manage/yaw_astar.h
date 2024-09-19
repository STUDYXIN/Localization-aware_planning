#ifndef _YAW_ASTAR_H_
#define _YAW_ASTAR_H_

#include "active_perception/frontier_finder.h"

#include "plan_manage/yaw_graph_utils.h"

using Eigen::Vector3d;
using std::vector;

namespace fast_planner {

class YawNode {
public:
  using Ptr = std::unique_ptr<YawNode>;

  std::vector<YawNode*> children_;
  YawNode* parent_ = nullptr;

  double yaw_;
  int yaw_id_;
  int layer_;
  vector<int> features_id_;
  vector<int> frontiers_id_;       // 自身能看到的frontier的id
  vector<int> frontiers_id_path_;  // 在路径上到该节点总共能看到的frontier的ids

  double cost_ = 0.0;

  YawNode() = default;
  YawNode(const Node& node) = delete;
  YawNode& operator=(const Node& node) = delete;

  void connectToNewParent(YawNode& parent, const double edge_cost) {
    parent_ = &parent;
    parent.children_.push_back(this);

    cost_ = parent_->cost_ + edge_cost_;
  }

  void disconnectFromParent() {
    parent_->children_.erase(find(parent_->children_.begin(), parent_->children_.end(), this));
    parent_ = nullptr;
  }

  double calEdgeDiff(const double yaw) const {
    double diff = fabs(yaw - yaw_);
    return std::min(diff, 2 * M_PI - diff);
  }

  double calEdgeDiff(const YawNode& node) const {
    return calEdgeDis(node.yaw_);
  }

  double calEdgeCost(const Node& node) const {
    double dis = calEdgeDis(node);
    return dis * (1.0 + 0.1 * (1 / (1.0001 - plane_->traversability_) + 1 / (1.0001 - node.plane_->traversability_)));
  };
};

struct NeighborInfo {
  YawNode* node_ = nullptr;
  double edge_cost_;
};

class YawAStar {
public:
  using Ptr = shared_ptr<YawAStar>;

  YawAStar(ros::NodeHandle& nh);

  void setTargetFrontier(const vector<Eigen::Vector3d>& target_frontier) {
    target_frontier_ = target_frontier;
  }

  void setFrontierFinder(shared_ptr<FrontierFinder> frontier_finder) {
    frontier_finder_ = frontier_finder;
  }

  void setFeatureMap(shared_ptr<FeatureMap>& feature_map) {
    feature_map_ = feature_map;
  }

  void setPos(vector<Vector3d>& pos) {
    pos_ = pos;
  }

  void setAcc(vector<Vector3d>& acc) {
    acc_ = acc;
  }

  bool search(const double start_yaw, const double end_yaw, const double& dt);
  // bool search(const double start_yaw, const double end_yaw, const vector<Vector3d>& pos, const vector<Vector3d>& acc,
  //     const double& dt, vector<double>& path);

  int countVisibleCells(const Vector3d& pos, const double& yaw);
  double calcInformationGain(const Vector3d& pos, const double& yaw);
  int YawAStar::getCoVisibileNum(const YawNode::Ptr& n1, const YawNode::Ptr& n2);

  void publishYawPath();

private:
  struct Param {
    int piece_num_;                  // 把360度等分为piece_num_份
    double max_yaw_rate_;            // 最大角速度
    double dt_;                      // 时间间隔
    int min_feature_num_;            // 最小可视特征点数量
    int min_covisible_feature_num_;  // 最小共视特征点数量
  };

  Param param_;

  // 规划过程要用到的外部辅助类
  shared_ptr<FeatureMap> feature_map_ = nullptr;
  shared_ptr<FrontierFinder> frontier_finder_ = nullptr;
  vector<Vector3d> target_frontier_;
  vector<Vector3d> pos_;
  vector<Vector3d> acc_;

  YawNode::Ptr start_node_ = nullptr;
  vector<vector<YawNode::Ptr>> graph_;
  vector<double> yaw_path_;

  // 规划过程调用的函数
  void calcId(YawNode::Ptr& node);

  void yaw2id(YawNode::Ptr& node);

  void id2yaw(YawNode::Ptr& node);

  template <typename T = YawNode::Ptr>
  void YawAStar::addNode(T&& node) {
    graph_[node->layer_].emplace_back(std::forward<T>(node));
  }

  void setFeatures(YawNode::Ptr& node);

  void generateNewNode(const YawNode::Ptr& cur_node, vector<YawNode::Ptr>& new_nodes);
  bool chooseParent(YawNode* new_node);
  void reWire(RRTNode* new_node);
  void updateSubTreeCost(YawNode* sub_tree_origin, const double& diff_cost);
  void traceBack(const YawNode* node);
  bool generatePath(const YawNode::Ptr& node);

  // Visualization
  ros::Publisher yaw_path_pub_;

  double yaw_diff_;
  int half_vert_num_;
  // double max_yaw_rate_, w_;

  double min_info_gain_;
};

}  // namespace fast_planner

#endif