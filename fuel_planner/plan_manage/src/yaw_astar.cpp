#include <plan_manage/yaw_astar.h>
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

/* ----------------------- class YawAStar ---------------------- */

YawAStar::YawAStar(ros::NodeHandle& nh) {
  yaw_path_pub_ = nh.advertise<visualization_msgs::Marker>("yaw_initial_planner/yaw_path", 20);

  param_.max_yaw_rate_ = Utils::getGlobalParam().max_yaw_rate_;
  param_.min_feature_num_ = Utils::getGlobalParam().min_feature_num_plan_;
  param_.min_covisible_feature_num_ = Utils::getGlobalParam().min_covisible_feature_num_plan_;

  nh.param("yaw_initial/piece_num", param_.piece_num_, -1);
}

void yaw2id(YawNode::Ptr& node) {
  ROS_ASSERT(node->yaw_ >= -M_PI && node->yaw_ <= M_PI);
  node->yaw_id_ = static_cast<int>((yaw_ + M_PI) / (2 * M_PI / param_.piece_num_));
}

void id2yaw(YawNode::Ptr& node) {
  ROS_ASSERT(node->yaw_id_ >= 0 && node->yaw_id_ < param_.piece_num_);
  node->yaw_ = node->yaw_id_ * 2 * M_PI / param_.piece_num_ - M_PI;
}

template <typename T = YawNode::Ptr>
void YawAStar::addNode(T&& node) {
  graph_[node->layer_].emplace_back(std::forward<T>(node));
}

void YawAStar::setFeatures(YawNode::Ptr& node) {
  node->features_id_.clear();

  vector<pair<int, Vector3d>> features;
  Quaterniond ori = Utils::calcOrientation(node->yaw_, acc_[node->layer_]);
  feature_map_->get_NumCloud_using_Odom(pos_[node->layer_], ori, features);
  for (const auto& feature : features) {
    node->features_id_.push_back(feature.first);
  }
}

void YawAStar::generateNewNode(const YawNode::Ptr& cur_node, vector<YawNode::Ptr>& new_nodes) {
  int next_layer = cur_node->layer_ + 1;

  vector<int> yaw_ids = { 0 };
  for (int i = 1; i < param_.piece_num_; i++) {
    // 起码保证能扩展出3个节点
    if (i != 1) {
      double yaw_diff = i * 2 * M_PI / param_.piece_num_;
      double yaw_rate = fabs(yaw_diff) / param_.dt_;
      if (yaw_rate > param_.max_yaw_rate_) break;
    }
    yaw_ids.push_back(i);
    yaw_ids.push_back(-i);
  }

  for (const auto& id : yaw_ids) {
    // 新建节点
    YawNode::Ptr new_node = make_unique<YawNode>();
    // 设置新节点的yaw_id
    new_node->yaw_id_ = cur_node->yaw_id_ + id;
    while (new_node->yaw_id_ < 0) new_node->yaw_id_ += param_.piece_num_;
    while (new_node->yaw_id_ >= param_.piece_num_) new_node->yaw_id_ -= param_.piece_num_;
    // 设置新节点的yaw
    id2yaw(new_node);
    // 设置新节点所属的层数
    new_node->layer_ = next_layer;
    // 设置新节点能观测到的特征点,如果可见特征点数目太少就放弃生成
    setFeatures(new_node);
    if (new_node->features_id_.size() <= min_feature_num) continue;

    new_nodes.push_back(std::move(new_node));
  }
}

bool YawAStar::chooseParent(YawNode* new_node) {
  NeighborInfo neighbor;

  vector<int> neighbor_ids;
  getNeighborIds(new_node, neighbor_ids);

  vector<NeighborInfo> candidates;
  for (const auto& node : graph_[new_node->layer_ - 1]) {
    // 如果上一层的相邻ID已经被搜索过了且两个节点之间满足共视关系，就生成一条候选边
    auto it = std::find(neighbor_ids.begin(), neighbor_ids.end(), node->yaw_id_);
    if (it != neighbor_ids.end()) {
      if (getCoVisibileNum(new_node, *it) > min_covisible_feature_num_) {
        neighbor.parent_node_ = *it;
        neighbor.edge_cost_ = new_node->calEdgeCost(*node_ptr);
        candidates.emplace_back(neighbor);
      }
    }
  }

  if (candidates.empty()) return false;

  static const auto compareFun = [&](const NeighborInfo& a, const NeighborInfo& b) -> bool {
    return a.node_->cost_ + a.edge_cost_ < b.node_->cost_ + b.edge_cost_;
  };

  auto it = std::min_element(candidates.begin(), candidates.end(), compareFun);
  new_node->connectToNewParent(*it->parent_node_, *it->edge_cost_);

  return true;
}

void YawAStar::reWire(RRTNode* new_node) {
  NeighborInfo neighbor;

  vector<int> neighbor_ids;
  getNeighborIds(new_node, neighbor_ids);

  vector<NeighborInfo> candidates;
  for (const auto& node : graph_[new_node->layer_ + 1]) {
    // 如果下一层的相邻ID已经被搜索过了且两个节点之间满足共视关系，就生成一条候选边
    auto it = std::find(neighbor_ids.begin(), neighbor_ids.end(), node->yaw_id_);
    if (it != neighbor_ids.end()) {
      if (getCoVisibileNum(new_node, *it) > min_covisible_feature_num_) {
        neighbor.parent_node_ = *it;
        neighbor.edge_cost_ = new_node->calEdgeCost(*node_ptr);
        candidates.emplace_back(neighbor);
      }
    }
  }

  for (const auto& neighbor : candidates) {
    double new_cost = new_node->cost_ + neighbor.edge_cost_;
    double diff_cost = neighbor.node_->cost_ - new_cost;
    if (diff_cost > 0) {
      neighbor.node_->disconnectFromParent();
      neighbor.node_->connectToNewParent(*new_node, neighbor.edge_cost_);

      updateSubTreeCost(neighbor.node_, diff_cost);
    }
  }
}

void YawAStar::updateSubTreeCost(YawNode* sub_tree_origin, const double& diff_cost) {
  std::for_each(std::execution::par, sub_tree_origin->children_.begin(), sub_tree_origin->children_.end(), [&](YawNode* child) {
    child->cost_ -= diff_cost;
    updateSubTreeCost(child, diff_cost);
  });
}

void YawAStar::traceBack(const YawNode* node) {
  while (node != nullptr) {
    yaw_path_.emplace_back(node->yaw_);
    node = node->parent_;
  }
}

bool YawAStar::generatePath(const YawNode::Ptr& node) {
  // Step1: 检查目标节点和当前节点的动力学约束
  vector<int> neighbor_ids;
  getNeighborIds(node, neighbor_ids);
  auto it = std::find(neighbor_ids.begin(), neighbor_ids.end(), end_node_->yaw_id_);
  if (it == neighbor_ids.end()) return false;

  // Step2: 检查目标节点和当前节点的共视约束
  if (getCoVisibileNum(new_node, end_node_) <= min_covisible_feature_num_) return false;

  // Step3: 生成路径
  yaw_path_.clear();
  yaw_path_.emplace_back(end_node_->yaw_);
  traceBack(node);
  std::reverse(path_.begin(), path_.end());

  return true;
}

bool YawAStar::search(const double start_yaw, const double end_yaw, const double& dt) {

  // Step1: 清空之前规划数据

  // Step1: 清空之前规划数据
  clear();

  // Step1: 读取dt参数
  param_.dt_ = dt;
  size_t num_layers = pos.size();
  ROS_ASSERT(num_layers >= 2);

  // Step2: 设置起始节点(不检查定位约束，传都传进来了)
  start_node_ = make_unique<YawNode>();
  start_node_->yaw = start_yaw;
  Utils::roundPi(start_node_->yaw);
  calcId(start_node_);
  start_node->layer_ = 0;
  setFeatures(start_node_);

  start_node->g_score = 0.0;
  addNode(start_node);

  // Step3: 设置目标节点并检查定位约束
  end_node_ = make_unique<YawNode>();
  end_node_->yaw = end_yaw;
  Utils::roundPi(end_node_->yaw);
  calcId(end_node_);
  end_node_->layer_ = num_layers - 1;
  setFeatures(end_node_);
  if (end_node_->features_id_.size() <= min_feature_num_) {
    ROS_ERROR("[yaw initial planner]: End point is not localizable!!!");
    return false;
  }

  while (!open_set_.empty()) {
    // Step4.1: 将f_score最小的节点取出
    cur_node = open_set_.top();

    // 已经到达目标节点，尝试连接
    if (cur_node.layer_ == num_layers - 2) {
      if (generatePath()) {
        ROS_INFO("[yaw initial planner]: Successfully generate yaw path!!!");
        publishYawPath();
        return true;
      }
    }

    // Step4.2: 从当前节点向下一层生成新节点
    vector<YawNode::Ptr> new_nodes;
    generateNewNode(cur_node, new_nodes);

    for (const auto& new_node : new_nodes) {
      // Step4.3: 记录当前节点能看到的frontier
      new_node->calcObservedFrontiers();
      RRTNode* new_node_ptr = new_node.get();

      // Step4.4: 在上一层搜索，选择最好（到当前节点cost最小）的父节点,如果没有合适的父节点就放弃生成
      if (!chooseParent(new_node_ptr, neighbors)) continue;

      addNode(std::move(new_node));

      // Step4.5: 在下一层搜索，选择最好（到当前节点cost最小）的父节点
      reWire(new_node_ptr, neighbors);
    }
  }

  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return false;
}

int YawAStar::countVisibleCells(const Vector3d& pos, const double& yaw) {
  return frontier_finder_->countVisibleCells(pos, yaw, target_frontier_);
}

double YawAStar::calcInformationGain(const Vector3d& pos, const double& yaw) {
  return static_cast<double>(countVisibleCells(pos, yaw));
}

int YawAStar::getCoVisibileNum(const YawNode::Ptr& n1, const YawNode::Ptr& n2) {
  int commonFeatureCount = 0;
  for (const auto& id1 : n1->features_id_) {
    for (const auto& id2 : n2->features_id_) {
      if (id1 == id2) {
        commonFeatureCount++;
      }
    }
  }

  // cout << "commonFeatureCount: " << commonFeatureCount << endl;
  return commonFeatureCount;
}

void YawAStar::publishYawPath() {
  vector<Vector3d> pts2;

  for (size_t i = 0; i < yaw_path_.size(); ++i) {
    Vector3d dir(cos(yaw_path_[i]), sin(yaw_path_[i]), 0);
    Vector3d pdir;
    if (i == 0 || i == yaw.size() - 1) {
      pdir = pos_[i] + 1.5 * dir;
    }

    else {
      pdir = pos_[i] + 0.9 * dir;
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

    cnt++;
  }

  ROS_INFO("[publishYawPath]: %d", cnt);
  yaw_path_pub_.publish(mk);
}
}  // namespace fast_planner