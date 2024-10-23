#include "path_searching/rrt_star.h"

#include <plan_env/sdf_map.h>
#include <plan_env/utils.hpp>

#include <execution>
#include <random>

using namespace std;
using namespace Eigen;

namespace fast_planner {

void RRTParameters::readParameters(ros::NodeHandle& nh) {
  nh.param<bool>("rrt/use_kd_tree", use_kd_tree_, false);

  nh.param<double>("rrt/goal_biased_prob", goal_biased_prob_, 0.15);
  nh.param<double>("rrt/step_size", step_size_, 0.2);
  nh.param<double>("rrt/neighbor_radius", neighbor_radius_, 1.0);

  nh.param<double>("rrt/goal_dist_threshold", goal_dist_threshold_, 1.0);
  nh.param<double>("rrt/max_search_time", max_search_time_, 100.0);

  nh.param<int>("rrt/yaw_sample_piece_num", yaw_sample_piece_num_, 6);
}

void RRTVisualizer::init(ros::NodeHandle& nh) {
  path_vis_pub_ = nh.advertise<visualization_msgs::Marker>("rrt/path", 20);
  tree_vis_pub_ = nh.advertise<visualization_msgs::Marker>("rrt/tree", 1);
}

void RRTVisualizer::visPath(const vector<Vector3d>& path, const bool pub_pt) {
  cout << "input path size: " << path.size() << endl;

  visualization_msgs::Marker Points, Line;
  Points.header.frame_id = Line.header.frame_id = "world";
  Points.header.stamp = Line.header.stamp = ros::Time::now();
  Line.pose.orientation.w = 1.0;
  Points.ns = Line.ns = "Path";
  Points.id = 0;
  Line.id = 1;

  Points.type = visualization_msgs::Marker::POINTS;
  Line.type = visualization_msgs::Marker::LINE_STRIP;

  Points.scale.x = Points.scale.y = 0.1;
  Line.scale.x = 0.1;

  Points.color.g = Points.color.a = 1.0;
  Line.color.b = Line.color.a = 1.0;

  if (path.size() > 1) {
    for (const auto& pos : path) {
      geometry_msgs::Point geo_pt = eigen2geo(pos);
      if (pub_pt) {
        cout << "pos: " << pos.transpose() << endl;
        Points.points.push_back(geo_pt);
      }
      Line.points.push_back(geo_pt);
    }
  }

  else
    Points.action = Line.action = visualization_msgs::Marker::DELETEALL;

  if (pub_pt) path_vis_pub_.publish(Points);
  path_vis_pub_.publish(Line);
}

void RRTVisualizer::visTree(const vector<RRTNode::Ptr>& tree) {
  visualization_msgs::Marker Points, Line;
  Points.header.frame_id = Line.header.frame_id = "world";
  Points.header.stamp = Line.header.stamp = ros::Time::now();
  Points.ns = Line.ns = "Tree";
  if (tree.empty())
    Points.action = Line.action = visualization_msgs::Marker::DELETEALL;
  else {
    Points.action = Line.action = visualization_msgs::Marker::ADD;
    Points.pose.orientation.w = Line.pose.orientation.w = 1.0;
    Points.id = 0;
    Line.id = 1;

    Points.type = visualization_msgs::Marker::POINTS;
    Line.type = visualization_msgs::Marker::LINE_LIST;

    Points.scale.x = Points.scale.y = 0.05;
    Line.scale.x = 0.01;

    // Points are green and Line Strip is blue
    Points.color.g = Points.color.a = 0.5;
    // Points.color.g = Points.color.r = 255*traversability;
    Line.color.b = Line.color.a = 0.75;

    geometry_msgs::Point pt;
    geometry_msgs::Point parent_pt;
    for (const auto& node : tree) {
      pt = eigen2geo(node->position_);
      Points.points.push_back(pt);

      if (node->parent_ != nullptr) {
        Line.points.push_back(pt);
        parent_pt = eigen2geo(node->parent_->position_);
        Line.points.push_back(parent_pt);
      }
    }
  }

  tree_vis_pub_.publish(Points);
  tree_vis_pub_.publish(Line);
}

void RRTStar::init(ros::NodeHandle& nh, const EDTEnvironment::Ptr& env) {
  param_.readParameters(nh);

  visualizer_.reset(new RRTVisualizer());
  visualizer_->init(nh);

  this->edt_env_ = env;

  if (param_.use_kd_tree_) {
    tree_point_cloud_.reset(new RRTPointCloud());
    kdtree_.reset(new pcl::KdTreeFLANN<RRTPoint>());
  }
}

bool RRTStar::makeProblem(const Vector3d& start_pos, const double start_yaw, const Vector3d& end_pos, const double end_yaw) {

  // if (edt_env_->sdf_map_->getInflateOccupancy(start_pos) == 1 ||
  // edt_env_->sdf_map_->getOccupancy(start_pos) == SDFMap::UNKNOWN) {

  //   ROS_WARN("Start position is in unkown space or obstacle!!!");
  //   return false;
  // }

  // if (edt_env_->sdf_map_->getInflateOccupancy(end_pos) == 1 ||
  // edt_env_->sdf_map_->getOccupancy(end_pos) == SDFMap::UNKNOWN) {

  //   ROS_WARN("End position is in unkown space or obstacle!!!");
  //   return false;
  // }

  path_.reset();
  tree_.clear();
  if (param_.use_kd_tree_) {
    tree_point_cloud_->clear();
  }

  close_check_record_.clear();

  origin_node_ = nullptr;
  generateNewNode(start_pos, origin_node_);
  if (origin_node_ == nullptr) {
    ROS_WARN("Start position is in unkown space or obstacle!!!");
    return false;
  }
  origin_node_->yaw_ = start_yaw;
  addNode(std::move(origin_node_));

  target_node_ = nullptr;
  generateNewNode(end_pos, target_node_);
  if (target_node_ == nullptr) {
    ROS_WARN("End position is in unkown space or obstacle!!!");
    return false;
  }
  target_node_->yaw_ = end_yaw;

  cout << "start_pos: " << start_pos.transpose() << endl;
  cout << "end_pos: " << end_pos.transpose() << endl;
  cout << "test pos: " << tree_.front()->position_.transpose() << endl;
  closeCheck(tree_.front().get());

  visualizer_->visPath({});
  visualizer_->visTree({});

  return true;
}

double RRTStar::getRandomNum() {
  std::random_device rand_rd;
  std::mt19937 rand_gen(rand_rd());
  std::uniform_real_distribution<double> rand_unif(0, 1.0);
  return rand_unif(rand_gen);
}

Vector3d RRTStar::getRandom3DPoint() {
  Vector3d sample_lb;
  Vector3d sample_ub;

  edt_env_->sdf_map_->getBox(sample_lb, sample_ub);

  Vector3d rand_point;
  random_device rand_rd;
  mt19937 rand_gen(rand_rd());

  uniform_real_distribution<double> rand_unif_x(sample_lb.x(), sample_ub.x());
  rand_point.x() = rand_unif_x(rand_gen);

  uniform_real_distribution<double> rand_unif_y(sample_lb.y(), sample_ub.y());
  rand_point.y() = rand_unif_y(rand_gen);

  uniform_real_distribution<double> rand_unif_z(sample_lb.z(), sample_ub.z());
  rand_point.z() = rand_unif_z(rand_gen);

  return rand_point;
}

Vector3d RRTStar::sampleRandomPoint() {
  Vector3d point_sample;
  if (getRandomNum() < param_.goal_biased_prob_)
    point_sample = target_node_->position_;
  else
    point_sample = getRandom3DPoint();

  return point_sample;
}

RRTNode* RRTStar::findNearestNode(const Vector3d& point) {
  if (param_.use_kd_tree_) {
    vector<int> k_indices(1);
    vector<float> k_distances(1);
    // cout << "tree size: " << tree_point_cloud_->size() << endl;
    // cout << "tree first: " << tree_point_cloud_->points.front().x << " " <<
    // tree_point_cloud_->points.front().y << " "
    //      << tree_point_cloud_->points.front().z << endl;

    // cout << "start pos: " << tree_.front()->position_.transpose() << endl;
    // cout << "start yaw: " << tree_.front()->yaw_ << endl;

    // cout << "input point: " << point.transpose() << endl;

    kdtree_->nearestKSearch(eigen2pcl(point), 1, k_indices, k_distances);
    return tree_[k_indices.front()].get();
  }

  else {
    float min_dis = std::numeric_limits<float>::max();
    RRTNode* node_closest = nullptr;

    for (const auto& node : tree_) {
      float tmp_dis = fabs(point(0) - node->position_(0)) + fabs(point(1) - node->position_(1));
      if (tmp_dis < min_dis) {
        min_dis = tmp_dis;
        node_closest = node.get();
      }
    }
    return node_closest;
  }
}

Vector3d RRTStar::steer(const Vector3d& point_rand, RRTNode* node_nearest) {
  Vector3d point_new;
  Vector3d point_nearest = node_nearest->position_;
  Vector3d steer_p = point_rand - point_nearest;
  double steer_norm = steer_p.norm();
  if (steer_norm > param_.step_size_)
    point_new = point_nearest + steer_p * param_.step_size_ / steer_norm;
  else
    point_new = point_rand;

  return point_new;
}

void RRTStar::generateNewNode(const Vector3d& pos, RRTNode::Ptr& node) {
  node = nullptr;

  if (edt_env_->sdf_map_->getInflateOccupancy(pos) == 1 || edt_env_->sdf_map_->getOccupancy(pos) == SDFMap::UNKNOWN) {
    return;
  }

  node = make_unique<RRTNode>();
  node->position_ = pos;
}

void RRTStar::generateNewNode(const Vector3d& pos, vector<RRTNode::Ptr>& nodes) {
  nodes.clear();

  if (edt_env_->sdf_map_->getInflateOccupancy(pos) == 1 || edt_env_->sdf_map_->getOccupancy(pos) == SDFMap::UNKNOWN) {
    return;
  }

  vector<double> yaws;
  generateYaw(yaws);

  // std::for_each(std::execution::par, yaws.begin())

  // std::for_each(std::execution::par, yaws.begin(), yaws.end(), [&](double
  // yaw) {
  //   Eigen::Matrix3d rot = Eigen::AngleAxisd(yaw,
  //   Eigen::Vector3d::UnitZ()).toRotationMatrix(); Quaterniond quat(rot);

  //   vector<Eigen::Vector3d> res;

  //   if (feature_map_->get_NumCloud_using_PosOrient(pos, quat, res) >
  //   param_.min_feature_num_) {
  //     RRTNode::Ptr node = make_unique<RRTNode>();
  //     node->position_ = pos;
  //     node->yaw_ = yaw;
  //     nodes.emplace_back(std::move(node));
  //   }
  // });

  for (const auto& yaw : yaws) {
    Eigen::Matrix3d rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Quaterniond quat(rot);

    vector<Eigen::Vector3d> res;

    // std::cout << "input pos: " << pos.transpose() << std::endl;
    // std::cout << "input yaw: " << yaw << std::endl;

    auto feature_num = feature_map_->get_NumCloud_using_Odom(pos, quat, res);
    // RRTNode* node = tree_.front().get();
    // auto feature_num = feature_map_->get_NumCloud_using_Odom(node->position_,
    // Quaterniond::Identity(), res); cout << "feature_num: " << feature_num <<
    // endl;

    int min_feature_num = Utils::getGlobalParam().min_feature_num_plan_;
    if (feature_num > min_feature_num) {

      // if (feature_map_->get_NumCloud_using_PosOrient(pos, quat, res) >
      // param_.min_feature_num_) {
      RRTNode::Ptr node = make_unique<RRTNode>();
      node->position_ = pos;
      node->yaw_ = yaw;
      nodes.emplace_back(std::move(node));
      // std::cout << "success!!!" << yaw << std::endl;
      break;
    }
  }
}

void RRTStar::generateYaw(vector<double>& yaws) {
  double yaw_res = 2 * M_PI / param_.yaw_sample_piece_num_;
  for (int i = 0; i < param_.yaw_sample_piece_num_; i++) {
    yaws.push_back(-M_PI + i * yaw_res);
  }
}

bool RRTStar::checkCollision(const RRTNode& node_start, const RRTNode& node_end) {
  Vector3d dir = node_end.position_ - node_start.position_;
  double len = dir.norm();
  dir.normalize();
  for (double l = 0.1; l < len; l += 0.1) {
    Vector3d ckpt = node_start.position_ + l * dir;
    if (edt_env_->sdf_map_->getInflateOccupancy(ckpt) == 1 || edt_env_->sdf_map_->getOccupancy(ckpt) == SDFMap::UNKNOWN) {

      return false;
    }
  }

  return true;
}

void RRTStar::findNeighbors(const RRTNode* new_node, vector<neighbor_info_t>& neighbors) {
  neighbor_info_t neighbor_info;

  if (param_.use_kd_tree_) {
    vector<int> k_indices;
    vector<float> k_distances;
    int neighbor_num = kdtree_->radiusSearch(eigen2pcl(new_node->position_), param_.neighbor_radius_, k_indices, k_distances);

    // std::for_each(std::execution::par, k_indices.begin(), k_indices.end(),
    // [&](int i) {
    //   RRTNode* node_ptr = tree_[i].get();
    //   if (checkCollision(*new_node, *node_ptr)) {
    //     neighbor_info.node_ = node_ptr;
    //     neighbor_info.edge_dis_ = new_node->calEdgeDis(*node_ptr);
    //     neighbor_info.edge_cost_ = new_node->calEdgeCost(*node_ptr);
    //     neighbors.emplace_back(neighbor_info);
    //   }
    // });

    for (int i = 0; i < neighbor_num; i++) {
      RRTNode* node_ptr = tree_[k_indices[i]].get();
      if (checkCollision(*new_node, *node_ptr)) {
        neighbor_info.node_ = node_ptr;
        neighbor_info.edge_dis_ = new_node->calEdgeDis(*node_ptr);
        neighbor_info.edge_cost_ = new_node->calEdgeCost(*node_ptr);
        neighbors.emplace_back(neighbor_info);
      }
    }
  }

  else {
    for (const auto& node : tree_) {
      RRTNode* node_ptr = node.get();
      double dis = new_node->calEdgeDis(*node_ptr);
      if (dis < param_.neighbor_radius_ && checkCollision(*new_node, *node_ptr)) {
        neighbor_info.node_ = node_ptr;
        neighbor_info.edge_dis_ = dis;
        neighbor_info.edge_cost_ = new_node->calEdgeCost(*node_ptr);
        neighbors.emplace_back(neighbor_info);
      }
    }
  }
}

void RRTStar::chooseParent(RRTNode* node_new, const vector<neighbor_info_t>& neighbors) {
  static const auto compareFun = [&](const neighbor_info_t& a, const neighbor_info_t& b) -> bool {
    return a.node_->cost_ + a.edge_cost_ < b.node_->cost_ + b.edge_cost_;
  };

  auto it = std::min_element(std::execution::par, neighbors.begin(), neighbors.end(), compareFun);
  node_new->connectToNewParent(*it->node_);
  updateNewNodeCost(node_new, &(*it));
}

void RRTStar::updateNewNodeCost(RRTNode* node_new, const neighbor_info_t* neighbor_info) {
  if (neighbor_info == nullptr) {
    node_new->dis_ = node_new->parent_->dis_ + node_new->calEdgeDis(*node_new->parent_);
    node_new->cost_ = node_new->parent_->cost_ + node_new->calEdgeCost(*node_new->parent_);
  }

  else {
    node_new->dis_ = node_new->parent_->dis_ + neighbor_info->edge_dis_;
    node_new->cost_ = node_new->parent_->cost_ + neighbor_info->edge_cost_;
  }
}

void RRTStar::reWire(RRTNode* new_node, vector<neighbor_info_t>& neighbors) {
  for (const auto& neighbor : neighbors) {
    double new_cost = new_node->cost_ + neighbor.edge_cost_;
    double diff_cost = neighbor.node_->cost_ - new_cost;
    if (diff_cost > 0) {
      double new_dis = new_node->dis_ + neighbor.edge_dis_;
      double diff_dis = neighbor.node_->dis_ - new_dis;

      neighbor.node_->disconnectFromParent();
      neighbor.node_->connectToNewParent(*new_node);
      updateNewNodeCost(neighbor.node_, &neighbor);

      updateSubTreeCost(neighbor.node_, diff_dis, diff_cost);
    }
  }
}

void RRTStar::updateSubTreeCost(RRTNode* sub_tree_origin, const double& diff_dis, const double& diff_cost) {
  // for (const auto& child : sub_tree_origin->children_) {
  //   child->dis_ -= diff_dis;
  //   child->cost_ -= diff_cost;
  //   updateSubTreeCost(child, diff_dis, diff_cost);
  // }

  std::for_each(std::execution::par, sub_tree_origin->children_.begin(), sub_tree_origin->children_.end(), [&](RRTNode* child) {
    child->dis_ -= diff_dis;
    child->cost_ -= diff_cost;
    updateSubTreeCost(child, diff_dis, diff_cost);
  });
}

void RRTStar::closeCheck(const RRTNode* node) {
  double dis = node->calEdgeDis(*target_node_);
  if (dis < param_.goal_dist_threshold_ && checkCollision(*node, *target_node_))
    close_check_record_.emplace_back(node, node->calEdgeCost(*target_node_));
}

void RRTStar::generatePath() {
  static const auto compareFun = [](const PairType& a, const PairType& b) -> bool {
    return a.first->cost_ + a.second < b.first->cost_ + b.second;
  };

  if (!close_check_record_.empty()) {
    auto it = min_element(close_check_record_.begin(), close_check_record_.end(), compareFun);
    double min_cost = it->first->cost_ + it->second;

    path_.waypoints_.clear();
    // path_.waypoints_.emplace_back(target_node_->position_);
    path_.traceBack(it->first);
    // path_.addNodeRecursively(it->first);
    path_.dis_ = it->first->dis_ + it->first->calEdgeDis(*target_node_);
    path_.cost_ = min_cost;

    std::reverse(path_.waypoints_.begin(), path_.waypoints_.end());
  }
}

bool RRTStar::makePlan() {
  if (feature_map_ == nullptr) {
    ROS_ERROR("Feature map is not set!!!");
    return false;
  }

  curr_time_ = 0.0;
  iter_num_ = 0;

  double time_now = curr_time_;
  ros::Time time_start = ros::Time::now();
  while (curr_time_ < param_.max_search_time_) {
    iter_num_++;
    curr_time_ = (ros::Time::now() - time_start).toSec() * 1000.0 + time_now;

    Vector3d rand_point = sampleRandomPoint();

    if (param_.use_kd_tree_) {
      kdtree_->setInputCloud(tree_point_cloud_);
    }

    RRTNode* nearest_node = findNearestNode(rand_point);
    Vector3d new_point = steer(rand_point, nearest_node);

    // RRTNode::Ptr new_node = nullptr;
    vector<RRTNode::Ptr> new_nodes;
    generateNewNode(new_point, new_nodes);

    for (auto& new_node : new_nodes) {
      if (new_node != nullptr) {
        RRTNode* new_node_ptr = new_node.get();
        vector<neighbor_info_t> neighbors;
        findNeighbors(new_node_ptr, neighbors);
        if (!neighbors.empty()) {
          addNode(std::move(new_node));
          chooseParent(new_node_ptr, neighbors);
          reWire(new_node_ptr, neighbors);
          closeCheck(new_node_ptr);
        }
      }
      // if (new_node != nullptr) {
      //   RRTNode* new_node_ptr = new_node.get();
      //   vector<neighbor_info_t> neighbors;
      //   findNeighbors(new_node_ptr, neighbors);
      //   if (!neighbors.empty()) {
      //     addNode(std::move(new_node));
      //     chooseParent(new_node_ptr, neighbors);
      //     reWire(new_node_ptr, neighbors);
      //     closeCheck(new_node_ptr);
      //   }
      // }

      // for (const auto& node : tree_) {
      //   if (fabs(node->dis_ - node->cost_) > 1e-3) {
      //     ROS_ERROR("Path length and cost are not equal!!!");
      //     ROS_BREAK();
      //   }
      // }
    }
  }

  generatePath();
  printStatistics();
  visualizer_->visPath(path_.waypoints_, true);
  visualizer_->visTree(tree_);

  return !path_.waypoints_.empty();
}

void RRTStar::printStatistics() {
  cout << "Nodes num in tree: " << tree_.size() << endl;
  if (!path_.waypoints_.empty()) {
    cout << "Iter num: " << iter_num_ << endl;
    cout << "Nodes num in path: " << path_.waypoints_.size() << endl;
    cout << "Path length: " << path_.dis_ << endl;
    cout << "Path cost: " << path_.cost_ << endl;
    cout << "Linear Distance: " << tree_.front()->calEdgeDis(*target_node_) << endl;

    // if (fabs(path_.dis_ - path_.cost_) > 1e-3) {
    //   ROS_ERROR("Path length and cost are not equal!!!");
    //   ROS_BREAK();
    // }
  }
}
}  // namespace fast_planner
