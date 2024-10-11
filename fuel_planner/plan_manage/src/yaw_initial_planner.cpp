#include <plan_manage/yaw_initial_planner.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <iostream>

#define ANSI_COLOR_YELLOW_BOLD "\033[1;33m"
#define ANSI_COLOR_GREEN_BOLD "\033[1;32m"
#define ANSI_COLOR_RED_BOLD "\033[1;31m"
#define NORMAL_FONT "\033[0m"

using namespace std;

namespace fast_planner {

YawInitialPlanner::YawInitialPlanner(ros::NodeHandle& nh) {
  yaw_path_pub_ = nh.advertise<visualization_msgs::Marker>("yaw_initial_planner/yaw_path", 20);

  param_.max_yaw_rate_ = Utils::getGlobalParam().max_yaw_rate_;
  param_.min_feature_num_ = Utils::getGlobalParam().min_feature_num_plan_;
  param_.min_covisible_feature_num_ = Utils::getGlobalParam().min_covisible_feature_num_plan_;

  nh.param("yaw_initial/piece_num", param_.piece_num_, -1);
  nh.param("yaw_initial/ld_frontier", param_.ld_frontier_, 0.0);
  nh.param("yaw_initial/ld_final_goal", param_.ld_final_goal_, 0.0);

  param_.basic_cost_ = 2 * M_PI / param_.piece_num_;

  param_.yaw_samples_.resize(param_.piece_num_);
  for (int i = 0; i < param_.piece_num_; ++i) {
    param_.yaw_samples_[i] = i * 2 * M_PI / param_.piece_num_ - M_PI;
  }

  camera_param_ = Utils::getGlobalParam().camera_param_;
}

void YawInitialPlanner::preprocessFrontier() {
  // 根据每一层的pos排除掉：1、太近或太远的frontier;2、被障碍物遮挡的frontier
  size_t num_layers = pos_.size();
  ROS_ASSERT(num_layers >= 2);

  target_frontier_aft_preprocess_.clear();
  target_frontier_aft_preprocess_.resize(num_layers);

  // ROS_INFO_STREAM("[yaw initial planner]: target_frontier size: " << target_frontier_.size());

  for (size_t i = 0; i < num_layers; i++) {
    Vector3d pos = pos_[i];
    for (size_t j = 0; j < target_frontier_.size(); j++) {
      Vector3d cell = target_frontier_[j];
      if (sdf_map_->getVisibility(pos, cell)) target_frontier_aft_preprocess_[i].emplace_back(j);
    }
  }

  // for (size_t i = 0; i < num_layers; i++) {
  //   cout << "layer " << i << endl;
  //   for (const auto& id : target_frontier_aft_preprocess_[i]) cout << id << " ";
  //   cout << endl;
  // }
}

void YawInitialPlanner::yaw2id(const YawVertex::Ptr& v) {
  ROS_ASSERT(v->yaw_ >= -M_PI && v->yaw_ <= M_PI);
  v->yaw_id_ = static_cast<int>((v->yaw_ + M_PI) / (2 * M_PI / param_.piece_num_));
}

void YawInitialPlanner::id2yaw(const YawVertex::Ptr& v) {
  ROS_ASSERT(v->yaw_id_ >= 0 && v->yaw_id_ < param_.piece_num_);
  v->yaw_ = v->yaw_id_ * 2 * M_PI / param_.piece_num_ - M_PI;
}

void YawInitialPlanner::addVertex(YawVertex::Ptr& vertex) {
  vertice_.push_back(vertex);
  vertex->graph_id_ = graph_id_++;
}

double YawInitialPlanner::calEdgeCost(const YawVertex::Ptr& from, const YawVertex::Ptr& to) {
  double diff = from->calEdgeDiff(to);

  // 利用set去重的特性，计算新增的frontier数量
  std::set<int> new_frontier_ids;
  new_frontier_ids.insert(to->frontiers_id_.begin(), to->frontiers_id_.end());
  new_frontier_ids.insert(from->frontiers_id_path_.begin(), from->frontiers_id_path_.end());

  int new_frontier_num = from->frontiers_id_path_.size() + to->frontiers_id_.size() - new_frontier_ids.size();
  ROS_ASSERT(new_frontier_num >= 0);

  return (diff + param_.basic_cost_) *
         (1 + std::exp(-param_.ld_frontier_ * new_frontier_num + (to->if_vis_final_goal_ ? -param_.ld_final_goal_ : 0)));
};

void YawInitialPlanner::estimateHeuristic(const YawVertex::Ptr& v) {
  size_t diff_layer = end_vert_->layer_ - v->layer_;
  double diff_yaw = end_vert_->calEdgeDiff(v);

  v->h_value_ = diff_layer * param_.basic_cost_ + diff_yaw;
}

void YawInitialPlanner::checkIfVisGoal(const YawVertex::Ptr& v) {
  v->if_vis_final_goal_ = sdf_map_->getVisibility(pos_[v->layer_], v->yaw_, final_goal_);
  // if (v->if_vis_final_goal_) {
  //   ROS_INFO_STREAM("YawInitialPlanner::checkIfVisGoal: " << pos_[v->layer_].transpose() << " can see final goal!");
  // }
}

bool YawInitialPlanner::checkFeasibility(const YawVertex::Ptr& v1, const YawVertex::Ptr& v2) {
  int diff_id = abs(v1->yaw_id_ - v2->yaw_id_);
  int diff_id_round = std::min(diff_id, param_.piece_num_ - diff_id);

  return (diff_id_round <= param_.max_diff_yaw_id_);
}

bool YawInitialPlanner::astarSearch() {
  start_vert_->g_value_ = 0.0;

  set<int> open_set_id;
  set<int> close_set;

  open_set_.push(start_vert_);
  open_set_id.emplace(start_vert_->graph_id_);

  while (!open_set_.empty()) {
    auto vc = open_set_.top();
    open_set_.pop();
    open_set_id.erase(vc->graph_id_);
    close_set.emplace(vc->graph_id_);

    // reach target
    if (vc == end_vert_) {
      YawVertex::Ptr vit = vc;
      while (vit != nullptr) {
        vert_path_.push_back(vit);
        vit = vit->parent_;
      }
      reverse(vert_path_.begin(), vert_path_.end());
      return true;
    }

    for (auto& vb : vc->edges_) {
      // skip vertex in close set
      if (close_set.find(vb->graph_id_) != close_set.end()) continue;

      // update new or open vertex
      double g_tmp = vc->g_value_ + calEdgeCost(vc, vb);
      // cout << "g_tmp: " << g_tmp << endl;

      if (open_set_id.find(vb->graph_id_) == open_set_id.end()) {
        open_set_id.emplace(vb->graph_id_);
        open_set_.push(vb);
      }

      else if (g_tmp > vb->g_value_)
        continue;

      vb->parent_ = vc;
      vb->g_value_ = g_tmp;
      vb->f_value_ = vb->g_value_ + vb->h_value_;
      // vb->f_value_ = vb->g_value_;
      vb->frontiers_id_path_ = vc->frontiers_id_path_;
      vb->frontiers_id_path_.insert(vb->frontiers_id_.begin(), vb->frontiers_id_.end());
    }
  }

  ROS_ERROR("AStar can't find path!");
  return false;
}

void YawInitialPlanner::setVisbleFrontiers(const YawVertex::Ptr& v) {
  v->frontiers_id_.clear();

  sdf_map_->countVisibleCells(
      pos_[v->layer_], v->yaw_, target_frontier_, target_frontier_aft_preprocess_[v->layer_], v->frontiers_id_);

  // cout << "result: " << endl;
  // for (const auto& id : v->frontiers_id_) cout << id << " ";
  // cout << endl;
}

void YawInitialPlanner::setVisbleFeatures(const YawVertex::Ptr& v) {
  v->features_id_.clear();

  vector<pair<int, Vector3d>> features;
  Quaterniond ori = Utils::calcOrientation(v->yaw_, acc_[v->layer_]);
  feature_map_->get_NumCloud_using_Odom(pos_[v->layer_], ori, features);
  for (const auto& feature : features) {
    v->features_id_.push_back(feature.first);
  }
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

void YawInitialPlanner::reset() {
  graph_id_ = 0;
  vertice_.clear();
  vert_path_.clear();
  std::priority_queue<YawVertex::Ptr, std::vector<YawVertex::Ptr>, YawNodeComparator> empty;
  open_set_.swap(empty);
}

// bool YawInitialPlanner::search(
//     const double start_yaw, const vector<double>& end_yaw_vec, const double& dt, vector<double>& path) {

//   reset();

//   // Step1: 准备参数
//   param_.dt_ = dt;
//   param_.max_diff_yaw_id_ = (param_.max_yaw_rate_ * dt) / (2 * M_PI / param_.piece_num_);
//   param_.max_diff_yaw_id_ = std::max(1, param_.max_diff_yaw_id_);

//   size_t num_layers = pos_.size();
//   ROS_ASSERT(num_layers >= 2);

//   // Step2: 设置起始节点(不检查定位约束，传都传进来了)
//   start_vert_.reset(new YawVertex(start_yaw, 0));
//   yaw2id(start_vert_);
//   setVisbleFeatures(start_vert_);
//   setVisbleFrontiers(start_vert_);
//   start_vert_->frontiers_id_path_ = start_vert_->frontiers_id_;
//   //(不检查定位约束，传都传进来了)

//   // Step3: 设置目标节点并检查定位约束
//   for (const auto& end_yaw : end_yaw_vec) {
//     YawVertex::Ptr end_vert(new YawVertex(end_yaw, num_layers - 1));
//     // end_vert_.reset(new YawVertex(end_yaw, num_layers - 1));
//     yaw2id(end_vert_);
//     setVisbleFeatures(end_vert_);
//     if (static_cast<int>(end_vert_->features_id_.size()) <= param_.min_feature_num_) {
//       ROS_ERROR("[yaw initial planner]: End yaw %lf is not localizable!!!", end_yaw);
//       // return false;
//       continue;
//     }
//     setVisbleFrontiers(end_vert_);
//     //estimateHeuristic(end_vert_);
//     checkIfVisGoal(end_vert_);

//     end_vert_vec_.push_back(end_vert);
//   }

//   vector<YawVertex::Ptr> layer, last_layer;

//   for (size_t i = 0; i < num_layers; ++i) {
//     if (i == 0) {
//       addVertex(start_vert_);
//       layer.push_back(start_vert_);
//     }

//     else if (i == num_layers - 1) {
//       addVertex(end_vert_);
//       layer.push_back(end_vert_);
//     }

//     else {
//       for (const auto& yaw : param_.yaw_samples_) {
//         YawVertex::Ptr vert(new YawVertex(yaw, i));
//         yaw2id(vert);
//         setVisbleFeatures(vert);
//         if (static_cast<int>(vert->features_id_.size()) <= param_.min_feature_num_) continue;
//         setVisbleFrontiers(vert);
//         estimateHeuristic(vert);
//         checkIfVisGoal(vert);

//         // addVertex(vert);
//         layer.push_back(vert);
//       }
//     }

//     // 检查与上一层是否存在连接
//     if (i != 0) {
//       int add_edge_num = 0;
//       // cout << "last layer size: " << last_layer.size() << endl;
//       // cout << "layer size: " << layer.size() << endl;

//       for (const auto& v1 : last_layer) {
//         // if (i == num_layers - 1) cout << v1->yaw_ << endl;
//         for (const auto& v2 : layer) {
//           // if (i == num_layers - 1) cout << v2->yaw_ << endl;
//           if (checkFeasibility(v1, v2) && getCoVisibileNum(v1, v2) > param_.min_covisible_feature_num_) {
//             v1->edges_.push_back(v2);
//             add_edge_num++;
//             v2->candiate_parent_num_++;
//           }
//         }
//       }

//       layer.erase(
//           remove_if(layer.begin(), layer.end(), [](YawVertex::Ptr v) { return v->candiate_parent_num_ == 0; }), layer.end());

//       for (auto& v : layer) {
//         addVertex(v);
//       }

//       if (add_edge_num == 0) {
//         ROS_ERROR(
//             "[yaw initial planner]: Fail to add edge between layer %zu and layer %zu,total tayer: %zu", i, i + 1, num_layers);
//         return false;  // 中间有断裂的层，search失败
//       }
//     }

//     last_layer.clear();
//     last_layer.swap(layer);
//   }

//   // Step2: 使用A*算法对yaw图进行搜索
//   if (!astarSearch(0, graph_id_ - 1)) {
//     return false;
//   }

//   // Step3: 将存储在vert_path中的yaw角数值提取出来
//   for (const auto& vert : vert_path_) {
//     path.push_back(vert->yaw_);
//   }

//   publishYawPath();

//   return true;
// }

bool YawInitialPlanner::search(
    const double start_yaw, const vector<double>& end_yaw_vec, const double& dt, vector<double>& path) {

  reset();

  // Step1: 准备参数
  param_.dt_ = dt;
  param_.max_diff_yaw_id_ = (param_.max_yaw_rate_ * dt) / (2 * M_PI / param_.piece_num_);
  param_.max_diff_yaw_id_ = std::max(1, param_.max_diff_yaw_id_);

  size_t num_layers = pos_.size();
  ROS_ASSERT(num_layers >= 2);

  // Step2: 设置起始节点(不检查定位约束，传都传进来了)
  start_vert_.reset(new YawVertex(start_yaw, 0));
  yaw2id(start_vert_);
  setVisbleFeatures(start_vert_);
  setVisbleFrontiers(start_vert_);
  start_vert_->frontiers_id_path_ = start_vert_->frontiers_id_;
  //(不检查定位约束，传都传进来了)

  for (size_t i = 0; i < end_yaw_vec.size(); i++) {
    cout << "end yaw " << i << ": " << end_yaw_vec[i] << endl;
  }

  vector<YawVertex::Ptr> layer, last_layer;

  for (size_t i = 0; i < num_layers; ++i) {
    if (i == 0) {
      addVertex(start_vert_);
      layer.push_back(start_vert_);
    }

    else if (i != num_layers - 1) {
      for (const auto& yaw : param_.yaw_samples_) {
        YawVertex::Ptr vert(new YawVertex(yaw, i));
        yaw2id(vert);
        setVisbleFeatures(vert);
        if (static_cast<int>(vert->features_id_.size()) <= param_.min_feature_num_) continue;
        setVisbleFrontiers(vert);
        checkIfVisGoal(vert);
        layer.push_back(vert);
      }
    }

    else {
      for (const auto& end_yaw : end_yaw_vec) {
        YawVertex::Ptr vert(new YawVertex(end_yaw, i));
        yaw2id(vert);
        setVisbleFeatures(vert);
        if (static_cast<int>(vert->features_id_.size()) <= param_.min_feature_num_) {
          ROS_WARN("[yaw initial planner]: End yaw %lf is not localizable!!!", end_yaw);
          continue;
        }

        setVisbleFrontiers(vert);
        checkIfVisGoal(vert);
        layer.push_back(vert);
      }
    }

    // 检查与上一层是否存在连接
    if (i != 0) {
      int add_edge_num = 0;
      // cout << "last layer size: " << last_layer.size() << endl;
      // cout << "layer size: " << layer.size() << endl;

      for (const auto& v1 : last_layer) {
        // if (i == num_layers - 1) cout << v1->yaw_ << endl;
        for (const auto& v2 : layer) {
          // if (i == num_layers - 1) cout << v2->yaw_ << endl;
          if (checkFeasibility(v1, v2) && getCoVisibileNum(v1, v2) > param_.min_covisible_feature_num_) {
            v1->edges_.push_back(v2);
            add_edge_num++;
            v2->candiate_parent_num_++;
          }
        }
      }

      layer.erase(
          remove_if(layer.begin(), layer.end(), [](YawVertex::Ptr v) { return v->candiate_parent_num_ == 0; }), layer.end());

      for (auto& v : layer) {
        addVertex(v);
      }

      if (add_edge_num == 0) {
        ROS_ERROR(
            "[yaw initial planner]: Fail to add edge between layer %zu and layer %zu,total tayer: %zu", i, i + 1, num_layers);
        return false;  // 中间有断裂的层，search失败
      }
    }

    last_layer.clear();
    last_layer.swap(layer);
  }

  // Step2: 记录最后一层数据到end_vert_vec中
  end_vert_vec_.swap(last_layer);
  end_vert_ = end_vert_vec_.front();  // 暂时将目标节点定为end_vert_vec首位，既然能建立出图就起码有一条合法路径
  for (const auto& v : vertice_) {  // 根据确定好的目标节点更新启发函数值
    estimateHeuristic(v);
  }

  // for (size_t i = 0; i < end_vert_vec_.size(); i++) {
  //   cout << "end yaw " << i << " after build graph: " << end_vert_vec_[i]->yaw_ << endl;
  // }

  // Step2: 使用A*算法对yaw图进行搜索
  if (!astarSearch()) {
    return false;
  }

  // Step3: 将存储在vert_path中的yaw角数值提取出来
  for (const auto& vert : vert_path_) {
    path.push_back(vert->yaw_);
  }

  publishYawPath();

  return true;
}

void YawInitialPlanner::publishYawPath() {
  vector<Vector3d> pts2;

  for (size_t i = 0; i < vert_path_.size(); ++i) {
    double yaw = vert_path_[i]->yaw_;
    Vector3d dir(cos(yaw), sin(yaw), 0);
    Vector3d pdir;
    if (i == 0 || i == vert_path_.size() - 1) {
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
  for (size_t i = 0; i < pos_.size(); ++i) {
    pt.x = pos_[i](0);
    pt.y = pos_[i](1);
    pt.z = pos_[i](2);
    mk.points.push_back(pt);

    pt.x = pts2[i](0);
    pt.y = pts2[i](1);
    pt.z = pts2[i](2);
    mk.points.push_back(pt);
  }

  yaw_path_pub_.publish(mk);
}

void YawInitialPlanner::prepareOptData(const YawOptData::Ptr& data) {
  // Step1: 准备观测特征点数据
  data->observed_features_.resize(vert_path_.size() - 1);

  for (size_t layer = 0; layer < vert_path_.size() - 1; ++layer) {
    YawVertex::Ptr v1 = vert_path_[layer];
    YawVertex::Ptr v2 = vert_path_[layer + 1];

    // ROS_ASSERT(static_cast<int>(v1->features_id_.size()) > param_.min_feature_num_);
    ROS_ASSERT(static_cast<int>(v2->features_id_.size()) > param_.min_feature_num_);

    vector<pair<int, Vector3d>> features_1;
    Quaterniond ori_1 = Utils::calcOrientation(v1->yaw_, acc_[v1->layer_]);
    feature_map_->get_NumCloud_using_Odom(pos_[v1->layer_], ori_1, features_1);

    vector<pair<int, Vector3d>> features_2;
    Quaterniond ori_2 = Utils::calcOrientation(v2->yaw_, acc_[v2->layer_]);
    feature_map_->get_NumCloud_using_Odom(pos_[v2->layer_], ori_2, features_2);

    for (size_t j = 0; j < features_1.size(); j++) {
      int id1 = features_1[j].first;

      for (size_t k = 0; k < features_2.size(); k++) {
        int id2 = features_2[k].first;

        if (id1 == id2) {
          data->observed_features_[layer].push_back(features_1[j].second);
          continue;
        }
      }
    }

    ROS_ASSERT(static_cast<int>(data->observed_features_[layer].size()) > param_.min_covisible_feature_num_);
  }

  // Step2: 准备观测frontier数据
  data->frontier_status_.resize(vert_path_.size());

  for (size_t layer = 0; layer < vert_path_.size(); ++layer) {
    YawVertex::Ptr vertex = vert_path_[layer];

    data->frontier_status_[layer].resize(target_frontier_.size(), NOT_AVAILABLE);  // 不可能被观测到

    for (const auto& id : target_frontier_aft_preprocess_[layer]) {
      data->frontier_status_[layer][id] = AVAILABLE;  // 可能被观测到
    }

    for (const auto& id : vertex->frontiers_id_) {
      ROS_ASSERT(std::find(target_frontier_aft_preprocess_[layer].begin(), target_frontier_aft_preprocess_[layer].end(), id) !=
                 target_frontier_aft_preprocess_[layer].end());

      data->frontier_status_[layer][id] = VISIBLE;  // 当前节点已经观测到
    }

    // 要写在后面覆盖掉2的情况
    if (layer != 0) {
      YawVertex::Ptr parent_vertex = vert_path_[layer - 1];
      for (const auto& id : parent_vertex->frontiers_id_path_) {
        data->frontier_status_[layer][id] = HAS_BEEN_OBSERVED;  // 先前节点已经观测到
      }
    }

    // Vector3d pos = pos_[layer];
    // for (const auto& id : target_frontier_aft_preprocess_[layer]) {
    //   if (data->frontier_status_[layer][id] == NOT_AVAILABLE) {

    //     Vector3d cell = target_frontier_[id];
    //     Vector2d yaw_range = camera_param_->calculateYawRange(pos, cell);

    //     double diff1 = fabs(vertex->yaw_ - yaw_range(0));
    //     diff1 = min(diff1, 2 * M_PI - diff1);

    //     double diff2 = fabs(vertex->yaw_ - yaw_range(1));
    //     diff2 = min(diff2, 2 * M_PI - diff2);

    //     if (diff1 < 0.3 || diff2 < 0.3) data->frontier_status_[layer][id] = AVAILABLE;  // 可能被观测到
    //   }
    // }
  }

  // Step3: 顺便在这里把yaw initial planner的一些统计数据打印出来
  cout << ANSI_COLOR_GREEN_BOLD;
  cout << "================================================================== Yaw Initial Planner "
          "Statistics====================================="

       << endl;
  cout << std::setw(15) << std::right << "layer_id: ";
  for (size_t i = 0; i < vert_path_.size(); ++i) cout << std::setw(10) << std::right << i << " ";
  cout << endl;
  cout << "------------------------------------------------------------------------------"
          "--------------------------------------------------------\n";

  cout << std::setw(15) << "not available: ";
  for (size_t layer = 0; layer < vert_path_.size(); ++layer) {
    int count = std::count(data->frontier_status_[layer].begin(), data->frontier_status_[layer].end(), NOT_AVAILABLE);
    cout << std::setw(10) << std::right << count << " ";
  }
  std::cout << std::endl;

  cout << std::setw(15) << "has observed: ";
  for (size_t layer = 0; layer < vert_path_.size(); ++layer) {
    int count = std::count(data->frontier_status_[layer].begin(), data->frontier_status_[layer].end(), HAS_BEEN_OBSERVED);
    cout << std::setw(10) << std::right << count << " ";
  }
  std::cout << std::endl;

  cout << std::setw(15) << "available: ";
  for (size_t layer = 0; layer < vert_path_.size(); ++layer) {
    int count = std::count(data->frontier_status_[layer].begin(), data->frontier_status_[layer].end(), AVAILABLE);
    cout << std::setw(10) << std::right << count << " ";
  }
  std::cout << std::endl;

  cout << std::setw(15) << "visible: ";
  for (size_t layer = 0; layer < vert_path_.size(); ++layer) {
    int count = std::count(data->frontier_status_[layer].begin(), data->frontier_status_[layer].end(), VISIBLE);
    cout << std::setw(10) << std::right << count << " ";
  }
  std::cout << std::endl;

  cout << std::setw(15) << "vis num: ";
  for (size_t layer = 0; layer < vert_path_.size(); ++layer) {
    int count = vert_path_[layer]->frontiers_id_.size();
    cout << std::setw(10) << std::right << count << " ";
  }
  std::cout << std::endl;

  std::cout << "=============================================================================="
               "========================================================\n"
            << std::endl;
  cout << NORMAL_FONT;
}

int YawInitialPlanner::getObservedNum() {
  return end_vert_->frontiers_id_path_.size();
}

}  // namespace fast_planner