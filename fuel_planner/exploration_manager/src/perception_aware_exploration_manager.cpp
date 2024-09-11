#include <active_perception/frontier_finder.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <exploration_manager/expl_data.h>
#include <exploration_manager/perception_aware_exploration_manager.h>
#include <lkh_tsp_solver/lkh_interface.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <plan_env/edt_environment.h>
#include <plan_env/feature_map.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>

#include <plan_manage/perception_aware_planner_manager.h>
#include <plan_env/utils.hpp>

#include <visualization_msgs/Marker.h>

#include <fstream>
#include <iostream>
#include <thread>

#define GET_FSM_STATE()                                                                                                          \
  const auto& start_pos = expl_fsm_.lock()->start_pos_;                                                                          \
  const auto& start_vel = expl_fsm_.lock()->start_vel_;                                                                          \
  const auto& start_acc = expl_fsm_.lock()->start_acc_;                                                                          \
  const auto& start_yaw = expl_fsm_.lock()->start_yaw_;                                                                          \
  const auto& final_goal = expl_fsm_.lock()->final_goal_;

using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query
PAExplorationManager::PAExplorationManager(shared_ptr<PAExplorationFSM> expl_fsm) : expl_fsm_(expl_fsm) {
}

void PAExplorationManager::initialize(ros::NodeHandle& nh) {
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);

  nh.param("feature/using_feature", ep_->using_feature, false);

  if (ep_->using_feature) {
    global_sdf_map_.reset(new SDFMap);
    global_sdf_map_->using_global_map = true;
    global_sdf_map_->initMap(nh);
    feature_map_.reset(new FeatureMap);
    feature_map_->setMap(global_sdf_map_);
    feature_map_->initMap(nh);

    planner_manager_->setFeatureMap(feature_map_);
    frontier_finder_.reset(new FrontierFinder(edt_environment_, feature_map_, nh));
    planner_manager_->setFrontierFinder(frontier_finder_);
  }

  else
    frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));

  nh.param("exploration/refine_local", ep_->refine_local_, true);
  nh.param("exploration/refined_num", ep_->refined_num_, -1);
  nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.param("exploration/relax_time", ep_->relax_time_, 1.0);

  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

  nh.param("exploration/feature_num_max", ep_->feature_num_max, -1);
  nh.param("exploration/visb_max", ep_->visb_max, -1);
  nh.param("exploration/we", ep_->we, -1.0);
  nh.param("exploration/wg", ep_->wg, -1.0);
  nh.param("exploration/wf", ep_->wf, -1.0);
  nh.param("exploration/wc", ep_->wc, -1.0);

  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh, edt_environment_);
  ViewNode::map_ = sdf_map_;

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);
}

NEXT_GOAL_TYPE PAExplorationManager::selectNextGoal(Vector3d& next_pos, double& next_yaw) {
  const auto& start_pos = expl_fsm_.lock()->start_pos_;
  const auto& start_vel = expl_fsm_.lock()->start_vel_;
  const auto& start_acc = expl_fsm_.lock()->start_acc_;
  const auto& start_yaw = expl_fsm_.lock()->start_yaw_;
  const auto& final_goal = expl_fsm_.lock()->final_goal_;
  const auto& gains = expl_fsm_.lock()->gains_;
  const auto& points = ed_->points_;
  const auto& yaws = ed_->yaws_;
  const auto& frontier_cells = ed_->frontier_cells_;

  bool reach_end_flag = false;

  // 目标点已经出现在已知区域，直接前往
  if (sdf_map_->getOccupancy(final_goal) == SDFMap::FREE && sdf_map_->getDistance(final_goal) > 0.2) {
    next_pos = final_goal;
    ROS_INFO("Select final goal as next goal");

    // next_yaw = 0.0;

    vector<double> candidate_yaws;
    searchYaw(next_pos, candidate_yaws);

    bool success = false;
    vector<Vector3d> empty;

    for (const auto& yaw : candidate_yaws) {
      if (planToNextGoal(next_pos, yaw, empty)) {
        ROS_INFO(
            "Successfully planned to final goal----x: %f, y: %f, z: %f, yaw: %f", next_pos(0), next_pos(1), next_pos(2), yaw);
        success = true;
        reach_end_flag = true;
        break;
      }
    }

    // vector<Vector3d> empty;
    // if (planToNextGoal(next_pos, next_yaw, empty)) {
    //   ROS_INFO("Successfully planned to final goal.");
    //   reach_end_flag = true;
    // }

    if (!success) {
      ROS_INFO("Failed to plan to final goal.Try to search frontier.");
    }
  }

  // 需要渐进地通过前沿区域朝目标点摸索

  // 使用混合A*算法搜索最优路径

  // Search frontiers and group them into clusters 在fsm中的frontier定时器里实时更新

  if (reach_end_flag) return REACH_END;

  if (ed_->frontiers_.empty()) {
    ROS_WARN("No coverable frontier.");
    return NO_AVAILABLE_FRONTIER;
  }

  // Select point with highest score
  // const double dg = (final_goal - start_pos).norm();
  // vector<pair<size_t, double>> gains;
  // for (size_t i = 0; i < ed_->points_.size(); ++i) {
  //   double visb_score = static_cast<double>(ed_->visb_num_[i]) / static_cast<double>(ep_->visb_max);
  //   double goal_score = (dg - (final_goal - ed_->points_[i]).norm()) / dg;
  //   double feature_score = static_cast<double>(feature_map_->get_NumCloud_using_justpos(ed_->points_[i])) /
  //                          static_cast<double>(ep_->feature_num_max);
  //   double motioncons_score =
  //       std::sin((ed_->points_[i] - start_pos).dot(start_vel) / ((ed_->points_[i] - start_pos).norm() * start_vel.norm())) /
  //       (M_PI / 2);
  //   double score = ep_->we * visb_score + ep_->wg * goal_score + ep_->wf * feature_score + ep_->wc * motioncons_score;
  //   // cout << "[PAExplorationManager] SCORE DEUBUG NUM: " << i << " visb_score: " << visb_score << " goal_score: " <<
  //   goal_score
  //   //      << " feature_score: " << feature_score << " score: " << score << " motioncons_score: " << motioncons_score << endl;
  //   gains.emplace_back(i, score);
  // }

  // std::sort(gains.begin(), gains.end(), [&](const auto& a, const auto& b) { return a.second > b.second; });

  // 按序遍历前沿点，直到找到一个可以规划成功的点

  // expl_fsm_.lock()->best_frontier_id默认是0，但是若是失败会外部递增
  size_t idx = gains[expl_fsm_.lock()->best_frontier_id].first;
  next_pos = points[idx];
  next_yaw = yaws[idx];
  vector<Vector3d> next_frontier_cells = frontier_cells[idx];
  if (planToNextGoal(next_pos, next_yaw, next_frontier_cells)) {
    expl_fsm_.lock()->last_used_viewpoint_pos = next_pos;
    return SEARCH_FRONTIER;
  }
  ROS_ERROR("[PAExplorationManager::selectNextGoal] The best Viewpoint can't be reached!!!!");
  return NO_AVAILABLE_FRONTIER;
}

bool PAExplorationManager::planToNextGoal(
    const Vector3d& next_pos, const double& next_yaw, const vector<Vector3d>& frontire_cells) {
  const auto& start_pos = expl_fsm_.lock()->start_pos_;
  const auto& start_vel = expl_fsm_.lock()->start_vel_;
  const auto& start_acc = expl_fsm_.lock()->start_acc_;
  const auto& start_yaw = expl_fsm_.lock()->start_yaw_;
  const auto& final_goal = expl_fsm_.lock()->final_goal_;

  double diff = fabs(next_yaw - start_yaw[0]);
  double time_lb = min(diff, 2 * M_PI - diff) / ViewNode::yd_;

  // if (!planner_manager_->kinodynamicReplan(
  //         start_pos, start_vel, start_acc, start_yaw(0), next_pos, Vector3d::Zero(), next_yaw, time_lb)) {
  //   return false;
  // }

  // if (!planner_manager_->sampleBasedReplan(start_pos, start_vel, start_acc, start_yaw(0), next_pos, next_yaw, time_lb)) {
  //   return false;
  // }
  if (!planner_manager_->planPosPerceptionAware(
          start_pos, start_vel, start_acc, start_yaw(0), next_pos, Vector3d::Zero(), next_yaw, time_lb)) {
    return false;
  }

  if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1) {
    ROS_ERROR("Lower bound not satified!");
  }

  // planner_manager_->planYawExplore(start_yaw, next_yaw, true, ep_->relax_time_);
  if (!planner_manager_->planYawPerceptionAware(start_yaw, next_yaw, frontire_cells)) return false;

  return true;
}

void PAExplorationManager::shortenPath(vector<Vector3d>& path) {
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }

  // Shorten the tour, only critical intermediate points are reserved.
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = { path.front() };
  for (int i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }

  if ((path.back() - short_tour.back()).norm() > 1e-3) {
    short_tour.push_back(path.back());
  }

  // Ensure at least three points in the path
  if (short_tour.size() == 2) {
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  }

  path = short_tour;
}

bool PAExplorationManager::findJunction(const vector<Vector3d>& path, Vector3d& point, double& yaw) {
  if (path.empty()) {
    ROS_ERROR("[PAExplorationManager] Empty path provided to find junction");
    return false;
  }

  for (int i = path.size() - 1; i >= 0; --i) {
    if (edt_environment_->sdf_map_->getOccupancy(path[i]) == SDFMap::FREE) {  // FREE point
      point = path[i];
      cout << "[PAExplorationManager] find Junction points: " << point.transpose() << endl;
      if (i == path.size() - 1 && i > 0) {  // goal is in FREE
        Eigen::Vector3d direction = path[i] - path[i - 1];
        yaw = atan2(direction.y(), direction.x());
      } else if (i == path.size() - 1 && i == 0) {
        return false;
      } else {
        Eigen::Vector3d direction = path[i + 1] - path[i];
        yaw = atan2(direction.y(), direction.x());
      }
      return true;
    }
  }
  ROS_ERROR("[PAExplorationManager] No free point found, defaulting to path's start point.");
  point = path.front();
  return false;
}

void PAExplorationManager::searchYaw(const Vector3d& pos, vector<double>& yaw_samples_res) {
  const int half_vert_num_ = 5;
  const double yaw_diff_ = M_PI / 6;

  yaw_samples_res.clear();

  vector<double> yaw_samples;
  int vert_num = 2 * half_vert_num_ + 1;
  for (int j = 0; j < vert_num; j++) yaw_samples.push_back((j - half_vert_num_) * yaw_diff_);
  yaw_samples.push_back(M_PI);

  for (const auto& yaw : yaw_samples) {
    Vector3d acc = Vector3d::Zero();
    Quaterniond ori = Utils::calcOrientation(yaw, acc);
    auto f_num = feature_map_->get_NumCloud_using_Odom(pos, ori);

    int min_feature_num = Utils::getGlobalParam().min_feature_num_plan_;
    if (f_num > min_feature_num) yaw_samples_res.push_back(yaw);
  }
}

}  // namespace fast_planner
