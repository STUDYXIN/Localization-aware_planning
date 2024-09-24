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
  const auto& frontier_cells = expl_fsm_.lock()->choose_frontier_cell;
  auto& last_fail_reason = expl_fsm_.lock()->last_fail_reason;

  bool reach_end_flag = false;

  // 目标点已经出现在已知区域，直接前往
  if (sdf_map_->getOccupancy(final_goal) == SDFMap::FREE && sdf_map_->getDistance(final_goal) > 0.2) {
    last_fail_reason = planToFinalGoal(next_pos, next_yaw, frontier_cells);
    if (last_fail_reason == NO_NEED_CHANGE) {
      ROS_INFO("SUCCESS FIND END!!!!!.");
      return REACH_END;
    } else {
      ROS_WARN("FAIL TO FIND END!!!!!.");
      return NO_AVAILABLE_FRONTIER;
    }
  }

  if (ed_->frontiers_.empty()) return NO_FRONTIER;

  last_fail_reason = planToNextGoal(next_pos, next_yaw, frontier_cells);
  if (last_fail_reason == NO_NEED_CHANGE) return SEARCH_FRONTIER;

  return NO_AVAILABLE_FRONTIER;
}  // namespace fast_planner

VIEWPOINT_CHANGE_REASON PAExplorationManager::planToNextGoal(
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
  // setLastErrorType(expl_fsm_.lock()->last_fail_reason);
  const auto& last_fail_reason = expl_fsm_.lock()->last_fail_reason;
  //简单先把pos和yaw分开来
  int position_traj_statu = planner_manager_->planPosPerceptionAware(
      start_pos, start_vel, start_acc, start_yaw(0), next_pos, Vector3d::Zero(), next_yaw, frontire_cells, time_lb);
  if (position_traj_statu == PATH_SEARCH_ERROR)
    return PATH_SEARCH_FAIL;
  else if (position_traj_statu == POSISION_OPT_ERROR)
    return POSITION_OPT_FAIL;

  if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1) {
    ROS_ERROR("Lower bound not satified!");
  }

  // planner_manager_->planYawExplore(start_yaw, next_yaw, true, ep_->relax_time_);
  int yaw_traj_statu = planner_manager_->planYawPerceptionAware(start_yaw, next_yaw, frontire_cells);
  if (yaw_traj_statu == YAW_INIT_ERROR)
    return YAW_INIT_FAIL;
  else if (yaw_traj_statu == YAW_OPT_ERROR)
    return YAW_OPT_FAIL;

  if (!planner_manager_->checkTrajLocalizabilityOnKnots()) return LOCABILITY_CHECK_FAIL;

  if (!planner_manager_->checkTrajExplorationOnKnots(frontire_cells)) return EXPLORABILITI_CHECK_FAIL;

  planner_manager_->printStatistics();

  return NO_NEED_CHANGE;
}

VIEWPOINT_CHANGE_REASON PAExplorationManager::planToFinalGoal(
    const Vector3d& next_pos, const double& next_yaw, const vector<Vector3d>& frontire_cells) {
  const auto& start_pos = expl_fsm_.lock()->start_pos_;
  const auto& start_vel = expl_fsm_.lock()->start_vel_;
  const auto& start_acc = expl_fsm_.lock()->start_acc_;
  const auto& start_yaw = expl_fsm_.lock()->start_yaw_;

  double diff = fabs(next_yaw - start_yaw[0]);
  double time_lb = min(diff, 2 * M_PI - diff) / ViewNode::yd_;

  const auto& last_fail_reason = expl_fsm_.lock()->last_fail_reason;
  //简单先把pos和yaw分开来
  int position_traj_statu = planner_manager_->planPosPerceptionAware(
      start_pos, start_vel, start_acc, start_yaw(0), next_pos, Vector3d::Zero(), next_yaw, frontire_cells, time_lb);
  if (position_traj_statu == PATH_SEARCH_ERROR)
    return PATH_SEARCH_FAIL;
  else if (position_traj_statu == POSISION_OPT_ERROR)
    return POSITION_OPT_FAIL;

  if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1) {
    ROS_ERROR("Lower bound not satified!");
  }

  // planner_manager_->planYawExplore(start_yaw, next_yaw, true, ep_->relax_time_);
  int yaw_traj_statu = planner_manager_->planYawPerceptionAware(start_yaw, next_yaw, frontire_cells);
  if (yaw_traj_statu == YAW_INIT_ERROR)
    return YAW_INIT_FAIL;
  else if (yaw_traj_statu == YAW_OPT_ERROR)
    return YAW_OPT_FAIL;

  if (!planner_manager_->checkTrajLocalizabilityOnKnots()) return LOCABILITY_CHECK_FAIL;

  planner_manager_->printStatistics();

  return NO_NEED_CHANGE;
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
      // cout << "[PAExplorationManager] find Junction points: " << point.transpose() << endl;
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

void PAExplorationManager::setSampleYaw(const Vector3d& pos, const double& yaw_now) {
  yaw_samples.clear();
  yaw_choose_id = 0;

  // 终点可以不用考虑看向未知区域，只用考虑特征点数量，和与当前状态的一致性，简单从当前yaw开始采样
  double yaw_step = M_PI / 12.0;  // 15度一步进，一共24个采样点

  // 遍历采样的 yaw 值
  for (double yaw = -M_PI; yaw < M_PI; yaw += yaw_step) {
    Vector3d acc = Vector3d::Zero();
    Quaterniond ori = Utils::calcOrientation(yaw, acc);
    auto f_num = feature_map_->get_NumCloud_using_Odom(pos, ori);
    if (f_num > Utils::getGlobalParam().min_feature_num_plan_) yaw_samples.push_back(yaw);
  }
  double norm_target_yaw = normalizeYaw(yaw_now);  // 归一化当前 yaw
                                                   // 对 yaw_samples 按照与当前 yaw_now 的差值进行排序
  std::sort(yaw_samples.begin(), yaw_samples.end(), [this, norm_target_yaw](double yaw1, double yaw2) {
    double diff1 = this->normalizeYaw(yaw1 - norm_target_yaw);
    double diff2 = this->normalizeYaw(yaw2 - norm_target_yaw);
    return std::abs(diff1) < std::abs(diff2);
  });
}

double PAExplorationManager::getNextYaw() {
  if (yaw_choose_id >= yaw_samples.size()) return std::numeric_limits<double>::quiet_NaN();
  return yaw_samples[yaw_choose_id++];
}

void PAExplorationManager::setLastErrorType(VIEWPOINT_CHANGE_REASON reason) {
  switch (reason) {
    case NO_NEED_CHANGE:
      planner_manager_->last_error_type = SUCCESS_FIND_POSISION_TRAJ;  // 假设 NO_NEED_CHANGE 表示成功
      break;
    case PATH_SEARCH_FAIL:
      planner_manager_->last_error_type = PATH_SEARCH_ERROR;
      break;
    case POSITION_OPT_FAIL:
      planner_manager_->last_error_type = POSISION_OPT_ERROR;
      break;
    case YAW_INIT_FAIL:
      planner_manager_->last_error_type = YAW_INIT_ERROR;
      break;
    case YAW_OPT_FAIL:
      planner_manager_->last_error_type = YAW_OPT_ERROR;
      break;
    default:
      planner_manager_->last_error_type = -1;  // 未知错误
      break;
  }
}

}  // namespace fast_planner
