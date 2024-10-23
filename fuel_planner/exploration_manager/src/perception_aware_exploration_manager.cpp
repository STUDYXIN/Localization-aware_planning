#include <active_perception/frontier_finder.h>
#include <active_perception/perception_utils.h>
#include <exploration_manager/expl_data.h>
#include <exploration_manager/perception_aware_exploration_manager.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <plan_env/edt_environment.h>
#include <plan_env/feature_map.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>

#include <plan_env/utils.hpp>
#include <plan_manage/perception_aware_planner_manager.h>
#include <stepping_debug.hpp>

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
  planner_manager_->getSteppingDebug(stepping_debug_);
  planner_manager_->initPlanModules(nh);
  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;

  ngd_.reset(new NextGoalData);
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
  }

  nh.param("exploration/feature_num_max", ep_->feature_num_max, -1);
  nh.param("exploration/visb_max", ep_->visb_max, -1);

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
}

bool PAExplorationManager::FindFinalGoal() {
  const auto& final_goal = expl_fsm_.lock()->final_goal_;

  return (sdf_map_->getOccupancy(final_goal) == SDFMap::FREE && sdf_map_->getDistance(final_goal) > 0.2);
}

}  // namespace fast_planner
