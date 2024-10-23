#ifndef _PA_EXPLORATION_MANAGER_H_
#define _PA_EXPLORATION_MANAGER_H_

#include <exploration_manager/perception_aware_exploration_fsm.h>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>

#include <Eigen/Eigen>
#include <memory>
#include <vector>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
using std::weak_ptr;

namespace fast_planner {
class EDTEnvironment;
class SDFMap;
class FeatureMap;
class FastPlannerManager;
class PAExplorationFSM;
class FrontierFinder;
struct ExplorationParam;
struct ExplorationData;
struct NextGoalData;
class SteppingDebug;

class PAExplorationManager {
public:
  PAExplorationManager(shared_ptr<PAExplorationFSM> expl_fsm);

  void initialize(ros::NodeHandle& nh);

  bool FindFinalGoal();

  void selectNextGoal();
  VIEWPOINT_CHANGE_REASON planToNextGoal(const Vector3d& next_pos, const vector<double>& next_yaw_vec, double& next_yaw,
      const vector<Vector3d>& frontire_cells, const bool check_exploration);

  bool findJunction(const vector<Vector3d>& path, Vector3d& point, double& yaw);
  weak_ptr<PAExplorationFSM> expl_fsm_;

  shared_ptr<NextGoalData> ngd_ = nullptr;
  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FrontierFinder> frontier_finder_;
  shared_ptr<FeatureMap> feature_map_;
  shared_ptr<SteppingDebug> stepping_debug_;
  void getSteppingDebug(shared_ptr<SteppingDebug> stepping_debug) {
    stepping_debug_ = stepping_debug;
  }

  // 为终点选择合适的采样点
  vector<double> yaw_samples;
  int yaw_choose_id;
  void setSampleYaw(const Vector3d& pos, const double& yaw_now);
  double getNextYaw();
  double normalizeYaw(double yaw) {
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    return yaw;
  }

  void gereateEndYawForFinalGoal(const Vector3d& pos, const double& cur_yaw, vector<double>& yaw_samples_res);

private:
  shared_ptr<EDTEnvironment> edt_environment_;
  shared_ptr<SDFMap> sdf_map_, global_sdf_map_;

  void searchYaw(const Vector3d& pos, vector<double>& yaw_samples_res);

public:
  typedef shared_ptr<PAExplorationManager> Ptr;
};

}  // namespace fast_planner

#endif