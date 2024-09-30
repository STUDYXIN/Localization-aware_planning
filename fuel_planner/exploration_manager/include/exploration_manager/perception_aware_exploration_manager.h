#ifndef _PA_EXPLORATION_MANAGER_H_
#define _PA_EXPLORATION_MANAGER_H_

#include <exploration_manager/perception_aware_exploration_fsm.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <stepping_debug.hpp>

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
class SteppingDebug;

enum NEXT_GOAL_TYPE { REACH_END, SEARCH_FRONTIER, NO_FRONTIER, NO_AVAILABLE_FRONTIER };

class PAExplorationManager {
public:
  PAExplorationManager(shared_ptr<PAExplorationFSM> expl_fsm);

  void initialize(ros::NodeHandle& nh);

  NEXT_GOAL_TYPE selectNextGoal(Vector3d& next_pos, double& next_yaw);
  VIEWPOINT_CHANGE_REASON planToNextGoal(
      const Vector3d& next_pos, const double& next_yaw, const vector<Vector3d>& frontire_cells, const bool check_exploration);

  bool findJunction(const vector<Vector3d>& path, Vector3d& point, double& yaw);
  void setLastErrorType(VIEWPOINT_CHANGE_REASON reason);
  weak_ptr<PAExplorationFSM> expl_fsm_;

  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FrontierFinder> frontier_finder_;
  shared_ptr<FeatureMap> feature_map_;
  shared_ptr<SteppingDebug> stepping_debug_;
  bool start_debug_mode_;
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

private:
  shared_ptr<EDTEnvironment> edt_environment_;
  shared_ptr<SDFMap> sdf_map_, global_sdf_map_;

  void shortenPath(vector<Vector3d>& path);
  void searchYaw(const Vector3d& pos, vector<double>& yaw_samples_res);

public:
  typedef shared_ptr<PAExplorationManager> Ptr;
};

}  // namespace fast_planner

#endif