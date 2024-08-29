#ifndef _PA_EXPLORATION_MANAGER_H_
#define _PA_EXPLORATION_MANAGER_H_

#include <exploration_manager/perception_aware_exploration_fsm.h>
#include <ros/ros.h>

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

enum NEXT_GOAL_TYPE { REACH_END, SEARCH_FRONTIER, NO_FRONTIER, NO_AVAILABLE_FRONTIER };

class PAExplorationManager {
public:
  PAExplorationManager(shared_ptr<PAExplorationFSM> expl_fsm);

  void initialize(ros::NodeHandle& nh);

  NEXT_GOAL_TYPE selectNextGoal(Vector3d& next_pos, double& next_yaw);
  bool planToNextGoal(const Vector3d& next_pos, const double& next_yaw);

  weak_ptr<PAExplorationFSM> expl_fsm_;

  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FrontierFinder> frontier_finder_;

private:
  shared_ptr<EDTEnvironment> edt_environment_;
  shared_ptr<SDFMap> sdf_map_, global_sdf_map_;
  shared_ptr<FeatureMap> feature_map_;

  // Find optimal tour for coarse viewpoints of all frontiers
  void findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw, vector<int>& indices);

  // Refine local tour for next few frontiers, using more diverse viewpoints
  void refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
      const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws, vector<Vector3d>& refined_pts,
      vector<double>& refined_yaws);

  void shortenPath(vector<Vector3d>& path);

public:
  typedef shared_ptr<PAExplorationManager> Ptr;
};

}  // namespace fast_planner

#endif