#ifndef _PERCEPTION_AWARE_PLANNER_MANAGER_H_
#define _PERCEPTION_AWARE_PLANNER_MANAGER_H_

#include <active_perception/frontier_finder.h>
// #include <active_perception/heading_planner.h>
#include <bspline/non_uniform_bspline.h>
#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/astar2.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/rrt_star.h>
#include <path_searching/topo_prm.h>
#include <plan_env/edt_environment.h>
#include <ros/ros.h>

#include <plan_manage/plan_container.hpp>
#include <plan_manage/yaw_initial_planner.h>

namespace fast_planner {
// Fast Planner Manager
// Key algorithms of mapping and planning are called

class FastPlannerManager {
  // SECTION stable
public:
  /* main planning interface */
  bool planPosPerceptionAware(const Vector3d& start_pt, const Vector3d& start_vel, const Vector3d& start_acc,
      const double start_yaw, const Vector3d& end_pt, const Vector3d& end_vel, const double end_yaw, const double& time_lb);
  bool sampleBasedReplan(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_vel, const Eigen::Vector3d& start_acc,
      const double start_yaw, const Eigen::Vector3d& end_pt, const double end_yaw, const double& time_lb = -1);
  bool kinodynamicReplan(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_vel, const Eigen::Vector3d& start_acc,
      const double start_yaw, const Eigen::Vector3d& end_pt, const Eigen::Vector3d& end_vel, const double end_yaw,
      const double& time_lb = -1);
  void planExploreTraj(const vector<Eigen::Vector3d>& tour, const Eigen::Vector3d& cur_vel, const Eigen::Vector3d& cur_acc,
      const double& time_lb = -1);

  void planYaw(const Eigen::Vector3d& start_yaw);
  void planYawExplore(const Eigen::Vector3d& start_yaw, const double& end_yaw, bool lookfwd, const double& relax_time);
  void planYawPerceptionAware(const Eigen::Vector3d& start_yaw, const double& end_yaw, const vector<Vector3d>& frontier_cells);

  void initPlanModules(ros::NodeHandle& nh);
  void setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints);

  bool checkTrajCollision(double& distance);
  bool checkCurrentLocalizability(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient, int& feature_num);
  bool checkTrajLocalizability(double& distance);
  void calcNextYaw(const double& last_yaw, double& yaw);

  void setFeatureMap(shared_ptr<FeatureMap>& feature_map) {
    feature_map_ = feature_map;
  }

  void setFrontierFinder(shared_ptr<FrontierFinder> frontier_finder) {
    frontier_finder_ = frontier_finder;
  }

  PlanParameters pp_;

  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  MidPlanData plan_data_;
  EDTEnvironment::Ptr edt_environment_;
  shared_ptr<Astar> path_finder_;
  unique_ptr<TopologyPRM> topo_prm_;
  shared_ptr<FrontierFinder> frontier_finder_;

private:
  /* main planning algorithms & modules */
  shared_ptr<SDFMap> sdf_map_;
  shared_ptr<FeatureMap> feature_map_;

  unique_ptr<KinodynamicAstar> kino_path_finder_;
  unique_ptr<RRTStar> sample_path_finder_;
  vector<BsplineOptimizer::Ptr> bspline_optimizers_;

  void updateTrajInfo();

  // Heading planning

  unique_ptr<YawInitialPlanner> yaw_initial_planner_;

  // !SECTION stable

  // SECTION developing

public:
  typedef shared_ptr<FastPlannerManager> Ptr;

  void planYawActMap(const Eigen::Vector3d& start_yaw);
  void test();
  void searchFrontier(const Eigen::Vector3d& p);

private:
  // unique_ptr<HeadingPlanner> heading_planner_;
  unique_ptr<VisibilityUtil> visib_util_;

  // Benchmark method, local exploration
public:
  bool localExplore(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc, Eigen::Vector3d end_pt);

  // !SECTION
};
}  // namespace fast_planner

#endif