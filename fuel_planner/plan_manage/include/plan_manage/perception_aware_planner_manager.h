#ifndef _PERCEPTION_AWARE_PLANNER_MANAGER_H_
#define _PERCEPTION_AWARE_PLANNER_MANAGER_H_

#include <active_perception/frontier_finder.h>
// #include <active_perception/heading_planner.h>
#include <bspline/non_uniform_bspline.h>
#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/astar2.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/kino_astar_4degree.h>
#include <path_searching/rrt_star.h>
#include <path_searching/topo_prm.h>
#include <plan_env/edt_environment.h>
#include <ros/ros.h>

#include <plan_manage/plan_container.hpp>
#include <plan_manage/yaw_initial_planner.h>

namespace fast_planner {
// Fast Planner Manager
// Key algorithms of mapping and planning are called
#define SUCCESS_FIND_POSISION_TRAJ 0
#define PATH_SEARCH_ERROR 1
#define POSISION_OPT_ERROR 2
#define SUCCESS_FIND_YAW_TRAJ 3
#define YAW_INIT_ERROR 4
#define YAW_OPT_ERROR 5
struct Statistics {
  double time_kinodynamic_astar_;          // 位置轨迹规划混合A*耗时(s)
  double time_pos_traj_opt_;               // 位置轨迹规划B样条优化耗时(s)
  double time_yaw_initial_planner_;        // yaw轨迹规划搜索初值耗时(s)
  double time_yaw_traj_opt_;               // yaw轨迹规划B样条优化耗时(s)
  double time_total_;                      // 规划总耗时(s)
  double mean_vel_;                        // 位置轨迹平均速度(m/s)
  double max_vel_;                         // 位置轨迹最大速度(m/s)
  double mean_acc_;                        // 位置轨迹平均加速度(m/s)
  double max_acc_;                         // 位置轨迹最大加速度(m^2/s)
  double mean_yaw_rate_;                   // yaw轨迹平均角速度(rad/s)
  double max_yaw_rate_;                    // yaw轨迹最大角速度(rad/s)
  int observed_frontier_num_yaw_initial_;  // 本条轨迹可观测到的目标frontier cell数量(yaw_initial_planner)
  int observed_frontier_num_;              // 本条轨迹可观测到的目标frontier cell数量
  double dt_;                              // 均匀B样条轨迹时间间隔(s)
  double last_success_time_;               // 上一次成功局部规划的时间
  int local_plan_time_ = 0;                // 局部规划总次数，每次成功后清零
};
class SteppingDebug;
class FastPlannerManager {
  // SECTION stable
public:
  /* main planning interface */
  bool planPosPerceptionAware(const Vector3d& start_pt, const Vector3d& start_vel, const Vector3d& start_acc,
      const double start_yaw, const Vector3d& end_pt, const Vector3d& end_vel);
  int planPosPerceptionAware(const Vector3d& start_pt, const Vector3d& start_vel, const Vector3d& start_acc,
      const double start_yaw, const Vector3d& end_pt, const Vector3d& end_vel, const double end_yaw,
      const vector<Vector3d>& frontier_cells, const double& time_lb);

  bool sampleBasedReplan(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_vel, const Eigen::Vector3d& start_acc,
      const double start_yaw, const Eigen::Vector3d& end_pt, const double end_yaw, const double& time_lb = -1);

  bool kinodynamicReplan(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_vel, const Eigen::Vector3d& start_acc,
      const double start_yaw, const Eigen::Vector3d& end_pt, const Eigen::Vector3d& end_vel, const double end_yaw,
      const double& time_lb = -1);

  void planExploreTraj(const vector<Eigen::Vector3d>& tour, const Eigen::Vector3d& cur_vel, const Eigen::Vector3d& cur_acc,
      const double& time_lb = -1);

  void planYawExplore(const Eigen::Vector3d& start_yaw, const double& end_yaw, bool lookfwd, const double& relax_time);
  int planYawPerceptionAware(
      const Vector3d& start_yaw, const double& end_yaw, const vector<Vector3d>& frontier_cells, const Vector3d& final_goal);

  void initPlanModules(ros::NodeHandle& nh);
  void setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints);

  bool checkTrajCollision(double& distance);
  bool checkCurrentLocalizability(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient, int& feature_num);
  bool checkCurrentLocalizability(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient);
  bool checkTrajLocalizability(double& distance);
  bool checkTrajLocalizabilityOnKnots();
  bool checkTrajExplorationOnKnots(const vector<Vector3d>& target_frontier);

  void setFeatureMap(shared_ptr<FeatureMap>& feature_map) {
    feature_map_ = feature_map;
  }

  void setFrontierFinder(shared_ptr<FrontierFinder> frontier_finder) {
    frontier_finder_ = frontier_finder;
  }

  void printStatistics(const vector<Vector3d>& target_frontier);

  PlanParameters pp_;

  Statistics statistics_;
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  MidPlanData plan_data_;
  EDTEnvironment::Ptr edt_environment_;
  shared_ptr<Astar> path_finder_;
  unique_ptr<TopologyPRM> topo_prm_;
  shared_ptr<FrontierFinder> frontier_finder_;
  shared_ptr<SteppingDebug> stepping_debug_;
  shared_ptr<SDFMap> sdf_map_;
  void getSteppingDebug(shared_ptr<SteppingDebug> stepping_debug) {
    stepping_debug_ = stepping_debug;
  }

  int last_error_type;
  bool is_pos_search_reach_end;
  vector<BsplineOptimizer::Ptr> bspline_optimizers_;

private:
  /* main planning algorithms & modules */
  shared_ptr<FeatureMap> feature_map_;

  unique_ptr<KinodynamicAstar> kino_path_finder_;
  unique_ptr<KinodynamicAstar4Degree> kino_path_4degree_finder_;
  unique_ptr<RRTStar> sample_path_finder_;

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
  // unique_ptr<VisibilityUtil> visib_util_;
  bool use_4degree_kinoAstar, use_apace_pose_opt_, use_fvp_opt_;
  // !SECTION
};
}  // namespace fast_planner

#endif