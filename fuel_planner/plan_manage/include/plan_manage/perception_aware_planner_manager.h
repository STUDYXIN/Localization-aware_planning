#ifndef _PERCEPTION_AWARE_PLANNER_MANAGER_H_
#define _PERCEPTION_AWARE_PLANNER_MANAGER_H_

#include <active_perception/frontier_finder.h>
// #include <active_perception/heading_planner.h>
#include <bspline/non_uniform_bspline.h>
#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/astar2.h>
#include <path_searching/kino_astar_4degree.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/rrt_star.h>
#include <path_searching/topo_prm.h>
#include <plan_env/edt_environment.h>
#include <ros/ros.h>

#include <plan_manage/plan_container.hpp>

#include <sophus/se3.hpp>

namespace fast_planner {
// Fast Planner Manager
// Key algorithms of mapping and planning are called

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

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
  int observed_frontier_num_yaw_initial_;  // 本条轨迹可观测到的目标frontier
                                           // cell数量(yaw_initial_planner)
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

  void initPlanModules(ros::NodeHandle& nh);

  bool checkTrajCollision(double& distance);
  bool checkCurrentLocalizability(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient, int& feature_num);
  bool checkCurrentLocalizability(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient);
  bool checkTrajLocalizability(double& distance);
  bool checkTrajLocalizabilityOnKnots();

  void setFinalGoal(const Vector3d& final_goal) {
    final_goal_ = final_goal;
  }

  void setFeatureMap(shared_ptr<FeatureMap>& feature_map) {
    feature_map_ = feature_map;
  }

  void printStatistics(const vector<Vector3d>& target_frontier);

  void updateStartState(
      const Vector3d& start_pos, const Vector3d& start_vel, const Vector3d& start_acc, const Vector3d& start_yaw);

  void solvePnPByGaussNewton(
      const vector<Vector3d>& points_3d, const vector<Vector2d>& points_2d, const Matrix3d& K, Sophus::SE3d& pose, Matrix6d& H);
  vector<Vector3d> generateEndPoint();
  bool calPerceptionCost(const size_t id, double& R_perc);
  double calGoalProcessCost(const size_t id);
  double calCollisionProb(const size_t id);
  void planYaw();
  void selectBestTraj(const Vector3d& start_pos, const Vector3d& start_vel, const Vector3d& start_acc, const Vector3d& start_yaw,
      const Vector3d& final_goal);

  PlanParameters pp_;

  CameraParam::Ptr camera_param = nullptr;
  Statistics statistics_;
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  MidPlanData plan_data_;
  EDTEnvironment::Ptr edt_environment_;
  shared_ptr<SteppingDebug> stepping_debug_;
  shared_ptr<SDFMap> sdf_map_;
  void getSteppingDebug(shared_ptr<SteppingDebug> stepping_debug) {
    stepping_debug_ = stepping_debug;
  }

  vector<BsplineOptimizer::Ptr> bspline_optimizers_;

private:
  /* main planning algorithms & modules */
  shared_ptr<FeatureMap> feature_map_;

  Eigen::Vector3d start_pos_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d final_goal_;

  void updateTrajInfo();

public:
  typedef shared_ptr<FastPlannerManager> Ptr;
};
}  // namespace fast_planner

#endif