#include "plan_manage/perception_aware_planner_manager.h"
#include "plan_env/utils.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>

#include <visualization_msgs/Marker.h>

#include <thread>
#include <stepping_debug.hpp>

#define ANSI_COLOR_YELLOW_BOLD "\033[1;33m"
#define ANSI_COLOR_GREEN_BOLD "\033[1;32m"
#define ANSI_COLOR_RED_BOLD "\033[1;31m"
#define NORMAL_FONT "\033[0m"

using namespace std;
using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query

void FastPlannerManager::initPlanModules(ros::NodeHandle& nh) {
  /* read algorithm parameters */

  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_yawdot", pp_.max_yawdot_, -1.0);
  nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
  nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
  nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
  nh.param("manager/bspline_degree", pp_.bspline_degree_, 3);
  nh.param("manager/min_time", pp_.min_time_, false);
  nh.param("manager/min_observed_ratio", pp_.min_observed_ratio_, 0.6);

  bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization, use_active_perception;
  bool use_sample_path;
  nh.param("manager/use_geometric_path", use_geometric_path, false);
  nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, false);
  nh.param("manager/use_sample_path", use_sample_path, false);
  nh.param("manager/use_topo_path", use_topo_path, false);
  nh.param("manager/use_optimization", use_optimization, false);
  nh.param("manager/use_active_perception", use_active_perception, false);
  nh.param("manager/use_4degree_kinoAstar", use_4degree_kinoAstar, false);
  nh.param("manager/use_apace_pose_opt", use_apace_pose_opt_, false);
  nh.param("manager/use_fvp_opt", use_fvp_opt_, false);

  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  if (use_sample_path) {
    sample_path_finder_.reset(new RRTStar);
    sample_path_finder_->init(nh, edt_environment_);
  }

  if (use_geometric_path) {
    path_finder_.reset(new Astar);
    path_finder_->init(nh, edt_environment_);
  }

  if (use_kinodynamic_path) {
    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(nh);
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->init();
  }

  if (use_4degree_kinoAstar) {
    kino_path_4degree_finder_.reset(new KinodynamicAstar4Degree);
    kino_path_4degree_finder_->init(nh, edt_environment_);
  }

  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->getSteppingDebug(stepping_debug_);
      bspline_optimizers_[i]->setParam(nh);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }

  if (use_topo_path) {
    topo_prm_.reset(new TopologyPRM);
    topo_prm_->setEnvironment(edt_environment_);
    topo_prm_->init(nh);
  }

  yaw_initial_planner_.reset(new YawInitialPlanner(nh));
}

void FastPlannerManager::updateTrajInfo() {
  local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();

  local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_ = local_data_.position_traj_.getTimeSum();

  local_data_.traj_id_++;

  local_data_.position_traj_.getMeanAndMaxVel(statistics_.mean_vel_, statistics_.max_vel_);
  local_data_.position_traj_.getMeanAndMaxAcc(statistics_.mean_acc_, statistics_.max_acc_);
  statistics_.dt_ = local_data_.position_traj_.getKnotSpan();
}

bool FastPlannerManager::checkTrajCollision(double& distance) {
  double t_now = (ros::Time::now() - local_data_.start_time_).toSec();

  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoorT(t_now);
  double radius = 0.0;
  Eigen::Vector3d fut_pt;
  double fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoorT(t_now + fut_t);
    // double dist = edt_environment_->sdf_map_->getDistance(fut_pt);
    if (sdf_map_->getInflateOccupancy(fut_pt) == 1) {
      distance = radius;
      // std::cout << "collision at: " << fut_pt.transpose() << ", dist: " << dist << std::endl;
      std::cout << "collision at: " << fut_pt.transpose() << std::endl;
      return false;
    }
    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

// 获取当前帧的可视特征点数量，连续几帧不满足才返回false
bool FastPlannerManager::checkCurrentLocalizability(const Vector3d& pos, const Quaterniond& orient, int& feature_num) {
  if (feature_map_ == nullptr) return true;

  feature_num = feature_map_->get_NumCloud_using_Odom(pos, orient);
  static int error_times = 0;

  int min_feature_num = Utils::getGlobalParam().min_feature_num_act_;
  if (feature_num <= min_feature_num) {
    ROS_ERROR("ERROR STATE with feature: %d", feature_num);
    error_times++;
  }

  else
    error_times = 0;

  return feature_num > min_feature_num || error_times <= 10;
}

// 获取当前帧的可视特征点数量，一旦不满足直接返回false
bool FastPlannerManager::checkCurrentLocalizability(const Vector3d& pos, const Quaterniond& orient) {
  if (feature_map_ == nullptr) return true;
  int feature_num = feature_map_->get_NumCloud_using_Odom(pos, orient);
  int min_feature_num = Utils::getGlobalParam().min_feature_num_act_;
  return feature_num > min_feature_num;
}

bool FastPlannerManager::checkTrajLocalizability(double& distance) {
  double t_now = (ros::Time::now() - local_data_.start_time_).toSec();

  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoorT(t_now);
  double radius = 0.0;
  Eigen::Vector3d fut_pt;
  double fut_yaw;
  Eigen::Vector3d fut_acc;
  double fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoorT(t_now + fut_t);
    fut_yaw = local_data_.yaw_traj_.evaluateDeBoorT(t_now + fut_t)[0];
    fut_acc = local_data_.acceleration_traj_.evaluateDeBoorT(t_now + fut_t);

    Quaterniond fut_orient = Utils::calcOrientation(fut_yaw, fut_acc);

    int feature_num;
    if (!checkCurrentLocalizability(fut_pt, fut_orient, feature_num)) {
      distance = radius;
      // std::cout << "collision at: " << fut_pt.transpose() << ", dist: " << dist << std::endl;
      std::cout << "poor localizability at: " << fut_pt.transpose() << std::endl;
      return false;
    }
    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

bool FastPlannerManager::checkTrajLocalizabilityOnKnots() {
  vector<Vector3d> knots_pos;
  vector<Vector3d> knots_acc;
  vector<Vector3d> knots_yaw;

  local_data_.position_traj_.getKnotPoint(knots_pos);
  local_data_.acceleration_traj_.getKnotPoint(knots_acc);
  local_data_.yaw_traj_.getKnotPoint(knots_yaw);

  ROS_ASSERT(knots_pos.size() == knots_yaw.size());

  for (size_t i = 0; i < knots_pos.size(); i++) {
    Quaterniond orient = Utils::calcOrientation(knots_yaw[i][0], knots_acc[i]);
    if (!checkCurrentLocalizability(knots_pos[i], orient)) {
      ROS_WARN("[FastPlannerManager::checkTrajLocalizabilityOnKnots] Poor localizability");
      return false;
    }
  }

  return true;
}

bool FastPlannerManager::checkTrajExplorationOnKnots(const vector<Vector3d>& target_frontier) {
  vector<Vector3d> knots_pos;
  vector<Vector3d> knots_acc;
  vector<Vector3d> knots_yaw;

  local_data_.position_traj_.getKnotPoint(knots_pos);
  local_data_.acceleration_traj_.getKnotPoint(knots_acc);
  local_data_.yaw_traj_.getKnotPoint(knots_yaw);

  ROS_ASSERT(knots_pos.size() == knots_yaw.size());

  set<int> observed_features;

  for (size_t i = 0; i < knots_pos.size(); i++) {
    set<int> observed_features_knots;
    sdf_map_->countVisibleCells(knots_pos[i], knots_yaw[i][0], target_frontier, observed_features_knots);
    observed_features.insert(observed_features_knots.begin(), observed_features_knots.end());
  }

  statistics_.observed_frontier_num_ = observed_features.size();

  double ratio = static_cast<double>(observed_features.size()) / target_frontier.size();

  if (ratio < pp_.min_observed_ratio_) {
    // ROS_WARN("[FastPlannerManager::checkTrajExplorationOnKnots] Poor exploration");
    return false;
  }

  return true;
}

void FastPlannerManager::printStatistics(const vector<Vector3d>& target_frontier) {
  cout << ANSI_COLOR_GREEN_BOLD;
  cout << "====================Local Planner Statistics====================" << endl;
  cout << fixed << setprecision(3);
  cout << "Time of Kinodynamic A*:      " << statistics_.time_kinodynamic_astar_ << " (sec)" << endl;
  cout << "Time of Pos Traj Optimize:   " << statistics_.time_pos_traj_opt_ << " (sec)" << endl;
  cout << "Time of Yaw Initial Planner: " << statistics_.time_yaw_initial_planner_ << " (sec)" << endl;
  cout << "Time of Yaw Traj Optimize:   " << statistics_.time_yaw_traj_opt_ << " (sec)" << endl;
  statistics_.time_total_ = statistics_.time_kinodynamic_astar_ + statistics_.time_pos_traj_opt_ +
                            statistics_.time_yaw_initial_planner_ + statistics_.time_yaw_traj_opt_;
  cout << "Time of Total Planning:      " << statistics_.time_total_ << " (sec)" << endl;

  cout << fixed << setprecision(3);
  double max_vel = Utils::getGlobalParam().max_vel_;
  double max_acc = Utils::getGlobalParam().max_acc_;
  cout << "Mean Vel on Pos Traj:               " << statistics_.mean_vel_ << " (m/s)" << endl;

  if (statistics_.max_vel_ < max_vel * 1.5)
    cout << ANSI_COLOR_GREEN_BOLD;
  else
    cout << ANSI_COLOR_RED_BOLD;
  cout << "Max Vel on Pos Traj:                " << statistics_.max_vel_ << " (m/s)" << endl << ANSI_COLOR_GREEN_BOLD;
  cout << "Mean Acc on Pos Traj:               " << statistics_.mean_acc_ << " (m^2/s)" << endl;

  if (statistics_.max_acc_ < max_acc * 1.5)
    cout << ANSI_COLOR_GREEN_BOLD;
  else
    cout << ANSI_COLOR_RED_BOLD;
  cout << "Max Acc on Pos Traj:                " << statistics_.max_acc_ << " (m^2/s)" << endl << ANSI_COLOR_GREEN_BOLD;

  double max_yaw_rate = Utils::getGlobalParam().max_yaw_rate_;
  if (statistics_.max_yaw_rate_ < max_yaw_rate)
    cout << ANSI_COLOR_GREEN_BOLD;
  else
    cout << ANSI_COLOR_RED_BOLD;
  cout << "Max Yaw Rate on Yaw Traj:           " << statistics_.max_yaw_rate_ << " (rad/s)" << endl << ANSI_COLOR_GREEN_BOLD;

  cout << "Knot Span:                          " << statistics_.dt_ << " (s)" << endl;

  if (!target_frontier.empty()) {
    cout << "Observed Frontiers Num(Yaw Intial): " << statistics_.observed_frontier_num_yaw_initial_ << "/"
         << target_frontier.size() << endl;
    cout << "Observed Frontiers Num:             " << statistics_.observed_frontier_num_ << "/" << target_frontier.size() << endl;
  }
  cout.unsetf(ios::fixed);
  cout << "===============================================================" << endl;
  cout << NORMAL_FONT;
}
// !SECTION

// SECTION perception aware replanning

int FastPlannerManager::planPosPerceptionAware(const Vector3d& start_pt, const Vector3d& start_vel, const Vector3d& start_acc,
    const double start_yaw, const Vector3d& end_pt, const Vector3d& end_vel, const double end_yaw,
    const vector<Vector3d>& frontier_cells, const double& time_lb) {
  // std::cout << "[Kino replan]: start pos: " << start_pt.transpose() << endl;
  // std::cout << "[Kino replan]: start vel: " << start_vel.transpose() << endl;
  // std::cout << "[Kino replan]: start acc: " << start_acc.transpose() << endl;
  // std::cout << "[Kino replan]: start yaw: " << start_yaw << endl;
  // std::cout << "[Kino replan]: end pos: " << end_pt.transpose() << endl;
  // std::cout << "[Kino replan]: end vel: " << end_vel.transpose() << endl;
  // std::cout << "[Kino replan]: end yaw: " << end_yaw << endl;

  // frontier_finder_->setLatestViewpoint(end_pt, end_yaw, 0);
  if ((start_pt - end_pt).norm() < 1e-2) {
    cout << "Close goal" << endl;
    return PATH_SEARCH_ERROR;
  }

  // Kinodynamic path searching

  // Step1: 调用混合A*得到初始waypoints
  auto time_start = ros::Time::now();

  int status;
  if (use_4degree_kinoAstar) {
    kino_path_4degree_finder_->setFeatureMap(feature_map_);
    kino_path_4degree_finder_->reset();
    status = kino_path_4degree_finder_->search(start_pt, start_vel, start_acc, start_yaw, end_pt, end_vel, end_yaw);
    if (status == KinodynamicAstar::NO_PATH) {
      ROS_ERROR("Kinodynamic A* search fail");
      return PATH_SEARCH_ERROR;
    }
    plan_data_.kino_path_ = kino_path_4degree_finder_->getKinoTraj(0.01);
  }

  else {
    kino_path_finder_->reset();
    cout << "-start_pt  " << start_pt.transpose() << endl
         << "-start_vel " << start_vel.transpose() << endl
         << "-start_acc " << start_acc.transpose() << endl
         << "-end_pt    " << end_pt.transpose() << endl
         << "-end_vel   " << end_vel.transpose() << endl
         << "-distance  " << (end_pt - start_pt).norm() << endl;
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);
    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[Kino replan]: search 1 fail." << endl;
      // Retry
      kino_path_finder_->reset();
      status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);
      if (status == KinodynamicAstar::NO_PATH) {
        ROS_ERROR("[Kino replan]: Can't find path.");
        return PATH_SEARCH_ERROR;
      }
    }
    plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);
  }
  stepping_debug_->debug_type_now_ = DEBUG_TYPE::BEFORE_POS_OPT;
  stepping_debug_->calldebug(DEBUG_TYPE::BEFORE_POS_OPT, plan_data_.kino_path_);
  statistics_.time_kinodynamic_astar_ = (ros::Time::now() - time_start).toSec();

  // Step2: 基于B样条曲线的轨迹优化，首先生成一条均匀B样条曲线
  auto time_start_2 = ros::Time::now();

  double dt = pp_.ctrl_pt_dist / pp_.max_vel_;
  vector<Vector3d> point_set, start_end_derivatives;
  if (use_4degree_kinoAstar)
    kino_path_4degree_finder_->getSamples(dt, point_set, start_end_derivatives);
  else
    kino_path_finder_->getSamples(dt, point_set, start_end_derivatives);

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivatives, pp_.bspline_degree_, ctrl_pts);
  NonUniformBspline init_traj(ctrl_pts, pp_.bspline_degree_, dt);

  // Step3: 在上面得到的均匀B样条曲线基础上进行B样条曲线优化，得到最终的轨迹
  vector<Vector3d> start, end;
  vector<bool> start_idx, end_idx;

  if (status == KinodynamicAstar::REACH_END) {
    init_traj.getBoundaryStates(2, 2, start, end);
    start_idx = { true, true, true };
    end_idx = { true, true, true };
  }

  else {
    init_traj.getBoundaryStates(2, 0, start, end);
    start_idx = { true, true, true };
    end_idx = { true, false, false };
  }

  bspline_optimizers_[0]->setBoundaryStates(start, end, start_idx, end_idx);
  if (time_lb > 0) bspline_optimizers_[0]->setTimeLowerBound(time_lb);

  // 这里使用了平滑约束、动力学可行性约束、起点约束、终点约束、避障约束、视差约束、垂直可见性
  // **增加了未知区域可见性约束FRONTIERVISIBILITY
  int cost_func = 0;
  cost_func |= BsplineOptimizer::SMOOTHNESS;
  cost_func |= BsplineOptimizer::FEASIBILITY;
  cost_func |= BsplineOptimizer::START;
  cost_func |= BsplineOptimizer::END;
  cost_func |= BsplineOptimizer::MINTIME;
  cost_func |= BsplineOptimizer::DISTANCE;

  if (use_apace_pose_opt_) {
    cost_func |= BsplineOptimizer::PARALLAX;
    cost_func |= BsplineOptimizer::VERTICALVISIBILITY;
    bspline_optimizers_[0]->setFeatureMap(feature_map_);
  }
  if (use_fvp_opt_) {
    cost_func |= BsplineOptimizer::FRONTIERVISIBILITY_POS;
    bspline_optimizers_[0]->setViewpoint(end_pt, end_yaw);
    bspline_optimizers_[0]->setFrontierCells(frontier_cells);
  }

  // Set params
  if (cost_func & BsplineOptimizer::PARALLAX || cost_func & BsplineOptimizer::VERTICALVISIBILITY ||
      cost_func & BsplineOptimizer::FRONTIERVISIBILITY_POS) {
    bspline_optimizers_[0]->setFeatureMap(feature_map_);
    if (cost_func & BsplineOptimizer::FRONTIERVISIBILITY_POS) {
      // bspline_optimizers_[0]->setFrontierFinder(frontier_finder_);
      // bspline_optimizers_[0]->setFrontiercenter(frontier_center);
      bspline_optimizers_[0]->setViewpoint(end_pt, end_yaw);
      bspline_optimizers_[0]->setFrontierCells(frontier_cells);
      bspline_optimizers_[0]->setFrontierNormals();
    }
  }
  stepping_debug_->debug_type_now_ = DEBUG_TYPE::EVERY_POS_OPT;
  bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 1, 1);
  if (!bspline_optimizers_[0]->issuccess) return POSISION_OPT_ERROR;
  local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  statistics_.time_pos_traj_opt_ = (ros::Time::now() - time_start_2).toSec();
  updateTrajInfo();

  return SUCCESS_FIND_POSISION_TRAJ;
}

// !SECTION

int FastPlannerManager::planYawPerceptionAware(const Vector3d& start_yaw, const vector<double>& end_yaw_vec, double& end_yaw,
    const vector<Vector3d>& frontier_cells, const Vector3d& final_goal) {

  auto time_start = ros::Time::now();

  // Yaw b-spline has same segment number as position b-spline
  Eigen::MatrixXd position_ctrl_pts = local_data_.position_traj_.getControlPoint();
  int ctrl_pts_num = position_ctrl_pts.rows();
  double dt_yaw = local_data_.position_traj_.getKnotSpan();

  // Yaw traj control points
  Eigen::MatrixXd yaw(ctrl_pts_num, 1);
  yaw.setZero();

  // Step1: 设置起始状态
  Vector3d start_yaw3d = start_yaw;
  Utils::roundPi(start_yaw3d[0]);

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0, dt_yaw,
      (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block<3, 1>(0, 0) = states2pts * start_yaw3d;

  // Step2: 设置中间的waypoints约束
  // Calculate knot pos and acc
  // [u[p],u[m-p]] -> [0*dt, (m-2p)*dt] -> [0*dt, (n-2)*dt]
  vector<Vector3d> knot_pos, knot_acc;
  local_data_.position_traj_.getKnotPoint(knot_pos);
  local_data_.acceleration_traj_.getKnotPoint(knot_acc);

  vector<double> yaw_waypoints;
  yaw_initial_planner_->setFeatureMap(feature_map_);
  // yaw_initial_planner_->setFrontierFinder(frontier_finder_);
  yaw_initial_planner_->setSDFmap(sdf_map_);
  yaw_initial_planner_->setFinalGoal(final_goal);
  yaw_initial_planner_->setPos(knot_pos);
  yaw_initial_planner_->setAcc(knot_acc);
  // yaw_initial_planner预处理frontier过程需要用到pos和acc信息，所以必须最后设置frontier
  yaw_initial_planner_->setTargetFrontier(frontier_cells);

  if (!yaw_initial_planner_->search(start_yaw[0], end_yaw_vec, dt_yaw, yaw_waypoints)) {
    ROS_ERROR("Yaw Trajectory Planning Failed in Graph Search!!!");
    return YAW_INIT_ERROR;
  }

  statistics_.observed_frontier_num_yaw_initial_ = yaw_initial_planner_->getObservedNum();

  statistics_.time_yaw_initial_planner_ = (ros::Time::now() - time_start).toSec();

  // 后面优化选项选择了WAYPOINTS，所以这里需要把waypoints设置好
  vector<Vector3d> waypts;
  vector<int> waypt_idx;
  double last_yaw = yaw_waypoints[0];
  for (size_t i = 0; i < yaw_waypoints.size(); ++i) {
    Vector3d waypt = Vector3d::Zero();
    waypt(0) = yaw_waypoints[i];
    Utils::calcNextYaw(last_yaw, waypt(0));
    last_yaw = waypt(0);
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // 设置终止状态
  end_yaw = yaw_waypoints.back();
  Eigen::Vector3d end_yaw3d(end_yaw, 0, 0);
  Utils::calcNextYaw(last_yaw, end_yaw3d(0));
  yaw.block<3, 1>(yaw.rows() - 3, 0) = states2pts * end_yaw3d;

  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  vector<Vector3d> start = { Vector3d(start_yaw3d[0], 0, 0), Vector3d(start_yaw3d[1], 0, 0), Vector3d(start_yaw3d[2], 0, 0) };
  vector<Vector3d> end = { Vector3d(end_yaw3d[0], 0, 0), zero, zero };
  vector<bool> start_idx = { true, true, true };
  vector<bool> end_idx = { true, false, false };

  statistics_.time_yaw_initial_planner_ = (ros::Time::now() - time_start).toSec();

  // Call B-spline optimization solver
  auto time_start_2 = ros::Time::now();

  // Add noise to yaw if variance is too small
  double yaw_variance = Utils::calcMeanAndVariance(yaw).second;
  if (yaw_variance < 1e-4) {
    cout << "add gaussian noise to yaw control points" << endl;

    double sigma = 0.1;  // adjust the noise level as needed

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> distribution(0.0, sigma);  // 均值为 0，标准差为 0.1

    for (int i = 0; i < yaw.rows(); ++i) {
      double noise = distribution(gen);  // 生成一个符合正态分布的随机数
      yaw(i, 0) += noise;
    }
    // cout << "yaw control point before optimize(add noise): " << yaw << endl;
  }

  // Step2: 调用B样条曲线优化器优化yaw轨迹
  // 这里使用了平滑约束、路径点约束、起点约束、终点约束、主要就是添加了yaw共视约束
  int cost_func = 0;
  cost_func |= BsplineOptimizer::SMOOTHNESS;
  cost_func |= BsplineOptimizer::FEASIBILITY_YAW;
  cost_func |= BsplineOptimizer::WAYPOINTS;
  cost_func |= BsplineOptimizer::START;
  cost_func |= BsplineOptimizer::END;
  cost_func |= BsplineOptimizer::YAWCOVISIBILITY;
  cost_func |= BsplineOptimizer::FRONTIERVISIBILITY_YAW;

  if (cost_func & BsplineOptimizer::YAWCOVISIBILITY || cost_func & BsplineOptimizer::FRONTIERVISIBILITY_YAW) {
    bspline_optimizers_[1]->setPosAndAcc(knot_pos, knot_acc);

    YawOptData::Ptr opt_data = make_shared<YawOptData>();
    yaw_initial_planner_->prepareOptData(opt_data);
    bspline_optimizers_[1]->setOptData(opt_data);

    if (cost_func & BsplineOptimizer::FRONTIERVISIBILITY_YAW) {
      bspline_optimizers_[1]->setFrontierCells(frontier_cells);
    }
  }

  stepping_debug_->getPosBspline(local_data_.position_traj_);
  stepping_debug_->debug_type_now_ = DEBUG_TYPE::YAW_INIT;
  stepping_debug_->calldebug(DEBUG_TYPE::YAW_INIT, waypts);
  stepping_debug_->debug_type_now_ = DEBUG_TYPE::EVERY_YAW_OPT;

  bspline_optimizers_[1]->setBoundaryStates(start, end, start_idx, end_idx);
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  bspline_optimizers_[1]->setFeatureMap(feature_map_);

  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 2, 2);
  if (!bspline_optimizers_[1]->issuccess) return YAW_OPT_ERROR;

  statistics_.time_yaw_traj_opt_ = (ros::Time::now() - time_start_2).toSec();

  // Update traj info
  // Step3: 更新yaw及其导数的轨迹
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  local_data_.yaw_traj_.getMeanAndMaxVel(statistics_.mean_yaw_rate_, statistics_.max_yaw_rate_);

  return SUCCESS_FIND_YAW_TRAJ;
}

}  // namespace fast_planner
