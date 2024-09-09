#include "plan_manage/perception_aware_planner_manager.h"
#include "plan_manage/utils.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>

#include <visualization_msgs/Marker.h>

#include <thread>

using namespace std;
using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query

void FastPlannerManager::initPlanModules(ros::NodeHandle& nh) {
  /* read algorithm parameters */

  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  nh.param("manager/accept_vel", pp_.accept_vel_, pp_.max_vel_ + 0.5);
  nh.param("manager/accept_acc", pp_.accept_acc_, pp_.max_acc_ + 0.5);
  nh.param("manager/max_yawdot", pp_.max_yawdot_, -1.0);
  nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
  nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
  nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
  nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
  nh.param("manager/bspline_degree", pp_.bspline_degree_, 3);
  nh.param("manager/min_time", pp_.min_time_, false);

  nh.param("manager/min_feature_num", pp_.min_feature_num_, -1);

  bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization, use_active_perception;
  bool use_sample_path;
  nh.param("manager/use_geometric_path", use_geometric_path, false);
  nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, false);
  nh.param("manager/use_sample_path", use_sample_path, false);
  nh.param("manager/use_topo_path", use_topo_path, false);
  nh.param("manager/use_optimization", use_optimization, false);
  nh.param("manager/use_active_perception", use_active_perception, false);

  local_data_.traj_id_ = 0;
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
    kino_path_finder_->init(nh, edt_environment_);
  }

  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->setParam(nh);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }

  if (use_topo_path) {
    topo_prm_.reset(new TopologyPRM);
    topo_prm_->setEnvironment(edt_environment_);
    topo_prm_->init(nh);
  }

  if (use_active_perception) {
    visib_util_.reset(new VisibilityUtil(nh));
    visib_util_->setEDTEnvironment(edt_environment_);
    plan_data_.view_cons_.idx_ = -1;
  }

  yaw_initial_planner_.reset(new YawInitialPlanner(nh));
}

void FastPlannerManager::updateTrajInfo() {
  local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();

  local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_ = local_data_.position_traj_.getTimeSum();

  local_data_.traj_id_++;
}

void FastPlannerManager::setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints) {
  plan_data_.global_waypoints_ = waypoints;
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

bool FastPlannerManager::checkCurrentLocalizability(const Vector3d& pos, const Quaterniond& orient, int& feature_num) {
  if (feature_map_ == nullptr) return true;

  feature_num = feature_map_->get_NumCloud_using_Odom(pos, orient);

  return feature_num > pp_.min_feature_num_;
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

// !SECTION

// SECTION perception aware replanning

bool FastPlannerManager::planPosPerceptionAware(const Vector3d& start_pt, const Vector3d& start_vel, const Vector3d& start_acc,
    const double start_yaw, const Vector3d& end_pt, const Vector3d& end_vel, const double end_yaw, const double& time_lb) {
  std::cout << "[Kino replan]: start pos: " << start_pt.transpose() << endl;
  std::cout << "[Kino replan]: start vel: " << start_vel.transpose() << endl;
  std::cout << "[Kino replan]: start acc: " << start_acc.transpose() << endl;
  std::cout << "[Kino replan]: start yaw: " << start_yaw << endl;
  std::cout << "[Kino replan]: end pos: " << end_pt.transpose() << endl;
  std::cout << "[Kino replan]: end vel: " << end_vel.transpose() << endl;
  std::cout << "[Kino replan]: end yaw: " << end_yaw << endl;

  frontier_finder_->setLatestViewpoint(end_pt, end_yaw, 0);
  if ((start_pt - end_pt).norm() < 1e-2) {
    cout << "Close goal" << endl;
    return false;
  }

  // Kinodynamic path searching

  // Step1: 调用混合A*得到初始waypoints
  auto time_start = ros::Time::now();

  kino_path_finder_->setFeatureMap(feature_map_);
  kino_path_finder_->reset();

  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, start_yaw, end_pt, end_vel, end_yaw);
  if (status == KinodynamicAstar::NO_PATH) {
    ROS_ERROR("Kinodynamic A* search fail");
    return false;
  }
  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

  double time_search = (ros::Time::now() - time_start).toSec();
  ROS_WARN("Time cost of kinodynamic A*: %lf(sec)", time_search);

  // Step2: 基于B样条曲线的轨迹优化，首先生成一条均匀B样条曲线
  auto time_start_2 = ros::Time::now();

  double dt = pp_.ctrl_pt_dist / pp_.max_vel_;
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
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

  // cout << "pos control point before optimize: " << endl << ctrl_pts << endl;

  // 这里使用了平滑约束、动力学可行性约束、起点约束、终点约束、避障约束、视差约束、垂直可见性
  // **增加了未知区域可见性约束FRONTIERVISIBILITY
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::FEASIBILITY | BsplineOptimizer::START | BsplineOptimizer::END |
                  BsplineOptimizer::MINTIME | BsplineOptimizer::DISTANCE | BsplineOptimizer::PARALLAX |
                  BsplineOptimizer::VERTICALVISIBILITY | BsplineOptimizer::FRONTIERVISIBILITY_POS;

  // int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::FEASIBILITY | BsplineOptimizer::START |
  // BsplineOptimizer::END |
  //                 BsplineOptimizer::MINTIME | BsplineOptimizer::DISTANCE | BsplineOptimizer::PARALLAX |
  //                 BsplineOptimizer::VERTICALVISIBILITY;

  // int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::FEASIBILITY | BsplineOptimizer::START |
  // BsplineOptimizer::END |
  //                 BsplineOptimizer::MINTIME | BsplineOptimizer::DISTANCE;
  bspline_optimizers_[0]->setFeatureMap(feature_map_);
  bspline_optimizers_[0]->setFrontierFinder(frontier_finder_);
  bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 1, 1);
  local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  // cout << "pos control point after optimize: " << endl << ctrl_pts << endl;

  double time_opt = (ros::Time::now() - time_start_2).toSec();
  ROS_WARN("Time cost of optimize: %lf(sec)", time_opt);

  updateTrajInfo();

  return true;
}

// !SECTION

// SECTION sample-based replanning

bool FastPlannerManager::sampleBasedReplan(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_vel,
    const Eigen::Vector3d& start_acc, const double start_yaw, const Eigen::Vector3d& end_pt, const double end_yaw,
    const double& time_lb) {

  if ((start_pt - end_pt).norm() < 1e-2) {
    cout << "Close goal" << endl;
    return false;
  }

  // Kinodynamic path searching

  auto time_start = ros::Time::now();

  sample_path_finder_->setFeatureMap(feature_map_);
  if (!sample_path_finder_->makeProblem(start_pt, start_yaw, end_pt, end_yaw)) {
    return false;
  }

  auto status = sample_path_finder_->makePlan();
  if (!status) {
    ROS_ERROR("Sample based search fail!!!");
    return false;
  }

  double time_search = (ros::Time::now() - time_start).toSec();
  ROS_WARN("Time cost of waypoint search: %lf(sec)", time_search);

  const auto tour = sample_path_finder_->getWaypoints();

  // Generate traj through waypoints-based method
  const int pt_num = tour.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) {
    pos.row(i) = tour[i];
  }

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd times(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i) {
    times(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);
  }

  PolynomialTraj init_traj;
  PolynomialTraj::waypointsTraj(pos, start_vel, zero, start_acc, zero, times, init_traj);

  // B-spline-based optimization
  vector<Vector3d> points, boundary_deri;
  double duration = init_traj.getTotalTime();
  int seg_num = init_traj.getLength() / pp_.ctrl_pt_dist;
  seg_num = max(8, seg_num);
  double dt = duration / seg_num;

  std::cout << "duration: " << duration << ", seg_num: " << seg_num << ", dt: " << dt << std::endl;

  for (double ts = 0.0; ts <= duration + 1e-4; ts += dt) points.push_back(init_traj.evaluate(ts, 0));

  boundary_deri.push_back(init_traj.evaluate(0.0, 1));
  boundary_deri.push_back(init_traj.evaluate(duration, 1));
  boundary_deri.push_back(init_traj.evaluate(0.0, 2));
  boundary_deri.push_back(init_traj.evaluate(duration, 2));

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, points, boundary_deri, pp_.bspline_degree_, ctrl_pts);
  NonUniformBspline tmp_traj(ctrl_pts, pp_.bspline_degree_, dt);

  int cost_func = BsplineOptimizer::NORMAL_PHASE;
  if (pp_.min_time_) cost_func |= BsplineOptimizer::MINTIME;

  vector<Vector3d> start, end;
  tmp_traj.getBoundaryStates(2, 0, start, end);
  bspline_optimizers_[0]->setBoundaryStates(start, end);
  if (time_lb > 0) {
    bspline_optimizers_[0]->setTimeLowerBound(time_lb);
  }

  bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 1, 1);
  local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  updateTrajInfo();

  return true;
}

// !SECTION

// SECTION kinodynamic replanning

bool FastPlannerManager::kinodynamicReplan(const Vector3d& start_pt, const Vector3d& start_vel, const Vector3d& start_acc,
    const double start_yaw, const Vector3d& end_pt, const Vector3d& end_vel, const double end_yaw, const double& time_lb) {
  std::cout << "[Kino replan]: start pos: " << start_pt.transpose() << endl;
  std::cout << "[Kino replan]: start vel: " << start_vel.transpose() << endl;
  std::cout << "[Kino replan]: start acc: " << start_acc.transpose() << endl;
  std::cout << "[Kino replan]: start yaw: " << start_yaw << endl;
  std::cout << "[Kino replan]: end pos: " << end_pt.transpose() << endl;
  std::cout << "[Kino replan]: end vel: " << end_vel.transpose() << endl;
  std::cout << "[Kino replan]: end yaw: " << end_yaw << endl;

  if ((start_pt - end_pt).norm() < 1e-2) {
    cout << "Close goal" << endl;
    return false;
  }

  // Kinodynamic path searching

  auto time_start = ros::Time::now();

  kino_path_finder_->setFeatureMap(feature_map_);
  kino_path_finder_->reset();
  // int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);
  // if (status == KinodynamicAstar::NO_PATH) {
  //   ROS_ERROR("Init kinodynamic A* search fail");
  //   // Retry
  //   kino_path_finder_->reset();
  //   status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);
  //   if (status == KinodynamicAstar::NO_PATH) {
  //     return false;
  //   }
  // }

  ROS_INFO("Start search");
  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, start_yaw, end_pt, end_vel, end_yaw);
  ROS_INFO("End search");
  if (status == KinodynamicAstar::NO_PATH) {
    ROS_ERROR("Kinodynamic A* search fail");
    return false;
  }
  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

  double time_search = (ros::Time::now() - time_start).toSec();
  ROS_WARN("Time cost of kinodynamic A*: %lf(sec)", time_search);

  auto time_start_2 = ros::Time::now();

  // Parameterize path to B-spline
  double ts = pp_.ctrl_pt_dist / pp_.max_vel_;
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, pp_.bspline_degree_, ctrl_pts);
  NonUniformBspline init(ctrl_pts, pp_.bspline_degree_, ts);

  // B-spline-based optimization
  int cost_function = BsplineOptimizer::NORMAL_PHASE;
  if (pp_.min_time_) cost_function |= BsplineOptimizer::MINTIME;
  vector<Eigen::Vector3d> start, end;
  init.getBoundaryStates(2, 0, start, end);
  vector<bool> start_idx = { true, true, true };
  vector<bool> end_idx = { true, false, false };
  bspline_optimizers_[0]->setBoundaryStates(start, end, start_idx, end_idx);
  if (time_lb > 0) bspline_optimizers_[0]->setTimeLowerBound(time_lb);

  bspline_optimizers_[0]->optimize(ctrl_pts, ts, cost_function, 1, 1);
  local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, ts);

  vector<Eigen::Vector3d> start2, end2;
  local_data_.position_traj_.getBoundaryStates(2, 0, start2, end2);
  std::cout << "State error: (" << (start2[0] - start[0]).norm() << ", " << (start2[1] - start[1]).norm() << ", "
            << (start2[2] - start[2]).norm() << ")" << std::endl;

  double time_opt = (ros::Time::now() - time_start_2).toSec();
  ROS_WARN("Time cost of optimize: %lf(sec)", time_opt);

  updateTrajInfo();

  return true;
}

void FastPlannerManager::planExploreTraj(
    const vector<Eigen::Vector3d>& tour, const Eigen::Vector3d& cur_vel, const Eigen::Vector3d& cur_acc, const double& time_lb) {
  if (tour.empty()) ROS_ERROR("Empty path to traj planner");

  // Generate traj through waypoints-based method
  const int pt_num = tour.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = tour[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd times(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i) times(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);

  PolynomialTraj init_traj;
  PolynomialTraj::waypointsTraj(pos, cur_vel, zero, cur_acc, zero, times, init_traj);

  // B-spline-based optimization
  vector<Vector3d> points, boundary_deri;
  double duration = init_traj.getTotalTime();
  int seg_num = init_traj.getLength() / pp_.ctrl_pt_dist;
  seg_num = max(8, seg_num);
  double dt = duration / seg_num;

  for (double ts = 0.0; ts <= duration + 1e-4; ts += dt) points.push_back(init_traj.evaluate(ts, 0));

  boundary_deri.push_back(init_traj.evaluate(0.0, 1));
  boundary_deri.push_back(init_traj.evaluate(duration, 1));
  boundary_deri.push_back(init_traj.evaluate(0.0, 2));
  boundary_deri.push_back(init_traj.evaluate(duration, 2));

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, points, boundary_deri, pp_.bspline_degree_, ctrl_pts);
  NonUniformBspline tmp_traj(ctrl_pts, pp_.bspline_degree_, dt);

  int cost_func = BsplineOptimizer::NORMAL_PHASE;
  if (pp_.min_time_) cost_func |= BsplineOptimizer::MINTIME;

  vector<Vector3d> start, end;
  tmp_traj.getBoundaryStates(2, 0, start, end);
  bspline_optimizers_[0]->setBoundaryStates(start, end);
  if (time_lb > 0) bspline_optimizers_[0]->setTimeLowerBound(time_lb);

  bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 1, 1);
  local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

  updateTrajInfo();
}

// !SECTION

void FastPlannerManager::planYawExplore(
    const Vector3d& start_yaw, const double& end_yaw, bool lookfwd, const double& relax_time) {
  const int seg_num = 12;
  double dt_yaw = local_data_.duration_ / seg_num;  // time of B-spline segment
  Eigen::Vector3d start_yaw3d = start_yaw;
  std::cout << "dt_yaw: " << dt_yaw << ", start yaw: " << start_yaw3d.transpose() << ", end: " << end_yaw << std::endl;

  while (start_yaw3d[0] < -M_PI) start_yaw3d[0] += 2 * M_PI;
  while (start_yaw3d[0] > M_PI) start_yaw3d[0] -= 2 * M_PI;
  double last_yaw = start_yaw3d[0];

  // Yaw traj control points
  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  // Initial state
  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0, dt_yaw,
      (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block<3, 1>(0, 0) = states2pts * start_yaw3d;

  // Add waypoint constraints if look forward is enabled
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;
  if (lookfwd) {
    const double forward_t = 2.0;
    const int relax_num = relax_time / dt_yaw;
    for (int i = 1; i < seg_num - relax_num; ++i) {
      double tc = i * dt_yaw;
      Eigen::Vector3d pc = local_data_.position_traj_.evaluateDeBoorT(tc);
      double tf = min(local_data_.duration_, tc + forward_t);
      Eigen::Vector3d pf = local_data_.position_traj_.evaluateDeBoorT(tf);
      Eigen::Vector3d pd = pf - pc;
      Eigen::Vector3d waypt;
      if (pd.norm() > 1e-6) {
        waypt(0) = atan2(pd(1), pd(0));
        waypt(1) = waypt(2) = 0.0;
        calcNextYaw(last_yaw, waypt(0));
      }

      else
        waypt = waypts.back();

      last_yaw = waypt(0);
      waypts.push_back(waypt);
      waypt_idx.push_back(i);
    }
  }

  // Final state
  Eigen::Vector3d end_yaw3d(end_yaw, 0, 0);
  calcNextYaw(last_yaw, end_yaw3d(0));
  yaw.block<3, 1>(seg_num, 0) = states2pts * end_yaw3d;

  cout << "states2pts * start_yaw3d: " << (states2pts * start_yaw3d).transpose() << endl;
  cout << "yaw waypts: " << endl;
  for (const auto& waypt : waypts) cout << "yaw: " << waypt[0] << endl;
  cout << "dt_yaw: " << dt_yaw << endl;
  cout << "states2pts * end_yaw3d: " << (states2pts * end_yaw3d).transpose() << endl;

  // Debug rapid change of yaw
  if (fabs(start_yaw3d[0] - end_yaw3d[0]) >= M_PI) {
    ROS_ERROR("Yaw change rapidly!");
    std::cout << "start yaw: " << start_yaw3d[0] << ", " << end_yaw3d[0] << std::endl;
  }

  // Call B-spline optimization solver
  // int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::START | BsplineOptimizer::END | BsplineOptimizer::WAYPOINTS;
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::START | BsplineOptimizer::WAYPOINTS;
  vector<Eigen::Vector3d> start = { Eigen::Vector3d(start_yaw3d[0], 0, 0), Eigen::Vector3d(start_yaw3d[1], 0, 0),
    Eigen::Vector3d(start_yaw3d[2], 0, 0) };
  vector<Eigen::Vector3d> end = { Eigen::Vector3d(end_yaw3d[0], 0, 0), Eigen::Vector3d(0, 0, 0) };

  vector<bool> start_idx = { true, true, true };
  vector<bool> end_idx = { true, true, false };

  bspline_optimizers_[1]->setBoundaryStates(start, end, start_idx, end_idx);
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  bspline_optimizers_[1]->setFeatureMap(feature_map_);
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1);

  cout << "yaw after optimize: " << endl;
  cout << yaw << endl;

  // Update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();
  plan_data_.dt_yaw_ = dt_yaw;
}

bool FastPlannerManager::planYawPerceptionAware(
    const Vector3d& start_yaw, const double& end_yaw, const vector<Vector3d>& frontier_cells) {

  // Yaw b-spline has same segment number as position b-spline
  Eigen::MatrixXd position_ctrl_pts = local_data_.position_traj_.getControlPoint();
  int ctrl_pts_num = position_ctrl_pts.rows();
  double dt_yaw = local_data_.position_traj_.getKnotSpan();

  // Yaw traj control points
  Eigen::MatrixXd yaw(ctrl_pts_num, 1);
  yaw.setZero();

  // Calculate knot pos and acc
  // [u[p],u[m-p]] -> [0*dt, (m-2p)*dt] -> [0*dt, (n-2)*dt]
  vector<Vector3d> knot_pos, knot_acc;
  for (int i = 0; i < ctrl_pts_num - 2; ++i) {
    double t = i * dt_yaw;
    knot_pos.emplace_back(local_data_.position_traj_.evaluateDeBoorT(t));
    knot_acc.emplace_back(local_data_.acceleration_traj_.evaluateDeBoorT(t));
  }

  // TODO: only need to calculate nn features once! Feed to yaw_initial_planner & optimizer

  auto time_start = ros::Time::now();

  // Step1: 使用图搜索算法给出一条初始的yaw轨迹(yaw_waypoints)，跟position_traj一一对应
  vector<double> yaw_waypoints;
  yaw_initial_planner_->setFeatureMap(feature_map_);
  yaw_initial_planner_->setFrontierFinder(frontier_finder_);
  yaw_initial_planner_->setTargetFrontier(frontier_cells);

  if (!yaw_initial_planner_->searchPathOfYaw(start_yaw[0], end_yaw, knot_pos, knot_acc, dt_yaw, yaw_waypoints)) {
    ROS_ERROR("Yaw Trajectory Planning Failed in Graph Search!!!");
    return false;
  }

  // if (yaw_waypoints.back() == 0) yaw_waypoints.back() = 10e-1;  // 不科学！！！但是设置成这样避免末端采样为0的时候造成报错
  //  for (auto& yaw : yaw_waypoints) yaw = 0.0;

  cout << "yaw_waypoints: " << endl;
  for (const auto& yaw : yaw_waypoints) cout << "yaw: " << yaw << endl;

  // 后面优化选项选择了WAYPOINTS，所以这里需要把waypoints设置好
  vector<Vector3d> waypts;
  vector<int> waypt_idx;
  double last_yaw = yaw_waypoints[0];
  for (size_t i = 0; i < yaw_waypoints.size(); ++i) {
    Vector3d waypt = Vector3d::Zero();
    waypt(0) = yaw_waypoints[i];
    calcNextYaw(last_yaw, waypt(0));
    last_yaw = waypt(0);
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  double time_yaw_inital_planner = (ros::Time::now() - time_start).toSec();
  ROS_WARN("Time cost of yaw inital planner: %lf(sec)", time_yaw_inital_planner);

  //  Initial state
  Vector3d start_yaw3d = start_yaw;

  while (start_yaw3d[0] < -M_PI) start_yaw3d[0] += 2 * M_PI;
  while (start_yaw3d[0] > M_PI) start_yaw3d[0] -= 2 * M_PI;
  last_yaw = start_yaw3d[0];

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0, dt_yaw,
      (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block<3, 1>(0, 0) = states2pts * start_yaw3d;

  // Final state
  Eigen::Vector3d end_yaw3d(end_yaw, 0, 0);
  calcNextYaw(last_yaw, end_yaw3d(0));
  yaw.block<3, 1>(yaw.rows() - 3, 0) = states2pts * end_yaw3d;
  // yaw.block<3, 1>(ctrl_pts_num - 3, 0) = states2pts * Vector3d(yaw_waypoints.back(), 0, 0);

  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  cout << "start_yaw3d: " << start_yaw3d.transpose() << endl;
  cout << "end_yaw3d: " << end_yaw3d.transpose() << endl;

  vector<Vector3d> start = { Vector3d(start_yaw3d[0], 0, 0), Vector3d(start_yaw3d[1], 0, 0), Vector3d(start_yaw3d[2], 0, 0) };
  vector<Vector3d> end = { Vector3d(end_yaw3d[0], 0, 0), zero, zero };
  // vector<Vector3d> end = { Vector3d(end_yaw3d[0], 0, 0), Vector3d(0, 0, 0) };
  vector<bool> start_idx = { true, true, true };
  vector<bool> end_idx = { true, true, true };

  // Call B-spline optimization solver
  auto time_start_2 = ros::Time::now();

  // Step2: 调用B样条曲线优化器优化yaw轨迹
  // 这里使用了平滑约束、路径点约束、起点约束、终点约束、主要就是添加了yaw共视约束
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS | BsplineOptimizer::START | BsplineOptimizer::END |
                  BsplineOptimizer::YAWCOVISIBILITY | BsplineOptimizer::FRONTIERVISIBILITY_YAW;

  // int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS | BsplineOptimizer::START | BsplineOptimizer::END
  // |
  //                 BsplineOptimizer::FRONTIERVISIBILITY_YAW;

  if (cost_func & BsplineOptimizer::YAWCOVISIBILITY || cost_func & BsplineOptimizer::FRONTIERVISIBILITY_YAW) {
    vector<Vector3d> pos_knots, acc_knots;
    local_data_.position_traj_.getKnotPoint(pos_knots);
    local_data_.acceleration_traj_.getKnotPoint(acc_knots);
    bspline_optimizers_[1]->setPosAndAcc(pos_knots, acc_knots);

    if (cost_func & BsplineOptimizer::FRONTIERVISIBILITY_YAW) {
      bspline_optimizers_[1]->setFrontierCells(frontier_cells);
    }
  }

  bspline_optimizers_[1]->setBoundaryStates(start, end, start_idx, end_idx);
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  bspline_optimizers_[1]->setFeatureMap(feature_map_);

  cout << "yaw control point before optimize: " << yaw << endl;
  cout << "dt_yaw: " << dt_yaw << endl;

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
    cout << "yaw control point before optimize(add noise): " << yaw << endl;
  }

  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1);

  cout << "yaw control point after optimize: " << yaw << endl;

  double time_opt = (ros::Time::now() - time_start_2).toSec();
  ROS_WARN("Time cost of yaw traj optimize: %lf(sec)", time_opt);

  // Update traj info
  // Step3: 更新yaw及其导数的轨迹
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  vector<double> knot_yaw;
  for (int i = 0; i < ctrl_pts_num - 2; ++i) {
    double t = i * dt_yaw;
    knot_yaw.emplace_back(local_data_.yaw_traj_.evaluateDeBoorT(t)[0]);
  }

  ROS_INFO("Compare knot point of yaw");
  ROS_ASSERT(knot_yaw.size() == yaw_waypoints.size());
  for (size_t i = 0; i < knot_yaw.size(); ++i) {
    cout << "knot_yaw: " << knot_yaw[i] << ", yaw_waypoints: " << yaw_waypoints[i] << endl;
  }

  return true;
}

void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]
  double round_last = last_yaw;
  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;
  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

}  // namespace fast_planner
