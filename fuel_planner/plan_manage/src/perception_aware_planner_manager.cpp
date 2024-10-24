#include "plan_manage/perception_aware_planner_manager.h"
#include "plan_env/utils.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>

#include <visualization_msgs/Marker.h>

#include <stepping_debug.hpp>
#include <thread>

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

  nh.param("manager/lookforward_radius", pp_.lookforward_radius_, 5.0);
  nh.param("manager/k_theta", pp_.k_theta_, 1.0);
  nh.param("manager/max_theta", pp_.max_theta_, -1.0);
  nh.param("manager/sample_num", pp_.sample_num_, 5);
  nh.param("manager/delta_v", pp_.delta_v_, 0.5);
  nh.param("manager/sample_pose_num", pp_.sample_pose_num_, 5);
  nh.param("manager/safety_sphere_radius", pp_.safety_sphere_radius_, 0.5);
  pp_.safety_sphere_volume_ = 4 * M_PI * pow(pp_.safety_sphere_radius_, 3) / 3;
  nh.param("manager/k_goal", pp_.k_goal_, 10.0);
  nh.param("manager/d_critic", pp_.d_critic_, 15.0);
  nh.param("manager/k", pp_.k_, 3);
  nh.param("manager/col_prob_non_decreasing", pp_.col_prob_non_decreasing_, true);
  nh.param("manager/k_col", pp_.k_col_, -10000.0);
  nh.param("manager/k_perc", pp_.k_perc_, 1.5);

  bool use_optimization;
  nh.param("manager/use_optimization", use_optimization, false);

  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  camera_param = Utils::getGlobalParam().camera_param_;

  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->getSteppingDebug(stepping_debug_);
      bspline_optimizers_[i]->setParam(nh);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }
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
      // std::cout << "collision at: " << fut_pt.transpose() << ", dist: " <<
      // dist << std::endl;
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
      // std::cout << "collision at: " << fut_pt.transpose() << ", dist: " <<
      // dist << std::endl;
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
      ROS_WARN("[FastPlannerManager::checkTrajLocalizabilityOnKnots] Poor "
               "localizability");
      return false;
    }
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

  cout.unsetf(ios::fixed);
  cout << "===============================================================" << endl;
  cout << NORMAL_FONT;
}
// !SECTION

// SECTION perception aware replanning

void FastPlannerManager::updateStartState(
    const Vector3d& start_pos, const Vector3d& start_vel, const Vector3d& start_acc, const Vector3d& start_yaw) {

  start_pos_ = start_pos;
  start_vel_ = start_vel;
  start_acc_ = start_acc;
  start_yaw_ = start_yaw;
}

vector<Vector3d> FastPlannerManager::generateEndPoint() {
  double v2d_norm = start_vel_.head(2).norm();
  double theta = std::max(pp_.k_theta_ * v2d_norm, pp_.max_theta_);

  vector<double> yaw_samples;
  double step = (2 * theta) / (pp_.sample_num_ - 1);
  for (int i = 0; i < pp_.sample_num_; ++i) yaw_samples.push_back(-theta + i * step);

  // for (const auto& yaw : yaw_samples) {
  //   std::cout << "Yaw sample: " << yaw << std::endl;
  // }

  vector<Vector3d> end_points;
  for (size_t i = 0; i < yaw_samples.size(); ++i) {
    Vector3d t_se = pp_.lookforward_radius_ * Vector3d(cos(yaw_samples[i]), sin(yaw_samples[i]), 0);
    Quaterniond q_ws = Quaterniond(AngleAxisd(start_yaw_(0), Vector3d::UnitZ()));
    end_points.emplace_back(q_ws * t_se + start_pos_);
  }

  return end_points;
}

void FastPlannerManager::solvePnPByGaussNewton(
    const vector<Vector3d>& points_3d, const vector<Vector2d>& points_2d, const Matrix3d& K, Sophus::SE3d& pose, Matrix6d& H) {

  const int iterations = 10;
  double cost = 0, lastCost = 0;
  double fx = K(0, 0);
  double fy = K(1, 1);
  double cx = K(0, 2);
  double cy = K(1, 2);

  H.setZero();

  for (int iter = 0; iter < iterations; iter++) {
    H.setZero();
    Vector6d g = Vector6d::Zero();

    cost = 0;
    // compute cost
    for (int i = 0; i < points_3d.size(); i++) {
      Vector3d pc = pose * points_3d[i];
      double inv_z = 1.0 / pc[2];
      double inv_z2 = inv_z * inv_z;
      Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);
      Vector2d e = points_2d[i] - proj;

      cost += e.squaredNorm();
      Eigen::Matrix<double, 2, 6> J;
      J << -fx * inv_z, 0, fx * pc[0] * inv_z2, fx * pc[0] * pc[1] * inv_z2, -fx - fx * pc[0] * pc[0] * inv_z2,
          fx * pc[1] * inv_z, 0, -fy * inv_z, fy * pc[1] * inv_z2, fy + fy * pc[1] * pc[1] * inv_z2, -fy * pc[0] * pc[1] * inv_z2,
          -fy * pc[0] * inv_z;

      H += J.transpose() * J;
      g += -J.transpose() * e;
    }

    Vector6d dx;
    dx = H.ldlt().solve(g);

    if (isnan(dx[0])) {
      cout << "result is nan!" << endl;
      break;
    }

    if (iter > 0 && cost >= lastCost) {
      // cost increase, update is not good
      cout << "cost: " << cost << ", last cost: " << lastCost << endl;
      ROS_ERROR("Fuck You");
      ROS_BREAK();
      break;
    }

    // update your estimation
    pose = Sophus::SE3d::exp(dx) * pose;
    lastCost = cost;

    // cout << "iteration " << iter << " cost=" << std::setprecision(12) << cost << endl;
    //  已经收敛了
    if (dx.norm() < 1e-6) {
      // cout << "Gaussian Newton Converge!!!" << endl;
      break;
    }
  }

  // cout << "Information Matrix: " << endl << H << endl;
  //  cout << "pose by g-n: \n" << pose.matrix() << endl;
}

bool FastPlannerManager::calPerceptionCost(const size_t id, double& R_perc) {

  size_t J = plan_data_.candidate_pos_[id].size();
  MatrixXd H_total = MatrixXd::Zero(6 * J, 6 * J);

  for (size_t i = 0; i < J; i++) {
    // Step1: 获取基本数据
    auto pos = plan_data_.candidate_pos_[id][i];
    auto acc = plan_data_.candidate_acc_[id][i];
    auto yaw = plan_data_.candidate_yaw_[id][i];
    Quaterniond ori = Utils::calcOrientation(yaw, acc);

    // Step2: 获取本节点可视的特征点集合
    vector<Vector3d> landmark;
    int landmark_num = feature_map_->get_NumCloud_using_Odom(pos, ori, landmark);
    int min_feature_num = Utils::getGlobalParam().min_feature_num_plan_;
    if (landmark_num < min_feature_num) return false;

    // Step3: 用相机投影模型得到像素坐标
    vector<Vector2d> landmark_proj;
    Sophus::SE3d pose_gt(ori, pos);  // Twb
    Sophus::SE3d Tbw = pose_gt.inverse();
    Sophus::SE3d Tbc(camera_param->sensor2body);
    Sophus::SE3d Tcb = Tbc.inverse();

    for (const auto& pt_w : landmark) {
      Vector3d pt_b = Tbw * pt_w;
      Vector3d pt_c = Tcb * pt_b;

      Vector2d proj_pt = camera_param->camera2pixel(pt_c);
      double sigma = 1;  // adjust the noise level as needed

      std::random_device rd;
      std::mt19937 gen(rd());
      std::normal_distribution<double> distribution(0.0, sigma);  // 均值为 0，标准差为 0.1
      double noise_u = distribution(gen);
      // cout << "noise u: " << noise_u << endl;
      proj_pt(0) += noise_u;
      double noise_v = distribution(gen);
      // cout << "noise v: " << noise_v << endl;
      proj_pt(1) += noise_v;

      landmark_proj.emplace_back(proj_pt);
    }

    // Step4: 计算这个节点的信息矩阵
    Matrix3d K = camera_param->getK();
    Matrix6d H;
    Sophus::SE3d pose1 = Tcb * Tbw;      // Tcw
    Sophus::SE3d pose_esti = Tcb * Tbw;  // Tcw
    solvePnPByGaussNewton(landmark, landmark_proj, K, pose_esti, H);

    // cout << "pose1: " << pose1.matrix() << endl;
    // cout << "pose esti: " << endl << pose_esti.matrix() << endl;

    H_total.block(6 * i, 6 * i, 6, 6) = H;
  }

  // 使用论文说的D-opt准则计算最终cost
  // cout << "k perc: " << pp_.k_perc_ << endl;
  // cout << "fuck1: " << std::exp(std::log(pow(H_total.determinant(), 1.0 / (6.0 * J)))) << endl;
  R_perc = pp_.k_perc_ * std::exp(std::log(pow(H_total.determinant(), 1.0 / (6.0 * J))));

  return true;
}

double FastPlannerManager::calGoalProcessCost(const size_t id) {
  // cout << "start_pos: " << start_pos_.transpose() << endl;
  // cout << "final_goal: " << final_goal_.transpose() << endl;

  double d_cur = (start_pos_ - final_goal_).norm();
  Vector3d end_point = plan_data_.candidate_pos_[id].back();
  double d_end = (end_point - final_goal_).norm();
  double delta_d = d_cur - d_end;

  // cout << "k_goal: " << pp_.k_goal_ << endl;
  // cout << "d_critic: " << pp_.d_critic_ << endl;
  // cout << "d_cur: " << d_cur << endl;
  // cout << "k: " << pp_.k_ << endl;
  double R_goal = pp_.k_goal_ * delta_d * pow(pp_.d_critic_ / d_cur, pp_.k_);

  return R_goal;
}

double FastPlannerManager::calCollisionProb(const size_t id) {
  vector<double> p_col_vec;
  for (const auto& pos : plan_data_.candidate_pos_[id]) {
    double dist = sdf_map_->getDistance(pos);

    double sigma_d = 1.0, sigma_p = 1.0;
    Matrix3d cov = (sigma_d + sigma_p) * Matrix3d::Identity();

    double p = pp_.safety_sphere_volume_ * std::exp(-(dist / (sigma_d + sigma_p)) / 2) / (sqrt(2 * M_PI * cov.determinant()));

    if (pp_.col_prob_non_decreasing_) {
      if (!p_col_vec.empty() && p < p_col_vec.back())
        p_col_vec.emplace_back(p_col_vec.back());

      else
        p_col_vec.emplace_back(p);
    }

    else {
      p_col_vec.emplace_back(p);
    }
  }

  double p_col = 1.0;
  for (const auto& p : p_col_vec) p_col *= (1 - p);

  p_col = 1 - p_col;

  return p_col;
}

void FastPlannerManager::planYaw() {
  Eigen::MatrixXd position_ctrl_pts = local_data_.position_traj_.getControlPoint();
  int ctrl_pts_num = position_ctrl_pts.rows();
  double dt_yaw = local_data_.position_traj_.getKnotSpan();
  const double duration = local_data_.position_traj_.getTimeSum();

  // Yaw traj control points
  Eigen::MatrixXd yaw(ctrl_pts_num, 1);
  yaw.setZero();

  // 设置起始状态
  Vector3d start_yaw3d = start_yaw_;
  Utils::roundPi(start_yaw3d[0]);

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0, dt_yaw,
      (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block<3, 1>(0, 0) = states2pts * start_yaw3d;

  // 设置中间的waypoint约束
  vector<Vector3d> knot_vel;
  local_data_.velocity_traj_.getKnotPoint(knot_vel);

  double last_yaw = start_yaw3d[0];
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;

  for (size_t i = 0; i < knot_vel.size() - 1; ++i) {
    Vector3d waypt = Vector3d::Zero();

    if (i == 0) {
      waypt(0) = last_yaw;
      waypt(1) = waypt(2) = 0.0;
    }

    else {
      Vector3d vel = knot_vel[i];
      if (vel.norm() > 1e-6) {
        waypt(0) = atan2(vel(1), vel(0));
        waypt(1) = waypt(2) = 0.0;
        Utils::calcNextYaw(last_yaw, waypt(0));
      } else
        waypt = waypts.back();
    }

    last_yaw = waypt(0);
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // 设置终止状态
  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  double end_yaw = atan2(end_v(1), end_v(0));
  Vector3d end_yaw3d(end_yaw, 0, 0);
  Utils::calcNextYaw(last_yaw, end_yaw3d(0));
  yaw.block<3, 1>(yaw.rows() - 3, 0) = states2pts * end_yaw3d;

  const Vector3d zero = Eigen::Vector3d::Zero();
  vector<Vector3d> start = { Vector3d(start_yaw_[0], 0, 0), Vector3d(start_yaw_[1], 0, 0), Vector3d(start_yaw_[2], 0, 0) };
  vector<Vector3d> end = { Vector3d(end_yaw3d[0], 0, 0), zero, zero };
  vector<bool> start_idx = { true, true, true };
  vector<bool> end_idx = { true, true, true };

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

  int cost_func = 0;
  cost_func |= BsplineOptimizer::SMOOTHNESS;
  cost_func |= BsplineOptimizer::WAYPOINTS;
  cost_func |= BsplineOptimizer::START;
  cost_func |= BsplineOptimizer::END;
  cost_func |= BsplineOptimizer::FEASIBILITY_YAW;

  bspline_optimizers_[1]->setBoundaryStates(start, end, start_idx, end_idx);
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1);

  // update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();
}

void FastPlannerManager::selectBestTraj(const Vector3d& start_pos, const Vector3d& start_vel, const Vector3d& start_acc,
    const Vector3d& start_yaw, const Vector3d& final_goal) {

  // std::cout << "[Kino replan]: start pos: " << start_pos.transpose() << endl;
  // std::cout << "[Kino replan]: start vel: " << start_vel.transpose() << endl;
  // std::cout << "[Kino replan]: start acc: " << start_acc.transpose() << endl;
  // std::cout << "[Kino replan]: start yaw: " << start_yaw << endl;
  // std::cout << "[Kino replan]: final goal: " << final_goal.transpose() << endl;

  // Step1: 更新起始状态
  updateStartState(start_pos, start_vel, start_acc, start_yaw);
  setFinalGoal(final_goal);

  // Step1: 决定末端点和轨迹用时
  auto end_points = generateEndPoint();
  double v2d_norm = start_vel_.head(2).norm();
  double tf = std::min(pp_.lookforward_radius_ / (v2d_norm + pp_.delta_v_), pp_.lookforward_radius_ / pp_.max_vel_);

  // Step2: 生成备选轨迹簇
  plan_data_.candidate_trajs_.clear();
  for (size_t i = 0; i < end_points.size(); i++) {
    Eigen::MatrixXd pos(2, 3);
    pos.row(0) = start_pos_;
    pos.row(1) = end_points[i];

    Eigen::VectorXd times(1);
    times(0) = tf;

    Vector3d zero = Vector3d::Zero();

    PolynomialTraj traj;
    // PolynomialTraj::waypointsTraj(pos, start_vel, zero, start_acc, zero, times, traj);
    PolynomialTraj::OBVPTraj(pos, start_vel, zero, tf, traj);

    // cout << "debug1!!!" << endl;
    plan_data_.candidate_trajs_.emplace_back(traj);
  }

  // Step3: 在备选轨迹簇上按均匀时间分布采样出waypoint
  double dt = tf / pp_.sample_pose_num_;

  plan_data_.candidate_trajs_vis_.clear();
  plan_data_.candidate_pos_.clear();
  plan_data_.candidate_acc_.clear();
  plan_data_.candidate_yaw_.clear();

  for (size_t i = 0; i < plan_data_.candidate_trajs_.size(); i++) {
    vector<Vector3d> pos_vis;
    plan_data_.candidate_trajs_[i].getSamplePoints(pos_vis);
    plan_data_.candidate_trajs_vis_.emplace_back(pos_vis);

    vector<Vector3d> candidate_pos;
    vector<Vector3d> candidate_acc;
    vector<double> candidate_yaw;

    for (int j = 0; j <= pp_.sample_pose_num_; j++) {
      double t = j * dt;
      candidate_pos.emplace_back(plan_data_.candidate_trajs_[i].evaluate(t, 0));
      candidate_acc.emplace_back(plan_data_.candidate_trajs_[i].evaluate(t, 2));

      Vector3d vel = plan_data_.candidate_trajs_[i].evaluate(t, 1);
      candidate_yaw.emplace_back(atan2(vel.y(), vel.x()));
    }

    plan_data_.candidate_pos_.emplace_back(candidate_pos);
    plan_data_.candidate_acc_.emplace_back(candidate_acc);
    plan_data_.candidate_yaw_.emplace_back(candidate_yaw);
  }

  // Step4: 为每段备选waypoint计算各项cost
  // vector<bool> if_perc_cost_valid_;
  plan_data_.if_perc_cost_valid_.clear();
  vector<Eigen::Vector4d> metric_;

  for (size_t i = 0; i < plan_data_.candidate_pos_.size(); i++) {
    double R_perc;
    bool ifvalid = calPerceptionCost(i, R_perc);
    plan_data_.if_perc_cost_valid_.emplace_back(ifvalid);

    double R_goal = calGoalProcessCost(i);
    double R_col = pp_.k_col_;

    double p_col = calCollisionProb(i);

    metric_.emplace_back(p_col, R_perc, R_goal, R_col);

    double score = (1 - p_col) * (R_perc + R_goal) + p_col * R_col;

    // cout << "p_col: " << p_col << endl;
    // cout << "R_perc: " << R_perc << endl;
    // cout << "R_goal: " << R_goal << endl;
    // cout << "R_col: " << R_col << endl;
    // cout << "score: " << score << endl;
  }

  // 如果备选簇中有至少一个perc_cost有效的轨迹，那就让所有perc_cost有效的轨迹来参与排序
  vector<double> scores;
  if (std::any_of(plan_data_.if_perc_cost_valid_.begin(), plan_data_.if_perc_cost_valid_.end(), [](bool v) { return v; })) {
    // Handle the case where there is at least one valid perception cost
    ROS_INFO("There is at least one valid perception cost.");

    for (size_t i = 0; i < plan_data_.candidate_pos_.size(); i++) {
      if (!plan_data_.if_perc_cost_valid_[i]) {
        scores.emplace_back(std::numeric_limits<double>::min());
      }

      else {
        double p_col = metric_[i](0);
        double R_perc = metric_[i](1);
        double R_goal = metric_[i](2);
        double R_col = metric_[i](3);
        double score = (1 - p_col) * (R_perc + R_goal) + p_col * R_col;
        scores.emplace_back(score);
      }
    }

  }

  // 如果备选簇中一个perc_cost有效的轨迹都没有，那本次排序就不管perc_cost（保证规划不会失败）
  else {
    // Handle the case where there are no valid perception costs
    ROS_WARN("No valid perception costs found.");

    for (size_t i = 0; i < plan_data_.candidate_pos_.size(); i++) {
      double p_col = metric_[i](0);
      double R_perc = 0.0;
      double R_goal = metric_[i](2);
      double R_col = metric_[i](3);
      double score = (1 - p_col) * (R_perc + R_goal) + p_col * R_col;
      scores.emplace_back(score);
    }
  }

  // Step5: 对scores进行排序，选出最好的轨迹
  // cout << "Select Best Traj" << endl;
  // for (size_t i = 0; i < scores.size(); i++) {
  //   cout << "id: " << i << ",score: " << scores[i] << endl;
  // }

  auto max_score_it = std::max_element(scores.begin(), scores.end());
  plan_data_.best_traj_idx_ = std::distance(scores.begin(), max_score_it);
  auto& best_traj = plan_data_.candidate_trajs_[plan_data_.best_traj_idx_];
  // auto& best_waypoint = plan_data_.candidate_pos_[plan_data_.best_traj_idx_];

  double duration = best_traj.getTotalTime();
  int seg_num = best_traj.getLength() / pp_.ctrl_pt_dist;
  seg_num = max(8, seg_num);
  double knot_span = duration / seg_num;

  vector<Vector3d> points;
  for (double ts = 0.0; ts <= duration + 1e-4; ts += knot_span) points.push_back(best_traj.evaluate(ts, 0));

  // Step6: 通过最好的多项式轨迹生成Bspline轨迹(位置轨迹)

  // Evaluate velocity at start and end
  vector<Vector3d> boundary_deri;
  boundary_deri.push_back(best_traj.evaluate(0.0, 1));
  boundary_deri.push_back(best_traj.evaluate(tf, 1));
  // Evaluate acceleration at start and end
  boundary_deri.push_back(best_traj.evaluate(0.0, 2));
  boundary_deri.push_back(best_traj.evaluate(tf, 2));

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(knot_span, points, boundary_deri, pp_.bspline_degree_, ctrl_pts);
  NonUniformBspline tmp_traj(ctrl_pts, pp_.bspline_degree_, knot_span);

  vector<Vector3d> start, end;
  vector<bool> start_idx, end_idx;

  tmp_traj.getBoundaryStates(2, 0, start, end);
  start_idx = { true, true, true };
  end_idx = { true, false, false };

  bspline_optimizers_[0]->setBoundaryStates(start, end, start_idx, end_idx);

  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::FEASIBILITY | BsplineOptimizer::START | BsplineOptimizer::END;
  bspline_optimizers_[0]->optimize(ctrl_pts, knot_span, cost_func, 1, 1);

  local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, knot_span);
  updateTrajInfo();

  // Step8: 通过位置轨迹简单生成yaw轨迹
  planYaw();
}

}  // namespace fast_planner
