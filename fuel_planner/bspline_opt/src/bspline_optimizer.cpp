#include "bspline_opt/bspline_optimizer.h"

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_env/feature_map.h>

#include <plan_env/utils.hpp>

#include <active_perception/frontier_finder.h>

#include <nlopt.hpp>
#include <thread>

#include <stepping_debug.hpp>

using namespace std;
using namespace Eigen;

namespace fast_planner {
const int BsplineOptimizer::SMOOTHNESS = (1 << 1);
const int BsplineOptimizer::DISTANCE = (1 << 2);
const int BsplineOptimizer::FEASIBILITY = (1 << 3);
const int BsplineOptimizer::START = (1 << 4);
const int BsplineOptimizer::END = (1 << 5);
const int BsplineOptimizer::GUIDE = (1 << 6);
const int BsplineOptimizer::WAYPOINTS = (1 << 7);
const int BsplineOptimizer::MINTIME = (1 << 8);
const int BsplineOptimizer::PARALLAX = (1 << 9);
const int BsplineOptimizer::VERTICALVISIBILITY = (1 << 10);
const int BsplineOptimizer::YAWCOVISIBILITY = (1 << 11);
const int BsplineOptimizer::FRONTIERVISIBILITY_POS = (1 << 12);
const int BsplineOptimizer::FRONTIERVISIBILITY_YAW = (1 << 13);
const int BsplineOptimizer::FEASIBILITY_YAW = (1 << 14);
const int BsplineOptimizer::FINAL_GOAL = (1 << 15);

const int BsplineOptimizer::GUIDE_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE | BsplineOptimizer::START | BsplineOptimizer::END;
const int BsplineOptimizer::NORMAL_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE |
                                           BsplineOptimizer::FEASIBILITY | BsplineOptimizer::START | BsplineOptimizer::END;

void BsplineOptimizer::setParam(ros::NodeHandle& nh) {
  nh.param("optimization/ld_smooth", ld_smooth_, -1.0);
  nh.param("optimization/ld_dist", ld_dist_, -1.0);
  nh.param("optimization/ld_feasi", ld_feasi_, -1.0);
  nh.param("optimization/ld_feasi_yaw", ld_feasi_yaw_, -1.0);
  nh.param("optimization/ld_start", ld_start_, -1.0);
  nh.param("optimization/ld_end", ld_end_, -1.0);
  nh.param("optimization/ld_guide", ld_guide_, -1.0);
  nh.param("optimization/ld_waypt", ld_waypt_, -1.0);
  nh.param("optimization/ld_view", ld_view_, -1.0);
  nh.param("optimization/ld_time", ld_time_, -1.0);
  nh.param("optimization/ld_parallax", ld_parallax_, -1.0);
  nh.param("optimization/ld_vertical_visibility", ld_vertical_visibility_, -1.0);
  nh.param("optimization/ld_yaw_covisibility", ld_yaw_covisib_, -1.0);
  nh.param("optimization/ld_frontier_visibility_pos", ld_frontier_visibility_pos_, -1.0);
  nh.param("optimization/ld_frontier_visibility_yaw", ld_frontier_visibility_yaw_, -1.0);
  nh.param("optimization/ld_final_goal", ld_final_goal_, -1.0);
  nh.param("optimization/ld_weight1", ld_weight1_, -1.0);
  nh.param("optimization/ld_weight2", ld_weight2_, -1.0);

  nh.param("optimization/dist0", dist0_, -1.0);
  nh.param("optimization/max_vel", max_vel_, -1.0);
  nh.param("optimization/max_acc", max_acc_, -1.0);
  nh.param("optimization/dlmin", dlmin_, -1.0);
  nh.param("optimization/wnl", wnl_, -1.0);

  nh.param("optimization/parallax/estimator_freq", configPA_.estimator_freq_, -1.0);
  nh.param("optimization/parallax/max_parallax", configPA_.max_parallax_, -1.0);
  nh.param("optimization/parallax/pot_a", configPA_.pot_a_, -1.0);
  nh.param("optimization/min_covisible_feature_cost_", configPA_.min_covisible_feature_cost_, -1.0);
  nh.param("optimization/min_frontier_see_feature_num", configPA_.min_frontier_see_feature_num_, -1.0);
  nh.param("optimization/pot_fafv", configPA_.pot_fafv_, -1.0);
  nh.param("optimization/k1", configPA_.k1_, -1.0);
  nh.param("optimization/k2", configPA_.k2_, -1.0);
  nh.param("optimization/k3", configPA_.k3_, -1.0);

  nh.param("optimization/max_iteration_num1", max_iteration_num_[0], -1);
  nh.param("optimization/max_iteration_num2", max_iteration_num_[1], -1);
  nh.param("optimization/max_iteration_num3", max_iteration_num_[2], -1);
  nh.param("optimization/max_iteration_num4", max_iteration_num_[3], -1);
  nh.param("optimization/max_iteration_time1", max_iteration_time_[0], -1.0);
  nh.param("optimization/max_iteration_time2", max_iteration_time_[1], -1.0);
  nh.param("optimization/max_iteration_time3", max_iteration_time_[2], -1.0);
  nh.param("optimization/max_iteration_time4", max_iteration_time_[3], -1.0);

  nh.param("optimization/algorithm1", algorithm1_, -1);
  nh.param("optimization/algorithm2", algorithm2_, -1);
  nh.param("manager/bspline_degree", bspline_degree_, 3);

  time_lb_ = -1;  // Not used by in most case
  frontier_centre_ = Eigen::Vector3d::Zero();
  view_point_yaw_ = 2 * M_PI;  // yaw_not_available

  camera_param_ = Utils::getGlobalParam().camera_param_;
}

void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
  dynamic_ = false;
}

void BsplineOptimizer::setCostFunction(const int& cost_code) {
  cost_function_ = cost_code;
}

void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d>& guide_pt) {
  guide_pts_ = guide_pt;
}

void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d>& waypts, const vector<int>& waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

void BsplineOptimizer::enableDynamic(double time_start) {
  dynamic_ = true;
  start_time_ = time_start;
}

void BsplineOptimizer::setBoundaryStates(const vector<Eigen::Vector3d>& start, const vector<Eigen::Vector3d>& end) {
  start_state_ = start;
  end_state_ = end;
}

void BsplineOptimizer::setBoundaryStates(
    const vector<Vector3d>& start, const vector<Vector3d>& end, const vector<bool>& start_idx, const vector<bool>& end_idx) {
  start_state_ = start;
  end_state_ = end;
  start_con_index_ = start_idx;
  end_con_index_ = end_idx;
}

void BsplineOptimizer::setTimeLowerBound(const double& lb) {
  time_lb_ = lb;
}

// SECTION Perception Aware Optimization

void BsplineOptimizer::setPosAndAcc(const vector<Vector3d>& pos, const vector<Vector3d>& acc, const vector<int>& idx) {
  pos_ = pos;
  acc_ = acc;
  pos_idx_ = idx;
}

// !SECTION

void BsplineOptimizer::resetCostAndGrad() {
  f_smoothness_ = 0.0;
  f_distance_ = 0.0;
  f_feasibility_ = 0.0;
  f_feasibility_yaw_ = 0.0;
  f_start_ = 0.0;
  f_end_ = 0.0;
  f_guide_ = 0.0;
  f_waypoints_ = 0.0;
  f_view_ = 0.0;
  f_time_ = 0.0;
  f_parallax_ = 0.0;
  f_frontier_visibility_pos_ = 0.0;
  f_yaw_covisibility_ = 0.0;
  f_frontier_visibility_yaw_ = 0.0;

  g_q_.resize(point_num_);
  g_smoothness_.resize(point_num_);
  g_distance_.resize(point_num_);
  g_feasibility_.resize(point_num_);
  g_feasibility_yaw_.resize(point_num_);
  g_start_.resize(point_num_);
  g_end_.resize(point_num_);
  g_guide_.resize(point_num_);
  g_waypoints_.resize(point_num_);
  g_view_.resize(point_num_);
  g_time_.resize(point_num_);
  g_parallax_.resize(point_num_);
  g_frontier_visibility_pos_.resize(point_num_);
  g_yaw_covisibility_.resize(point_num_);
  g_frontier_visibility_yaw_.resize(point_num_);

  std::fill(g_q_.begin(), g_q_.end(), Eigen::Vector3d::Zero());
  std::fill(g_smoothness_.begin(), g_smoothness_.end(), Eigen::Vector3d::Zero());
  std::fill(g_distance_.begin(), g_distance_.end(), Eigen::Vector3d::Zero());
  std::fill(g_feasibility_.begin(), g_feasibility_.end(), Eigen::Vector3d::Zero());
  std::fill(g_feasibility_yaw_.begin(), g_feasibility_yaw_.end(), Eigen::Vector3d::Zero());
  std::fill(g_start_.begin(), g_start_.end(), Eigen::Vector3d::Zero());
  std::fill(g_end_.begin(), g_end_.end(), Eigen::Vector3d::Zero());
  std::fill(g_guide_.begin(), g_guide_.end(), Eigen::Vector3d::Zero());
  std::fill(g_waypoints_.begin(), g_waypoints_.end(), Eigen::Vector3d::Zero());
  std::fill(g_view_.begin(), g_view_.end(), Eigen::Vector3d::Zero());
  std::fill(g_time_.begin(), g_time_.end(), Eigen::Vector3d::Zero());
  std::fill(g_parallax_.begin(), g_parallax_.end(), Eigen::Vector3d::Zero());
  std::fill(g_frontier_visibility_pos_.begin(), g_frontier_visibility_pos_.end(), Eigen::Vector3d::Zero());
  std::fill(g_yaw_covisibility_.begin(), g_yaw_covisibility_.end(), Eigen::Vector3d::Zero());
  std::fill(g_frontier_visibility_yaw_.begin(), g_frontier_visibility_yaw_.end(), Eigen::Vector3d::Zero());
}

void BsplineOptimizer::optimize(
    Eigen::MatrixXd& points, double& dt, const int& cost_function, const int& max_num_id, const int& max_time_id) {

  if (start_state_.empty()) {
    ROS_ERROR("Initial state undefined!");
    return;
  }

  control_points_ = points;
  knot_span_ = dt;
  // max_num_id_ = max_num_id;
  // max_time_id_ = max_time_id;
  setCostFunction(cost_function);

  // Set necessary data and flag
  dim_ = control_points_.cols();
  if (dim_ == 1)
    order_ = 3;
  else
    order_ = bspline_degree_;
  point_num_ = control_points_.rows();
  optimize_time_ = cost_function_ & MINTIME;
  variable_num_ = optimize_time_ ? dim_ * point_num_ + 1 : dim_ * point_num_;
  if (variable_num_ <= 0) {
    ROS_ERROR("Empty varibale to optimization solver.");
    return;
  }

  pt_dist_ = 0.0;
  for (int i = 0; i < control_points_.rows() - 1; ++i) {
    pt_dist_ += (control_points_.row(i + 1) - control_points_.row(i)).norm();
  }
  // 得到控制点的平均距离
  pt_dist_ /= point_num_;

  iter_num_ = 0;
  min_cost_ = std::numeric_limits<double>::max();

  resetCostAndGrad();

  comb_time = 0.0;

  optimize();

  points = control_points_;
  dt = knot_span_;
  start_state_.clear();
  start_con_index_.clear();
  end_con_index_.clear();
  time_lb_ = -1;
}

void BsplineOptimizer::optimize() {
  // Optimize all control points and maybe knot span dt
  // Use NLopt solver

  // Step1：初始化NLopt非线性优化器
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
  opt.set_min_objective(BsplineOptimizer::costFunction, this);
  opt.set_xtol_rel(1e-4);

  // opt.set_maxeval(max_iteration_num_[max_num_id_]);
  // opt.set_maxtime(max_iteration_time_[max_time_id_]);
  // opt.set_xtol_rel(1e-5);

  // Set axis aligned bounding box for optimization

  // Step2：把control_points_的数据搬运到q中，记得考虑上下限
  Eigen::Vector3d bmin, bmax;
  edt_environment_->sdf_map_->getBox(bmin, bmax);
  for (int k = 0; k < 3; ++k) {
    bmin[k] += 0.1;
    bmax[k] -= 0.1;
  }

  vector<double> q(variable_num_);

  // Variables for control points
  for (int i = 0; i < point_num_; ++i) {
    for (int j = 0; j < dim_; ++j) {
      double cij = control_points_(i, j);
      if (dim_ != 1) cij = max(min(cij, bmax[j % 3]), bmin[j % 3]);
      q[dim_ * i + j] = cij;
    }
  }

  // Variables for knot span
  if (optimize_time_) q[variable_num_ - 1] = knot_span_;

  // Step3：为NLopt优化器设置优化的上下限
  if (dim_ != 1) {
    vector<double> lb(variable_num_), ub(variable_num_);
    const double bound = 10.0;
    for (int i = 0; i < 3 * point_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
      lb[i] = max(lb[i], bmin[i % 3]);
      ub[i] = min(ub[i], bmax[i % 3]);
    }
    if (optimize_time_) {
      lb[variable_num_ - 1] = 0.0;
      ub[variable_num_ - 1] = 5.0;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  // Step4：正式进行优化
  auto t1 = ros::Time::now();
  try {
    double final_cost;
    nlopt::result result = opt.optimize(q, final_cost);
    for (int i = 0; i < point_num_; ++i)
      for (int j = 0; j < dim_; ++j) control_points_(i, j) = best_variable_[dim_ * i + j];
    if (optimize_time_) knot_span_ = best_variable_[variable_num_ - 1];

    // if (cost_function_ & MINTIME) {
    //   std::cout << "Iter num: " << iter_num_ << ", time: " << (ros::Time::now() - t1).toSec() << ", point num: " << point_num_
    //             << ", comb time: " << comb_time << std::endl;
    // }
    issuccess = true;
  }

  catch (std::exception& e) {
    cout << e.what() << endl;
    ROS_ERROR("[BsplineOptimizer::optimize] Optimized_fail!!!!!!!");
    issuccess = false;
  }
}

void BsplineOptimizer::calcSmoothnessCost(const vector<Vector3d>& q, double& cost, vector<Vector3d>& gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);
  Eigen::Vector3d jerk, temp_j;

  for (size_t i = 0; i < q.size() - 3; i++) {
    /* evaluate jerk */
    // 3-rd order derivative = 1/(ts)^3*(q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i])

    // Test jerk cost
    Eigen::Vector3d ji = (q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i]) / pt_dist_;
    double cost_this = ji.squaredNorm();
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::SMOOTHNESS, cost_this);
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_YAW_OPT, COST_TYPE::SMOOTHNESS, cost_this);
    cost += cost_this;
    temp_j = 2 * ji / pt_dist_;

    gradient_q[i + 0] += -temp_j;
    gradient_q[i + 1] += 3.0 * temp_j;
    gradient_q[i + 2] += -3.0 * temp_j;
    gradient_q[i + 3] += temp_j;
  }
}

void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double dist;
  Eigen::Vector3d dist_grad;
  for (size_t i = 0; i < q.size(); i++) {
    edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);
    if (dist_grad.norm() > 1e-4) dist_grad.normalize();

    if (dist < dist0_) {
      double cost_this = pow(dist - dist0_, 2);
      stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::DISTANCE, cost_this);
      cost += cost_this;
      gradient_q[i] += 2.0 * (dist - dist0_) * dist_grad;
    } else
      stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::DISTANCE, 0.0);
  }
}

void BsplineOptimizer::calcFeasibilityCost(
    const vector<Vector3d>& q, const double& dt, double& cost, vector<Vector3d>& gradient_q, double& gt) {

  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);
  gt = 0.0;

  // Abbreviation of params
  const double dt_inv = 1 / dt;
  const double dt_inv2 = dt_inv * dt_inv;
  for (size_t i = 0; i < q.size() - 1; ++i) {
    // Control point of velocity
    Eigen::Vector3d vi = (q[i + 1] - q[i]) * dt_inv;
    for (int k = 0; k < 3; ++k) {
      // Calculate cost for each axis
      double vd = fabs(vi[k]) - max_vel_;
      if (vd > 0.0) {
        double cost_this = pow(vd, 2);
        stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::FEASIBILITY_VEL, cost_this);
        cost += cost_this;
        double sign = vi[k] > 0 ? 1.0 : -1.0;
        double tmp = 2 * vd * sign * dt_inv;
        gradient_q[i][k] += -tmp;
        gradient_q[i + 1][k] += tmp;
        if (optimize_time_) gt += tmp * (-vi[k]);
      } else
        stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::FEASIBILITY_VEL, 0.0);
    }
  }

  // Acc feasibility cost
  for (size_t i = 0; i < q.size() - 2; ++i) {
    Eigen::Vector3d ai = (q[i + 2] - 2 * q[i + 1] + q[i]) * dt_inv2;
    for (int k = 0; k < 3; ++k) {
      double ad = fabs(ai[k]) - max_acc_;
      if (ad > 0.0) {
        double cost_this = pow(ad, 2);
        stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::FEASIBILITY_ACC, cost_this);
        cost += cost_this;
        double sign = ai[k] > 0 ? 1.0 : -1.0;
        double tmp = 2 * ad * sign * dt_inv2;
        gradient_q[i][k] += tmp;
        gradient_q[i + 1][k] += -2 * tmp;
        gradient_q[i + 2][k] += tmp;
        if (optimize_time_) gt += tmp * ai[k] * (-2) * dt;
      } else
        stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::FEASIBILITY_ACC, 0.0);
    }
  }
}

void BsplineOptimizer::calcFeasibilityCostYaw(
    const vector<Vector3d>& q, const double& dt, double& cost, vector<Vector3d>& gradient_q, double& gt) {

  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);
  gt = 0.0;

  // Abbreviation of params
  const double dt_inv = 1 / dt;

  // Control point of velocity
  for (size_t i = 0; i < q.size() - 1; ++i) {
    Vector3d vi = (q[i + 1] - q[i]) * dt_inv;

    double max_yaw_rate = Utils::getGlobalParam().max_yaw_rate_;
    double vd = fabs(vi[0]) - max_yaw_rate;
    if (vd > 0.0) {
      double cost_this = pow(vd, 2);
      stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_YAW_OPT, COST_TYPE::FEASIBILITY_YAW, cost_this);
      cost += cost_this;
      double sign = vi[0] > 0 ? 1.0 : -1.0;
      double tmp = 2 * vd * sign * dt_inv;
      gradient_q[i][0] -= tmp;
      gradient_q[i + 1][0] += tmp;
      if (optimize_time_) gt += tmp * (-vi[0]);
    } else
      stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_YAW_OPT, COST_TYPE::FEASIBILITY_YAW, 0);
  }
}

void BsplineOptimizer::calcStartCost(
    const vector<Vector3d>& q, const double& dt, double& cost, vector<Vector3d>& gradient_q, double& gt) {

  if (start_con_index_.size() != 3) ROS_ERROR("Start state constraint is not set!");

  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  // std::fill(gradient_q.begin(), gradient_q.end(), zero);
  for (int i = 0; i < 3; ++i) gradient_q[i] = zero;
  gt = 0.0;

  Eigen::Vector3d q1, q2, q3, dq;
  q1 = q[0];
  q2 = q[1];
  q3 = q[2];

  // Start position
  if (start_con_index_[0]) {
    if (start_state_.size() < 1) ROS_ERROR_STREAM("(start pos),start state size: " << start_state_.size());

    static const double w_pos = 10.0;
    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - start_state_[0];
    double cost_this = w_pos * dq.squaredNorm();
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::START_POS, cost_this);
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_YAW_OPT, COST_TYPE::START_POS, cost_this);
    cost += cost_this;
    gradient_q[0] += w_pos * 2 * dq * (1 / 6.0);
    gradient_q[1] += w_pos * 2 * dq * (4 / 6.0);
    gradient_q[2] += w_pos * 2 * dq * (1 / 6.0);
  }

  // Start velocity
  if (start_con_index_[1]) {
    if (start_state_.size() < 2) ROS_ERROR_STREAM("(start vel),start state size: " << start_state_.size());

    dq = 1 / (2 * dt) * (q3 - q1) - start_state_[1];
    double cost_this = dq.squaredNorm();
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::START_VEL, cost_this);
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_YAW_OPT, COST_TYPE::START_VEL, cost_this);
    cost += cost_this;
    gradient_q[0] += 2 * dq * (-1.0) / (2 * dt);
    gradient_q[2] += 2 * dq * 1.0 / (2 * dt);
    if (optimize_time_) gt += dq.dot(q3 - q1) / (-dt * dt);
  }

  // Start acceleration
  if (start_con_index_[2]) {
    if (start_state_.size() < 3) ROS_ERROR_STREAM("(start acc),start state size: " << start_state_.size());

    dq = 1 / (dt * dt) * (q1 - 2 * q2 + q3) - start_state_[2];
    double cost_this = dq.squaredNorm();
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::START_ACC, cost_this);
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_YAW_OPT, COST_TYPE::START_ACC, cost_this);
    cost += cost_this;
    gradient_q[0] += 2 * dq * 1.0 / (dt * dt);
    gradient_q[1] += 2 * dq * (-2.0) / (dt * dt);
    gradient_q[2] += 2 * dq * 1.0 / (dt * dt);
    if (optimize_time_) gt += dq.dot(q1 - 2 * q2 + q3) / (-dt * dt * dt);
  }
}

void BsplineOptimizer::calcEndCost(
    const vector<Vector3d>& q, const double& dt, double& cost, vector<Vector3d>& gradient_q, double& gt) {

  if (end_con_index_.size() != 3) ROS_ERROR("End state constraint is not set!");

  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  // std::fill(gradient_q.begin(), gradient_q.end(), zero);
  for (size_t i = q.size() - 3; i < q.size(); ++i) gradient_q[i] = zero;
  gt = 0.0;

  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  // End position
  if (end_con_index_[0]) {
    if (end_state_.size() < 1) ROS_ERROR_STREAM("(end pos),end state size: " << end_state_.size());

    dq = 1 / 6.0 * (q_1 + 4 * q_2 + q_3) - end_state_[0];
    double cost_this = dq.squaredNorm();
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::END_POS, cost_this);
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_YAW_OPT, COST_TYPE::END_POS, cost_this);
    cost += cost_this;
    gradient_q[q.size() - 1] += 2 * dq * (1 / 6.0);
    gradient_q[q.size() - 2] += 2 * dq * (4 / 6.0);
    gradient_q[q.size() - 3] += 2 * dq * (1 / 6.0);
  }

  // End velocity
  if (end_con_index_[1]) {
    if (end_state_.size() < 2) ROS_ERROR_STREAM("(end vel),end state size: " << end_state_.size());

    dq = 1 / (2 * dt) * (q_1 - q_3) - end_state_[1];
    double cost_this = dq.squaredNorm();
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::END_VEL, cost_this);
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_YAW_OPT, COST_TYPE::END_VEL, cost_this);
    cost += cost_this;
    gradient_q[q.size() - 1] += 2 * dq * 1.0 / (2 * dt);
    gradient_q[q.size() - 3] += 2 * dq * (-1.0) / (2 * dt);
    if (optimize_time_) gt += dq.dot(q_1 - q_3) / (-dt * dt);
  }

  // End acceleration
  if (end_con_index_[2]) {
    if (end_state_.size() < 3) ROS_ERROR_STREAM("(end acc),end state size: " << end_state_.size());

    dq = 1 / (dt * dt) * (q_1 - 2 * q_2 + q_3) - end_state_[2];
    double cost_this = dq.squaredNorm();
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::END_ACC, cost_this);
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_YAW_OPT, COST_TYPE::END_ACC, cost_this);
    cost += cost_this;
    gradient_q[q.size() - 1] += 2 * dq * 1.0 / (dt * dt);
    gradient_q[q.size() - 2] += 2 * dq * (-2.0) / (dt * dt);
    gradient_q[q.size() - 3] += 2 * dq * 1.0 / (dt * dt);
    if (optimize_time_) gt += dq.dot(q_1 - 2 * q_2 + q_3) / (-dt * dt * dt);
  }
}

void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  Vector3d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    Vector3d waypt = waypoints_[i];
    int idx = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = (q1 + 4 * q2 + q3) / 6 - waypt;
    double cost_this = dq.squaredNorm();
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_YAW_OPT, COST_TYPE::WAYPOINTS, cost_this);
    cost += cost_this;

    gradient_q[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient_q[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient_q[idx + 2] += dq * (2.0 / 6.0);
  }
}

/* use the uniformly sampled points on a geomertic path to guide the
 * trajectory. For each control points to be optimized, it is assigned a
 * guiding point on the path and the distance between them is penalized */
void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  int end_idx = q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    Vector3d gpt = guide_pts_[i - order_];
    double cost_this = (q[i] - gpt).squaredNorm();
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::GUIDE, cost_this);
    cost += cost_this;
    gradient_q[i] += 2 * (q[i] - gpt);
  }
}

void BsplineOptimizer::calcTimeCost(const double& dt, double& cost, double& gt) {
  // Min time
  double duration = (point_num_ - order_) * dt;
  cost = duration;
  gt = double(point_num_ - order_);

  // Time lower bound
  if (time_lb_ > 0 && duration < time_lb_) {
    static const double w_lb = 10;
    double cost_this = w_lb * pow(duration - time_lb_, 2);
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::MINTIME, cost_this);
    cost += cost_this;
    gt += w_lb * 2 * (duration - time_lb_) * (point_num_ - order_);
  } else {
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::MINTIME, duration);
  }
}

// SECTION Perception Aware Optimization

void BsplineOptimizer::calcParaValueAndGradients(
    const Vector3d& v1, const Vector3d& v2, double& parallax, bool calc_grad, Vector3d& dpara_dv1, Vector3d& dpara_dv2) {

  parallax = acos(v1.dot(v2) / (v1.norm() * v2.norm()));

  // Calculate gradients dpara_dv1 and dpara_dv2
  if (!calc_grad) {
    double v1_norm_inv = 1 / v1.norm();
    double v2_norm_inv = 1 / v2.norm();

    // Use chain rule
    double u = v1.dot(v2) / (v1.norm() * v2.norm());
    double dpara_du = -1 / sqrt(1 - pow(u, 2));

    // Compute directly
    Eigen::Vector3d du_dv1 = v2_norm_inv * (v1_norm_inv * v2 - v1.dot(v2) * pow(v1_norm_inv, 3) * v1);
    Eigen::Vector3d du_dv2 = v1_norm_inv * (v2_norm_inv * v1 - v1.dot(v2) * pow(v2_norm_inv, 3) * v2);

    dpara_dv1 = dpara_du * du_dv1;
    dpara_dv2 = dpara_du * du_dv2;
  }
}

void BsplineOptimizer::calcffAngleValueAndGradients(const Vector3d& node_pos, const Vector3d& feature, const Vector3d& frontier,
    double& convisual_angle, bool calc_grad, Eigen::Vector3d& dfvb_dq) {

  Vector3d v1 = node_pos - feature;
  Vector3d v2 = node_pos - frontier;
  double v1_norm = v1.norm();
  double v2_norm = v2.norm();

  // 如果 v1 或 v2 的模长接近零，如果位置非常接近，夹角设为 0 或 π，梯度设为小值
  if (v1_norm < 1e-6 || v2_norm < 1e-6) {
    convisual_angle = (v1_norm < 1e-6 && v2_norm < 1e-6) ? 0.0 : M_PI;
    dfvb_dq = Eigen::Vector3d::Zero();
    return;
  }

  double u = v1.dot(v2) / (v1_norm * v2_norm);
  // 计算视差角
  convisual_angle = acos(u);

  if (!calc_grad) {
    return;
  }

  // 当 u 等于 1 或 -1 时，梯度设为 0
  if (fabs(u) >= 1.0) {
    dfvb_dq = Eigen::Vector3d::Zero();
    return;
  }

  // 计算 dfvb/du
  double dconvisual_du = -1 / sqrt(1 - u * u);
  // 计算 du/dP
  Vector3d du_dP =
      (v2 / (v1_norm * v2_norm)) + (v1 / (v1_norm * v2_norm)) - u * ((v1 / pow(v1_norm, 3)) + (v2 / pow(v2_norm, 3)));
  // 计算 dfvb/dq
  dfvb_dq = dconvisual_du * du_dP;
}

void BsplineOptimizer::calcParaPotentialAndGradients(
    const double parallax, const double dt, double& para_pot, double& dpot_dpara) {
  // Potential func: f(x) = 0 if x < max; f(x) = a(x-max)^2 otherwise
  double max_para_btw_knots = configPA_.max_parallax_ * configPA_.estimator_freq_ * dt;
  // ROS_INFO("max_para_btw_knots: %f", max_para_btw_knots);
  if (parallax < max_para_btw_knots) {
    para_pot = 0;
    dpot_dpara = 0;
  }

  else {
    para_pot = configPA_.pot_a_ * pow(parallax - max_para_btw_knots, 2);
    dpot_dpara = 2 * configPA_.pot_a_ * (parallax - max_para_btw_knots);
  }
}

double BsplineOptimizer::calcVCWeight(const Eigen::Vector3d& knot, const Eigen::Vector3d& f, const Eigen::Vector3d& thrust_dir) {
  double weight = 0.0;

  Eigen::Vector3d v = f - knot;
  double sin_theta = v.cross(thrust_dir).norm() / v.norm();
  // 经典垂直视场角alpha_v直接写死，跟论文对不上
  double fov_vertical = M_PI / 3.0;
  double sin_alpha = sin((M_PI - fov_vertical) / 2.0);
  weight = 1.0 / (1 + exp(-60 * (sin_theta - sin_alpha)));

  return weight;
}

void BsplineOptimizer::calcParaCostAndGradientsKnots(
    const vector<Vector3d>& q, const double dt, const vector<Vector3d>& features, double& cost, vector<Vector3d>& dcost_dq) {

  if (q.size() != 4) ROS_ERROR("Control points set should have exactly 4 points!");

  cost = 0;
  dcost_dq.clear();
  for (int i = 0; i < 4; i++) dcost_dq.push_back(Eigen::Vector3d::Zero());

  Vector3d knot_pos1 = (q[0] + 4 * q[1] + q[2]) / 6;
  Vector3d knot_pos2 = (q[1] + 4 * q[2] + q[3]) / 6;
  Vector3d knot_pos_mid = 0.5 * (knot_pos1 + knot_pos2);
  Vector3d acc1 = (q[2] - 2 * q[1] + q[0]) / pow(dt, 2);
  Vector3d thrust_dir1 = getThrustDirection(acc1);
  Vector3d acc2 = (q[3] - 2 * q[2] + q[1]) / pow(dt, 2);
  Vector3d thrust_dir2 = getThrustDirection(acc2);
  Vector3d thrust_mid = 0.5 * (thrust_dir1 + thrust_dir2);

  double total_weight = 0.0;
  for (const auto& f : features) {
    Vector3d v1 = knot_pos1 - f;
    Vector3d v2 = knot_pos2 - f;

    // 对应论文公式(13)
    // parallax就是论文里的视差角parallax angle
    double parallax;
    Vector3d dpara_dv1, dpara_dv2;
    calcParaValueAndGradients(v1, v2, parallax, true, dpara_dv1, dpara_dv2);

    // 对应论文公式(15)
    // 经典超过一定阈值就给惩罚
    double para_pot, dpot_dpara;
    calcParaPotentialAndGradients(parallax, dt, para_pot, dpot_dpara);

    // 对应论文公式(14)里出现的权重
    double w = calcVCWeight(knot_pos_mid, f, thrust_mid);

    // 这什么阴间加权方式
    cost = (cost * total_weight + para_pot * w) / (total_weight + w);

    vector<Vector3d> dpot_dq_cur;
    dpot_dq_cur.push_back(w * dpot_dpara * dpara_dv1 / 6);
    dpot_dq_cur.push_back(w * dpot_dpara * (dpara_dv1 * 4 / 6 + dpara_dv2 / 6));
    dpot_dq_cur.push_back(w * dpot_dpara * (dpara_dv1 / 6 + dpara_dv2 * 4 / 6));
    dpot_dq_cur.push_back(w * dpot_dpara * dpara_dv2 / 6);

    for (int i = 0; i < 4; i++) dcost_dq[i] = (dcost_dq[i] * total_weight + dpot_dq_cur[i]) / (total_weight + w);

    total_weight += w;
  }
}

void BsplineOptimizer::calcVVValueAndGradients(
    const Vector3d& a, const Vector3d& b, double& cos_theta, bool calc_grad, Vector3d& dcos_theta_da, Vector3d& dcos_theta_db) {
  cos_theta = a.dot(b) / (a.norm() * b.norm());

  // Calculate gradients dcos_theta_da and dcos_theta_db
  if (calc_grad) {
    double a_norm_inv = 1 / a.norm();
    double b_norm_inv = 1 / b.norm();

    // Compute directly
    dcos_theta_da = b_norm_inv * (a_norm_inv * b - a.dot(b) * pow(a_norm_inv, 3) * a);
    dcos_theta_db = a_norm_inv * (b_norm_inv * a - a.dot(b) * pow(b_norm_inv, 3) * b);
  }
}

void BsplineOptimizer::calcVVPotentialAndGradients(const double cos_theta, double& cos_theta_pot, double& dpot_dcos_theta) {

  double max = cos(M_PI / 3.0);
  double min = cos(M_PI - M_PI / 3.0);

  double a = 10;
  if (cos_theta > max) {
    cos_theta_pot = a * pow(cos_theta - max, 2);
    dpot_dcos_theta = 2 * a * (cos_theta - max);
  }

  else if (cos_theta < min) {
    cos_theta_pot = a * pow(cos_theta - min, 2);
    dpot_dcos_theta = 2 * a * (cos_theta - min);
  }

  else {
    cos_theta_pot = 0;
    dpot_dcos_theta = 0;
  }
}

void BsplineOptimizer::calcVCVCostAndGradientsKnots(const vector<Vector3d>& q, const double& knot_span,
    const vector<Vector3d> features, double& cost, vector<Vector3d>& dcost_dq) {

  if (q.size() != 4) ROS_ERROR("Control points set should have exactly 4 points!");

  cost = 0.0;
  dcost_dq.clear();
  for (int i = 0; i < 4; i++) dcost_dq.push_back(Eigen::Vector3d::Zero());

  double knot_span_inv2 = 1 / pow(knot_span, 2);
  Eigen::Vector3d gravity(0, 0, -9.81);

  double total_weight = 0.0;
  for (const auto& f : features) {
    double w = 1.0;

    vector<double> cos_theta_pot_vec;
    vector<vector<Eigen::Vector3d>> dpoti_dqj_vec;

    // Calculate visibility cost and gradients for each knot
    for (int i = 0; i < 2; i++) {
      // Calculate vector a,b and their gradients
      Eigen::Vector3d acc = knot_span_inv2 * (q[i] - 2 * q[i + 1] + q[i + 2]);
      Eigen::Vector3d a = acc - gravity;  // thrust
      Eigen::Vector3d knot = (q[i] + 4 * q[i + 1] + q[i + 2]) / 6;
      Eigen::Vector3d b = f - knot;

      Eigen::Vector4d da_dq = Eigen::Vector4d::Zero();
      da_dq.segment(i, 3) << 1, -2, 1;
      da_dq.segment(i, 3) *= knot_span_inv2;

      Eigen::Vector4d db_dq = Eigen::Vector4d::Zero();
      db_dq.segment(i, 3) << -1 / 6.0, -4 / 6.0, -1 / 6.0;

      // Calculate cos_theta
      // 对应论文公式(10)
      // 与论文有出入，这里包括下面的公式(12)用来作对比的是cos值，而不是论文里的角度值
      double cos_theta;
      Eigen::Vector3d dcos_theta_da, dcos_theta_db;
      calcVVValueAndGradients(a, b, cos_theta, true, dcos_theta_da, dcos_theta_db);

      // Calculate potential cost function
      // 对应论文公式(12)，这里cos_theta_pot对应着等式左侧的g
      double cos_theta_pot, dpot_dcos_theta;
      calcVVPotentialAndGradients(cos_theta, cos_theta_pot, dpot_dcos_theta);

      cos_theta_pot_vec.push_back(cos_theta_pot);

      // Calculate gradients of potential cost
      vector<Eigen::Vector3d> dpot_dqj;
      for (int j = 0; j < 4; j++) {
        Eigen::Vector3d dpoti_dqj = w * dpot_dcos_theta * (dcos_theta_da * da_dq[j] + dcos_theta_db * db_dq[j]);
        dpot_dqj.push_back(dpoti_dqj);
      }
      dpoti_dqj_vec.push_back(dpot_dqj);
    }

    // Calculate co-visbility potential cost function and its gradient
    // f_cov(theta_1, theta_2) = (f_v(theta_1) + 1)(f_v(theta_2) + 1) - 1
    // 对应论文公式(11)
    double covisib_pot = (cos_theta_pot_vec[0] + 1) * (cos_theta_pot_vec[1] + 1) - 1;

    // 怎么又是你这种阴间加权方式
    cost = (cost * total_weight + covisib_pot * w) / (total_weight + w);

    for (int j = 0; j < 4; j++) {
      Eigen::Vector3d dcovisb_pot_dq_cur =
          dpoti_dqj_vec[0][j] * (cos_theta_pot_vec[1] + 1) + dpoti_dqj_vec[1][j] * (cos_theta_pot_vec[0] + 1);
      dcost_dq[j] = (dcost_dq[j] * total_weight + dcovisb_pot_dq_cur) / (total_weight + w);
    }

    total_weight += w;
  }
}

void BsplineOptimizer::calcPerceptionCost(const vector<Vector3d>& q, const double& dt, double& cost, vector<Vector3d>& gradient_q,
    const double ld_para, const double ld_vcv) {

  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double cost_para, cost_vcv;
  vector<Vector3d> dcost_para_dq, dcost_vcv_dq;

  for (size_t i = 0; i < q.size() - 3; ++i) {
    // For (q0, q1, q2, q3)->(knot1, knot2), calculate the parallax cost and gradient
    vector<Vector3d> q_cur;
    for (int j = 0; j < 4; j++) q_cur.push_back(q[i + j]);

    Vector3d knot_mid = ((q_cur[0] + 4 * q_cur[1] + q_cur[2]) + (q_cur[1] + 4 * q_cur[2] + q_cur[3])) / 12.0;
    vector<Vector3d> features;
    feature_map_->getFeatures(knot_mid, features);

    // 对应论文第5章B节计算视差cost部分
    calcParaCostAndGradientsKnots(q_cur, dt, features, cost_para, dcost_para_dq);
    // 对应论文第5章B节计算垂直共视性(vertical covisibility)cost部分
    calcVCVCostAndGradientsKnots(q_cur, dt, features, cost_vcv, dcost_vcv_dq);

    // cost += ld_para * cost_para;
    // cost += ld_vcv * cost_vcv;

    double cost_this = ld_para * cost_para + ld_vcv * cost_vcv;
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::APACE_POS, cost_this);
    cost += cost_this;

    for (int j = 0; j < 4; j++) {
      gradient_q[i + j] += ld_para * dcost_para_dq[j];
      gradient_q[i + j] += ld_vcv * dcost_vcv_dq[j];
    }
  }
  // cout << "BsplineOptimizer::calcPerceptionCost: " << cost << endl;
}

void BsplineOptimizer::calcFVBCostAndGradientsKnots(const vector<Vector3d>& q, const Vector3d& knots_pos,
    const vector<Vector3d>& features, double& cost, vector<Vector3d>& dcost_dq) {
  // 初始化以及一些可行性检查
  cost = 0;
  dcost_dq.clear();
  for (int i = 0; i < 3; i++) dcost_dq.push_back(Eigen::Vector3d::Zero());

  if (frontier_centre_.norm() < 1e-5) {
    ROS_ERROR("[BsplineOptimizer::calcFVBCostAndGradientsKnots] NO Frontiers!!!!");
    return;
  }
  if (q.size() != 3) {
    ROS_ERROR("[BsplineOptimizer::calcFVBCostAndGradientsKnots] Control points set should have exactly 3 points!");
    return;
  }
  if (features.empty()) {
    ROS_ERROR("[BsplineOptimizer::calcFVBCostAndGradientsKnots] This knot start with no feature!!!!");
    return;
  }

  double total_weight = 0.0;
  double frontier_visual_feature_num = 0;
  Vector3d frontier_visual_feature_gradient = Eigen::Vector3d::Zero();
  for (const auto& feature : features) {

    // 计算 convisual_angle：当前控制点看到特征点和前沿点的夹角
    double convisual_angle;
    Vector3d convisual_angle_gradient;
    calcffAngleValueAndGradients(knots_pos, feature, frontier_centre_, convisual_angle, true, convisual_angle_gradient);
    // 计算代价函数和梯度
    double is_feature_good, is_feature_good_gradient;
    double k1 = 10;
    is_feature_good =
        1.0 / (1.0 + std::exp(-k1 * (std::cos(convisual_angle) - std::cos(configPA_.max_feature_and_frontier_convisual_angle_))));
    is_feature_good_gradient = is_feature_good * (1.0 - is_feature_good) * k1 * std::sin(convisual_angle);

    // 计算每个frontier可视的feature的数量
    frontier_visual_feature_num += is_feature_good;
    frontier_visual_feature_gradient += is_feature_good_gradient * convisual_angle_gradient;
  }
  //====这里应该套个硬约束的壳，但是现在场景太好了，套壳估计没啥影响，先写一个简单的线性函数
  double final_cost = -1 * frontier_visual_feature_num / features.size();
  Vector3d dcost_dkont = -1 * frontier_visual_feature_gradient / features.size();
  //====
  cost += final_cost;
  dcost_dq[0] += dcost_dkont * (1.0 / 6.0);
  dcost_dq[1] += dcost_dkont * (4.0 / 6.0);
  dcost_dq[2] += dcost_dkont * (1.0 / 6.0);
}

void BsplineOptimizer::calcFVBCostAndGradientsKnots(
    const vector<Vector3d>& q, const Vector3d& knots_pos, double& cost, vector<Vector3d>& dcost_dq) {
  // 初始化以及一些可行性检查
  cost = 0;
  dcost_dq.clear();
  for (int i = 0; i < 3; i++) dcost_dq.push_back(Eigen::Vector3d::Zero());

  if (view_point_yaw_ == 2 * M_PI) {
    // 有点丑陋的验证viewpoint有没有被初始化的方法...
    ROS_ERROR("[BsplineOptimizer::calcFVBCostAndGradientsKnots] NO Frontiers!!!!");
    return;
  }

  if (q.size() != 3) {
    ROS_ERROR("[BsplineOptimizer::calcFVBCostAndGradientsKnots] Control points set should have exactly 3 points!");
    return;
  }

  Eigen::Vector3d diff = view_point_pos_ - knots_pos;
  double diff_norm = diff.norm();
  if (diff_norm < 1e-6) return;
  // 求解梯度
  Eigen::Vector3d dcost_dknot(Eigen::Vector3d::Zero());

  Eigen::Vector3d diff_normalized = diff / diff_norm;
  Eigen::Vector3d dir(std::cos(view_point_yaw_), std::sin(view_point_yaw_), 0);
  // cost = sqrt(pow(diff_norm, 2) - diff.dot(dir));
  // dcost_dkont = (diff - 0.5 * dir) / cost;
  double cos_theta = diff_normalized.dot(dir);

  cost = 1.0 - cos_theta;
  if (std::abs(cos_theta) < 1.0) {
    dcost_dknot = -(cos_theta * diff_normalized - dir) / diff_norm;
  } else {
    // 当 cos_theta 接近 1 或 -1 时，梯度设为零
    dcost_dknot = Eigen::Vector3d::Zero();
  }
  dcost_dq[0] += dcost_dknot * (1.0 / 6.0);
  dcost_dq[1] += dcost_dknot * (4.0 / 6.0);
  dcost_dq[2] += dcost_dknot * (1.0 / 6.0);
}
void BsplineOptimizer::calcEUAGoefficient(const vector<Vector3d>& q, const double& knot_span, vector<double>& coefficient,
    const bool& use_grad, vector<vector<Eigen::Vector3d>>& dcoefficient_dq) {
  coefficient.clear();
  dcoefficient_dq.clear();

  double knot_span_inv2 = 1 / pow(knot_span, 2);
  Eigen::Vector3d gravity(0, 0, -9.81);
  double fov_vertical = camera_param_->fov_vertical * M_PI / 180.0;
  double fov_horizontal = camera_param_->fov_horizontal * M_PI / 180.0;

  Eigen::Vector3d acc = knot_span_inv2 * (q[0] - 2 * q[1] + q[2]);
  Eigen::Vector3d a = acc - gravity;  // thrust
  Eigen::Vector3d knot = (q[0] + 4 * q[1] + q[2]) / 6;
  Eigen::Vector3d da_dq(1, -2, 1);
  da_dq *= knot_span_inv2;
  Eigen::Vector3d db_dq(-1 / 6.0, -4 / 6.0, -1 / 6.0);
  vector<Eigen::Vector3d> zero_vec_q(3, Eigen::Vector3d::Zero());

  // 首先计算垂直可见性===================================================================================
  vector<double> sin_theta_pot_vec;
  vector<vector<Eigen::Vector3d>> dpot_dqj_vec;
  sin_theta_pot_vec.reserve(frontier_cells_.size());
  dpot_dqj_vec.reserve(frontier_cells_.size());
  for (size_t i = 0; i < frontier_cells_.size(); i++) {
    Eigen::Vector3d b = frontier_cells_[i] - knot;

    // 计算sin_theta
    double sin_theta;
    Eigen::Vector3d dsin_theta_da, dsin_theta_db;

    double a_norm = a.norm();
    double b_norm = b.norm();
    double a_norm_inv = 1 / a_norm;
    double b_norm_inv = 1 / b_norm;
    Eigen::Vector3d cross_ab = a.cross(b);
    double cross_ab_norm = cross_ab.norm();

    if (a_norm == 0 || b_norm == 0 || cross_ab_norm == 0) {
      ROS_WARN("[BsplineOptimizer] Vector magnitude is zero, cannot compute gradients.");
      sin_theta_pot_vec.push_back(0);
      vector<Eigen::Vector3d> grad_zero(3, Eigen::Vector3d::Zero());
      dpot_dqj_vec.push_back(grad_zero);
      continue;
    }

    sin_theta = cross_ab_norm * a_norm_inv * b_norm_inv;

    // 计算 dsin_theta/da 和 dsin_theta/db
    dsin_theta_da = (b_norm_inv / cross_ab_norm) * cross_ab.cross(b) - (sin_theta / a_norm) * a;
    dsin_theta_db = (a_norm_inv / cross_ab_norm) * a.cross(cross_ab) - (sin_theta / b_norm) * b;

    // 对应论文公式(4)
    double sin_theta_pot, dpot_dsin_theta;
    double k1 = 60;
    double sin_alpha = sin((M_PI - fov_vertical) / 2.0);
    sin_theta_pot = 1.0 / (1 + exp(-k1 * (sin_theta - sin_alpha)));
    dpot_dsin_theta = k1 * sin_theta_pot * (1 - sin_theta_pot);

    // Calculate gradients of potential cost
    vector<Eigen::Vector3d> dpot_dqj;
    dpot_dqj.reserve(3);
    for (int j = 0; j < 3; j++) {
      Eigen::Vector3d dpoti_dqj = dpot_dsin_theta * (dsin_theta_da * da_dq[j] + dsin_theta_db * db_dq[j]);
      dpot_dqj.push_back(dpoti_dqj);
    }

    sin_theta_pot_vec.push_back(sin_theta_pot);
    dpot_dqj_vec.push_back(dpot_dqj);
  }
  // stepping_debug_->getCloudForVisualization(DEBUG_TYPE::SHOW_VERVIS, q, frontier_cells_, sin_theta_pot_vec, dpot_dqj_vec);
  // stepping_debug_->calldebug(DEBUG_TYPE::SHOW_VERVIS);
  // std::fill(sin_theta_pot_vec.begin(), sin_theta_pot_vec.end(), 1.0);
  std::fill(dpot_dqj_vec.begin(), dpot_dqj_vec.end(), zero_vec_q);
  //==================================================================================================

  // 用viewpoint和frontier_Cell的连线近似为cell所在平面====================================================
  vector<double> cos_alpha_coe_vec;
  vector<vector<Eigen::Vector3d>> dcoe_dqj_vec;
  cos_alpha_coe_vec.reserve(frontier_cells_.size());
  dcoe_dqj_vec.reserve(frontier_cells_.size());
  for (size_t i = 0; i < frontier_cells_.size(); i++) {
    Eigen::Vector3d knot2fc = frontier_cells_[i] - knot;
    Eigen::Vector3d vp2fc = frontier_cells_[i] - view_point_pos_;

    double cos_alpha;
    Eigen::Vector3d dcos_alpha_dknot;
    cos_alpha = knot2fc.dot(vp2fc) / (knot2fc.norm() * vp2fc.norm());
    Eigen::Vector3d dcos_alpha_dknot2fc =
        (vp2fc / knot2fc.norm() - knot2fc.dot(vp2fc) / pow(knot2fc.norm(), 3) * knot2fc) / vp2fc.norm();
    dcos_alpha_dknot = -dcos_alpha_dknot2fc;

    double cos_alpha_L, dcos_alpha_Ldcos_alpha;
    Eigen::Vector3d dcos_alpha_Ldknot;
    double mu = 0.1;
    if (cos_alpha <= 0) {
      cos_alpha_L = 0;
      dcos_alpha_Ldcos_alpha = 0;
    } else if (cos_alpha <= mu) {
      double factor = (mu - cos_alpha / 2) * (cos_alpha / mu) * (cos_alpha / mu);
      cos_alpha_L = factor * (cos_alpha / mu);
      dcos_alpha_Ldcos_alpha =
          (3 * (mu - cos_alpha / 2) * cos_alpha * cos_alpha) / (mu * mu * mu) - 0.5 * (cos_alpha / mu) * (cos_alpha / mu);
    } else {  // cos_alpha > mu
      cos_alpha_L = cos_alpha - mu / 2;
      dcos_alpha_Ldcos_alpha = 1;
    }

    dcos_alpha_Ldknot = dcos_alpha_Ldcos_alpha * dcos_alpha_dknot;

    vector<Eigen::Vector3d> dcoe_dqj;
    dcoe_dqj.resize(3);
    dcoe_dqj[0] = dcos_alpha_Ldknot * (1.0 / 6.0);
    dcoe_dqj[1] = dcos_alpha_Ldknot * (4.0 / 6.0);
    dcoe_dqj[2] = dcos_alpha_Ldknot * (1.0 / 6.0);

    cos_alpha_coe_vec.push_back(cos_alpha_L);
    dcoe_dqj_vec.push_back(dcoe_dqj);
  }
  // stepping_debug_->getCloudForVisualization(DEBUG_TYPE::SHOW_VERVIS, q, frontier_cells_, cos_alpha_coe_vec, dcoe_dqj_vec);
  // stepping_debug_->calldebug(DEBUG_TYPE::SHOW_VERVIS);
  // std::fill(cos_alpha_coe_vec.begin(), cos_alpha_coe_vec.end(), 1.0);
  // std::fill(dcoe_dqj_vec.begin(), dcoe_dqj_vec.end(), zero_vec_q);
  //==================================================================================================

  for (size_t i = 0; i < frontier_cells_.size(); i++) {
    double cost_each_goe = sin_theta_pot_vec[i] * cos_alpha_coe_vec[i];
    coefficient.push_back(cost_each_goe);
    std::vector<Eigen::Vector3d> dcost_each_goe_dqj(3);
    for (int j = 0; j < 3; ++j)
      dcost_each_goe_dqj[j] = dpot_dqj_vec[i][j] * cos_alpha_coe_vec[i] + sin_theta_pot_vec[i] * dcoe_dqj_vec[i][j];
    dcoefficient_dq.push_back(dcost_each_goe_dqj);
  }

  if (!use_grad) std::fill(dcoefficient_dq.begin(), dcoefficient_dq.end(), zero_vec_q);
}

void BsplineOptimizer::calcEUACostAndGradientsKnots(
    const vector<Vector3d>& q, const double& knot_span, double& cost, vector<Vector3d>& dcost_dq) {
  // 初始化以及一些可行性检查
  cost = 0;
  dcost_dq.clear();
  for (int i = 0; i < 3; i++) dcost_dq.push_back(Eigen::Vector3d::Zero());

  if (view_point_yaw_ == 2 * M_PI) {
    // 有点丑陋的验证viewpoint有没有被初始化的方法...
    ROS_ERROR("[BsplineOptimizer::calcEUACostAndGradientsKnots] NO Viewpoint!!!!");
    return;
  }

  if (frontier_cells_.size() == 0) {
    ROS_ERROR("[BsplineOptimizer::calcEUACostAndGradientsKnots] NO Frontiers!!!!");
    return;
  }

  if (q.size() != 3) {
    ROS_ERROR("[BsplineOptimizer::calcEUACostAndGradientsKnots] Control points set should have exactly 3 points!");
    return;
  }
  vector<double> coefficient;
  vector<vector<Eigen::Vector3d>> dcoefficient_dq;
  // 计算系数
  calcEUAGoefficient(q, knot_span, coefficient, true, dcoefficient_dq);
  // 扩张多少============================================================================================
  Eigen::Vector3d knot = (q[0] + 4 * q[1] + q[2]) / 6;
  vector<double> explan_scale_vec;
  vector<vector<Eigen::Vector3d>> dexplan_scale_dqj_vec;
  explan_scale_vec.reserve(frontier_cells_.size());
  dexplan_scale_dqj_vec.reserve(frontier_cells_.size());
  double max_length = camera_param_->frontier_visual_max;
  for (size_t i = 0; i < frontier_cells_.size(); i++) {
    auto& fc = frontier_cells_[i];
    double length_known = (fc - knot).norm();
    double explan_scale = length_known / max_length;
    Eigen::Vector3d dlength_known_dknot = (knot - fc) / length_known;
    Eigen::Vector3d dexplan_scale_dknot = dlength_known_dknot / max_length;

    vector<Eigen::Vector3d> des_dqj;
    des_dqj.resize(3);
    des_dqj[0] = dexplan_scale_dknot * (1.0 / 6.0);
    des_dqj[1] = dexplan_scale_dknot * (4.0 / 6.0);
    des_dqj[2] = dexplan_scale_dknot * (1.0 / 6.0);

    explan_scale_vec.push_back(explan_scale);
    dexplan_scale_dqj_vec.push_back(des_dqj);
  }

  // stepping_debug_->getCloudForVisualization(DEBUG_TYPE::SHOW_VERVIS, q, frontier_cells_, explan_scale_vec,
  // dexplan_scale_dqj_vec); stepping_debug_->calldebug(DEBUG_TYPE::SHOW_VERVIS);
  //====================================================================================================

  // 融合每个frontier的cost 和 grad
  vector<double> cost_each_fc_vec;
  vector<vector<Eigen::Vector3d>> dcost_each_fc_dqj_vec;
  for (size_t i = 0; i < frontier_cells_.size(); i++) {
    double cost_each_fc = coefficient[i] * explan_scale_vec[i];
    cost_each_fc_vec.push_back(cost_each_fc);
    std::vector<Eigen::Vector3d> dcost_each_fc_dqj(3);
    for (int j = 0; j < 3; ++j)
      dcost_each_fc_dqj[j] = dcoefficient_dq[i][j] * explan_scale_vec[i] + coefficient[i] * dexplan_scale_dqj_vec[i][j];
    dcost_each_fc_dqj_vec.push_back(dcost_each_fc_dqj);

    cost += cost_each_fc;
    for (int j = 0; j < 3; ++j) dcost_dq[j] += dcost_each_fc_dqj[j];
  }
  stepping_debug_->getCloudForVisualization(DEBUG_TYPE::SHOW_VERVIS, q, frontier_cells_, cost_each_fc_vec, dcost_each_fc_dqj_vec);
  stepping_debug_->calldebug(DEBUG_TYPE::SHOW_VERVIS);
}

// 原本考虑的太复杂，更换方法
//  void BsplineOptimizer::calcFVBCost(const vector<Vector3d>& q, double& cost, vector<Vector3d>& gradient_q) {
//    ros::Time start_time = ros::Time::now();

//   cost = 0.0;
//   Vector3d zero(0, 0, 0);
//   std::fill(gradient_q.begin(), gradient_q.end(), zero);

//   double cost_fvb;
//   vector<Vector3d> dcost_dfvb_q;
//   // cout << "[BsplineOptimizer::calcFrontierVisbilityCost] Begin!: " << endl;

//   for (size_t i = 0; i < q.size() - 2; ++i) {
//     // 获得q_cur和knot_pos，用于对这个knot进行优化
//     vector<Vector3d> q_cur;
//     for (int j = 0; j < 3; j++) q_cur.push_back(q[i + j]);
//     Vector3d knot_pos = (q[i + 0] + 4 * q[i + 1] + q[i + 2]) / 6;
//     // 计算这个knot的可视特征点（一个球形范围）
//     vector<Vector3d> features;
//     feature_map_->getFeatures(knot_pos, features);  // 得到feature
//     // 计算每个knot的cost和gradient
//     calcFVBCostAndGradientsKnots(q_cur, knot_pos, features, cost_fvb, dcost_dfvb_q);
//     // 累加cos和gradient到每一个控制点上
//     cost += cost_fvb;
//     for (int j = 0; j < 3; j++) gradient_q[i + j] += dcost_dfvb_q[j];
//   }
//   // cout << "BsplineOptimizer::calcFVBCost: " << cost << endl;
// }

void BsplineOptimizer::calcFVBCost(const vector<Vector3d>& q, double& cost, vector<Vector3d>& gradient_q) {
  ros::Time start_time = ros::Time::now();

  cost = 0.0;
  Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double cost_fvb;
  vector<Vector3d> dcost_dfvb_q;
  // cout << "[BsplineOptimizer::calcFrontierVisbilityCost] Begin!: " << endl;
  for (size_t i = 0; i < q.size() - 2; ++i) {
    // 获得q_cur和knot_pos，用于对这个knot进行优化
    vector<Vector3d> q_cur;
    for (int j = 0; j < 3; j++) q_cur.push_back(q[i + j]);
    Vector3d knot_pos = (q[i + 0] + 4 * q[i + 1] + q[i + 2]) / 6;
    // 计算每个knot的cost和gradient
    calcFVBCostAndGradientsKnots(q_cur, knot_pos, cost_fvb, dcost_dfvb_q);
    // cout << "cost_fvb: " << cost_fvb << endl;
    // 累加cos和gradient到每一个控制点上
    cost += cost_fvb;
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::FRONTIERVIS_POS, cost_fvb);

    for (int j = 0; j < 3; j++) gradient_q[i + j] += dcost_dfvb_q[j];
  }
}

void BsplineOptimizer::calcEUACost(const vector<Vector3d>& q, const double& dt, double& cost, vector<Vector3d>& gradient_q) {
  ros::Time start_time = ros::Time::now();

  cost = 0.0;
  Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double cost_eua;
  vector<Vector3d> dcost_deua_q;
  // cout << "[BsplineOptimizer::calcFrontierVisbilityCost] Begin!: " << endl;
  for (size_t i = 0; i < q.size() - 2; ++i) {
    // 获得q_cur和knot_pos，用于对这个knot进行优化
    vector<Vector3d> q_cur;
    for (int j = 0; j < 3; j++) q_cur.push_back(q[i + j]);
    calcEUACostAndGradientsKnots(q_cur, dt, cost_eua, dcost_deua_q);
    // cout << "cost_eua: " << cost_eua << endl;
    // 累加cos和gradient到每一个控制点上
    cost += cost_eua;
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_POS_OPT, COST_TYPE::FRONTIERVIS_POS, cost_eua);

    for (int j = 0; j < 3; j++) gradient_q[i + j] += dcost_deua_q[j];
  }
}

// void BsplineOptimizer::calcYawCVCostAndGradientsKnots(const vector<Vector3d>& q, const vector<Vector3d>& knots_pos,
//     const vector<Vector3d>& knots_acc, const vector<Vector3d>& features, double& cost, vector<Vector3d>& dcost_dq) {

//   ROS_ASSERT(q.size() == 4);
//   ROS_ASSERT(knots_pos.size() == 2);
//   ROS_ASSERT(knots_acc.size() == 2);

//   cost = 0.0;
//   dcost_dq.clear();
//   for (int i = 0; i < 4; i++) dcost_dq.push_back(Vector3d::Zero());

//   // Step1: 通过最准确的方式计算共视特征点数量，如果达标就直接走
//   Vector3d yaw_knot_0 = (q[0] + 4 * q[1] + q[2]) / 6;
//   double yaw_0 = yaw_knot_0(0);

//   Vector3d yaw_knot_1 = (q[1] + 4 * q[2] + q[3]) / 6;
//   double yaw_1 = yaw_knot_1(1);

//   Quaterniond ori_0 = Utils::calcOrientation(yaw_0, knots_acc[0]);
//   vector<pair<int, Vector3d>> feature_0;
//   feature_map_->get_NumCloud_using_Odom(knots_pos[0], ori_0, feature_0);

//   Quaterniond ori_1 = Utils::calcOrientation(yaw_1, knots_acc[1]);
//   vector<pair<int, Vector3d>> feature_1;
//   feature_map_->get_NumCloud_using_Odom(knots_pos[1], ori_1, feature_1);

//   int commonFeatureCount = 0;
//   for (size_t i = 0; i < feature_0.size(); i++) {
//     int id_i = feature_0[i].first;
//     for (size_t j = 0; j < feature_1.size(); j++) {
//       int id_j = feature_1[j].first;
//       if (id_i == id_j) {
//         commonFeatureCount++;
//       }
//     }
//   }

//   // ROS_INFO("Debug2");
//   // cout << "commonFeatureCount: " << commonFeatureCount << endl;

//   int min_covisible_feature_num_plan = Utils::getGlobalParam().min_covisible_feature_num_plan_;
//   if (commonFeatureCount > min_covisible_feature_num_plan) return;

//   // Step2: 获取两帧待选的特征点，保证不重复地加入优化特征点集合
//   vector<pair<int, Vector3d>> canditate_feature_0;
//   feature_map_->get_More_NumCloud_using_Odom(knots_pos[0], ori_0, canditate_feature_0);

//   vector<pair<int, Vector3d>> canditate_feature_1;
//   feature_map_->get_More_NumCloud_using_Odom(knots_pos[1], ori_1, canditate_feature_1);

//   vector<Vector3d> feature_opt;
//   list<int> feature_opt_id;
//   for (size_t i = 0; i < canditate_feature_0.size(); i++) {
//     feature_opt.push_back(canditate_feature_0[i].second);
//     feature_opt_id.push_back(canditate_feature_0[i].first);
//   }

//   for (size_t j = 0; j < canditate_feature_1.size(); j++) {
//     int id = canditate_feature_1[j].first;
//     if (std::find(feature_opt_id.begin(), feature_opt_id.end(), id) == feature_opt_id.end()) {
//       feature_opt.push_back(canditate_feature_1[j].second);
//       feature_opt_id.push_back(id);
//     }
//   }

//   Vector3d gravity(0, 0, -9.81);
//   double total_weight = 0.0;
//   for (const auto& f : feature_opt) {
//     // for (const auto& f : features) {
//     double w = 1.0;
//     vector<double> v3_theta3_vec, v1v2_vec;
//     vector<vector<Vector3d>> dv3_dyaw_vec;

//     // Calculate visibility cost and gradients for each knot
//     for (int i = 0; i < 2; i++) {
//       Vector3d yaw_knot = (q[i] + 4 * q[i + 1] + q[i + 2]) / 6;
//       double yaw = yaw_knot(0);

//       // Calculate vectors n1, ny, n3 ,b and their gradients
//       Vector3d n1, ny, n3, n2, b;
//       n1 = knots_acc[i] - gravity;  // thrust
//       ny << cos(yaw), sin(yaw), 0;
//       n3 = n1.cross(ny);
//       n2 = n3.cross(n1);
//       b = f - knots_pos[i];

//       Vector3d dn3_dyaw;
//       dn3_dyaw << -n1(2) * cos(yaw), -n1(2) * sin(yaw), n1(1) * sin(yaw) + n1(0) * cos(yaw);

//       Vector4d dyaw_dq = Vector4d::Zero();
//       dyaw_dq.segment(i, 3) = Vector3d(1, 4, 1) / 6;

//       // Calculate v1 * v2
//       double sin_theta1, v1_theta1, cos_theta2, v2_theta2, v1v2;
//       double fov_vertical = M_PI / 3.0;
//       sin_theta1 = n1.cross(b).norm() / (n1.norm() * b.norm());
//       v1_theta1 = 1 / (1 + exp(-configPA_.k1_ * (sin_theta1 - sin((M_PI - fov_vertical) / 2.0))));
//       cos_theta2 = n2.dot(b) / (n2.norm() * b.norm());
//       v2_theta2 = 1 / (1 + exp(-configPA_.k2_ * cos_theta2));
//       v1v2 = v1_theta1 * v2_theta2;

//       // Calculate v3(theta3) and gradients
//       double sin_theta3, v3_theta3;
//       double fov_horizontal = M_PI / 2.0;
//       sin_theta3 = n3.cross(b).norm() / (n3.norm() * b.norm());
//       v3_theta3 = 1 / (1 + exp(-configPA_.k3_ * (sin_theta3 - sin((M_PI - fov_horizontal) / 2.0))));

//       Eigen::Vector3d dsin_theta3_dn3;
//       Eigen::Vector3d c = n3.cross(b);
//       double n3_norm, b_norm, c_norm;
//       n3_norm = n3.norm();
//       b_norm = b.norm();
//       c_norm = c.norm();
//       dsin_theta3_dn3 = (pow(-n3_norm, 2) * b.cross(c) - pow(c_norm, 2) * n3) / (pow(n3_norm, 3) * b_norm * c_norm);
//       double dv3_dsin_theta3 =
//           configPA_.k3_ * exp(-configPA_.k3_ * (sin_theta3 - sin((M_PI - fov_horizontal) / 2.0))) * pow(v3_theta3, 2);

//       // Combine gradients using chain rule
//       double dv3_dyaw = dv3_dsin_theta3 * dsin_theta3_dn3.dot(dn3_dyaw);

//       // Store results
//       v1v2_vec.push_back(v1v2);
//       v3_theta3_vec.push_back(v3_theta3);
//       vector<Eigen::Vector3d> dv3_dyaw_i;
//       for (int j = 0; j < 4; j++) {
//         Eigen::Vector3d dv3_dqj = Eigen::Vector3d(dv3_dyaw, 0, 0) * dyaw_dq[j];
//         dv3_dyaw_i.push_back(dv3_dqj);
//       }
//       dv3_dyaw_vec.push_back(dv3_dyaw_i);
//     }

//     // Calculate co-visbility potential cost function and its gradient
//     double pot_cost;
//     vector<Vector3d> dcovisb_pot_dq_cur;

//     // double covisibility_cost = (v1v2_vec[0] * v1v2_vec[1] * v3_theta3_vec[0] * v3_theta3_vec[1]);

//     // if (covisibility_cost > 10) {
//     //   ROS_WARN("covisibility_cost: %f", covisibility_cost);

//     //   pot_cost = 0.0;
//     //   for (int j = 0; j < 4; j++) {
//     //     dcovisb_pot_dq_cur.emplace_back(Vector3d::Zero());
//     //   }
//     // }

//     // else {
//     //   pot_cost = pow(covisibility_cost - 10, 2);
//     //   for (int j = 0; j < 4; j++) {
//     //     dcovisb_pot_dq_cur.emplace_back(2 * (covisibility_cost - 10) * v1v2_vec[0] * v1v2_vec[1] *
//     //                                     (dv3_dyaw_vec[0][j] * v3_theta3_vec[1] + v3_theta3_vec[0] * dv3_dyaw_vec[1][j]));
//     //   }
//     // }

//     double covisibility_cost = -(v1v2_vec[0] * v1v2_vec[1] * v3_theta3_vec[0] * v3_theta3_vec[1]);

//     pot_cost = covisibility_cost;
//     for (int j = 0; j < 4; j++) {
//       dcovisb_pot_dq_cur.emplace_back(
//           -v1v2_vec[0] * v1v2_vec[1] * (dv3_dyaw_vec[0][j] * v3_theta3_vec[1] + v3_theta3_vec[0] * dv3_dyaw_vec[1][j]));
//     }

//     cost = (cost * total_weight + pot_cost * w) / (total_weight + w);

//     for (int j = 0; j < 4; j++) {
//       // Vector3d dcovisb_pot_dq_cur =
//       //     -v1v2_vec[0] * v1v2_vec[1] * (dv3_dyaw_vec[0][j] * v3_theta3_vec[1] + v3_theta3_vec[0] * dv3_dyaw_vec[1][j]);

//       dcost_dq[j] = (dcost_dq[j] * total_weight + dcovisb_pot_dq_cur[j] * w) / (total_weight + w);
//     }

//     total_weight += w;
//   }
// }

void BsplineOptimizer::calcYawCVCostAndGradientsKnots(const vector<Vector3d>& q, const vector<Vector3d>& knots_pos,
    const vector<Vector3d>& knots_acc, const vector<Vector3d>& features, double& cost, vector<Vector3d>& dcost_dq) {

  ROS_ASSERT(q.size() == 4);
  ROS_ASSERT(knots_pos.size() == 2);
  ROS_ASSERT(knots_acc.size() == 2);

  cost = 0.0;
  dcost_dq.clear();
  for (int i = 0; i < 4; i++) dcost_dq.push_back(Vector3d::Zero());

  Vector3d gravity(0, 0, -9.81);
  double total_weight = 0.0;
  for (const auto& f : features) {
    // for (const auto& f : features) {

    vector<double> v1v2_vec(2);
    vector<double> v3_vec(2);

    vector<vector<Vector3d>> dv3_dq_vec(2);

    // Calculate visibility cost and gradients for each knot
    for (int i = 0; i < 2; i++) {
      Vector3d yaw_knot = (q[i] + 4 * q[i + 1] + q[i + 2]) / 6;
      double yaw = yaw_knot(0);

      // Calculate vectors n1, ny, n3 ,b and their gradients
      Vector3d n1, ny, n3, n2, b;
      n1 = knots_acc[i] - gravity;  // thrust
      ny << cos(yaw), sin(yaw), 0;
      n3 = n1.cross(ny);
      n2 = n3.cross(n1);
      b = f - knots_pos[i];

      Vector3d dn3_dyaw;
      dn3_dyaw << -n1(2) * cos(yaw), -n1(2) * sin(yaw), n1(1) * sin(yaw) + n1(0) * cos(yaw);

      Vector4d dyaw_dq = Vector4d::Zero();
      dyaw_dq.segment(i, 3) = Vector3d(1, 4, 1) / 6;

      // 计算v1
      double k1 = configPA_.k1_;
      double fov_vertical = camera_param_->fov_vertical;
      double alpha1 = (M_PI - fov_vertical) / 2.0;
      double sin_theta1 = n1.cross(b).norm() / (n1.norm() * b.norm());
      double v1 = Utils::sigmoid(k1, (sin_theta1 - sin(alpha1)));

      // 计算v2
      double k2 = configPA_.k2_;
      double cos_theta2 = n2.dot(b) / (n2.norm() * b.norm());
      double v2 = Utils::sigmoid(k2, cos_theta2);

      double v1v2 = v1 * v2;

      // 计算v3
      double k3 = configPA_.k3_;
      double fov_horizontal = camera_param_->fov_horizontal;
      double alpha3 = (M_PI - fov_horizontal) / 2.0;
      double sin_theta3 = n3.cross(b).norm() / (n3.norm() * b.norm());
      double v3 = Utils::sigmoid(k3, (sin_theta3 - sin(alpha3)));

      // 计算v3的grad
      Vector3d c = n3.cross(b);
      double n3_norm = n3.norm();
      double b_norm = b.norm();
      double c_norm = c.norm();
      Vector3d dsin_theta3_dn3 = (pow(-n3_norm, 2) * b.cross(c) - pow(c_norm, 2) * n3) / (pow(n3_norm, 3) * b_norm * c_norm);
      double dv3_dsin_theta3 = k3 * exp(-k3 * (sin_theta3 - sin(alpha3))) * pow(v3, 2);

      // Combine gradients using chain rule
      double dv3_dyaw = dv3_dsin_theta3 * dsin_theta3_dn3.dot(dn3_dyaw);

      // Store results
      v1v2_vec[i] = v1v2;
      v3_vec[i] = v3;
      vector<Vector3d> dv3_dq;
      for (int j = 0; j < 4; j++) {
        Vector3d dv3_dqj = Vector3d(dv3_dyaw, 0, 0) * dyaw_dq[j];
        dv3_dq.push_back(dv3_dqj);
      }
      dv3_dq_vec[i] = dv3_dq;
    }

    // Calculate co-visbility potential cost function and its gradient
    double pot_cost;
    vector<Vector3d> dcovisb_pot_dq_cur;

    double covisibility = -(v1v2_vec[0] * v1v2_vec[1] * v3_vec[0] * v3_vec[1]);

    pot_cost = covisibility;
    for (int j = 0; j < 4; j++) {
      dcovisb_pot_dq_cur.emplace_back(-v1v2_vec[0] * v1v2_vec[1] * (dv3_dq_vec[0][j] * v3_vec[1] + v3_vec[0] * dv3_dq_vec[1][j]));
    }

    // double w = 1.0;
    // cost = (cost * total_weight + pot_cost * w) / (total_weight + w);
    // // ROS_INFO("fuck cost: %f", cost);

    // for (int j = 0; j < 4; j++) {
    //   dcost_dq[j] = (dcost_dq[j] * total_weight + dcovisb_pot_dq_cur[j] * w) / (total_weight + w);
    // }

    // total_weight += w;

    // 为了硬约束考虑，不要加权了
    cost += pot_cost;
    // ROS_INFO("fuck cost: %f", cost);

    for (int j = 0; j < 4; j++) {
      dcost_dq[j] += dcovisb_pot_dq_cur[j];
    }
  }
}

void BsplineOptimizer::calcFrontierVisibilityCostAndGradientsKnots(const vector<Vector3d>& q_cur, const Vector3d& knots_pos,
    const Vector3d& knots_acc, const int layer, double& cost, vector<Vector3d>& dcost_dq) {

  ROS_ASSERT(q_cur.size() == 3);

  cost = 0.0;
  dcost_dq.clear();
  for (int i = 0; i < 3; i++) dcost_dq.push_back(Vector3d::Zero());

  auto& status_vec = opt_data_->frontier_status_[layer];
  ROS_ASSERT(status_vec.size() == frontier_cells_.size());

  Vector3d gravity(0, 0, -9.81);
  double total_weight = 0.0;
  // for (const auto& cell : frontier_cells) {
  for (size_t i = 0; i < frontier_cells_.size(); i++) {
    if (status_vec[i] == NOT_AVAILABLE || status_vec[i] == HAS_BEEN_OBSERVED) continue;

    double w;
    if (status_vec[i] == VISIBLE)
      w = ld_weight1_;
    else if (status_vec[i] == AVAILABLE)
      w = ld_weight2_;

    Vector3d cell = frontier_cells_[i];

    Vector3d yaw_knot = (q_cur[0] + 4 * q_cur[1] + q_cur[2]) / 6;
    double yaw = yaw_knot(0);

    // Calculate vectors n1, ny, n3 ,b and their gradients
    Vector3d n1, ny, n3, n2, b;
    n1 = knots_acc - gravity;  // thrust
    ny << cos(yaw), sin(yaw), 0;
    n3 = n1.cross(ny);
    n2 = n3.cross(n1);
    b = cell - knots_pos;

    Vector3d dn3_dyaw;
    dn3_dyaw << -n1(2) * cos(yaw), -n1(2) * sin(yaw), n1(1) * sin(yaw) + n1(0) * cos(yaw);

    // 计算v1
    double k1 = configPA_.k1_;
    double fov_vertical = camera_param_->fov_vertical;
    double alpha1 = (M_PI - fov_vertical) / 2.0;
    double sin_theta1 = n1.cross(b).norm() / (n1.norm() * b.norm());
    double v1 = Utils::sigmoid(k1, (sin_theta1 - sin(alpha1)));

    // 计算v2
    double k2 = configPA_.k2_;
    double cos_theta2 = n2.dot(b) / (n2.norm() * b.norm());
    double v2 = Utils::sigmoid(k2, cos_theta2);

    double v1v2 = v1 * v2;

    // 计算v3
    double k3 = configPA_.k3_;
    double fov_horizontal = camera_param_->fov_horizontal;
    double alpha3 = (M_PI - fov_horizontal) / 2.0;
    double sin_theta3 = n3.cross(b).norm() / (n3.norm() * b.norm());
    double v3 = Utils::sigmoid(k3, (sin_theta3 - sin(alpha3)));

    // 计算v3的grad
    Vector3d c = n3.cross(b);
    double c_norm = c.norm();
    double n3_norm = n3.norm();
    double b_norm = b.norm();
    Vector3d dsin_theta3_dn3 = (pow(-n3_norm, 2) * b.cross(c) - pow(c_norm, 2) * n3) / (pow(n3_norm, 3) * b_norm * c_norm);
    double dv3_dsin_theta3 = k3 * exp(-k3 * (sin_theta3 - sin(alpha3))) * pow(v3, 2);

    // Combine gradients using chain rule
    double dv3_dyaw = dv3_dsin_theta3 * dsin_theta3_dn3.dot(dn3_dyaw);
    Vector3d dyaw_dq = Vector3d(1, 4, 1) / 6;

    // 计算对单个cell的cost
    double cost_per_cell = 1 - (v1v2 * v3);
    // 计算对单个cell的grad
    vector<Vector3d> dcost_per_cell_dq;
    for (int j = 0; j < 3; j++) dcost_per_cell_dq.emplace_back(-v1v2 * Vector3d(dv3_dyaw, 0, 0) * dyaw_dq[j]);

    cost = (cost * total_weight + cost_per_cell * w) / (total_weight + w);
    for (int j = 0; j < 3; j++) {
      dcost_dq[j] = (dcost_dq[j] * total_weight + dcost_per_cell_dq[j] * w) / (total_weight + w);
    }

    total_weight += w;
  }
}

void BsplineOptimizer::calcYawCoVisbilityCost(const vector<Vector3d>& q, double& cost, vector<Vector3d>& gradient_q) {
  ROS_ASSERT(q.size() - 2 == pos_.size());

  cost = 0.0;
  Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double cost_i;
  vector<Vector3d> dcost_dq_i;
  for (size_t i = 0; i < q.size() - 3; ++i) {

    // 如果当前的knot point离传入的初值还太远，在接下来具体计算cost和grad的过程中获取供优化特征点的步骤可能会出错
    // Vector3d yaw_knot_0 = (q[i] + 4 * q[i + 1] + q[i + 2]) / 6;
    // if ((yaw_knot_0 - waypoints_[i]).norm() > 0.5) continue;
    // Vector3d yaw_knot_1 = (q[i + 1] + 4 * q[i + 2] + q[i + 3]) / 6;
    // if ((yaw_knot_1 - waypoints_[i + 1]).norm() > 0.5) continue;

    // For (q0, q1, q2, q3)->(knot1, knot2), calculate the covisibility cost and gradient
    vector<Vector3d> q_cur;
    for (int j = 0; j < 4; j++) q_cur.push_back(q[i + j]);

    vector<Vector3d> knots_pos, knots_acc;
    for (int j = 0; j < 2; j++) {
      knots_pos.push_back(pos_[i + j]);
      knots_acc.push_back(acc_[i + j]);
    }

    Vector3d knot_mid = 0.5 * (pos_[i] + pos_[i + 1]);
    vector<Vector3d> features;
    feature_map_->getFeatures(knot_mid, features);

    calcYawCVCostAndGradientsKnots(q_cur, knots_pos, knots_acc, opt_data_->observed_features_[i], cost_i, dcost_dq_i);

    // cost_i是负值，越小说明两帧之间的共视性越好，比阈值还大的话就需要让它往小走
    double pot_cost;
    vector<Vector3d> dpot_cost_dq_i;
    for (int j = 0; j < 4; j++) dpot_cost_dq_i.push_back(Vector3d::Zero());

    int min_covisible_feature_num = Utils::getGlobalParam().min_covisible_feature_num_plan_;
    double threshold = -min_covisible_feature_num / configPA_.min_covisible_feature_cost_;
    // ROS_INFO("threshold: %f", threshold);
    // ROS_INFO("cost i: %f", cost_i);

    // double threshold = -configPA_.min_covisible_feature_cost_;

    double vd = cost_i - threshold;
    if (vd > 0.0) {
      pot_cost = pow(vd, 2);
      double dpot_cost_dcost_i = 2 * vd;
      for (int j = 0; j < 4; j++) dpot_cost_dq_i[j] = dpot_cost_dcost_i * dcost_dq_i[j];
    }

    else {
      pot_cost = 0.0;
      for (int j = 0; j < 4; j++) dpot_cost_dq_i[j] = Vector3d::Zero();
    }

    // cost += cost_i;
    // for (int j = 0; j < 4; j++) gradient_q[i + j] += dcost_dq_i[j];
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_YAW_OPT, COST_TYPE::YAWCOVISIBILITY, pot_cost);
    cost += pot_cost;
    for (int j = 0; j < 4; j++) {
      gradient_q[i + j] += dpot_cost_dq_i[j];
    }
  }
}

void BsplineOptimizer::calcFrontierVisbilityCostYaw(const vector<Vector3d>& q, double& cost, vector<Vector3d>& gradient_q) {
  cost = 0.0;
  Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double cost_i;
  vector<Vector3d> dcost_dq_i;
  // cout << "[BsplineOptimizer::calcFrontierVisbilityCostYaw] Begin!: " << endl;

  for (size_t i = 0; i < q.size() - 2; ++i) {
    vector<Vector3d> q_cur;
    for (int j = 0; j < 3; j++) q_cur.push_back(q[i + j]);

    Vector3d knots_pos = pos_[i];
    Vector3d knots_acc = acc_[i];

    calcFrontierVisibilityCostAndGradientsKnots(q_cur, knots_pos, knots_acc, i, cost_i, dcost_dq_i);
    stepping_debug_->addDebugCost(DEBUG_TYPE::EVERY_YAW_OPT, COST_TYPE::FRONTIERVIS_YAW, cost_i);

    cost += cost_i;
    for (int j = 0; j < 3; j++) gradient_q[i + j] += dcost_dq_i[j];
  }
}

void BsplineOptimizer::calcFinalGoalCostKnots(const vector<Vector3d>& q_cur, const Vector3d& knots_pos, const Vector3d& knots_acc,
    const int layer, double& cost, vector<Vector3d>& dcost_dq) {

  ROS_ASSERT(q_cur.size() == 3);

  cost = 0.0;
  dcost_dq.clear();
  for (int i = 0; i < 3; i++) dcost_dq.push_back(Vector3d::Zero());

  // auto& if_visible = opt_data_->final_status_[layer];

  // Vector3d gravity(0, 0, -9.81);
  // if (if_visible == 0) return;

  // // for (const auto& cell : frontier_cells) {
  // for (size_t i = 0; i < frontier_cells_.size(); i++) {
  //   if (status_vec[i] == 0 || status_vec[i] == 1) continue;

  //   double w;
  //   if (status_vec[i] == 2)
  //     w = ld_weight1_;
  //   else if (status_vec[i] == 3)
  //     w = ld_weight2_;

  //   Vector3d cell = frontier_cells_[i];

  //   Vector3d yaw_knot = (q_cur[0] + 4 * q_cur[1] + q_cur[2]) / 6;
  //   double yaw = yaw_knot(0);

  //   // Calculate vectors n1, ny, n3 ,b and their gradients
  //   Vector3d n1, ny, n3, n2, b;
  //   n1 = knots_acc - gravity;  // thrust
  //   ny << cos(yaw), sin(yaw), 0;
  //   n3 = n1.cross(ny);
  //   n2 = n3.cross(n1);
  //   b = cell - knots_pos;

  //   Vector3d dn3_dyaw;
  //   dn3_dyaw << -n1(2) * cos(yaw), -n1(2) * sin(yaw), n1(1) * sin(yaw) + n1(0) * cos(yaw);

  //   // 计算v1
  //   double k1 = configPA_.k1_;
  //   double fov_vertical = camera_param_->fov_vertical;
  //   double alpha1 = (M_PI - fov_vertical) / 2.0;
  //   double sin_theta1 = n1.cross(b).norm() / (n1.norm() * b.norm());
  //   double v1 = Utils::sigmoid(k1, (sin_theta1 - sin(alpha1)));

  //   // 计算v2
  //   double k2 = configPA_.k2_;
  //   double cos_theta2 = n2.dot(b) / (n2.norm() * b.norm());
  //   double v2 = Utils::sigmoid(k2, cos_theta2);

  //   double v1v2 = v1 * v2;

  //   // 计算v3
  //   double k3 = configPA_.k3_;
  //   double fov_horizontal = camera_param_->fov_horizontal;
  //   double alpha3 = (M_PI - fov_horizontal) / 2.0;
  //   double sin_theta3 = n3.cross(b).norm() / (n3.norm() * b.norm());
  //   double v3 = Utils::sigmoid(k3, (sin_theta3 - sin(alpha3)));

  //   // 计算v3的grad
  //   Vector3d c = n3.cross(b);
  //   double c_norm = c.norm();
  //   double n3_norm = n3.norm();
  //   double b_norm = b.norm();
  //   Vector3d dsin_theta3_dn3 = (pow(-n3_norm, 2) * b.cross(c) - pow(c_norm, 2) * n3) / (pow(n3_norm, 3) * b_norm * c_norm);
  //   double dv3_dsin_theta3 = k3 * exp(-k3 * (sin_theta3 - sin(alpha3))) * pow(v3, 2);

  //   // Combine gradients using chain rule
  //   double dv3_dyaw = dv3_dsin_theta3 * dsin_theta3_dn3.dot(dn3_dyaw);
  //   Vector3d dyaw_dq = Vector3d(1, 4, 1) / 6;

  //   // 计算对单个cell的cost
  //   double cost_per_cell = 1 - (v1v2 * v3);
  //   // 计算对单个cell的grad
  //   vector<Vector3d> dcost_per_cell_dq;
  //   for (int j = 0; j < 3; j++) dcost_per_cell_dq.emplace_back(-v1v2 * Vector3d(dv3_dyaw, 0, 0) * dyaw_dq[j]);

  //   cost = (cost * total_weight + cost_per_cell * w) / (total_weight + w);
  //   for (int j = 0; j < 3; j++) {
  //     dcost_dq[j] = (dcost_dq[j] * total_weight + dcost_per_cell_dq[j] * w) / (total_weight + w);
  //   }

  //   total_weight += w;
  // }
}

void BsplineOptimizer::calcFinalGoalCostYaw(const vector<Vector3d>& q, double& cost, vector<Vector3d>& gradient_q) {
  cost = 0.0;
  Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double cost_i;
  vector<Vector3d> dcost_dq_i;
  // cout << "[BsplineOptimizer::calcFrontierVisbilityCostYaw] Begin!: " << endl;

  for (size_t i = 0; i < q.size() - 2; ++i) {
    vector<Vector3d> q_cur;
    for (int j = 0; j < 3; j++) q_cur.push_back(q[i + j]);

    Vector3d knots_pos = pos_[i];
    Vector3d knots_acc = acc_[i];

    calcFinalGoalCostKnots(q_cur, knots_pos, knots_acc, i, cost_i, dcost_dq_i);

    cost += cost_i;
    for (int j = 0; j < 3; j++) gradient_q[i + j] += dcost_dq_i[j];
  }
}

// !SECTION

void BsplineOptimizer::combineCost(const std::vector<double>& x, vector<double>& grad, double& f_combine) {

  ros::Time t1 = ros::Time::now();

  for (int i = 0; i < point_num_; ++i) {
    for (int j = 0; j < dim_; ++j) g_q_[i][j] = x[dim_ * i + j];
    for (int j = dim_; j < 3; ++j) g_q_[i][j] = 0.0;
  }
  const double dt = optimize_time_ ? x[variable_num_ - 1] : knot_span_;

  f_combine = 0.0;
  grad.resize(variable_num_);
  fill(grad.begin(), grad.end(), 0.0);
  // Cost1：平滑度约束
  if (cost_function_ & SMOOTHNESS) {
    calcSmoothnessCost(g_q_, f_smoothness_, g_smoothness_);
    f_combine += ld_smooth_ * f_smoothness_;
    // cout << " f_smoothness_: " << ld_smooth_ * f_smoothness_ << endl;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_smooth_ * g_smoothness_[i](j);
  }

  // Cost2：避障约束
  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance_, g_distance_);
    f_combine += ld_dist_ * f_distance_;
    // cout << " f_distance_: " << ld_dist_ * f_distance_ << endl;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_dist_ * g_distance_[i](j);
  }

  // Cost3：动力学可行性约束
  if (cost_function_ & FEASIBILITY) {
    double gt_feasibility = 0.0;
    calcFeasibilityCost(g_q_, dt, f_feasibility_, g_feasibility_, gt_feasibility);
    // cout << " f_feasibility_: " << ld_feasi_ * f_feasibility_ << endl;
    f_combine += ld_feasi_ * f_feasibility_;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_feasi_ * g_feasibility_[i](j);
    if (optimize_time_) grad[variable_num_ - 1] += ld_feasi_ * gt_feasibility;
  }

  // Cost3：动力学可行性约束(适用于YAW轨迹)
  if (cost_function_ & FEASIBILITY_YAW) {
    double gt_feasibility_yaw = 0.0;
    calcFeasibilityCostYaw(g_q_, dt, f_feasibility_yaw_, g_feasibility_yaw_, gt_feasibility_yaw);
    // cout << " f_feasibility_yaw_: " << ld_feasi_yaw_ * f_feasibility_yaw_ << endl;
    f_combine += ld_feasi_yaw_ * f_feasibility_yaw_;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_feasi_yaw_ * g_feasibility_yaw_[i](j);
    if (optimize_time_) grad[variable_num_ - 1] += ld_feasi_yaw_ * gt_feasibility_yaw;
  }

  // Cost4：起始状态约束
  if (cost_function_ & START) {
    double gt_start = 0.0;
    calcStartCost(g_q_, dt, f_start_, g_start_, gt_start);
    // cout << " f_start_: " << ld_start_ * f_start_ << endl;
    f_combine += ld_start_ * f_start_;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_start_ * g_start_[i](j);
    if (optimize_time_) grad[variable_num_ - 1] += ld_start_ * gt_start;
  }

  // Cost5：终止状态约束
  if (cost_function_ & END) {
    double gt_end = 0.0;
    calcEndCost(g_q_, dt, f_end_, g_end_, gt_end);
    // cout << " f_end_: " << ld_end_ * f_end_ << endl;
    f_combine += ld_end_ * f_end_;
    for (int i = point_num_ - 3; i < point_num_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_end_ * g_end_[i](j);

    if (optimize_time_) grad[variable_num_ - 1] += ld_end_ * gt_end;
  }

  // Cost6：引导约束，指定的是控制点约束
  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, f_guide_, g_guide_);
    // cout << " f_guide_: " << ld_guide_ * f_guide_ << endl;
    f_combine += ld_guide_ * f_guide_;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_guide_ * g_guide_[i](j);
  }

  // Cost7：同样是引导约束，不过是使用waypoints，跟上面的区别是上面使用control point，这个使用knot point
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints_, g_waypoints_);
    f_combine += ld_waypt_ * f_waypoints_;
    // cout << " f_waypoints_: " << ld_waypt_ * f_waypoints_ << endl;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_waypt_ * g_waypoints_[i](j);
  }

  // Cost9：最小化时间约束
  if (cost_function_ & MINTIME) {
    double gt_time = 0.0;
    calcTimeCost(dt, f_time_, gt_time);
    // cout << " f_time_: " << ld_time_ * f_time_ << endl;
    f_combine += ld_time_ * f_time_;
    grad[variable_num_ - 1] += ld_time_ * gt_time;
  }

  // SECTION Perception Aware Optimization

  /// APACE新加的
  /// 用于Position Trajectory Optimization阶段
  /// Cost10：视差约束和垂直共视性约束
  // if (cost_function_ & VERTICALVISIBILITY) {
  //   calcPerceptionCost(g_q_, dt, f_parallax_, g_parallax_, ld_parallax_, ld_vertical_visibility_);
  //   f_combine += f_parallax_;
  //   for (int i = 0; i < point_num_; i++) {
  //     for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += g_parallax_[i](j);
  //   }
  // }

  if ((cost_function_ & PARALLAX) && (cost_function_ & VERTICALVISIBILITY)) {
    calcPerceptionCost(g_q_, dt, f_parallax_, g_parallax_, ld_parallax_, ld_vertical_visibility_);
    f_combine += f_parallax_;
    // cout << " f_parallax_: " << f_parallax_ << endl;
    for (int i = 0; i < point_num_; i++) {
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += g_parallax_[i](j);
    }
  }
  // Cost11：FRONTIER可见性约束
  if ((cost_function_ & FRONTIERVISIBILITY_POS)) {
    calcEUACost(g_q_, dt, f_frontier_visibility_pos_, g_frontier_visibility_pos_);
    // calcFVBCost(g_q_, f_frontier_visibility_pos_, g_frontier_visibility_pos_);
    f_combine += ld_frontier_visibility_pos_ * f_frontier_visibility_pos_;
    // cout << " f_frontier_visibility_pos_: " << ld_frontier_visibility_pos_ * f_frontier_visibility_pos_ << endl;
    for (int i = 0; i < point_num_; i++) {
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_frontier_visibility_pos_ * g_frontier_visibility_pos_[i](j);
    }
  }

  /// 用于Yaw Trajectory Optimization阶段
  /// Cost12：yaw共视性约束
  if (cost_function_ & YAWCOVISIBILITY) {
    calcYawCoVisbilityCost(g_q_, f_yaw_covisibility_, g_yaw_covisibility_);
    // cout << " f_yaw_covisibility_: " << ld_yaw_covisib_ * f_yaw_covisibility_ << endl;
    f_combine += ld_yaw_covisib_ * f_yaw_covisibility_;
    for (int i = 0; i < point_num_; i++) {
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_yaw_covisib_ * g_yaw_covisibility_[i](j);
    }
  }

  /// 我们新加的
  ///  Cost13：frontier可见性约束
  if (cost_function_ & FRONTIERVISIBILITY_YAW) {
    calcFrontierVisbilityCostYaw(g_q_, f_frontier_visibility_yaw_, g_frontier_visibility_yaw_);
    // cout << "frontier_cells_ size: " << frontier_cells_.size() << endl;
    // cout << "f_frontier_visibility: " << f_frontier_visibility_yaw << endl;
    // cout << "g_frontier_visibility_: " << endl;
    // for (const auto& g : g_frontier_visibility_yaw_) {
    //   cout << g.transpose() << endl;
    // }
    // cout << " f_frontier_visibility_yaw_: " << ld_frontier_visibility_yaw_ * f_frontier_visibility_yaw_ << endl;
    f_combine += ld_frontier_visibility_yaw_ * f_frontier_visibility_yaw_;
    for (int i = 0; i < point_num_; i++) {
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_frontier_visibility_yaw_ * g_frontier_visibility_yaw_[i](j);
    }
  }
  // cout << " f_combine: " << f_combine << endl;

  stepping_debug_->calldebug(DEBUG_TYPE::EVERY_POS_OPT, g_q_, order_, dt);
  stepping_debug_->calldebug(DEBUG_TYPE::EVERY_YAW_OPT, g_q_, order_, dt);

  // printVector(g_q_, "g_q_");
  // printVector(g_smoothness_, "g_smoothness_");
  // printVector(g_distance_, "g_distance_");
  // printVector(g_feasibility_, "g_feasibility_");
  // printVector(g_feasibility_yaw_, "g_feasibility_yaw_");
  // printVector(g_start_, "g_start_");
  // printVector(g_end_, "g_end_");
  // printVector(g_guide_, "g_guide_");
  // printVector(g_waypoints_, "g_waypoints_");
  // printVector(g_view_, "g_view_");
  // printVector(g_time_, "g_time_");
  // printVector(g_parallax_, "g_parallax_");
  // printVector(g_frontier_visibility_pos_, "g_frontier_visibility_pos_");
  // printVector(g_yaw_covisibility_, "g_yaw_covisibility_");
  // printVector(g_frontier_visibility_yaw_, "g_frontier_visibility_yaw_");
  // stepping_debug_->calldebug(DEBUG_TYPE::EVERY_YAW_OPT, g_q_, order_, dt);

  if (cost_function_ & FINAL_GOAL) {
    // calcFinalGoalCost(g_q_, f_final_goal_, g_final_goal_);
    // f_combine += ld_final_goal_ * f_final_goal_;
    // for (int i = 0; i < point_num_; i++)
    //   for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_final_goal_ * g_final_goal_[i](j);
  }

  // cout << " f_combine: " << f_combine << endl;
  stepping_debug_->calldebug(DEBUG_TYPE::EVERY_POS_OPT, g_q_, order_, dt);
  stepping_debug_->calldebug(DEBUG_TYPE::EVERY_YAW_OPT, g_q_, order_, dt);

  // !SECTION

  comb_time += (ros::Time::now() - t1).toSec();

  if (cost_function_ & YAWCOVISIBILITY) {

    // cout << "---------------------------------------------------------------------------" << endl;
    // cout << "[bspline optimizer]:iter_num: " << iter_num_ << endl;
    // cout << "[bspline optimizer]:total cost: " << f_combine << endl;

    // if (cost_function_ & SMOOTHNESS) cout << "[bspline optimizer]:smoothness cost: " << f_smoothness_ << endl;
    // if (cost_function_ & DISTANCE) cout << "[bspline optimizer]:distance cost: " << f_distance_ << endl;
    // if (cost_function_ & FEASIBILITY) cout << "[bspline optimizer]:feasibility cost: " << f_feasibility_ << endl;
    // if (cost_function_ & FEASIBILITY_YAW) cout << "[bspline optimizer]:feasibility yaw cost: " << f_feasibility_yaw_ << endl;
    // if (cost_function_ & START) cout << "[bspline optimizer]:start cost: " << f_start_ << endl;
    // if (cost_function_ & END) cout << "[bspline optimizer]:end cost: " << f_end_ << endl;
    // if (cost_function_ & GUIDE) cout << "[bspline optimizer]:guide cost: " << f_guide_ << endl;
    // if (cost_function_ & WAYPOINTS) cout << "[bspline optimizer]:waypoints cost: " << f_waypoints_ << endl;
    // if (cost_function_ & MINTIME) cout << "[bspline optimizer]:time cost: " << f_time_ << endl;
    // if ((cost_function_ & PARALLAX) && (cost_function_ & VERTICALVISIBILITY))
    //   cout << "[bspline optimizer]:parallax cost: " << f_parallax_ << endl;
    // if (cost_function_ & FRONTIERVISIBILITY_POS)
    //   cout << "[bspline optimizer]:view frontier(pos) cost: " << f_frontier_visibility_pos_ << endl;
    // if (cost_function_ & YAWCOVISIBILITY) cout << "[bspline optimizer]:yaw covisibility cost: " << f_yaw_covisibility_ <<
    // endl; if (cost_function_ & FRONTIERVISIBILITY_YAW)
    //   cout << "[bspline optimizer]:view frontier(yaw) cost: " << f_frontier_visibility_yaw_ << endl;
    // if (cost_function_ & FINAL_GOAL)
    //   cout << "[bspline optimizer]:final goal cost: " << f_final_goal_ << endl;
    // cout << "---------------------------------------------------------------------------" << endl;
  }
}

double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data) {
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_ = cost;
    opt->best_variable_ = x;
  }
  return cost;
}

vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints() {
  return this->control_points_;
}

bool BsplineOptimizer::isQuadratic() {
  if (cost_function_ == GUIDE_PHASE) {
    return true;
  }

  else if (cost_function_ == SMOOTHNESS) {
    return true;
  }

  else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    return true;
  }

  return false;
}

}  // namespace fast_planner