#include "bspline_opt/bspline_optimizer.h"

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_env/feature_map.h>
#include <active_perception/frontier_finder.h>

#include <nlopt.hpp>
#include <thread>

using namespace std;
using namespace Eigen;

namespace fast_planner {
const int BsplineOptimizer::SMOOTHNESS = (1 << 0);
const int BsplineOptimizer::DISTANCE = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::START = (1 << 3);
const int BsplineOptimizer::END = (1 << 4);
const int BsplineOptimizer::GUIDE = (1 << 5);
const int BsplineOptimizer::WAYPOINTS = (1 << 6);
const int BsplineOptimizer::VIEWCONS = (1 << 7);
const int BsplineOptimizer::MINTIME = (1 << 8);
const int BsplineOptimizer::PARALLAX = (1 << 9);
const int BsplineOptimizer::VERTICALVISIBILITY = (1 << 10);
const int BsplineOptimizer::YAWCOVISIBILITY = (1 << 11);
const int BsplineOptimizer::FRONTIERVISIBILITY_POS = (1 << 12);
const int BsplineOptimizer::FRONTIERVISIBILITY_YAW = (1 << 13);

const int BsplineOptimizer::GUIDE_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE | BsplineOptimizer::START | BsplineOptimizer::END;
const int BsplineOptimizer::NORMAL_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE |
                                           BsplineOptimizer::FEASIBILITY | BsplineOptimizer::START | BsplineOptimizer::END;

void BsplineOptimizer::setParam(ros::NodeHandle& nh) {
  nh.param("optimization/ld_smooth", ld_smooth_, -1.0);
  nh.param("optimization/ld_dist", ld_dist_, -1.0);
  nh.param("optimization/ld_feasi", ld_feasi_, -1.0);
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

  nh.param("optimization/dist0", dist0_, -1.0);
  nh.param("optimization/max_vel", max_vel_, -1.0);
  nh.param("optimization/max_acc", max_acc_, -1.0);
  nh.param("optimization/dlmin", dlmin_, -1.0);
  nh.param("optimization/wnl", wnl_, -1.0);

  nh.param("optimization/parallax/estimator_freq", configPA_.estimator_freq_, -1.0);
  nh.param("optimization/parallax/max_parallax", configPA_.max_parallax_, -1.0);
  nh.param("optimization/parallax/pot_a", configPA_.pot_a_, -1.0);
  nh.param("optimization/max_feature_and_frontier_convisual_angle", configPA_.max_feature_and_frontier_convisual_angle_, -1.0);
  nh.param("optimization/min_frontier_see_feature_num", configPA_.min_frontier_see_feature_num_, -1.0);
  nh.param("optimization/pot_fafv", configPA_.pot_fafv_, -1.0);

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
}

void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
  dynamic_ = false;
}

void BsplineOptimizer::setCostFunction(const int& cost_code) {
  cost_function_ = cost_code;

  // print optimized cost function
  string cost_str;
  if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";
  if (cost_function_ & DISTANCE) cost_str += " dist  |";
  if (cost_function_ & FEASIBILITY) cost_str += " feasi |";
  if (cost_function_ & START) cost_str += " start |";
  if (cost_function_ & END) cost_str += " end   |";
  if (cost_function_ & GUIDE) cost_str += " guide |";
  if (cost_function_ & WAYPOINTS) cost_str += " waypt |";
  if (cost_function_ & VIEWCONS) cost_str += " view  |";
  if (cost_function_ & MINTIME) cost_str += " time  |";
  if (cost_function_ & PARALLAX) cost_str += " parallax | ";
  if (cost_function_ & VERTICALVISIBILITY) cost_str += " veritcal_visibility | ";
  if (cost_function_ & YAWCOVISIBILITY) cost_str += " yaw_covisibility | ";
  if (cost_function_ & FRONTIERVISIBILITY_POS) cost_str += " frontier_covisibility_pos | ";
  if (cost_function_ & FRONTIERVISIBILITY_YAW) cost_str += " frontier_covisibility_yaw | ";

  // ROS_INFO_STREAM("cost func: " << cost_str);
}

void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d>& guide_pt) {
  guide_pts_ = guide_pt;
}

void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d>& waypts, const vector<int>& waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

void BsplineOptimizer::setViewConstraint(const ViewConstraint& vc) {
  view_cons_ = vc;
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
  double final_cost;
  try {
    double final_cost;
    nlopt::result result = opt.optimize(q, final_cost);
    for (int i = 0; i < point_num_; ++i)
      for (int j = 0; j < dim_; ++j) control_points_(i, j) = best_variable_[dim_ * i + j];
    if (optimize_time_) knot_span_ = best_variable_[variable_num_ - 1];

    if (cost_function_ & MINTIME) {
      std::cout << "Iter num: " << iter_num_ << ", time: " << (ros::Time::now() - t1).toSec() << ", point num: " << point_num_
                << ", comb time: " << comb_time << std::endl;
    }
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
    cost += ji.squaredNorm();
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
      cost += pow(dist - dist0_, 2);
      gradient_q[i] += 2.0 * (dist - dist0_) * dist_grad;
    }
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
        cost += pow(vd, 2);
        double sign = vi[k] > 0 ? 1.0 : -1.0;
        double tmp = 2 * vd * sign * dt_inv;
        gradient_q[i][k] += -tmp;
        gradient_q[i + 1][k] += tmp;
        if (optimize_time_) gt += tmp * (-vi[k]);
      }
    }
  }

  // Acc feasibility cost
  for (size_t i = 0; i < q.size() - 2; ++i) {
    Eigen::Vector3d ai = (q[i + 2] - 2 * q[i + 1] + q[i]) * dt_inv2;
    for (int k = 0; k < 3; ++k) {
      double ad = fabs(ai[k]) - max_acc_;
      if (ad > 0.0) {
        cost += pow(ad, 2);
        double sign = ai[k] > 0 ? 1.0 : -1.0;
        double tmp = 2 * ad * sign * dt_inv2;
        gradient_q[i][k] += tmp;
        gradient_q[i + 1][k] += -2 * tmp;
        gradient_q[i + 2][k] += tmp;
        if (optimize_time_) gt += tmp * ai[k] * (-2) * dt;
      }
    }
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
    cost += w_pos * dq.squaredNorm();
    gradient_q[0] += w_pos * 2 * dq * (1 / 6.0);
    gradient_q[1] += w_pos * 2 * dq * (4 / 6.0);
    gradient_q[2] += w_pos * 2 * dq * (1 / 6.0);
  }

  // Start velocity
  if (start_con_index_[1]) {
    if (start_state_.size() < 2) ROS_ERROR_STREAM("(start vel),start state size: " << start_state_.size());

    dq = 1 / (2 * dt) * (q3 - q1) - start_state_[1];
    cost += dq.squaredNorm();
    gradient_q[0] += 2 * dq * (-1.0) / (2 * dt);
    gradient_q[2] += 2 * dq * 1.0 / (2 * dt);
    if (optimize_time_) gt += dq.dot(q3 - q1) / (-dt * dt);
  }

  // Start acceleration
  if (start_con_index_[2]) {
    if (start_state_.size() < 3) ROS_ERROR_STREAM("(start acc),start state size: " << start_state_.size());

    dq = 1 / (dt * dt) * (q1 - 2 * q2 + q3) - start_state_[2];
    cost += dq.squaredNorm();
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
    cost += dq.squaredNorm();
    gradient_q[q.size() - 1] += 2 * dq * (1 / 6.0);
    gradient_q[q.size() - 2] += 2 * dq * (4 / 6.0);
    gradient_q[q.size() - 3] += 2 * dq * (1 / 6.0);
  }

  // End velocity
  if (end_con_index_[1]) {
    if (end_state_.size() < 2) ROS_ERROR_STREAM("(end vel),end state size: " << end_state_.size());

    dq = 1 / (2 * dt) * (q_1 - q_3) - end_state_[1];
    cost += dq.squaredNorm();
    gradient_q[q.size() - 1] += 2 * dq * 1.0 / (2 * dt);
    gradient_q[q.size() - 3] += 2 * dq * (-1.0) / (2 * dt);
    if (optimize_time_) gt += dq.dot(q_1 - q_3) / (-dt * dt);
  }

  // End acceleration
  if (end_con_index_[2]) {
    if (end_state_.size() < 3) ROS_ERROR_STREAM("(end acc),end state size: " << end_state_.size());

    dq = 1 / (dt * dt) * (q_1 - 2 * q_2 + q_3) - end_state_[2];
    cost += dq.squaredNorm();
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
    cost += dq.squaredNorm();

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
    cost += (q[i] - gpt).squaredNorm();
    gradient_q[i] += 2 * (q[i] - gpt);
  }
}

void BsplineOptimizer::calcViewCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);
  Eigen::Vector3d p = view_cons_.pt_;
  Eigen::Vector3d v = view_cons_.dir_.normalized();
  Eigen::Matrix3d vvT = v * v.transpose();
  Eigen::Matrix3d I_vvT = Eigen::Matrix3d::Identity() - vvT;

  // prependicular cost, increase visibility of points before blocked point
  int i = view_cons_.idx_;
  Eigen::Vector3d dn = (q[i] - p) - ((q[i] - p).dot(v)) * v;
  cost += dn.squaredNorm();
  gradient_q[i] += 2 * I_vvT * dn;
  double norm_dn = dn.norm();

  // parallel cost, increase projection along view direction
  Eigen::Vector3d dl = ((q[i] - p).dot(v)) * v;
  double norm_dl = dl.norm();
  double safe_dist = view_cons_.dir_.norm();
  if (norm_dl < safe_dist) {
    cost += wnl_ * pow(norm_dl - safe_dist, 2);
    gradient_q[i] += wnl_ * 2 * (norm_dl - safe_dist) * vvT * dl / norm_dl;
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
    cost += w_lb * pow(duration - time_lb_, 2);
    gt += w_lb * 2 * (duration - time_lb_) * (point_num_ - order_);
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

void BsplineOptimizer::calcFVBValueAndGradients(const Vector3d& node_pos, const Vector3d& feature, const Vector3d& frontier,
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

    cost += ld_para * cost_para;
    cost += ld_vcv * cost_vcv;

    for (int j = 0; j < 4; j++) {
      gradient_q[i + j] += ld_para * dcost_para_dq[j];
      gradient_q[i + j] += ld_vcv * dcost_vcv_dq[j];
    }
  }
}

void BsplineOptimizer::calcViewFrontierCost(const vector<Vector3d>& q, double& cost, vector<Vector3d>& gradient_q) {
  ros::Time start_time = ros::Time::now();
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  // 遍历每个控制点 q[i]
  vector<Vector3d> frontiers;
  frontier_finder_->getLatestFrontier(frontiers);  // 仅先考虑看向前沿frontiers
  // ROS_WARN("[BsplineOptimizer::calcViewFrontierCost] Debug Message---knot_size: %zu ---ld_this: %.2f ---frontier_this: %zu "
  //          "---Begin Compute ViewFrontierCost!",
  //     q.size(), ld_frontier_visibility_pos_, frontiers.size());

  if (frontiers.empty()) {
    ROS_ERROR("[BsplineOptimizer::calcViewFrontierCost] NO Frontiers!!!!");
    return;
  }

  for (size_t i = 0; i < q.size(); i++) {
    Vector3d node_pos = q[i];  // 当前控制点的位置
    vector<Vector3d> features;
    feature_map_->getFeatures(node_pos, features);  // 得到feature

    if (features.empty()) {
      std::cout << "Knot number " << i << "no feature..........." << endl;
      gradient_q[i] = zero;
      continue;
    }

    // 遍历每个特征点和前沿点
    double good_frontier_num = 0;
    Vector3d good_frontier_gradient = zero;
    for (const auto& frontier : frontiers) {
      double frontier_visual_feature_num = 0;
      Vector3d frontier_visual_feature_gradient = zero;
      for (const auto& feature : features) {
        // 计算 convisual_angle：当前控制点看到特征点和前沿点的夹角
        double convisual_angle;
        Vector3d convisual_angle_gradient;
        calcFVBValueAndGradients(node_pos, feature, frontier, convisual_angle, true, convisual_angle_gradient);

        // 计算代价函数和梯度
        double is_feature_good, is_feature_good_gradient;
        double k1 = 10;
        is_feature_good =
            1.0 /
            (1.0 + std::exp(-k1 * (std::cos(convisual_angle) - std::cos(configPA_.max_feature_and_frontier_convisual_angle_))));
        is_feature_good_gradient = is_feature_good * (1.0 - is_feature_good) * k1 * std::sin(convisual_angle);

        // 计算每个frontier可视的feature的数量
        frontier_visual_feature_num += is_feature_good;
        frontier_visual_feature_gradient += is_feature_good_gradient * convisual_angle_gradient;
      }
      // std::cout << "   frontier_visual_feature_num " << frontier_visual_feature_num << endl;
      double is_frontier_good, is_frontier_good_gradient;
      double k2 = 20;
      is_frontier_good = 1.0 / (1.0 + std::exp(-k2 * (frontier_visual_feature_num - configPA_.min_frontier_see_feature_num_)));
      is_frontier_good_gradient = is_frontier_good * (1.0 - is_frontier_good) * k2;
      good_frontier_num += is_frontier_good;
      good_frontier_gradient += is_frontier_good_gradient * frontier_visual_feature_gradient;
    }
    // std::cout << "    good_frontier_num " << good_frontier_num << " frontiers.size() " << frontiers.size() << endl;
    double good_frontier_percentage = good_frontier_num / frontiers.size();
    Vector3d good_frontier_percentage_gradient = good_frontier_gradient / frontiers.size();

    cost += 1 - good_frontier_percentage;
    gradient_q[i] = -1 * good_frontier_percentage_gradient;
    // std::cout << "Knot number " << i << ": cost_sum " << cost << " gradient_q: " << gradient_q[i].transpose() << endl;
  }

  ros::Time end_time = ros::Time::now();
  ros::Duration elapsed_time = end_time - start_time;
  // ROS_WARN("[BsplineOptimizer::calcViewFrontierCost] Execution time: %.6f seconds", elapsed_time.toSec());
}

void BsplineOptimizer::calcYawCVCostAndGradientsKnots(const vector<Vector3d>& q, const vector<Vector3d>& knots_pos,
    const vector<Vector3d>& knots_acc, const vector<Vector3d>& features, double& cost, vector<Vector3d>& dcost_dq) {

  if (q.size() != 4) ROS_ERROR("Control points set should have exactly 4 points!");

  cost = 0.0;
  dcost_dq.clear();
  for (int i = 0; i < 4; i++) dcost_dq.push_back(Vector3d::Zero());

  Vector3d gravity(0, 0, -9.81);
  double total_weight = 0.0;
  for (const auto& f : features) {
    double w = 1.0;
    vector<double> v3_theta3_vec, v1v2_vec;
    vector<vector<Vector3d>> dv3_dyaw_vec;

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

      // Calculate v1 * v2
      double sin_theta1, v1_theta1, cos_theta2, v2_theta2, v1v2;
      double k1 = 40;
      double k2 = 10;
      double fov_vertical = M_PI / 3.0;
      sin_theta1 = n1.cross(b).norm() / (n1.norm() * b.norm());
      v1_theta1 = 1 / (1 + exp(-k1 * (sin_theta1 - sin((M_PI - fov_vertical) / 2.0))));
      cos_theta2 = n2.dot(b) / (n2.norm() * b.norm());
      v2_theta2 = 1 / (1 + exp(-k2 * cos_theta2));
      v1v2 = v1_theta1 * v2_theta2;

      // Calculate v3(theta3) and gradients
      double sin_theta3, v3_theta3;
      double k3 = 20;
      double fov_horizontal = M_PI / 2.0;
      sin_theta3 = n3.cross(b).norm() / (n3.norm() * b.norm());
      v3_theta3 = 1 / (1 + exp(-k3 * (sin_theta3 - sin((M_PI - fov_horizontal) / 2.0))));

      Eigen::Vector3d dsin_theta3_dn3;
      Eigen::Vector3d c = n3.cross(b);
      double n3_norm, b_norm, c_norm;
      n3_norm = n3.norm();
      b_norm = b.norm();
      c_norm = c.norm();
      dsin_theta3_dn3 = (pow(-n3_norm, 2) * b.cross(c) - pow(c_norm, 2) * n3) / (pow(n3_norm, 3) * b_norm * c_norm);
      double dv3_dsin_theta3 = k3 * exp(-k3 * (sin_theta3 - sin((M_PI - fov_horizontal) / 2.0))) * pow(v3_theta3, 2);

      // Combine gradients using chain rule
      double dv3_dyaw = dv3_dsin_theta3 * dsin_theta3_dn3.dot(dn3_dyaw);

      // Store results
      v1v2_vec.push_back(v1v2);
      v3_theta3_vec.push_back(v3_theta3);
      vector<Eigen::Vector3d> dv3_dyaw_i;
      for (int j = 0; j < 4; j++) {
        Eigen::Vector3d dv3_dqj = Eigen::Vector3d(dv3_dyaw, 0, 0) * dyaw_dq[j];
        dv3_dyaw_i.push_back(dv3_dqj);
      }
      dv3_dyaw_vec.push_back(dv3_dyaw_i);
    }

    // Calculate co-visbility potential cost function and its gradient
    double pot_cost;
    vector<Vector3d> dcovisb_pot_dq_cur;

    // double covisibility_cost = -(v1v2_vec[0] * v1v2_vec[1] * v3_theta3_vec[0] * v3_theta3_vec[1]);
    double covisibility_cost = (v1v2_vec[0] * v1v2_vec[1] * v3_theta3_vec[0] * v3_theta3_vec[1]);

    if (covisibility_cost > 10) {
      ROS_WARN("covisibility_cost: %f", covisibility_cost);

      pot_cost = 0.0;
      for (int j = 0; j < 4; j++) {
        dcovisb_pot_dq_cur.emplace_back(Vector3d::Zero());
      }
    }

    else {
      pot_cost = pow(covisibility_cost - 10, 2);
      for (int j = 0; j < 4; j++) {
        dcovisb_pot_dq_cur.emplace_back(2 * (covisibility_cost - 10) * v1v2_vec[0] * v1v2_vec[1] *
                                        (dv3_dyaw_vec[0][j] * v3_theta3_vec[1] + v3_theta3_vec[0] * dv3_dyaw_vec[1][j]));
      }
    }

    cost = (cost * total_weight + pot_cost * w) / (total_weight + w);

    for (int j = 0; j < 4; j++) {
      // Vector3d dcovisb_pot_dq_cur =
      //     -v1v2_vec[0] * v1v2_vec[1] * (dv3_dyaw_vec[0][j] * v3_theta3_vec[1] + v3_theta3_vec[0] * dv3_dyaw_vec[1][j]);

      dcost_dq[j] = (dcost_dq[j] * total_weight + dcovisb_pot_dq_cur[j] * w) / (total_weight + w);
    }

    total_weight += w;
  }
}

void calcFrontierVisibilityCostAndGradientsKnots(const vector<Vector3d>& q_cur, const Vector3d& knots_pos,
    const Vector3d& knots_acc, const vector<Vector3d>& frontier_cells, double& cost_i, vector<Vector3d>& dcost_dq_i) {

  ROS_ASSERT(q_cur.size() == 3);

  cost_i = 0.0;
  dcost_dq_i.clear();
  for (int i = 0; i < 3; i++) dcost_dq_i.push_back(Vector3d::Zero());

  Vector3d gravity(0, 0, -9.81);
  for (const auto& cell : frontier_cells) {
    // vector<double> v3_theta3_vec, v1v2_vec;
    // vector<vector<Vector3d>> dv3_dyaw_vec;

    Vector3d yaw_knot = (q_cur[0] + 4 * q_cur[1] + q_cur[2]) / 6;
    double yaw = yaw_knot(0);

    // Calculate vectors n1, ny, n3 ,b and their gradients
    Vector3d n1, ny, n3, n2, b;
    n1 = knots_acc - gravity;  // thrust
    ny << cos(yaw), sin(yaw), 0;
    n3 = n1.cross(ny);
    n2 = n3.cross(n1);
    b = cell - knots_pos;

    if (b.norm() < 0.2 || b.norm() > 5.0) continue;

    Vector3d dn3_dyaw;
    dn3_dyaw << -n1(2) * cos(yaw), -n1(2) * sin(yaw), n1(1) * sin(yaw) + n1(0) * cos(yaw);

    // Calculate v1 * v2
    double k1 = 40;
    double k2 = 10;
    double fov_vertical = M_PI / 3.0;

    double sin_theta1, v1_theta1, cos_theta2, v2_theta2, v1v2;
    sin_theta1 = n1.cross(b).norm() / (n1.norm() * b.norm());
    v1_theta1 = 1 / (1 + exp(-k1 * (sin_theta1 - sin((M_PI - fov_vertical) / 2.0))));
    cos_theta2 = n2.dot(b) / (n2.norm() * b.norm());
    v2_theta2 = 1 / (1 + std::exp(-k2 * cos_theta2));
    v1v2 = v1_theta1 * v2_theta2;

    // Calculate v3(theta3) and gradients
    double k3 = 20;
    double fov_horizontal = M_PI / 2.0;

    double sin_theta3, v3_theta3;
    sin_theta3 = n3.cross(b).norm() / (n3.norm() * b.norm());
    v3_theta3 = 1 / (1 + exp(-k3 * (sin_theta3 - sin((M_PI - fov_horizontal) / 2.0))));

    // 到这里已经可以算出cost了
    cost_i = (v1v2 * v3_theta3);

    double n3_norm, b_norm;
    n3_norm = n3.norm();
    b_norm = b.norm();

    Vector3d dsin_theta3_dn3;
    Vector3d c = n3.cross(b);
    double c_norm = c.norm();
    dsin_theta3_dn3 = (pow(-n3_norm, 2) * b.cross(c) - pow(c_norm, 2) * n3) / (pow(n3_norm, 3) * b_norm * c_norm);
    double dv3_dsin_theta3 = k3 * exp(-k3 * (sin_theta3 - sin((M_PI - fov_horizontal) / 2.0))) * pow(v3_theta3, 2);

    // Combine gradients using chain rule
    double dv3_dyaw = dv3_dsin_theta3 * dsin_theta3_dn3.dot(dn3_dyaw);

    Vector3d dyaw_dq = Vector3d(1, 4, 1) / 6;
    for (int j = 0; j < 3; j++) {
      Vector3d dv3_dq = Vector3d(dv3_dyaw, 0, 0) * dyaw_dq[j];
      dcost_dq_i[j] += v1v2 * dv3_dq;
    }
    // vector<Vector3d> dv3_dq;
    // for (int j = 0; j < 3; j++) dv3_dq.emplace_back(Vector3d(dv3_dyaw, 0, 0) * dyaw_dq[j]);
    // for (int j = 0; j < 3; j++) {
    //   // Vector3d dcovisb_pot_dq_cur =
    //   //     -v1v2_vec[0] * v1v2_vec[1] * (dv3_dyaw_vec[0][j] * v3_theta3_vec[1] + v3_theta3_vec[0] * dv3_dyaw_vec[1][j]);

    //   dcost_dq[j] = (dcost_dq[j] * total_weight + dcovisb_pot_dq_cur[j] * w) / (total_weight + w);
    // }

    // cost += cost_i;
    // for (int j = 0; j < 3; j++) {
    //   dcovisb_pot_dq_cur.emplace_back(v1v2 * (dv3_dyaw_vec[0][j] * v3_theta3_vec[1]));
    // }

    // for (int j = 0; j < 4; j++) {
    //   // Vector3d dcovisb_pot_dq_cur =
    //   //     -v1v2_vec[0] * v1v2_vec[1] * (dv3_dyaw_vec[0][j] * v3_theta3_vec[1] + v3_theta3_vec[0] * dv3_dyaw_vec[1][j]);

    //   dcost_dq[j] = (dcost_dq[j] * total_weight + dcovisb_pot_dq_cur[j] * w) / (total_weight + w);
    // }
  }
}

void BsplineOptimizer::calcYawCoVisbilityCost(const vector<Vector3d>& q, double& cost, vector<Vector3d>& gradient_q) {
  // q.size = n+1, pos_.size = n-p+2 = (n+1) - 2, where p = 3

  ROS_ASSERT(q.size() - 2 == pos_.size());

  cost = 0.0;
  Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double cost_i;
  vector<Vector3d> dcost_dq_i;
  for (size_t i = 0; i < q.size() - 3; ++i) {
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

    calcYawCVCostAndGradientsKnots(q_cur, knots_pos, knots_acc, features, cost_i, dcost_dq_i);

    cost += cost_i;
    for (int j = 0; j < 4; j++) gradient_q[i + j] += dcost_dq_i[j];
  }
}

void BsplineOptimizer::calcFrontierVisbilityCost(const vector<Vector3d>& q, double& cost, vector<Vector3d>& gradient_q) {
  // ROS_INFO("calcFrontierVisbilityCost");

  cost = 0.0;
  Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double cost_i;
  vector<Vector3d> dcost_dq_i;
  // cout << "[BsplineOptimizer::calcFrontierVisbilityCost] Begin!: " << endl;

  for (size_t i = 0; i < q.size() - 2; ++i) {
    vector<Vector3d> q_cur;
    for (int j = 0; j < 3; j++) q_cur.push_back(q[i + j]);

    Vector3d knots_pos = pos_[i];
    Vector3d knots_acc = acc_[i];

    calcFrontierVisibilityCostAndGradientsKnots(q_cur, knots_pos, knots_acc, frontier_cells_, cost_i, dcost_dq_i);

    // cout << "[BsplineOptimizer::calcFrontierVisbilityCost] frontier_cells_.size: " << frontier_cells_.size() << endl;

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
    // double f_smoothness = 0.0;
    calcSmoothnessCost(g_q_, f_smoothness_, g_smoothness_);
    f_combine += ld_smooth_ * f_smoothness_;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_smooth_ * g_smoothness_[i](j);
  }

  // Cost2：避障约束
  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance_, g_distance_);
    f_combine += ld_dist_ * f_distance_;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_dist_ * g_distance_[i](j);
  }

  // Cost3：动力学可行性约束
  if (cost_function_ & FEASIBILITY) {
    double gt_feasibility = 0.0;
    calcFeasibilityCost(g_q_, dt, f_feasibility_, g_feasibility_, gt_feasibility);
    f_combine += ld_feasi_ * f_feasibility_;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_feasi_ * g_feasibility_[i](j);
    if (optimize_time_) grad[variable_num_ - 1] += ld_feasi_ * gt_feasibility;
  }

  // Cost4：起始状态约束
  if (cost_function_ & START) {
    double gt_start = 0.0;
    calcStartCost(g_q_, dt, f_start_, g_start_, gt_start);
    f_combine += ld_start_ * f_start_;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_start_ * g_start_[i](j);
    if (optimize_time_) grad[variable_num_ - 1] += ld_start_ * gt_start;
  }

  // Cost5：终止状态约束
  if (cost_function_ & END) {
    double gt_end = 0.0;
    calcEndCost(g_q_, dt, f_end_, g_end_, gt_end);
    f_combine += ld_end_ * f_end_;
    for (int i = point_num_ - 3; i < point_num_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_end_ * g_end_[i](j);

    if (optimize_time_) grad[variable_num_ - 1] += ld_end_ * gt_end;
  }

  // Cost6：引导约束，指定的是控制点约束
  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, f_guide_, g_guide_);
    f_combine += ld_guide_ * f_guide_;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_guide_ * g_guide_[i](j);
  }

  // Cost7：同样是引导约束，不过是使用waypoints，跟上面的区别是上面使用control point，这个使用knot point
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints_, g_waypoints_);
    f_combine += ld_waypt_ * f_waypoints_;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_waypt_ * g_waypoints_[i](j);
  }

  // Cost8：似乎是RAPTOR遗留下来的约束
  if (cost_function_ & VIEWCONS) {
    calcViewCost(g_q_, f_view_, g_view_);
    f_combine += ld_view_ * f_view_;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_view_ * g_view_[i](j);
  }

  // Cost9：最小化时间约束
  if (cost_function_ & MINTIME) {
    double gt_time = 0.0;
    calcTimeCost(dt, f_time_, gt_time);
    f_combine += ld_time_ * f_time_;
    grad[variable_num_ - 1] += ld_time_ * gt_time;
  }

  // SECTION Perception Aware Optimization

  /// APACE新加的
  /// 用于Position Trajectory Optimization阶段
  /// Cost10：视差约束和垂直共视性约束
  if ((cost_function_ & PARALLAX) && (cost_function_ & VERTICALVISIBILITY)) {
    calcPerceptionCost(g_q_, dt, f_parallax_, g_parallax_, ld_parallax_, ld_vertical_visibility_);
    f_combine += f_parallax_;
    for (int i = 0; i < point_num_; i++) {
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += g_parallax_[i](j);
    }
  }
  // Cost11：FRONTIER可见性约束
  if ((cost_function_ & FRONTIERVISIBILITY_POS)) {
    calcViewFrontierCost(g_q_, f_frontier_visibility_pos_, g_frontier_visibility_pos_);
    f_combine += ld_frontier_visibility_pos_ * f_frontier_visibility_pos_;
    for (int i = 0; i < point_num_; i++) {
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_frontier_visibility_pos_ * g_frontier_visibility_pos_[i](j);
    }
  }
  /// 用于Yaw Trajectory Optimization阶段
  /// Cost12：yaw共视性约束
  if (cost_function_ & YAWCOVISIBILITY) {
    calcYawCoVisbilityCost(g_q_, f_yaw_covisibility_, g_yaw_covisibility_);
    f_combine += ld_yaw_covisib_ * f_yaw_covisibility_;
    for (int i = 0; i < point_num_; i++) {
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_yaw_covisib_ * g_yaw_covisibility_[i](j);
    }
  }

  /// 我们新加的
  ///  Cost13：frontier可见性约束
  if (cost_function_ & FRONTIERVISIBILITY_YAW) {
    double f_frontier_visibility_yaw = 0.0;
    calcFrontierVisbilityCost(g_q_, f_frontier_visibility_yaw_, g_frontier_visibility_yaw_);
    // cout << "frontier_cells_ size: " << frontier_cells_.size() << endl;
    // cout << "f_frontier_visibility: " << f_frontier_visibility_yaw << endl;
    // cout << "g_frontier_visibility_: " << endl;
    // for (const auto& g : g_frontier_visibility_yaw_) {
    //   cout << g.transpose() << endl;
    // }
    f_combine += ld_frontier_visibility_yaw_ * f_frontier_visibility_yaw_;
    for (int i = 0; i < point_num_; i++) {
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += ld_frontier_visibility_yaw_ * g_frontier_visibility_yaw_[i](j);
    }
  }

  // !SECTION

  comb_time += (ros::Time::now() - t1).toSec();

  cout << "---------------------------------------------------------------------------" << endl;
  cout << "[bspline optimizer]:iter_num: " << iter_num_ << endl;
  cout << "[bspline optimizer]:total cost: " << f_combine << endl;

  if (cost_function_ & SMOOTHNESS) cout << "[bspline optimizer]:smoothness cost: " << f_smoothness_ << endl;
  if (cost_function_ & DISTANCE) cout << "[bspline optimizer]:distance cost: " << f_distance_ << endl;
  if (cost_function_ & FEASIBILITY) cout << "[bspline optimizer]:feasibility cost: " << f_feasibility_ << endl;
  if (cost_function_ & START) cout << "[bspline optimizer]:start cost: " << f_start_ << endl;
  if (cost_function_ & END) cout << "[bspline optimizer]:end cost: " << f_end_ << endl;
  if (cost_function_ & GUIDE) cout << "[bspline optimizer]:guide cost: " << f_guide_ << endl;
  if (cost_function_ & WAYPOINTS) cout << "[bspline optimizer]:waypoints cost: " << f_waypoints_ << endl;
  if (cost_function_ & VIEWCONS) cout << "[bspline optimizer]:view cost: " << f_view_ << endl;
  if (cost_function_ & MINTIME) cout << "[bspline optimizer]:time cost: " << f_time_ << endl;
  if ((cost_function_ & PARALLAX) && (cost_function_ & VERTICALVISIBILITY))
    cout << "[bspline optimizer]:parallax cost: " << f_parallax_ << endl;
  if (cost_function_ & FRONTIERVISIBILITY_POS)
    cout << "[bspline optimizer]:view frontier(pos) cost: " << f_frontier_visibility_pos_ << endl;
  if (cost_function_ & YAWCOVISIBILITY) cout << "[bspline optimizer]:yaw covisibility cost: " << f_yaw_covisibility_ << endl;
  if (cost_function_ & FRONTIERVISIBILITY_YAW)
    cout << "[bspline optimizer]:view frontier(yaw) cost: " << f_frontier_visibility_yaw_ << endl;
  cout << "---------------------------------------------------------------------------" << endl;
}

double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data) {
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  // double f_smoothness_;
  // double f_distance_;
  // double f_feasibility_;
  // double f_start_;
  // double f_end_;
  // double f_guide_;
  // double f_waypoints_;
  // double f_view_;
  // double f_time_;
  // double f_parallax_;
  // double f_frontier_visibility_pos_;
  // double f_yaw_covisibility_;
  // double f_frontier_visibility_yaw_;

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