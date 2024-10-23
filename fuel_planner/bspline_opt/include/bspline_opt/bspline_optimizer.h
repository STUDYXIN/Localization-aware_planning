#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <plan_env/edt_environment.h>
#include <plan_env/utils.hpp>
#include <ros/ros.h>

#include <Eigen/Eigen>

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point

using std::vector;

using Eigen::Vector3d;
namespace fast_planner {
class EDTEnvironment;
class FeatureMap;
class FrontierFinder;
class SteppingDebug;

class BsplineOptimizer {
public:
  static const int SMOOTHNESS;
  static const int DISTANCE;
  static const int FEASIBILITY;
  static const int FEASIBILITY_YAW;
  static const int START;
  static const int END;
  static const int GUIDE;
  static const int WAYPOINTS;
  static const int MINTIME;
  static const int PARALLAX;
  static const int VERTICALVISIBILITY;
  static const int YAWCOVISIBILITY;
  static const int FRONTIERVISIBILITY_POS;
  static const int FRONTIERVISIBILITY_YAW;
  static const int FINAL_GOAL;

  static const int GUIDE_PHASE;
  static const int NORMAL_PHASE;

  struct PerceptionAwareConfig {
    double estimator_freq_;  // frequency of consecutive frames in hz
    double max_parallax_;    // max parallax angle between consecutive frames in rad
    double pot_a_;           // potential func: a(x-max_parallax_)^2
    double min_covisible_feature_cost_;
    double max_feature_and_frontier_convisual_angle_;
    double min_frontier_see_feature_num_;
    double pot_fafv_;
    double k1_;
    double k2_;
    double k3_;
  };

  PerceptionAwareConfig configPA_;

  /* main API */
  void setEnvironment(const shared_ptr<EDTEnvironment>& env);
  void setParam(ros::NodeHandle& nh);
  void optimize(Eigen::MatrixXd& points, double& dt, const int& cost_function, const int& max_num_id, const int& max_time_id);

  /* helper function */

  // required inputs
  void setCostFunction(const int& cost_function);
  void setBoundaryStates(const vector<Vector3d>& start, const vector<Vector3d>& end);
  void setBoundaryStates(
      const vector<Vector3d>& start, const vector<Vector3d>& end, const vector<bool>& start_idx, const vector<bool>& end_idx);
  void setTimeLowerBound(const double& lb);

  // optional inputs
  void setGuidePath(const vector<Vector3d>& guide_pt);
  void setWaypoints(const vector<Vector3d>& waypts,
      const vector<int>& waypt_idx);  // N-2 constraints at most
  void enableDynamic(double time_start);

  // SECTION Perception Aware Optimization

  void setPosAndAcc(const vector<Vector3d>& pos, const vector<Vector3d>& acc, const vector<int>& idx = vector<int>());

  // !SECTION

  void optimize();

  Eigen::MatrixXd getControlPoints();
  vector<Eigen::Vector3d> matrixToVectors(const Eigen::MatrixXd& ctrl_pts);

private:
  // Wrapper of cost function
  static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
  void combineCost(const std::vector<double>& x, vector<double>& grad, double& cost);

  // Cost functions, q: control points, dt: knot span
  void calcSmoothnessCost(const vector<Vector3d>& q, double& cost, vector<Vector3d>& gradient_q);
  void calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient_q);
  void calcFeasibilityCost(const vector<Vector3d>& q, const double& dt, double& cost, vector<Vector3d>& gradient_q, double& gt);
  void calcFeasibilityCostYaw(
      const vector<Vector3d>& q, const double& dt, double& cost, vector<Vector3d>& gradient_q, double& gt);
  void calcStartCost(const vector<Vector3d>& q, const double& dt, double& cost, vector<Vector3d>& gradient_q, double& gt);
  void calcEndCost(const vector<Vector3d>& q, const double& dt, double& cost, vector<Vector3d>& gradient_q, double& gt);
  void calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient_q);
  void calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient_q);
  void calcViewCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient_q);
  void calcTimeCost(const double& dt, double& cost, double& gt);

  // SECTION Perception Aware Optimization
  void calcParaValueAndGradients(
      const Vector3d& vfea, const Vector3d& vfron, double& parallax, bool calc_grad, Vector3d& dpara_dv1, Vector3d& dpara_dv2);
  void calcParaPotentialAndGradients(const double parallax, const double dt, double& para_pot, double& dpot_dpara);

  double calcVCWeight(const Vector3d& knot, const Vector3d& f, const Vector3d& thrust_dir);

  // 位置轨迹规划阶段考虑frontier可见性
  void calcffAngleValueAndGradients(const Vector3d& node_pos, const Vector3d& feature, const Vector3d& frontier,
      double& convisual_angle, bool calc_grad, Eigen::Vector3d& dfvb_dq);
  void calcFVBCostAndGradientsKnots(const vector<Vector3d>& q, const Vector3d& knots_pos, const vector<Vector3d>& features,
      double& cost, vector<Vector3d>& dcost_dq);
  void calcFVBCostAndGradientsKnots(
      const vector<Vector3d>& q, const Vector3d& knots_pos, double& cost, vector<Vector3d>& dcost_dq);
  // EXPLORE UNKNOWN AREA
  void calcEUACostAndGradientsKnots(const vector<Vector3d>& q, const double& knot_span, double& cost, vector<Vector3d>& dcost_dq);
  void calcEUACoefficient(const vector<Vector3d>& q, const double& knot_span, vector<double>& coefficient, const bool& use_grad,
      vector<vector<Eigen::Vector3d>>& dcoefficient_dq);
  // ===========
  void calcParaCostAndGradientsKnots(const vector<Vector3d>& q, const double dt, const vector<Vector3d>& features, double& cost,
      vector<Vector3d>& dcost_dq, const bool& use_hard_constraint);

  void calcVVValueAndGradients(
      const Vector3d& a, const Vector3d& b, double& cos_theta, bool calc_grad, Vector3d& dcos_theta_da, Vector3d& dcos_theta_db);

  void calcVVPotentialAndGradients(const double cos_theta, double& cos_theta_pot, double& dpot_dcos_theta);

  // 计算单个pos knot垂直共视性cost
  void calcVCVCostAndGradientsKnots(const vector<Vector3d>& q, const double& knot_span, const vector<Vector3d> features,
      double& cost, vector<Vector3d>& dcost_dq, const bool& use_hard_constrai);

  // 计算整条位置轨迹的percpetion aware cost，包含视差和垂直共视性cost
  void calcPerceptionCost(const vector<Vector3d>& q, const double& dt, double& cost, vector<Vector3d>& gradient_q,
      const double ld_para, const double ld_vcv, const bool& use_hard_constraint);

  void calcFVBCost(const vector<Vector3d>& q, double& cost, vector<Vector3d>& gradient_q);
  void calcEUACost(const vector<Vector3d>& q, const double& dt, double& cost, vector<Vector3d>& gradient_q);

  void calcYawCVCostAndGradientsKnots(const vector<Vector3d>& q, const vector<Vector3d>& knots_pos,
      const vector<Vector3d>& knots_acc, const vector<Vector3d>& features, double& pot_cost, vector<Vector3d>& dpot_dq);

  // 计算整条yaw轨迹的共视性
  void calcYawCoVisbilityCost(const vector<Vector3d>& q, double& cost, vector<Vector3d>& gradient_q);

  void calcFrontierVisibilityCostAndGradientsKnots(const vector<Vector3d>& q_cur, const Vector3d& knots_pos,
      const Vector3d& knots_acc, const int layer, double& cost_i, vector<Vector3d>& dcost_dq_i);

  void calcFrontierVisbilityCostYaw(const vector<Vector3d>& q, double& cost, vector<Vector3d>& gradient_q);

  void calcFinalGoalCostKnots(const vector<Vector3d>& q_cur, const Vector3d& knots_pos, const Vector3d& knots_acc,
      const int layer, double& cost, vector<Vector3d>& dcost_dq);
  void calcFinalGoalCostYaw(const vector<Vector3d>& q, double& cost, vector<Vector3d>& gradient_q);

  static Vector3d getThrustDirection(const Vector3d& acc) {
    Vector3d gravity(0, 0, -9.81);
    Vector3d thrust_dir = (acc - gravity).normalized();
    return thrust_dir;
  }
  // !SECTION

  bool isQuadratic();

  shared_ptr<EDTEnvironment> edt_environment_;
  shared_ptr<FrontierFinder> frontier_finder_;

  // Optimized variables
  Eigen::MatrixXd control_points_;  // B-spline control points, N x dim
  double knot_span_;                // B-spline knot span

  // Input to solver
  int dim_;  // dimension of the B-spline
  vector<Eigen::Vector3d> start_state_, end_state_;
  vector<bool> start_con_index_,
      end_con_index_;                  // whether constraint on (pos, vel, acc)
  vector<Eigen::Vector3d> guide_pts_;  // geometric guiding path points, N-6
  vector<Eigen::Vector3d> waypoints_;  // waypts constraints
  vector<int> waypt_idx_;
  int max_num_id_, max_time_id_;  // stopping criteria
  int cost_function_;
  double time_lb_;
  bool dynamic_;       // moving obstacles ?
  double start_time_;  // global time for moving obstacles

  /* Parameters of optimization  */
  int order_;  // bspline degree
  int bspline_degree_;
  double ld_smooth_, ld_dist_, ld_feasi_, ld_feasi_yaw_, ld_start_, ld_end_, ld_guide_, ld_waypt_, ld_view_, ld_time_;

  // SECTION Perception Aware Optimization
  double ld_parallax_;
  double ld_vertical_visibility_;
  double ld_frontier_visibility_pos_;
  double ld_yaw_covisib_;
  double ld_frontier_visibility_yaw_;
  double ld_final_goal_;
  double ld_weight1_, ld_weight2_;

  vector<Eigen::Vector3d> pos_, acc_;                 // knot points position and acceleration
  vector<vector<Eigen::Vector3d>> knot_nn_features_;  // neighboring features at each knot midpoint
  vector<int> pos_idx_;

  // !SECTION

  double dist0_;              // safe distance
  double max_vel_, max_acc_;  // dynamic limits
  double wnl_, dlmin_;
  int algorithm1_;                // optimization algorithms for quadratic cost
  int algorithm2_;                // optimization algorithms for general cost
  int max_iteration_num_[4];      // stopping criteria that can be used
  double max_iteration_time_[4];  // stopping criteria that can be used

  // Data of opt
  vector<Eigen::Vector3d> g_q_, g_smoothness_, g_distance_, g_feasibility_, g_feasibility_yaw_, g_start_, g_end_, g_guide_,
      g_waypoints_, g_view_, g_time_;
  void printVector(const std::vector<Eigen::Vector3d>& vec, const std::string& name) {
    std::cout << name << " contains " << vec.size() << " elements:\n";
    for (const auto& elem : vec) {
      std::cout << "[" << elem.x() << ", " << elem.y() << ", " << elem.z() << "]\n";
    }
  }
  double f_smoothness_;
  double f_distance_;
  double f_feasibility_;
  double f_feasibility_yaw_;
  double f_start_;
  double f_end_;
  double f_guide_;
  double f_waypoints_;
  double f_view_;
  double f_time_;
  double f_parallax_;
  double f_frontier_visibility_pos_;
  double f_yaw_covisibility_;
  double f_frontier_visibility_yaw_;

  // SECTION Perception Aware Optimization
  vector<Vector3d> g_parallax_;
  vector<Vector3d> g_frontier_visibility_pos_;
  vector<Vector3d> g_yaw_covisibility_;
  vector<Vector3d> g_frontier_visibility_yaw_;

  // !SECTION

  int variable_num_;  // optimization variables
  int point_num_;
  bool optimize_time_;
  int iter_num_;                       // iteration of the solver
  std::vector<double> best_variable_;  //
  double min_cost_;                    //
  double pt_dist_;

  /* for benckmark evaluation only */
public:
  vector<double> vec_cost_;
  vector<double> vec_time_;
  bool issuccess;
  ros::Time time_start_;

  // SECTION Perception Aware Optimization
  shared_ptr<FeatureMap> feature_map_;

  void setFeatureMap(shared_ptr<FeatureMap>& feature_map) {
    feature_map_ = feature_map;
  }

  void setFrontierFinder(shared_ptr<FrontierFinder> frontier_finder) {
    frontier_finder_ = frontier_finder;
  }

  Vector3d frontier_centre_, view_point_pos_;
  double view_point_yaw_;

  void setFrontierCells(const vector<Vector3d>& frontier_cells) {
    frontier_cells_ = frontier_cells;
  }

  void setFrontiercenter(const Vector3d& frontier_centre) {
    frontier_centre_ = frontier_centre;
  }

  void setViewpoint(const Vector3d& view_point_pos, double view_point_yaw) {
    view_point_pos_ = view_point_pos;
    view_point_yaw_ = view_point_yaw;
  }

  CameraParam::Ptr camera_param_ = nullptr;
  vector<Vector3d> frontier_cells_;

  // vector<vector<int>> frontier_status_;

  // vector<vector<Vector3d>> observed_features_;

  YawOptData::Ptr opt_data_ = nullptr;
  void setOptData(const YawOptData::Ptr& data) {
    opt_data_ = data;
  }

  // !SECTION

  // SECTION Debug msg
  shared_ptr<SteppingDebug> stepping_debug_;
  void getSteppingDebug(shared_ptr<SteppingDebug> stepping_debug) {
    stepping_debug_ = stepping_debug;
  }
  // !SECTION

  void resetCostAndGrad();

  void getCostCurve(vector<double>& cost, vector<double>& time) {
    cost = vec_cost_;
    time = vec_time_;
  }

  double comb_time;

  typedef unique_ptr<BsplineOptimizer> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace fast_planner
#endif