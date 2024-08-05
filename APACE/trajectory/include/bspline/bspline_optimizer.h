#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include "parallax_util.h"
#include "plan_env/edt_environment.h"
#include "traj_visibility.h"

#include <Eigen/Eigen>
#include <nlopt.hpp>
#include <ros/ros.h>

#include <thread>

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace fast_planner
{
  class MapServer;

  class BsplineOptimizer
  {
  public:
    static const int SMOOTHNESS;
    static const int DISTANCE;
    static const int FEASIBILITY;
    static const int START;
    static const int END;
    static const int GUIDE;
    static const int WAYPOINTS;
    static const int MINTIME;
    static const int PARALLAX;
    static const int VERTICALVISIBILITY;
    static const int YAWCOVISIBILITY;

    static const int GUIDE_PHASE;
    static const int NORMAL_PHASE;

    using ParallaxUtilPtr = shared_ptr<ParallaxUtil>;

    /* main API */
    void setEnvironment(const shared_ptr<EDTEnvironment> &env);
    void setParam(ros::NodeHandle &nh);
    void initParallaxUtil(ros::NodeHandle &nh);

    /**
     * 外部调用B样条曲线优化的API
     *
     * @param points 这条B样条曲线的控制点，列数代表维度（一般为3），行数代表控制点个数
     * @param dt B样条曲线的节点间隔（knot span)
     * @param cost_function 指定要添加哪些约束
     */
    void optimize(Eigen::MatrixXd &points, double &dt, const int &cost_function);

    /**
     * @brief 上面那个optimize把必要的数据和标志位处理好后进入这个函数
     *
     * @note 然而这个函数也不是重头戏，它主要是进一步把完成了NLopt优化器的初始化和配置
     */
    void optimize();

    void setCostFunction(const int &cost_function);
    void setBoundaryStates(const vector<Eigen::Vector3d> &start, const vector<Eigen::Vector3d> &end, const vector<bool> &start_idx, const vector<bool> &end_idx);
    void setTimeLowerBound(const double &lb);

    // optional inputs
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // N-2 constraints at most
    void setPosAndAcc(const vector<Eigen::Vector3d> &pos, const vector<Eigen::Vector3d> &acc,
                      const vector<int> &idx = vector<int>());
    void setViewConstraint(const ViewConstraint &vc);
    void setParallaxUtil(const ParallaxUtilPtr &pu);

    void debugVisualization(const std::vector<Eigen::Vector3d> &q,
                            const std::vector<Eigen::Vector3d> &q_grad);

    Eigen::MatrixXd getControlPoints();

  private:
    // Wrapper of cost function

    /**
     * Calculates the cost function value and gradient for the optimization problem.
     *
     * @param x The input vector.
     * @param grad The gradient vector.
     * @param func_data Additional data required for the cost function calculation.
     * @return The cost function value.
     */
    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);

    /**
     * @brief 由costFunction函数调用，这个函数里头会分别调用各个约束的计算函数并进行求和
     *
     * @note 这个函数是真正的重头戏，精华所在
     *
     * @param x[in] 优化变量x
     * @param grad[out] 梯度g
     * @param cost[out] 最终计算得到的cost值
     */
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

    // Cost functions, q: control points, dt: knot span
    void calcSmoothnessCost(const vector<Eigen::Vector3d> &q, double &cost, vector<Eigen::Vector3d> &gradient_q);
    void calcDistanceCost(const vector<Eigen::Vector3d> &q, double &cost,
                          vector<Eigen::Vector3d> &gradient_q);
    void calcFeasibilityCost(const vector<Eigen::Vector3d> &q, const double &dt, double &cost,
                             vector<Eigen::Vector3d> &gradient_q, double &gt);
    void calcStartCost(const vector<Eigen::Vector3d> &q, const double &dt, double &cost,
                       vector<Eigen::Vector3d> &gradient_q, double &gt);
    void calcEndCost(const vector<Eigen::Vector3d> &q, const double &dt, double &cost,
                     vector<Eigen::Vector3d> &gradient_q, double &gt);
    void calcGuideCost(const vector<Eigen::Vector3d> &q, double &cost, vector<Eigen::Vector3d> &gradient_q);
    void calcWaypointsCost(const vector<Eigen::Vector3d> &q, double &cost, vector<Eigen::Vector3d> &gradient_q);
    void calcTimeCost(const double &dt, double &cost, double &gt);
    void calcPerceptionCost(const vector<Eigen::Vector3d> &q, const double &dt, double &cost,
                            vector<Eigen::Vector3d> &gradient_q, const double ld_para,
                            const double ld_vcv);
    void calcYawCoVisbilityCost(const vector<Eigen::Vector3d> &q, const double &dt, double &cost, vector<Eigen::Vector3d> &gradient_q);

    bool isQuadratic();

    shared_ptr<EDTEnvironment> edt_environment_;

    // Optimized variables
    Eigen::MatrixXd control_points_; // B-spline control points, N x dim
    double knot_span_;               // B-spline knot span

    // Input to solver
    int dim_; // dimension of the B-spline
    vector<Eigen::Vector3d> start_state_, end_state_;
    vector<bool> start_con_index_, end_con_index_; // whether constraint on (pos, vel, acc)
    vector<Eigen::Vector3d> guide_pts_;            // geometric guiding path points, N-6
    vector<Eigen::Vector3d> waypoints_;            // waypts constraints
    vector<int> waypt_idx_;
    vector<Eigen::Vector3d> pos_, acc_;                // knot points position and acceleration
    vector<vector<Eigen::Vector3d>> knot_nn_features_; // neighboring features at each knot midpoint
    vector<int> pos_idx_;
    int max_num_id_, max_time_id_; // stopping criteria
    int cost_function_;
    double time_lb_;
    double start_time_; // global time for moving obstacles

    /* Parameters of optimization  */
    int order_; // bspline degree
    int bspline_degree_;
    double ld_smooth_, ld_dist_, ld_feasi_, ld_start_, ld_end_, ld_guide_, ld_waypt_, ld_view_,
        ld_time_, ld_yaw_feasi_, ld_parallax_, ld_vertical_visibility_, ld_yaw_covisib_;
    double dist0_;                                                     // safe distance
    double max_vel_, max_acc_;                                         // dynamic limits
    double max_vel_yaw_;                                               // dynamic limits for yaw
    double thresh_info_, quad_k_info_, linear_k_info_, linear_b_info_; // info potential function
    double wnl_, dlmin_;
    int algorithm1_;               // optimization algorithms for quadratic cost
    int algorithm2_;               // optimization algorithms for general cost
    int max_iteration_num_[4];     // stopping criteria that can be used
    double max_iteration_time_[4]; // stopping criteria that can be used

    // Data of opt
    vector<Eigen::Vector3d> g_q_, g_smoothness_, g_distance_, g_feasibility_, g_start_, g_end_,
        g_guide_, g_waypoints_, g_view_, g_time_, g_yaw_feasibility_, g_parallax_, g_vertical_visib_,
        g_yaw_covisib_;

    int variable_num_; // optimization variables
    int point_num_;
    bool optimize_time_;
    std::vector<double> best_variable_; //
    double min_cost_;                   //
    ViewConstraint view_cons_;
    ParallaxUtilPtr parallax_util_;
    double pt_dist_;

    bool vis_;
    ros::Publisher vis_pub_;

    /* for benckmark evaluation only */
  public:
    vector<double> vec_cost_;
    vector<double> vec_time_;
    ros::Time time_start_;

    void getCostCurve(vector<double> &cost, vector<double> &time)
    {
      cost = vec_cost_;
      time = vec_time_;
    }

    int getWaypointsNumber() { return waypoints_.size(); }

    typedef unique_ptr<BsplineOptimizer> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace fast_planner
#endif