#include "bspline/bspline_optimizer.h"

namespace fast_planner
{
  const int BsplineOptimizer::SMOOTHNESS = (1 << 0);
  const int BsplineOptimizer::DISTANCE = (1 << 1);
  const int BsplineOptimizer::FEASIBILITY = (1 << 2);
  const int BsplineOptimizer::START = (1 << 3);
  const int BsplineOptimizer::END = (1 << 4);
  const int BsplineOptimizer::GUIDE = (1 << 5);
  const int BsplineOptimizer::WAYPOINTS = (1 << 6);
  const int BsplineOptimizer::MINTIME = (1 << 8);

  const int BsplineOptimizer::PARALLAX = (1 << 11);
  const int BsplineOptimizer::VERTICALVISIBILITY = (1 << 12);
  const int BsplineOptimizer::YAWCOVISIBILITY = (1 << 13);

  const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE | BsplineOptimizer::START | BsplineOptimizer::END;

  const int BsplineOptimizer::NORMAL_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY |
                                             BsplineOptimizer::START | BsplineOptimizer::END;

  void BsplineOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/visualization", vis_, false);

    nh.param("optimization/ld_smooth", ld_smooth_, -1.0);
    nh.param("optimization/ld_dist", ld_dist_, -1.0);
    nh.param("optimization/ld_feasi", ld_feasi_, -1.0);
    nh.param("optimization/ld_start", ld_start_, -1.0);
    nh.param("optimization/ld_end", ld_end_, -1.0);
    nh.param("optimization/ld_guide", ld_guide_, -1.0);
    nh.param("optimization/ld_waypt", ld_waypt_, -1.0);
    nh.param("optimization/ld_view", ld_view_, -1.0);
    nh.param("optimization/ld_time", ld_time_, -1.0);
    nh.param("optimization/ld_yaw_feasi", ld_yaw_feasi_, -1.0);
    nh.param("optimization/ld_parallax", ld_parallax_, -1.0);
    nh.param("optimization/ld_vertical_visibility", ld_vertical_visibility_, -1.0);
    nh.param("optimization/ld_yaw_covisibility", ld_yaw_covisib_, -1.0);

    nh.param("optimization/dist0", dist0_, -1.0);
    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);
    nh.param("optimization/max_vel_yaw", max_vel_yaw_, -1.0);
    nh.param("optimization/dlmin", dlmin_, -1.0);
    nh.param("optimization/wnl", wnl_, -1.0);

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

    time_lb_ = -1; // Not used by in most case
    vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("optimization/debug_vis", 10);
  }

  void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr &env)
  {
    this->edt_environment_ = env;
  }

  void BsplineOptimizer::initParallaxUtil(ros::NodeHandle &nh)
  {
    ParallaxConfig config;
    nh.param("optimization/parallax/estimator_freq", config.estimator_freq_, -1.0);
    nh.param("optimization/parallax/max_parallax", config.max_parallax_, -1.0);
    nh.param("optimization/parallax/pot_a", config.pot_a_, -1.0);
    parallax_util_.reset(new ParallaxUtil(config));
  }

  void BsplineOptimizer::setCostFunction(const int &cost_code)
  {
    cost_function_ = cost_code;

    // Print cost function
    bool verbose = false;
    if (verbose)
    {
      string cost_str;
      if (cost_function_ & SMOOTHNESS)
        cost_str += "smooth | ";
      if (cost_function_ & DISTANCE)
        cost_str += " dist | ";
      if (cost_function_ & FEASIBILITY)
        cost_str += " feasi | ";
      if (cost_function_ & START)
        cost_str += " start | ";
      if (cost_function_ & END)
        cost_str += " end | ";
      if (cost_function_ & GUIDE)
        cost_str += " guide | ";
      if (cost_function_ & WAYPOINTS)
        cost_str += " waypt | ";
      if (cost_function_ & MINTIME)
        cost_str += " time | ";
      if (cost_function_ & PARALLAX)
        cost_str += " parallax | ";
      if (cost_function_ & VERTICALVISIBILITY)
        cost_str += " veritcal_visibility | ";
      if (cost_function_ & YAWCOVISIBILITY)
        cost_str += " yaw_covisibility | ";
      ROS_INFO_STREAM("cost func: " << cost_str);
    }
  }

  void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d> &guide_pt)
  {
    guide_pts_ = guide_pt;
  }

  void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d> &waypts, const vector<int> &waypt_idx)
  {
    waypoints_ = waypts;
    waypt_idx_ = waypt_idx;
  }

  void BsplineOptimizer::setPosAndAcc(const vector<Eigen::Vector3d> &pos,
                                      const vector<Eigen::Vector3d> &acc, const vector<int> &idx)
  {
    pos_ = pos;
    acc_ = acc;
    pos_idx_ = idx;
  }
  void BsplineOptimizer::setViewConstraint(const ViewConstraint &vc) { view_cons_ = vc; }

  void BsplineOptimizer::setParallaxUtil(const ParallaxUtilPtr &pu) { parallax_util_ = pu; }

  void BsplineOptimizer::setBoundaryStates(const vector<Eigen::Vector3d> &start,
                                           const vector<Eigen::Vector3d> &end,
                                           const vector<bool> &start_idx,
                                           const vector<bool> &end_idx)
  {
    start_state_ = start;
    end_state_ = end;
    start_con_index_ = start_idx;
    end_con_index_ = end_idx;
  }

  void BsplineOptimizer::setTimeLowerBound(const double &lb) { time_lb_ = lb; }

  void BsplineOptimizer::optimize(Eigen::MatrixXd &points, double &dt, const int &cost_function)
  {
    if (start_state_.empty())
    {
      ROS_ERROR("Initial state undefined!");
      return;
    }

    control_points_ = points;
    knot_span_ = dt;
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
    if (variable_num_ <= 0)
    {
      ROS_ERROR("Empty varibale to optimization solver.");
      return;
    }

    pt_dist_ = 0.0;
    for (int i = 0; i < control_points_.rows() - 1; ++i)
      pt_dist_ += (control_points_.row(i + 1) - control_points_.row(i)).norm();
    pt_dist_ /= point_num_;

    min_cost_ = std::numeric_limits<double>::max();
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
    g_yaw_feasibility_.resize(point_num_);
    g_parallax_.resize(point_num_);
    g_vertical_visib_.resize(point_num_);
    g_yaw_covisib_.resize(point_num_);

    optimize();

    points = control_points_;
    dt = knot_span_;
    start_state_.clear();
    start_con_index_.clear();
    end_con_index_.clear();
    time_lb_ = -1;
  }

  void BsplineOptimizer::optimize()
  {
    // Optimize all control points and maybe knot span dt
    // Use NLopt solver

    // Step1：初始化NLopt非线性优化器
    nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
    opt.set_min_objective(BsplineOptimizer::costFunction, this);
    opt.set_xtol_rel(1e-4);

    // Set axis aligned bounding box for optimization

    // Step2：把control_points_的数据搬运到q中，记得考虑上下限
    Eigen::Vector3d bmin, bmax;
    edt_environment_->map_server_->getBox(bmin, bmax);
    for (int k = 0; k < 3; ++k)
    {
      bmin[k] += 0.1;
      bmax[k] -= 0.1;
    }

    vector<double> q(variable_num_);
    for (int i = 0; i < point_num_; ++i)
    {
      for (int j = 0; j < dim_; ++j)
      {
        double cij = control_points_(i, j);
        if (dim_ != 1)
          cij = max(min(cij, bmax[j % 3]), bmin[j % 3]);
        q[dim_ * i + j] = cij;
      }
    }
    // 如果要优化knot_span，加一个优化变量
    if (optimize_time_)
      q[variable_num_ - 1] = knot_span_;

    // Step3：为NLopt优化器设置优化的上下限
    if (dim_ != 1)
    {
      vector<double> lb(variable_num_), ub(variable_num_);
      const double bound = 10.0;
      for (int i = 0; i < 3 * point_num_; ++i)
      {
        lb[i] = q[i] - bound;
        ub[i] = q[i] + bound;
        lb[i] = max(lb[i], bmin[i % 3]);
        ub[i] = min(ub[i], bmax[i % 3]);
      }
      if (optimize_time_)
      {
        lb[variable_num_ - 1] = 0.0;
        ub[variable_num_ - 1] = 5.0;
      }
      opt.set_lower_bounds(lb);
      opt.set_upper_bounds(ub);
    }

    // Step4：正式进行优化
    try
    {
      double final_cost;
      nlopt::result result = opt.optimize(q, final_cost);
    }
    catch (std::exception &e)
    {
      cout << e.what() << endl;
    }

    // Step5：把优化变量反写回control_points_和knot_span_
    // Note：best_variable_在优化过程中就不断更新
    for (int i = 0; i < point_num_; ++i)
    {
      for (int j = 0; j < dim_; ++j)
        control_points_(i, j) = best_variable_[dim_ * i + j];
    }

    if (optimize_time_)
      knot_span_ = best_variable_[variable_num_ - 1];
  }

  void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d> &q, double &cost, vector<Eigen::Vector3d> &gradient_q)
  {
    cost = 0.0;
    Eigen::Vector3d zero(0, 0, 0);
    std::fill(gradient_q.begin(), gradient_q.end(), zero);
    Eigen::Vector3d jerk, temp_j;

    for (int i = 0; i < q.size() - 3; i++)
    {
      /* evaluate jerk */
      // 3-rd order derivative = 1/(ts)^3*(q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i])

      // 这里用三阶差分也就是jerk来当作最小化目标，然后这个pt_dist_我不知道为啥要过来
      Eigen::Vector3d ji = (q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i]) / pt_dist_;
      cost += ji.squaredNorm();
      temp_j = 2 * ji / (pt_dist_); // 这里原本只有一个pt_dist_感觉写错了

      /* jerk gradient_q */
      // d cost / d q[i] = d cost / d jerk * d jerk / d q[i]
      // gradient_q = d cost / d q[i] for each i

      // 这里使用链式求导法则做梯度计算
      gradient_q[i + 0] += -temp_j;
      gradient_q[i + 1] += 3.0 * temp_j;
      gradient_q[i + 2] += -3.0 * temp_j;
      gradient_q[i + 3] += temp_j;
    }
  }

  void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d> &q, double &cost, vector<Eigen::Vector3d> &gradient_q)
  {
    cost = 0.0;
    Eigen::Vector3d zero(0, 0, 0);
    std::fill(gradient_q.begin(), gradient_q.end(), zero);

    double dist;
    Eigen::Vector3d ddist_dq;
    for (int i = 0; i < q.size(); i++)
    {
      // TODO：这里把距离和梯度都丢给了EDT的接口去计算，这里我还没有看懂
      // 更新：距离怎么算的看懂了，算梯度太复杂了
      edt_environment_->evaluateEDTWithGrad(q[i], dist, ddist_dq);
      if (ddist_dq.norm() > 1e-4)
        ddist_dq.normalize();

      if (dist < dist0_)
      {
        cost += pow(dist - dist0_, 2);
        gradient_q[i] += 2.0 * (dist - dist0_) * ddist_dq;
      }
    }
  }

  void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d> &q, const double &dt,
                                             double &cost, vector<Eigen::Vector3d> &gradient_q,
                                             double &gt)
  {
    cost = 0.0;
    Eigen::Vector3d zero(0, 0, 0);
    std::fill(gradient_q.begin(), gradient_q.end(), zero);
    gt = 0.0;

    // Abbreviation of params
    const double dt_inv = 1 / dt;
    const double dt_inv2 = dt_inv * dt_inv;

    // 使用control points的一阶差分（带时间）作为速度
    // 速度超过设定的速度阈值就按超过部分的平方作为cost，其实是很简单的做法
    for (int i = 0; i < q.size() - 1; ++i)
    {
      // Control point of velocity
      Eigen::Vector3d vi = (q[i + 1] - q[i]) * dt_inv;
      for (int k = 0; k < 3; ++k)
      {
        // Calculate cost for each axis
        double vd = fabs(vi[k]) - max_vel_;
        if (vd > 0.0)
        {
          cost += pow(vd, 2);
          double sign = vi[k] > 0 ? 1.0 : -1.0;
          double tmp = 2 * vd * sign * dt_inv;
          gradient_q[i][k] += -tmp;
          gradient_q[i + 1][k] += tmp;

          if (optimize_time_)
            gt += tmp * (-vi[k]);
        }
      }
    }

    // 使用control points的二阶差分（带时间）作为加速度
    // 照样是超过设定的加速度阈值就按超过部分的平方作为cost
    for (int i = 0; i < q.size() - 2; ++i)
    {
      Eigen::Vector3d ai = (q[i + 2] - 2 * q[i + 1] + q[i]) * dt_inv2;
      for (int k = 0; k < 3; ++k)
      {
        double ad = fabs(ai[k]) - max_acc_;
        if (ad > 0.0)
        {
          cost += pow(ad, 2);
          double sign = ai[k] > 0 ? 1.0 : -1.0;
          double tmp = 2 * ad * sign * dt_inv2;
          gradient_q[i][k] += tmp;
          gradient_q[i + 1][k] += -2 * tmp;
          gradient_q[i + 2][k] += tmp;
          if (optimize_time_)
            gt += tmp * ai[k] * (-2) * dt;
        }
      }
    }
  }

  void BsplineOptimizer::calcStartCost(const vector<Eigen::Vector3d> &q, const double &dt,
                                       double &cost, vector<Eigen::Vector3d> &gradient_q,
                                       double &gt)
  {
    CHECK_EQ(start_con_index_.size(), 3) << "Start state constraint is not set!";

    cost = 0.0;
    Eigen::Vector3d zero(0, 0, 0);
    // 讲道理这里真的得全赋值为0吧
    //  std::fill(gradient_q.begin(), gradient_q.end(), zero);
    for (int i = 0; i < 3; ++i)
      gradient_q[i] = zero;
    gt = 0.0;

    Eigen::Vector3d q1, q2, q3, dq;
    q1 = q[0];
    q2 = q[1];
    q3 = q[2];

    // Start position
    if (start_con_index_[0])
    {
      static const double w_pos = 10.0;
      dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - start_state_[0];
      cost += w_pos * dq.squaredNorm();
      gradient_q[0] += w_pos * 2 * dq * (1 / 6.0);
      gradient_q[1] += w_pos * 2 * dq * (4 / 6.0);
      gradient_q[2] += w_pos * 2 * dq * (1 / 6.0);
    }

    // Start velocity
    if (start_con_index_[1])
    {
      dq = 1 / (2 * dt) * (q3 - q1) - start_state_[1];
      cost += dq.squaredNorm();
      gradient_q[0] += 2 * dq * (-1.0) / (2 * dt);
      gradient_q[2] += 2 * dq * 1.0 / (2 * dt);
      if (optimize_time_)
        gt += dq.dot(q3 - q1) / (-dt * dt);
    }

    // Start acceleration
    if (start_con_index_[2])
    {
      dq = 1 / (dt * dt) * (q1 - 2 * q2 + q3) - start_state_[2];
      cost += dq.squaredNorm();
      gradient_q[0] += 2 * dq * 1.0 / (dt * dt);
      gradient_q[1] += 2 * dq * (-2.0) / (dt * dt);
      gradient_q[2] += 2 * dq * 1.0 / (dt * dt);
      if (optimize_time_)
        gt += dq.dot(q1 - 2 * q2 + q3) / (-dt * dt * dt);
    }
  }

  void BsplineOptimizer::calcEndCost(const vector<Eigen::Vector3d> &q, const double &dt, double &cost,
                                     vector<Eigen::Vector3d> &gradient_q, double &gt)
  {
    CHECK_EQ(end_con_index_.size(), 3) << "End state constraint is not set!";

    cost = 0.0;
    Eigen::Vector3d zero(0, 0, 0);
    // std::fill(gradient_q.begin(), gradient_q.end(), zero);
    for (int i = q.size() - 3; i < q.size(); ++i)
      gradient_q[i] = zero;
    gt = 0.0;

    Eigen::Vector3d q_3, q_2, q_1, dq;
    q_3 = q[q.size() - 3];
    q_2 = q[q.size() - 2];
    q_1 = q[q.size() - 1];

    // End position
    if (end_con_index_[0])
    {
      dq = 1 / 6.0 * (q_1 + 4 * q_2 + q_3) - end_state_[0];
      cost += dq.squaredNorm();
      gradient_q[q.size() - 1] += 2 * dq * (1 / 6.0);
      gradient_q[q.size() - 2] += 2 * dq * (4 / 6.0);
      gradient_q[q.size() - 3] += 2 * dq * (1 / 6.0);
    }

    // End velocity
    if (end_con_index_[1])
    {
      dq = 1 / (2 * dt) * (q_1 - q_3) - end_state_[1];
      cost += dq.squaredNorm();
      gradient_q[q.size() - 1] += 2 * dq * 1.0 / (2 * dt);
      gradient_q[q.size() - 3] += 2 * dq * (-1.0) / (2 * dt);
      if (optimize_time_)
        gt += dq.dot(q_1 - q_3) / (-dt * dt);
    }

    // End acceleration
    if (end_con_index_[2])
    {
      dq = 1 / (dt * dt) * (q_1 - 2 * q_2 + q_3) - end_state_[2];
      cost += dq.squaredNorm();
      gradient_q[q.size() - 1] += 2 * dq * 1.0 / (dt * dt);
      gradient_q[q.size() - 2] += 2 * dq * (-2.0) / (dt * dt);
      gradient_q[q.size() - 3] += 2 * dq * 1.0 / (dt * dt);
      if (optimize_time_)
        gt += dq.dot(q_1 - 2 * q_2 + q_3) / (-dt * dt * dt);
    }
  }

  void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d> &q, double &cost, vector<Eigen::Vector3d> &gradient_q)
  {
    cost = 0.0;
    Eigen::Vector3d zero(0, 0, 0);
    std::fill(gradient_q.begin(), gradient_q.end(), zero);

    Eigen::Vector3d q1, q2, q3, dq;

    for (int i = 0; i < waypoints_.size(); ++i)
    {
      Eigen::Vector3d waypt = waypoints_[i];
      int idx = waypt_idx_[i];

      q1 = q[idx];
      q2 = q[idx + 1];
      q3 = q[idx + 2];

      dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
      cost += dq.squaredNorm();

      gradient_q[idx] += dq * (2.0 / 6.0);     // 2*dq*(1/6)
      gradient_q[idx + 1] += dq * (8.0 / 6.0); // 2*dq*(4/6)
      gradient_q[idx + 2] += dq * (2.0 / 6.0);
    }
  }

  /* use the uniformly sampled points on a geomertic path to guide the
   * trajectory. For each control points to be optimized, it is assigned a
   * guiding point on the path and the distance between them is penalized */
  void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d> &q, double &cost, vector<Eigen::Vector3d> &gradient_q)
  {
    cost = 0.0;
    Eigen::Vector3d zero(0, 0, 0);
    std::fill(gradient_q.begin(), gradient_q.end(), zero);

    int end_idx = q.size() - order_;

    for (int i = order_; i < end_idx; i++)
    {
      Eigen::Vector3d gpt = guide_pts_[i - order_];
      cost += (q[i] - gpt).squaredNorm();
      gradient_q[i] += 2 * (q[i] - gpt);
    }
  }

  void BsplineOptimizer::calcTimeCost(const double &dt, double &cost, double &gt)
  {
    // Min time
    // 这段轨迹用时越短越好
    double duration = (point_num_ - order_) * dt;
    cost = duration;
    gt = point_num_ - order_;

    // Time lower bound
    // 但是太短了也不行，给点惩罚
    if (time_lb_ > 0 && duration < time_lb_)
    {
      static const double w_lb = 10000;
      cost += w_lb * pow(duration - time_lb_, 2);
      gt += w_lb * 2 * (duration - time_lb_) * (point_num_ - order_);
    }
  }

  void BsplineOptimizer::calcPerceptionCost(const vector<Eigen::Vector3d> &q, const double &dt,
                                            double &cost, vector<Eigen::Vector3d> &gradient_q,
                                            const double ld_para, const double ld_vcv)
  {
    cost = 0.0;
    Eigen::Vector3d zero(0, 0, 0);
    std::fill(gradient_q.begin(), gradient_q.end(), zero);

    double cost_para, cost_vcv;
    vector<Eigen::Vector3d> dcost_para_dq, dcost_vcv_dq;
    for (int i = 0; i < q.size() - 3; ++i)
    {
      // For (q0, q1, q2, q3)->(knot1, knot2), calculate the parallax cost and gradient
      vector<Eigen::Vector3d> q_cur;
      for (int j = 0; j < 4; j++)
        q_cur.push_back(q[i + j]);

      Eigen::Vector3d knot_mid = ((q_cur[0] + 4 * q_cur[1] + q_cur[2]) + (q_cur[1] + 4 * q_cur[2] + q_cur[3])) / 12.0;
      vector<Eigen::Vector3d> features;
      edt_environment_->getFeaturesInFovDepth(knot_mid, features);

      // 对应论文第5章B节计算视差cost部分
      parallax_util_->calcParaCostAndGradientsKnots(q_cur, dt, features, cost_para, dcost_para_dq);
      // 对应论文第5章B节计算垂直共视性(vertical covisibility)cost部分
      parallax_util_->calcVCVCostAndGradientsKnots(q_cur, dt, features, cost_vcv, dcost_vcv_dq);

      cost += ld_para * cost_para;
      cost += ld_vcv * cost_vcv;
      for (int j = 0; j < 4; j++)
      {
        gradient_q[i + j] += ld_para * dcost_para_dq[j];
        gradient_q[i + j] += ld_vcv * dcost_vcv_dq[j];
      }
    }
  }

  void BsplineOptimizer::calcYawCoVisbilityCost(const vector<Eigen::Vector3d> &q, const double &dt, double &cost, vector<Eigen::Vector3d> &gradient_q)
  {
    // q.size = n+1, pos_.size = n-p+2 = (n+1) - 2, where p = 3
    CHECK_EQ(q.size() - 2, pos_.size()) << "q and pos_ have incompatible size!";

    cost = 0.0;
    Eigen::Vector3d zero(0, 0, 0);
    std::fill(gradient_q.begin(), gradient_q.end(), zero);

    double pot_cost;
    vector<Eigen::Vector3d> dpot_dq;
    for (int i = 0; i < q.size() - 3; ++i)
    {
      // For (q0, q1, q2, q3)->(knot1, knot2), calculate the covisibility cost and gradient
      vector<Eigen::Vector3d> q_cur;
      for (int j = 0; j < 4; j++)
        q_cur.push_back(q[i + j]);

      vector<Eigen::Vector3d> knots_pos, knots_acc;
      for (int j = 0; j < 2; j++)
      {
        knots_pos.push_back(pos_[i + j]);
        knots_acc.push_back(acc_[i + j]);
      }

      Eigen::Vector3d knot_mid = (pos_[i] + pos_[i + 1]) / 2.0;
      vector<Eigen::Vector3d> features;
      edt_environment_->getFeaturesInFovDepth(knot_mid, features);

      parallax_util_->calcYawCVCostAndGradientsKnots(q_cur, knots_pos, knots_acc, features, pot_cost, dpot_dq);

      cost += pot_cost;
      for (int j = 0; j < 4; j++)
        gradient_q[i + j] += dpot_dq[j];
    }
  }

  void BsplineOptimizer::combineCost(const std::vector<double> &x, std::vector<double> &grad, double &f_combine)
  {
    // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store each control
    // point. For 1D case, the second and third elements are zero, and similar for the 2D case.

    for (int i = 0; i < point_num_; ++i)
    {
      for (int j = 0; j < dim_; ++j)
        g_q_[i][j] = x[dim_ * i + j];
      for (int j = dim_; j < 3; ++j)
        g_q_[i][j] = 0.0;
    }
    const double dt = optimize_time_ ? x[variable_num_ - 1] : knot_span_;

    f_combine = 0.0;            // 总cost
    grad.resize(variable_num_); // 梯度grad
    fill(grad.begin(), grad.end(), 0.0);

    double f_smoothness, f_distance, f_feasibility, gt_feasibility, f_start, gt_start,
        f_end, gt_end, f_guide, f_waypoints, f_view, f_time, gt_time,
        f_parallax, f_vertical_visbility, f_yaw_covisibility = 0.0;

    // Cost1：平滑度约束
    if (cost_function_ & SMOOTHNESS)
    {
      calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
      f_combine += ld_smooth_ * f_smoothness;
      for (int i = 0; i < point_num_; i++)
      {
        for (int j = 0; j < dim_; j++)
          grad[dim_ * i + j] += ld_smooth_ * g_smoothness_[i](j);
      }
    }

    // Cost2：避障约束
    if (cost_function_ & DISTANCE)
    {
      calcDistanceCost(g_q_, f_distance, g_distance_);
      f_combine += ld_dist_ * f_distance;
      for (int i = 0; i < point_num_; i++)
      {
        for (int j = 0; j < dim_; j++)
          grad[dim_ * i + j] += ld_dist_ * g_distance_[i](j);
      }
    }

    // Cost3：动力学可行性约束
    if (cost_function_ & FEASIBILITY)
    {
      calcFeasibilityCost(g_q_, dt, f_feasibility, g_feasibility_, gt_feasibility);
      f_combine += ld_feasi_ * f_feasibility;
      for (int i = 0; i < point_num_; i++)
      {
        for (int j = 0; j < dim_; j++)
          grad[dim_ * i + j] += ld_feasi_ * g_feasibility_[i](j);
      }

      if (optimize_time_)
        grad[variable_num_ - 1] += ld_feasi_ * gt_feasibility;
    }

    // Cost4：起始状态约束
    if (cost_function_ & START)
    {
      calcStartCost(g_q_, dt, f_start, g_start_, gt_start);
      f_combine += ld_start_ * f_start;
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < dim_; j++)
          grad[dim_ * i + j] += ld_start_ * g_start_[i](j);
      }

      if (optimize_time_)
        grad[variable_num_ - 1] += ld_start_ * gt_start;
    }

    // Cost5：终止状态约束（具体原理和上面的起始状态约束几乎一样）
    if (cost_function_ & END)
    {
      calcEndCost(g_q_, dt, f_end, g_end_, gt_end);
      f_combine += ld_end_ * f_end;
      for (int i = point_num_ - 3; i < point_num_; i++)
      {
        for (int j = 0; j < dim_; j++)
          grad[dim_ * i + j] += ld_end_ * g_end_[i](j);
      }

      if (optimize_time_)
        grad[variable_num_ - 1] += ld_end_ * gt_end;
    }

    // Cost6：引导约束
    if (cost_function_ & GUIDE)
    {
      calcGuideCost(g_q_, f_guide, g_guide_);
      f_combine += ld_guide_ * f_guide;
      for (int i = 0; i < point_num_; i++)
      {
        for (int j = 0; j < dim_; j++)
          grad[dim_ * i + j] += ld_guide_ * g_guide_[i](j);
      }
    }

    // Cost7：同样是引导约束，不过是使用waypoints，具体方式与上面的引导约束差不多，懒得细分
    if (cost_function_ & WAYPOINTS)
    {
      calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
      f_combine += ld_waypt_ * f_waypoints;
      for (int i = 0; i < point_num_; i++)
      {
        for (int j = 0; j < dim_; j++)
          grad[dim_ * i + j] += ld_waypt_ * g_waypoints_[i](j);
      }
    }

    // Cost8：最小化时间约束
    if (cost_function_ & MINTIME)
    {
      calcTimeCost(dt, f_time, gt_time);
      f_combine += ld_time_ * f_time;
      grad[variable_num_ - 1] += ld_time_ * gt_time;
    }

    /// APACE新加的
    /// 用于Position Trajectory Optimization阶段
    /// Cost9：视差约束和垂直共视性约束
    if ((cost_function_ & PARALLAX) && (cost_function_ & VERTICALVISIBILITY))
    {
      calcPerceptionCost(g_q_, dt, f_parallax, g_parallax_, ld_parallax_, ld_vertical_visibility_);
      f_combine += f_parallax;
      for (int i = 0; i < point_num_; i++)
      {
        for (int j = 0; j < dim_; j++)
          grad[dim_ * i + j] += g_parallax_[i](j);
      }
    }
    /// 用于Yaw Trajectory Optimization阶段
    /// Cost10：yaw共视性约束
    if (cost_function_ & YAWCOVISIBILITY)
    {
      cout << "calcYawCoVisbilityCost start" << endl;

      calcYawCoVisbilityCost(g_q_, dt, f_yaw_covisibility, g_yaw_covisib_);
      f_combine += ld_yaw_covisib_ * f_yaw_covisibility;
      for (int i = 0; i < point_num_; i++)
      {
        for (int j = 0; j < dim_; j++)
          grad[dim_ * i + j] += ld_yaw_covisib_ * g_yaw_covisib_[i](j);
      }

      cout << "calcYawCoVisbilityCost end" << endl;
    }

    bool verbose = false;

    if (verbose)
    {
      if (cost_function_ & SMOOTHNESS)
        ROS_INFO("Smoothness cost:      %f", ld_smooth_ * f_smoothness);
      if (cost_function_ & DISTANCE)
        ROS_INFO("Distance cost:        %f", ld_dist_ * f_distance);
      if (cost_function_ & FEASIBILITY)
        ROS_INFO("Feasibility cost:     %f", ld_feasi_ * f_feasibility);
      if (cost_function_ & START)
        ROS_INFO("Start cost:           %f", ld_start_ * f_start);
      if (cost_function_ & END)
        ROS_INFO("End cost:             %f", ld_end_ * f_end);
      if (cost_function_ & GUIDE)
        ROS_INFO("Guide cost:           %f", ld_guide_ * f_guide);
      if (cost_function_ & WAYPOINTS)
        ROS_INFO("Waypoint cost:        %f", ld_waypt_ * f_waypoints);
      if (cost_function_ & MINTIME)
        ROS_INFO("Time cost:            %f", ld_time_ * f_time);
      if (cost_function_ & PARALLAX)
        ROS_INFO("Parallax cost:        %f", ld_parallax_ * f_parallax);
      if (cost_function_ & VERTICALVISIBILITY)
        ROS_INFO("Vertical Visib cost:  %f", ld_vertical_visibility_ * f_vertical_visbility);
      if (cost_function_ & YAWCOVISIBILITY)
        ROS_INFO("Yaw Covisib cost:     %f", ld_yaw_covisib_ * f_yaw_covisibility);
      ROS_INFO("--------------------");
      ROS_INFO("TOTAL:                %f", f_combine);
      ROS_INFO("------------------------------");
    }

    // Visualize intermediate results
    if (vis_)
    {
      vector<Eigen::Vector3d> grad_3d;
      for (int i = 0; i < point_num_; i++)
      {
        Eigen::Vector3d temp;
        for (int j = 0; j < dim_; j++)
        {
          temp(j) = grad[dim_ * i + j];
        }
        grad_3d.push_back(temp);
      }
      debugVisualization(g_q_, grad_3d);
      ros::Duration(0.05).sleep();
    }
  }

  double BsplineOptimizer::costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    double cost;
    opt->combineCost(x, grad, cost);

    /* save the min cost result */
    if (cost < opt->min_cost_)
    {
      opt->min_cost_ = cost;
      opt->best_variable_ = x;
    }
    return cost;
  }

  Eigen::MatrixXd BsplineOptimizer::getControlPoints() { return this->control_points_; }

  bool BsplineOptimizer::isQuadratic()
  {
    if (cost_function_ == GUIDE_PHASE)
      return true;
    else if (cost_function_ == SMOOTHNESS)
      return true;
    else if (cost_function_ == (SMOOTHNESS | WAYPOINTS))
      return true;

    return false;
  }

  void BsplineOptimizer::debugVisualization(const std::vector<Eigen::Vector3d> &q,
                                            const std::vector<Eigen::Vector3d> &q_grad)
  {
    if (vis_pub_.getNumSubscribers() == 0)
      return;

    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < q.size(); ++i)
    {
      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = ros::Time::now();
      mk.id = i;
      mk.type = visualization_msgs::Marker::ARROW;
      mk.action = visualization_msgs::Marker::ADD;
      mk.pose.orientation.w = 1.0;
      mk.scale.x = 0.1;
      mk.scale.y = 0.2;
      mk.scale.z = 0.3;
      mk.color.r = 1.0;
      mk.color.g = 0.5;
      mk.color.b = 0.0;
      mk.color.a = 1.0;

      geometry_msgs::Point pt;
      pt.x = q[i](0);
      pt.y = q[i](1);
      pt.z = q[i](2);
      mk.points.push_back(pt);
      pt.x = q[i](0) + q_grad[i](0);
      pt.y = q[i](1) + q_grad[i](1);
      pt.z = q[i](2) + q_grad[i](2);
      mk.points.push_back(pt);

      marker_array.markers.push_back(mk);
    }

    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    // mk.action = visualization_msgs::Marker::DELETE;
    mk.id = q.size();
    // pubs_[pub_id].publish(mk);

    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;

    mk.scale.x = 0.2;
    mk.scale.y = 0.2;
    mk.scale.z = 0.2;

    geometry_msgs::Point pt;
    for (size_t i = 0; i < q.size(); ++i)
    {
      pt.x = q[i](0);
      pt.y = q[i](1);
      pt.z = q[i](2);
      mk.points.push_back(pt);
    }

    marker_array.markers.push_back(mk);

    vis_pub_.publish(marker_array);
  }
} // namespace fast_planner