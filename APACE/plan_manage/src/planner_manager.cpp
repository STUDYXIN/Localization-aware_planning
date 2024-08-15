// #include <fstream>
#include <plan_manage/planner_manager.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include <visualization_msgs/Marker.h>

#define ANSI_COLOR_YELLOW_BOLD "\033[1;33m"
#define ANSI_COLOR_GREEN_BOLD "\033[1;32m"
#define ANSI_COLOR_RED_BOLD "\033[1;31m"

namespace fast_planner
{
  // SECTION interfaces for setup and query
  void FastPlannerManager::initPlanModules(ros::NodeHandle &nh)
  {
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/max_yawdot", pp_.max_yawdot_, -1.0);
    nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
    nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/bspline_degree", pp_.bspline_degree_, 3);
    nh.param("manager/min_time", pp_.min_time_, false);

    bool use_geometric_path, use_optimization, use_initial_yaw, use_parallax;
    nh.param("manager/use_geometric_path", use_geometric_path, false);
    nh.param("manager/use_optimization", use_optimization, false);
    nh.param("manager/use_initial_yaw", use_initial_yaw, false);
    nh.param("manager/use_parallax", use_parallax, false);

    nh.param("manager/occupancy_map_file", occupancy_map_file_, string(""));
    nh.param("manager/esdf_map_file", esdf_map_file_, string(""));
    nh.param("manager/feature_map_file", feature_map_file_, string(""));

    local_data_.traj_id_ = 0;
    map_server_.reset(new voxel_mapping::MapServer(nh));
    edt_environment_.reset(new EDTEnvironment);
    edt_environment_->setMap(map_server_);

    double resolution = edt_environment_->map_server_->getResolution();
    Eigen::Vector3d origin, size;
    edt_environment_->map_server_->getRegion(origin, size);
    caster_.reset(new RayCaster);
    caster_->setParams(resolution, origin);

    if (use_geometric_path)
    {
      path_finder_.reset(new Astar);
      path_finder_->init(nh, edt_environment_);
    }

    if (use_optimization)
    {
      bspline_optimizers_.resize(10);
      for (int i = 0; i < 10; ++i)
      {
        bspline_optimizers_[i].reset(new BsplineOptimizer);
        bspline_optimizers_[i]->setParam(nh);
        bspline_optimizers_[i]->setEnvironment(edt_environment_);
      }
    }

    if (use_initial_yaw)
    {
      yaw_initial_planner_.reset(new YawInitialPlanner(nh));
      yaw_initial_planner_->setMap(map_server_);
    }

    if (use_parallax)
    {
      for (int i = 0; i < 10; ++i)
        bspline_optimizers_[i]->initParallaxUtil(nh);
    }
  }

  void FastPlannerManager::saveMapService()
  {
    map_server_->saveMap(occupancy_map_file_);
  }

  void FastPlannerManager::loadMapService()
  {
    map_server_->loadMap(occupancy_map_file_, esdf_map_file_, feature_map_file_);
  }

  bool FastPlannerManager::checkTrajCollision(double &distance)
  {
    double t_now = (ros::Time::now() - local_data_.start_time_).toSec();

    double tm, tmp;
    local_data_.position_traj_.getTimeSpan(tm, tmp);
    Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now);

    double radius = 0.0;
    Eigen::Vector3d fut_pt;
    double fut_t = 0.02;

    while (radius < 6.0 && t_now + fut_t < local_data_.duration_)
    {
      fut_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);

      double dist = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
      if (dist < 0.1)
      {
        distance = radius;
        return false;
      }

      radius = (fut_pt - cur_pt).norm();
      fut_t += 0.02;
    }

    return true;
  }

  // !SECTION
  void FastPlannerManager::planExploreTraj(const vector<Eigen::Vector3d> &tour, const Eigen::Vector3d &cur_vel, const Eigen::Vector3d &cur_acc,
                                           const bool reach_end, const double &time_lb)
  {
    if (tour.empty())
      ROS_ERROR("Empty path to traj planner");

    TicToc t2;

    // Generate traj through waypoints-based method
    const int pt_num = tour.size();
    Eigen::MatrixXd pos(pt_num, 3);
    for (int i = 0; i < pt_num; ++i)
      pos.row(i) = tour[i];

    Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    Eigen::VectorXd times(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
      times(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);

    // Step1: 调用waypointsTraj函数生成初始的分段多项式轨迹
    PolynomialTraj init_traj;
    PolynomialTraj::waypointsTraj(pos, cur_vel, zero, cur_acc, zero, times, init_traj);

    ROS_WARN("[Local Planner] Plan t2: %fs", t2.toc());

    TicToc t3;

    // Step2: 基于B样条曲线的轨迹优化，首先生成一条均匀B样条曲线
    double duration = init_traj.getTotalTime();
    int seg_num = init_traj.getLength() / pp_.ctrl_pt_dist;
    seg_num = max(8, seg_num);
    double dt = duration / seg_num;

    vector<Vector3d> points;
    for (double ts = 0.0; ts <= duration + 1e-4; ts += dt)
      points.push_back(init_traj.evaluate(ts, 0));

    // Evaluate velocity at start and end
    vector<Vector3d> boundary_deri;
    boundary_deri.push_back(init_traj.evaluate(0.0, 1));
    boundary_deri.push_back(init_traj.evaluate(duration, 1));
    // Evaluate acceleration at start and end
    boundary_deri.push_back(init_traj.evaluate(0.0, 2));
    boundary_deri.push_back(init_traj.evaluate(duration, 2));

    Eigen::MatrixXd ctrl_pts;
    NonUniformBspline::parameterizeToBspline(dt, points, boundary_deri, pp_.bspline_degree_, ctrl_pts);
    NonUniformBspline tmp_traj(ctrl_pts, pp_.bspline_degree_, dt);

    ROS_WARN("[Local Planner] Plan t3: %fs", t3.toc());

    // Step3: 在上面得到的均匀B样条曲线基础上进行B样条曲线优化，得到最终的轨迹
    TicToc t4;

    vector<Vector3d> start, end;
    vector<bool> start_idx, end_idx;

    if (reach_end)
    {
      tmp_traj.getBoundaryStates(2, 2, start, end);
      start_idx = {true, true, true};
      end_idx = {true, true, true};
    }
    else
    {
      tmp_traj.getBoundaryStates(2, 0, start, end);
      start_idx = {true, true, true};
      end_idx = {true, false, false};
    }

    bspline_optimizers_[0]->setBoundaryStates(start, end, start_idx, end_idx);
    if (time_lb > 0)
      bspline_optimizers_[0]->setTimeLowerBound(time_lb);

    // 这里使用了平滑约束、动力学可行性约束、起点约束、终点约束、避障约束、
    // int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::FEASIBILITY |
    //                 BsplineOptimizer::START | BsplineOptimizer::END | BsplineOptimizer::MINTIME |
    //                 BsplineOptimizer::DISTANCE | BsplineOptimizer::PARALLAX |
    //                 BsplineOptimizer::VERTICALVISIBILITY;

    int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::FEASIBILITY |
                    BsplineOptimizer::START | BsplineOptimizer::END | BsplineOptimizer::MINTIME |
                    BsplineOptimizer::DISTANCE;

    bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func);
    local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();

    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();

    ROS_WARN("[Local Planner] Plan t4: %fs", t4.toc());
  }

  int FastPlannerManager::planLocalMotion(const Vector3d &next_pos, const Vector3d &pos, const Vector3d &vel, const Vector3d &acc, bool &truncated, const double &time_lb)
  {
    // Start optimization
    // Plan trajectory (position and yaw) to the next viewpoint

    // Step1: 调用A*算法接口进行全局规划，并简单处理得到缩短的路径（由一堆三维路径点组成）
    TicToc t1;
    path_finder_->reset();
    if (path_finder_->search(pos, next_pos) != Astar::REACH_END)
    {
      ROS_ERROR("No path to point (%f, %f, %f)", next_pos.x(), next_pos.y(), next_pos.z());
      std::cout << "pos:" << pos.transpose() << std::endl;
      std::cout << "next_pos:" << next_pos.transpose() << std::endl;
      return LOCAL_FAIL;
    }

    vector<Vector3d> path_waypoint = path_finder_->getPath();
    shortenPath(path_waypoint);

    vector<Vector3d> truncated_path;

    const double radius_far = 50.0;

    // Next viewpoint is far away, truncate the far goal to an intermediate goal
    if (Astar::pathLength(path_waypoint) > radius_far)
    {
      truncated = true;
      truncated_path.push_back(path_waypoint.front());
      double len2 = 0.0;
      for (int i = 1; i < path_waypoint.size() && len2 < radius_far; ++i)
      {
        auto cur_pt = path_waypoint[i];
        len2 += (cur_pt - truncated_path.back()).norm();
        truncated_path.push_back(cur_pt);
      }
    }

    ROS_WARN("[Local Planner] Plan t1: %fs", t1.toc());

    // Step2: 到这里拿到了路径规划生成的waypoints，接下来用这些waypoints生成trajetory
    planExploreTraj(truncated ? truncated_path : path_waypoint, vel, acc, !truncated, time_lb);

    return LOCAL_SUCCEED;
  }

  void FastPlannerManager::shortenPath(vector<Vector3d> &path)
  {
    if (path.empty())
    {
      ROS_ERROR("Empty path to shorten");
      return;
    }
    // Shorten the tour, only critical intermediate points are reserved.
    // See Fig.8 from "Robust Real-time UAV Replanning Using Guided Gradient-based
    // Optimization and Topological Paths"
    const double dist_thresh = 3.0;
    vector<Vector3d> short_tour = {path.front()};
    for (int i = 1; i < path.size() - 1; ++i)
    {
      if ((path[i] - short_tour.back()).norm() > dist_thresh)
        short_tour.push_back(path[i]);
      else
      {
        // Add waypoints to shorten path only to avoid collision
        caster_->input(short_tour.back(), path[i + 1]);
        Eigen::Vector3i idx;
        while (caster_->nextId(idx) && ros::ok())
        {
          if (edt_environment_->map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::OCCUPIED ||
              edt_environment_->map_server_->getOccupancy(idx) == voxel_mapping::OccupancyType::UNKNOWN)
          {
            short_tour.push_back(path[i]);
            break;
          }
        }
      }
    }
    if ((path.back() - short_tour.back()).norm() > 1e-3)
      short_tour.push_back(path.back());

    // Ensure at least three points in the path
    if (short_tour.size() == 2)
      short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));

    path = short_tour;
  }

  void FastPlannerManager::planYawCovisibility()
  {
    TicToc t5;
    // Yaw b-spline has same segment number as position b-spline
    Eigen::MatrixXd position_ctrl_pts = local_data_.position_traj_.getControlPoint();
    int ctrl_pts_num = position_ctrl_pts.rows();
    double dt_yaw = local_data_.position_traj_.getKnotSpan();

    // Yaw traj control points
    Eigen::MatrixXd yaw(ctrl_pts_num, 1);
    yaw.setZero();

    // Calculate knot pos and acc
    // [u[p],u[m-p]] -> [0*dt, (m-2p)*dt] -> [0*dt, (n-2)*dt]
    vector<Eigen::Vector3d> twb_pos, twb_acc;
    for (int i = 0; i < ctrl_pts_num - 2; ++i)
    {
      double t = i * dt_yaw;
      Eigen::Vector3d pos = local_data_.position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d acc = local_data_.acceleration_traj_.evaluateDeBoorT(t);
      twb_pos.push_back(pos);
      twb_acc.push_back(acc);
    }

    // TODO: only need to calculate nn features once! Feed to yaw_initial_planner & optimizer

    // Step1: 使用图搜索算法给出一条初始的yaw轨迹(yaw_waypoints)，跟position_traj一一对应，除了首尾各添了个0
    vector<double> yaw_waypoints;
    yaw_initial_planner_->searchPathOfYaw(twb_pos, twb_acc, dt_yaw, yaw_waypoints);

    ROS_WARN("[Local Planner] Plan t5: %fs", t5.toc());

    TicToc t6;

    // 后面优化选项选择了WAYPOINTS，所以这里需要把waypoints设置好
    vector<Eigen::Vector3d> waypts;
    vector<int> waypt_idx;
    double last_yaw = yaw_waypoints[0];
    for (int i = 0; i < yaw_waypoints.size(); ++i)
    {
      Eigen::Vector3d waypt = Eigen::Vector3d::Zero();
      waypt(0) = yaw_waypoints[i];
      calcNextYaw(last_yaw, waypt(0));
      last_yaw = waypt(0);
      waypts.push_back(waypt);
      waypt_idx.push_back(i);
    }

    // 吐槽：这里是在干嘛？这里结果上讲就是将yaw的前三个元素赋值成waypts的首个元素，这个矩阵其他元素是来干嘛的？
    // 结论：照抄了fast_planner没有改
    //  Initial state
    Eigen::Matrix3d states2pts;
    states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw,
        1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
        1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
    Eigen::Vector3d start_yaw3d = waypts[0];
    yaw.block<3, 1>(0, 0) = states2pts * start_yaw3d;

    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    vector<Eigen::Vector3d> start = {zero, zero, zero};
    vector<Eigen::Vector3d> end = {zero, zero, zero};
    vector<bool> start_idx = {false, true, true};
    vector<bool> end_idx = {false, true, true};

    // Call B-spline optimization solver
    // Step2: 调用B样条曲线优化器优化yaw轨迹
    // 这里使用了平滑约束、路径点约束、起点约束、终点约束、主要就是添加了yaw共视约束
    int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS |
                    BsplineOptimizer::START | BsplineOptimizer::END |
                    BsplineOptimizer::YAWCOVISIBILITY;

    if (cost_func & BsplineOptimizer::YAWCOVISIBILITY)
    {
      vector<Eigen::Vector3d> pos_knots, acc_knots;
      local_data_.position_traj_.getKnotPoint(pos_knots);
      local_data_.acceleration_traj_.getKnotPoint(acc_knots);
      bspline_optimizers_[1]->setPosAndAcc(pos_knots, acc_knots);
    }

    bspline_optimizers_[1]->setBoundaryStates(start, end, start_idx, end_idx);
    bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
    bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func);

    // Update traj info
    // Step3: 更新yaw及其导数的轨迹
    local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
    local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
    local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

    ROS_WARN("[Local Planner] Plan t6: %fs", t6.toc());
  }

  void FastPlannerManager::callYawPrepare(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc, const Vector3d &yaw)
  {
    ROS_WARN("[Planner_Manager] Yaw preparation starts");

    double yaw_cur = yaw(0);
    double yaw_cmd = local_data_.yaw_traj_.evaluateDeBoorT(0.0)[0];

    // Round yaw_cur to [-PI, PI]
    while (yaw_cur < -M_PI)
      yaw_cur += 2 * M_PI;
    while (yaw_cur > M_PI)
      yaw_cur -= 2 * M_PI;

    double diff = yaw_cmd - yaw_cur;
    if (fabs(diff) <= M_PI)
      yaw_cmd = yaw_cur + diff;
    else if (diff > M_PI)
      yaw_cmd = yaw_cur + diff - 2 * M_PI;
    else if (diff < -M_PI)
      yaw_cmd = yaw_cur + diff + 2 * M_PI;

    diff = yaw_cmd - yaw_cur;

    // Set b-spline params
    const int seg_num = 12;
    const double yaw_vel = M_PI / 12.0;
    const double duration = fabs(diff) / yaw_vel;
    const double dt_yaw = duration / seg_num; // time of B-spline segment

    // Define waypoints such that yaw change is uniform
    vector<Eigen::Vector3d> waypoints;
    for (int i = 0; i < seg_num + 1; i++)
    {
      double t = i * dt_yaw;
      double yaw = yaw_cur + diff * t / duration;
      Eigen::Vector3d waypt;
      waypt(0) = yaw;
      waypt(1) = waypt(2) = 0.0;
      waypoints.push_back(waypt);
    }

    Eigen::MatrixXd points(waypoints.size(), 3);
    for (size_t i = 0; i < waypoints.size(); ++i)
      points.row(i) = waypoints[i].transpose();

    Eigen::VectorXd times(waypoints.size() - 1);
    times.setConstant(dt_yaw);

    // Given desired waypoints and corresponding time stamps, fit a B-spline and execute it
    PolynomialTraj poly;
    PolynomialTraj::waypointsTraj(points, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), times, poly);

    // Fit the polynomial with B-spline
    vector<Eigen::Vector3d> point_set, boundary_der;
    for (double ts = 0; ts <= 1e-3 + duration; ts += dt_yaw)
      point_set.push_back(poly.evaluate(ts, 0));

    boundary_der.push_back(poly.evaluate(0, 1));
    boundary_der.push_back(poly.evaluate(duration, 1));
    boundary_der.push_back(poly.evaluate(0, 2));
    boundary_der.push_back(poly.evaluate(duration, 2));

    Eigen::MatrixXd yaw_ctrl_pts;
    NonUniformBspline::parameterizeToBspline(dt_yaw, point_set, boundary_der, 3, yaw_ctrl_pts);

    prepare_yaw_data_.yaw_traj_.setUniformBspline(yaw_ctrl_pts, 3, dt_yaw);
    prepare_yaw_data_.yawdot_traj_ = prepare_yaw_data_.yaw_traj_.getDerivative();
    prepare_yaw_data_.yawdotdot_traj_ = prepare_yaw_data_.yawdot_traj_.getDerivative();

    Eigen::MatrixXd pos_ctrl_pts = yaw_ctrl_pts;
    for (int i = 0; i < pos_ctrl_pts.rows(); i++)
      pos_ctrl_pts.row(i) = pos.transpose();

    prepare_yaw_data_.position_traj_.setUniformBspline(pos_ctrl_pts, pp_.bspline_degree_, dt_yaw);
    prepare_yaw_data_.velocity_traj_ = prepare_yaw_data_.position_traj_.getDerivative();
    prepare_yaw_data_.acceleration_traj_ = prepare_yaw_data_.velocity_traj_.getDerivative();

    prepare_yaw_data_.start_pos_ = pos;
    prepare_yaw_data_.duration_ = prepare_yaw_data_.position_traj_.getTimeSum();

    ROS_WARN("[Planner_Manager] Yaw preparation done");
  }

  void FastPlannerManager::calcNextYaw(const double &last_yaw, double &yaw)
  {
    // round yaw to [-PI, PI]
    double round_last = last_yaw;
    while (round_last < -M_PI)
      round_last += 2 * M_PI;
    while (round_last > M_PI)
      round_last -= 2 * M_PI;

    double diff = yaw - round_last;
    if (fabs(diff) <= M_PI)
      yaw = last_yaw + diff;
    else if (diff > M_PI)
      yaw = last_yaw + diff - 2 * M_PI;
    else if (diff < -M_PI)
      yaw = last_yaw + diff + 2 * M_PI;
  }

} // namespace fast_planner
