#include <stepping_debug.hpp>
#include <traj_utils/planning_visualization.h>
#include <fstream>
#include <iostream>
#include <thread>
#include <iomanip>
namespace fast_planner {
// ANSI 颜色代码
const std::string COLOR_GREEN = "\033[32m";
const std::string COLOR_RED = "\033[31m";
const std::string COLOR_RESET = "\033[0m";

SteppingDebug::SteppingDebug() {
  init_visual = false;
  init_success = false;
  showcloud = false;
  debug_count = 0;
}

void SteppingDebug::init(ros::NodeHandle& nh) {
  proceed_ = false;
  init_success = true;
  // 从参数服务器获取bool值并赋值到map
  debug_map[BEFORE_COMPUTE] = false;
  debug_map[BEFORE_POS_OPT] = false;
  debug_map[EVERY_POS_OPT] = false;
  debug_map[YAW_INIT] = false;
  debug_map[EVERY_YAW_OPT] = false;
  debug_map[SHOW_VERVIS] = false;
  nh.param("debug/stop_before_compute", debug_map[BEFORE_COMPUTE], false);
  nh.param("debug/stop_before_pos_opt", debug_map[BEFORE_POS_OPT], false);
  nh.param("debug/stop_every_pos_opt", debug_map[EVERY_POS_OPT], false);
  nh.param("debug/stop_before_yaw_init", debug_map[YAW_INIT], false);
  nh.param("debug/stop_every_yaw_opt", debug_map[EVERY_YAW_OPT], false);
  nh.param("debug/show_vertical_visibility_cost_every_control_point", debug_map[SHOW_VERVIS], false);
  nh.param("debug/debug_delay_time", debug_delay_time_, -1.0);

  nh.param("optimization/ld_smooth", ld_cost[SMOOTHNESS], -1.0);
  nh.param("optimization/ld_dist", ld_cost[DISTANCE], -1.0);
  nh.param("optimization/ld_feasi", ld_cost[FEASIBILITY_VEL], -1.0);
  nh.param("optimization/ld_feasi", ld_cost[FEASIBILITY_ACC], -1.0);
  nh.param("optimization/ld_start", ld_cost[START_POS], -1.0);
  nh.param("optimization/ld_start", ld_cost[START_VEL], -1.0);
  nh.param("optimization/ld_start", ld_cost[START_ACC], -1.0);
  nh.param("optimization/ld_end", ld_cost[END_POS], -1.0);
  nh.param("optimization/ld_end", ld_cost[END_VEL], -1.0);
  nh.param("optimization/ld_end", ld_cost[END_ACC], -1.0);
  nh.param("optimization/ld_guide", ld_cost[GUIDE], -1.0);
  nh.param("optimization/ld_waypt", ld_cost[WAYPOINTS], -1.0);
  nh.param("optimization/ld_time", ld_cost[MINTIME], -1.0);
  ld_cost[APACE_POS] = 1.0;
  nh.param("optimization/ld_yaw_covisibility", ld_cost[YAWCOVISIBILITY], -1.0);
  nh.param("optimization/ld_frontier_visibility_pos", ld_cost[FRONTIERVIS_POS], -1.0);
  nh.param("optimization/ld_frontier_visibility_yaw", ld_cost[FRONTIERVIS_YAW], -1.0);
  nh.param("optimization/ld_feasi_yaw", ld_cost[FEASIBILITY_YAW], -1.0);
}

void SteppingDebug::calldebug(DEBUG_TYPE type, const vector<Eigen::Vector3d>& path, const int& order, const double& interval) {
  if (!init_visual || !init_success) return;
  if (type != debug_type_now_ && !showcloud) return;
  if (!debug_map[type]) return;

  if (debug_count == 0) reason_count_[debugTypeStrings[type]] = 0;
  Eigen::MatrixXd points;
  NonUniformBspline bspline;
  vector<Vector3d> knot_pos;
  switch (type) {
    case BEFORE_POS_OPT:
      visualization_->drawGeometricPath(path, 0.05, Eigen::Vector4d(1.0, 0.647, 0, 0.5), 99);  // 新开一个id，防止被使用,使用橙色
      break;
    case EVERY_POS_OPT:
      points.resize(path.size(), 3);
      for (size_t i = 0; i < path.size(); ++i) points.row(i) = path[i];
      bspline.setUniformBspline(points, order, interval);
      visualization_->drawDebugPosBspline(bspline, debug_count);
      coutDebugMsg(type, path.size());
      break;
    case YAW_INIT:
      bspline_pos_.getKnotPoint(knot_pos);
      visualization_->drawDebugPosBspline(bspline_pos_, debug_count);
      visualization_->drawYawTraj(knot_pos, path);
      break;
    case EVERY_YAW_OPT:
      points.resize(path.size(), 3);
      for (size_t i = 0; i < path.size(); ++i) points.row(i) = path[i];
      bspline.setUniformBspline(points, order, interval);
      visualization_->drawDebugPosBspline(bspline_pos_, debug_count);
      visualization_->drawYawTraj(bspline_pos_, bspline, bspline.getKnotSpan());
      coutDebugMsg(type, path.size());
      break;
    case SHOW_VERVIS:
      visualization_->drawDebugControlpoint(control_point_, control_point_grad_);
      visualization_->drawDebugCloud(cloud_, intense_);
      showcloud = false;
      break;
    default:
      break;
  }

  debug_count++;

  if (debug_delay_time_ < 0)
    waitForInput(debugTypeStrings[type]);
  else
    ros::Duration(debug_delay_time_).sleep();
}

void SteppingDebug::waitForInput(const std::string& reason) {
  if (!init_visual || !init_success) return;
  std::unique_lock<std::mutex> lock(mtx_);

  std::cout << "Paused due to reason: \033[36m" << reason << "\033[0m" << std::endl;
  reason_count_[reason]++;
  std::thread input_thread(&SteppingDebug::keyboardInput, this);
  cv_.wait(lock, [this]() { return proceed_; });
  std::cout << "Input received. Continuing..." << std::endl;
  std::cout << "Times reason '" << reason << "' occurred: " << reason_count_[reason] << std::endl;
  proceed_ = false;
  input_thread.join();
}

// 等待键盘输入的辅助函数
void SteppingDebug::keyboardInput() {
  std::string user_input;
  std::cout << "Please press Enter to continue..." << std::endl;

  std::getline(std::cin, user_input);
  {
    std::lock_guard<std::mutex> lock(mtx_);
    proceed_ = true;
  }
  cv_.notify_one();
}

// 输出字符调用次数的统计
void SteppingDebug::printReasonCounts() {
  std::cout << "Reason call counts:" << std::endl;
  for (const auto& entry : reason_count_) {
    std::cout << "Reason: " << entry.first << ", Count: " << entry.second << std::endl;
  }
}

// 统计cost信息，用于调试
void SteppingDebug::addDebugCost(DEBUG_TYPE type, COST_TYPE cost_type, const double& cost) {
  if (!init_visual || !init_success) return;
  if (!debug_map[type]) return;
  if (type != debug_type_now_) return;
  cost_record[cost_type].push_back(cost);
}

// SteppingDebug::coutDebugMsg 函数
void SteppingDebug::coutDebugMsg(DEBUG_TYPE type, const size_t& max_size) {
  if (!init_visual || !init_success) return;
  if (!debug_map[type]) return;
  if (type != debug_type_now_) return;
  // 用来记录每个控制点的总cost
  ctrl_point_cost.resize(max_size + 1);
  std::fill(ctrl_point_cost.begin(), ctrl_point_cost.end(), 0.0);
  // 表头
  std::cout << "\n================================================================== Cost Change Statistics "
               "=================================================================="
            << std::endl;
  std::cout << std::setw(15) << std::right << "ctrl_id"
            << ": ";
  for (size_t i = 0; i < max_size; ++i) std::cout << std::setw(10) << std::right << i << " ";
  std::cout << std::setw(10) << std::right << " cost_total";
  std::cout << std::endl;
  std::cout << "------------------------------------------------------------------------------"
               "------------------------------------------------------------------------------\n";
  // 内容
  for (auto& entry : cost_record) {
    COST_TYPE cost_type = entry.first;
    std::vector<double>& values = entry.second;
    std::cout << std::setw(15) << std::right << costTypeStrings[cost_type] << ": ";
    size_t i = 0;
    double total_cost = 0.0;
    // 遍历当前记录的值，并与 last_cost_record 比较
    for (; i < values.size() && i < max_size; ++i) {
      double last_value = 0;
      if (last_cost_record.count(cost_type) && i < last_cost_record.at(cost_type).size())
        last_value = last_cost_record.at(cost_type)[i];

      total_cost += ld_cost[cost_type] * values[i];          // 计算总和
      ctrl_point_cost[i] += ld_cost[cost_type] * values[i];  // 累计这个控制点

      // 判断 cost 是增加、减少还是不变，并使用对应颜色输出
      if (values[i] < last_value)
        std::cout << COLOR_GREEN << std::setw(10) << std::right << values[i] << COLOR_RESET << " ";
      else if (values[i] > last_value)
        std::cout << COLOR_RED << std::setw(10) << std::right << values[i] << COLOR_RESET << " ";
      else
        std::cout << std::setw(10) << std::right << values[i] << " ";
    }

    // 填充 0 如果不足 max_size
    for (; i < max_size; ++i) std::cout << std::setw(10) << std::right << 0 << " ";

    // 获取上次的总和，用于比较
    double last_total_cost = 0;
    if (last_cost_record.count(cost_type) && last_cost_record.at(cost_type).size() > max_size)
      last_total_cost = last_cost_record.at(cost_type)[max_size];

    // 判断总和的变化，并使用对应颜色输出
    if (total_cost < last_total_cost)
      std::cout << COLOR_GREEN << std::setw(10) << std::right << total_cost << COLOR_RESET;
    else if (total_cost > last_total_cost)
      std::cout << COLOR_RED << std::setw(10) << std::right << total_cost << COLOR_RESET;
    else
      std::cout << std::setw(10) << std::right << total_cost;

    std::cout << std::endl;

    // 将总和推入 cost_record 的末尾
    while (values.size() < max_size) values.push_back(0);
    values.push_back(total_cost);  // 更新已经存在的总和值
  }

  // 输出 Total 行
  double ctrl_point_total = 0.0;
  std::cout << "------------------------------------------------------------------------------"
               "------------------------------------------------------------------------------\n";
  std::cout << std::setw(15) << std::right << "Total"
            << ": ";

  for (size_t i = 0; i < max_size; ++i) {
    // 如果存在上次的 ctrl_point_cost，获取上次的值
    double last_ctrl_value = 0;
    if (i < last_ctrl_point_cost.size()) last_ctrl_value = last_ctrl_point_cost[i];
    // 计算每个控制点的总和
    ctrl_point_total += ctrl_point_cost[i];
    // 判断控制点的 cost 是增加、减少还是不变，并使用对应颜色输出
    if (ctrl_point_cost[i] < last_ctrl_value)
      std::cout << COLOR_GREEN << std::setw(10) << std::right << ctrl_point_cost[i] << COLOR_RESET << " ";
    else if (ctrl_point_cost[i] > last_ctrl_value)
      std::cout << COLOR_RED << std::setw(10) << std::right << ctrl_point_cost[i] << COLOR_RESET << " ";
    else
      std::cout << std::setw(10) << std::right << ctrl_point_cost[i] << " ";
  }

  // 获取上次的总和
  double last_ctrl_point_total = 0;
  if (!last_ctrl_point_cost.empty() && last_ctrl_point_cost.size() > max_size)
    last_ctrl_point_total = last_ctrl_point_cost[max_size];

  // 判断总和的变化，并使用对应颜色输出
  if (ctrl_point_total < last_ctrl_point_total)
    std::cout << COLOR_GREEN << std::setw(10) << std::right << ctrl_point_total << COLOR_RESET;
  else if (ctrl_point_total > last_ctrl_point_total)
    std::cout << COLOR_RED << std::setw(10) << std::right << ctrl_point_total << COLOR_RESET;
  else
    std::cout << std::setw(10) << std::right << ctrl_point_total;
  ctrl_point_cost[max_size] = ctrl_point_total;
  std::cout << std::endl;
  // 结束
  last_cost_record = cost_record;
  last_ctrl_point_cost = ctrl_point_cost;
  cost_record.clear();
  ctrl_point_cost.clear();
  std::cout << "=============================================================================="
               "==============================================================================\n"
            << std::endl;
}

}  // namespace fast_planner
