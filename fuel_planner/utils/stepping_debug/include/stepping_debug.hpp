#ifndef STEPPING_DEBUG_HPP
#define STEPPING_DEBUG_HPP

#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include <utility>
#include <map>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>

#include <bspline/non_uniform_bspline.h>

using namespace Eigen;
using namespace std;

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::vector;
namespace fast_planner {
class PlanningVisualization;

enum DEBUG_TYPE {
  BEFORE_COMPUTE = 0,
  BEFORE_POS_OPT = 1,
  EVERY_POS_OPT = 2,
  YAW_INIT = 3,
  EVERY_YAW_OPT = 4,
  SHOW_VERVIS = 5,
  SHOW_VCV = 6
};
const string debugTypeStrings[] = { "BEFORE_COMPUTE", "BEFORE_POS_OPT", "EVERY_POS_OPT", "YAW_INIT", "EVERY_YAW_OPT",
  "SHOW_VERVIS", "SHOW_VCV" };

enum COST_TYPE {
  SMOOTHNESS,
  DISTANCE,
  FEASIBILITY_VEL,
  FEASIBILITY_ACC,
  START_POS,
  START_VEL,
  START_ACC,
  END_POS,
  END_VEL,
  END_ACC,
  GUIDE,
  WAYPOINTS,
  MINTIME,
  APACE_POS,
  YAWCOVISIBILITY,
  FRONTIERVIS_POS,
  FRONTIERVIS_YAW,
  FEASIBILITY_YAW
};
const std::string costTypeStrings[] = { "SMOOTHNESS", "DISTANCE", "FEASIBILITY_VEL", "FEASIBILITY_ACC", "START_POS", "START_VEL",
  "START_ACC", "END_POS", "END_VEL", "END_ACC", "GUIDE", "WAYPOINTS", "MINTIME", "APACE_POS", "YAWCOVISIBILITY",
  "FRONTIERVIS_POS", "FRONTIERVIS_YAW", "FEASIBILITY_YAW" };

class SteppingDebug {
private:
  std::mutex mtx_;
  std::condition_variable cv_;
  bool proceed_;
  double debug_delay_time_;
  bool init_visual, init_success;
  std::map<DEBUG_TYPE, bool> debug_map;
  std::map<std::string, int> reason_count_;
  std::map<COST_TYPE, vector<double>> cost_record, last_cost_record;
  std::map<COST_TYPE, double> ld_cost;
  vector<double> ctrl_point_cost, last_ctrl_point_cost;
  NonUniformBspline bspline_pos_;

public:
  SteppingDebug();
  void init(ros::NodeHandle& nh);
  void calldebug(DEBUG_TYPE type, const vector<Eigen::Vector3d>& path = vector<Eigen::Vector3d>(), const int& order = 0,
      const double& interval = 0.0);
  void waitForInput(const std::string& reason);
  void keyboardInput();
  void printReasonCounts();

  shared_ptr<PlanningVisualization> visualization_;
  void getvisualization(shared_ptr<PlanningVisualization> visualization) {
    visualization_ = visualization;
    init_visual = true;
  }

  int debug_count;
  DEBUG_TYPE debug_type_now_;

  void addDebugCost(DEBUG_TYPE type, COST_TYPE cost_type, const double& cost);
  void coutDebugMsg(DEBUG_TYPE type, const size_t& max_size);
  void getPosBspline(NonUniformBspline bspline_pos) {
    if (!init_visual || !init_success) return;
    bspline_pos_ = bspline_pos;
  }
  vector<Eigen::Vector3d> control_point_, cloud_;
  vector<double> intense_;
  vector<Eigen::Vector3d> control_point_grad_;
  bool showcloud;
  void getCloudForVisualization(DEBUG_TYPE type, const vector<Eigen::Vector3d>& control_point,
      const vector<Eigen::Vector3d>& cloud1, const vector<double>& intense, const vector<vector<Eigen::Vector3d>>& intense_grad) {
    if (!init_visual || !init_success) return;
    if (!debug_map[type]) return;
    if (control_point.size() != 3 && control_point.size() != 4) {
      ROS_ERROR("[getCloudForVisualization] CONTROL POINT SIZE ERROR: %zu", control_point.size());
      return;
    }
    showcloud = true;
    control_point_ = control_point;
    cloud_ = cloud1;
    intense_ = intense;
    control_point_grad_.assign(control_point.size(), Eigen::Vector3d::Zero());
    for (size_t i = 0; i < intense_grad.size(); i++) {
      if (intense_grad[i].size() != control_point.size()) {
        ROS_ERROR("[getCloudForVisualization] INPUT SIZE ERROR: %zu", intense_grad[i].size());
        return;
      }
      for (size_t j = 0; j < control_point.size(); j++) control_point_grad_[j] += intense_grad[i][j];
    }
  }
};

}  // namespace fast_planner

#endif  // STEPPING_DEBUG_HPP
