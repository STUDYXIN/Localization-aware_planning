#ifndef _ACTIVE_LOCAL_FSM_H_
#define _ACTIVE_LOCAL_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline/bspline_optimizer.h>
#include <pathfinding/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_manage/planner_manager.h>
#include <tic_toc.h>
#include <traj_utils/planning_visualization.h>
#include <trajectory/Bspline.h>

using std::vector;

namespace fast_planner
{
  class AgilePerceptionFSM
  {
  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      YAW_PREPARE,
      EXEC_TRAJ,
      REPLAN_TRAJ
    };

    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    FastPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_;
    double last_arrive_time_;

    /* planning data */
    bool trigger_, have_target_, have_odom_;
    FSM_EXEC_STATE exec_state_;

    Eigen::Vector3d odom_pos_, odom_vel_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d start_yaw_, end_yaw_;
    Eigen::Vector3d start_pt_, start_vel_, start_acc_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                 // target state
    int current_wp_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_;
    ros::ServiceServer map_save_service_, map_load_service_, start_eval_service_, end_eval_service_;
    ros::Publisher bspline_pub_, replan_pub_, new_pub_;

    /* ROS callbacks */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);

    void waypointCallback(const nav_msgs::PathConstPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);

    /* ROS services */
    bool saveMapCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
    bool loadMapCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

    /* Helper functions */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);

    /**
     * \brief 调用Agile Perception replan算法，整个项目核心创新点的入口
     *
     * \note 集成了运动规划的前端和后端方法
     *
     * \return True代表规划成功，False代表规划失败
     */
    bool callAgilePerceptionReplan();

    bool calcTrajLenInKnownSpace(double &len);

    void pubBspline(LocalTrajData &traj_data);

  public:
    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace fast_planner

#endif