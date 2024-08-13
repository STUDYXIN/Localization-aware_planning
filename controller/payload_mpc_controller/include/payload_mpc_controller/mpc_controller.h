#pragma once

#include <thread>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <flatness.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>

#include <queue>
#include "mpc_wrapper.h"
#include "mpc_params.h"
#include "polynomial_trajectory.h"

namespace PayloadMPC
{
  // 状态变量
  enum STATE
  {
    kPosX = 0,
    kPosY = 1,
    kPosZ = 2,
    kOriW = 3,
    kOriX = 4,
    kOriY = 5,
    kOriZ = 6,
    kVelX = 7,
    kVelY = 8,
    kVelZ = 9
  };

  enum INPUT_BODYRATE
  {
    kAccZ = 0,
    kRateX = 1,
    kRateY = 2,
    kRateZ = 3
  };

  struct TorquesAndThrust
  {
    Eigen::Vector3d body_torques;
    double collective_thrust;
  };

  class MpcController
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static_assert(kStateSize == 10,
                  "MpcController: Wrong model size. Number of states does not match.");
    static_assert(kInputSize == 4,
                  "MpcController: Wrong model size. Number of inputs does not match.");

    MpcController(MpcParams &params);

    void execMPC(const Eigen::Matrix<real_t, kStateSize, kSamples + 1> &reference_state,
                 const Eigen::Matrix<real_t, kInputSize, kSamples + 1> &reference_input,
                 const Eigen::Matrix<real_t, kStateSize, 1> &estimated_state,
                 Eigen::Matrix<real_t, kStateSize, kSamples + 1> &predicted_states,
                 Eigen::Matrix<real_t, kInputSize, kSamples> &control_inputs);
    void execMPC(const Eigen::Matrix<real_t, kStateSize, 1> &estimated_state,
                 Eigen::Matrix<real_t, kStateSize, kSamples + 1> &predicted_states,
                 Eigen::Matrix<real_t, kInputSize, kSamples> &control_inputs);
    void setHoverReference(const Eigen::Ref<const Eigen::Vector3d> &quad_position, const double yaw);
    void setTrajectoyReference(Trajectory &traj, double tstart,double start_yaw);
    double getTimeStep(){return mpc_time_step_;}

    quadrotor_msgs::Px4ctrlDebug debug;

  private:
    double last_yaw_;
    double last_yaw_dot_;
    // Internal helper functions.

    // void offCallback(const std_msgs::Empty::ConstPtr& msg);
    void calculate_yaw(Eigen::Vector3d &vel, const double dt, double &yaw, double &yawdot);
    
    void preparationThread();
    double angle_limit(double ang);
    double angle_diff(double a, double b);

    // Parameters
    MpcParams& params_;

    Flatness payload_flat_;

    // MPC
    MpcWrapper mpc_wrapper_;
    const double mpc_time_step_;

    // Preparation Thread
    std::thread preparation_thread_;

    // Variables
    real_t timing_feedback_, timing_preparation_;
    bool solve_from_scratch_;
    
  public:
    Eigen::Matrix<real_t, kStateSize, kSamples + 1> reference_states_;
    Eigen::Matrix<real_t, kInputSize, kSamples + 1> reference_inputs_;
    Eigen::Matrix<real_t, kInputSize, kSamples + 1> hover_input_;
  };

} // namespace MPC
