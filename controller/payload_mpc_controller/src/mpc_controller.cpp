#include "mpc_controller.h"
#include <nav_msgs/Odometry.h>
#include <ctime>

namespace PayloadMPC
{

  MpcController::MpcController(MpcParams &params) : params_(params), mpc_time_step_(params_.step_T_)
  {
    auto & q_gain_ = params_.q_gain_;
    auto & r_gain_ = params_.r_gain_;
    
    Eigen::Matrix<real_t, kCostSize, kCostSize> Q = (Eigen::Matrix<real_t, kCostSize, 1>() << q_gain_.Q_pos_xy, q_gain_.Q_pos_xy, q_gain_.Q_pos_z,
                                                     q_gain_.Q_attitude_rp, q_gain_.Q_attitude_rp, q_gain_.Q_attitude_rp, q_gain_.Q_attitude_yaw,
                                                     q_gain_.Q_velocity, q_gain_.Q_velocity, q_gain_.Q_velocity)
                                                        .finished()
                                                        .asDiagonal();
    Eigen::Matrix<real_t, kInputSize, kInputSize> R = (Eigen::Matrix<real_t, kInputSize, 1>() << r_gain_.R_accz, r_gain_.R_pitchroll, r_gain_.R_pitchroll, r_gain_.R_yaw).finished().asDiagonal();
    // TAG: initial_state pos_z设置为0
    Eigen::Matrix<real_t, kStateSize, 1> initial_state = (Eigen::Matrix<real_t, kStateSize, 1>() << 0.0, 0.0, 0.0,
                                                          1.0, 0.0, 0.0, 0.0,
                                                          0.0, 0.0, 0.0)
                                                            .finished();
    reference_states_ = initial_state.replicate(1, kSamples+1); // kStateSize * （kSamples+1）的矩阵
    Eigen::Matrix<real_t, kInputSize, 1> initial_input = (Eigen::Matrix<real_t, kInputSize, 1>() << params_.gravity_, 0.0, 0.0, 0.0).finished(); // 初始化输入矩阵 
    // ROS_INFO("initial_input: %f, %f, %f, %f", initial_input(0), initial_input(1), initial_input(2), initial_input(3));

    // 悬停状态的输入矩阵
    hover_input_ = initial_input.replicate(1, kSamples + 1);
    reference_inputs_ = hover_input_;

    mpc_wrapper_.initialize(Q, R, initial_state, initial_input, params_.state_cost_exponential_, params_.input_cost_exponential_);

    // mpc_wrapper_.setLimits(
    //     params_.max_acceleration_z_,
    //     params_.max_bodyrate_xy_, params_.max_bodyrate_z_);

    mpc_wrapper_.setLimits(
        params_.max_acceleration_z_,
        params_.max_bodyrate_xy_, params_.max_bodyrate_z_,
        params_.max_velocity_);

    // first_traj_received_ = false;
    solve_from_scratch_ = false;
    timing_feedback_ = 0;
    timing_preparation_ = 0;
    preparation_thread_ = std::thread(&MpcWrapper::prepare, mpc_wrapper_);
    
  }

  void MpcController::execMPC(const Eigen::Matrix<real_t, kStateSize, kSamples + 1> &reference_states,
                              const Eigen::Matrix<real_t, kInputSize, kSamples + 1> &reference_inputs,
                              const Eigen::Matrix<real_t, kStateSize, 1> &estimated_state,
                              Eigen::Matrix<real_t, kStateSize, kSamples + 1> &predicted_states,
                              Eigen::Matrix<real_t, kInputSize, kSamples> &control_inputs)
  {
    const clock_t start = clock();

    preparation_thread_.join(); // waiting the preparation_thread_ finished

    // Get the feedback from MPC.

    mpc_wrapper_.setTrajectory(reference_states, reference_inputs);

    if (solve_from_scratch_) // 从头开始解决MPC问题，使用悬停状态作为初始猜测
    {
      ROS_INFO("Solving MPC with hover as initial guess.");
      mpc_wrapper_.solve(estimated_state);
      solve_from_scratch_ = false;
    }
    else // 上一次结果作为初始猜测
    {
      constexpr bool do_preparation_step(false); // the preparation step has been done by another thread
      mpc_wrapper_.update(estimated_state, do_preparation_step);
    }

    mpc_wrapper_.getStates(predicted_states);
    mpc_wrapper_.getInputs(control_inputs);

    // Start a thread to prepare for the next execution.
    preparation_thread_ = std::thread(&MpcController::preparationThread, this);

    // Timing
    const clock_t end = clock();
    timing_feedback_ = 0.9*timing_feedback_ + 0.1* double(end - start) / CLOCKS_PER_SEC;
    if (params_.print_info_)
      ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.2f ms  |  Total: %1.2f ms",
                      timing_feedback_ * 1000, (timing_feedback_ + timing_preparation_) * 1000);
  }

  // 重载函数
  void MpcController::execMPC(const Eigen::Matrix<real_t, kStateSize, 1> &estimated_state,
                              Eigen::Matrix<real_t, kStateSize, kSamples + 1> &predicted_states,
                              Eigen::Matrix<real_t, kInputSize, kSamples> &control_inputs)
  {
    execMPC(reference_states_, reference_inputs_, estimated_state, predicted_states, control_inputs);
  }

  // DONE: maybe it's right?
  void MpcController::setHoverReference(const Eigen::Ref<const Eigen::Vector3d> &quad_position, const double yaw)
  {
    Eigen::Matrix<real_t, 3, 1> quad_pos = quad_position.cast<real_t>();
    
    Eigen::Matrix<real_t, kStateSize, 1> reference_state;
    // q.normalize();
    
    // reference_state << quad_pos.x(), quad_pos.y(), quad_pos.z(),
    //                    1.0, 0.0, 0.0, 0.0,
    //                    0.0, 0.0, 0.0;

    reference_state << 0.0, 0.0, 0.5,
                       0.707, 0.0, 0.0, 0.707,
                       0.0, 0.0, 0.0;

    Eigen::Matrix<real_t, kInputSize, 1> hover_in; 
    hover_in << params_.gravity_, 0.0, 0.0, 0.0;
    // hover_in << 0.0, 0.0, 0.0, 0.0;
    reference_states_ = reference_state.replicate(1, kSamples + 1);
    reference_inputs_ = hover_in.replicate(1, kSamples + 1);
  }

  void MpcController::setTrajectoyReference(Trajectory &traj, double tstart, double start_yaw)
  {
    const double t_step = mpc_time_step_;
    double t_all = traj.getTotalDuration() - 1.0e-3;
    double t = tstart;
    double yaw, yaw_dot;
    Eigen::Vector3d pos, vel, acc, jerk, snap, crackle;

    Eigen::Quaterniond quat;
    double accz;
    Eigen::Vector3d omg;

    // last_yaw_ = start_yaw;  // must reset the last_yaw_ 

    for (int i = 0; i < (kSamples + 1); i++)
    {
      Eigen::MatrixXd pvajs;
      if (t > t_all)
      { // if t is larger than the total time, use the last point
        // TODO : Consider 2 trajectories
        t = t_all;
        pvajs = traj.getPVAJSC(t);
        pos = pvajs.col(0);
        vel = Eigen::Vector3d::Zero();
        acc =  Eigen::Vector3d::Zero();
        jerk = Eigen::Vector3d::Zero();
        snap = Eigen::Vector3d::Zero();
        crackle =  Eigen::Vector3d::Zero();
      }
      else
      {
        pvajs = traj.getPVAJSC(t);
        pos = pvajs.col(0);
        vel = pvajs.col(1);
        acc = pvajs.col(2);
        jerk = pvajs.col(3);
        snap = pvajs.col(4);
        crackle = pvajs.col(5);
      }

      // ROS_WARN("%f, %f, %f", vel(0), vel(1), vel(2));

      if (params_.use_fix_yaw_)
      {
        yaw = start_yaw;
        yaw_dot = 0.0; 
      }
      else
      {
        if(i == 0) // reset last_yaw_ from the first point
        {
          if(t - t_step <= 0.0) // 在轨迹开始的时刻
          {
            if (fabs(vel(0))+fabs(vel(1))>0.1) // 速度足够大
            {
              last_yaw_ = atan2(vel(1), vel(0));
              last_yaw_dot_ = 0.0;
            }
            else
            {
              last_yaw_ = start_yaw;
              last_yaw_dot_ = 0.0;
            }
          }
          else // Get yaw in last step ：从上一时刻获取yaw角
          {
            Eigen::MatrixXd last_pvajs = traj.getPVAJSC(t-t_step);
            Eigen::Vector3d last_pos= last_pvajs.col(0);
            Eigen::Vector3d last_vel = last_pvajs.col(1);
            Eigen::Vector3d last_acc = last_pvajs.col(2);
            Eigen::Vector3d last_jerk = last_pvajs.col(3);
            Eigen::Vector3d last_snap = last_pvajs.col(4);
            Eigen::Vector3d last_crackle = last_pvajs.col(5); 

            if (fabs(last_vel(0))+fabs(last_vel(1))>0.1)
            {
              last_yaw_ = atan2(last_vel(1), last_vel(0));
              last_yaw_dot_ = 0.0;
            }
            else
            {
              last_yaw_ = start_yaw;
              last_yaw_dot_ = 0.0;
            }
          }
        }
        calculate_yaw(vel, t_step, yaw, yaw_dot);  
      }

      accz = acc(2); 
      payload_flat_.forward_rot(acc, jerk, yaw, yaw_dot, quat, omg, accz);
      
      
      Eigen::Matrix<real_t, kStateSize, 1> reference_state;
      reference_state << pos.x(), pos.y(), pos.z(),
                         quat.w(), quat.x(), quat.y(), quat.z(),
                         vel.x(), vel.y(), vel.z();

      
      reference_states_.col(i) = reference_state;

      Eigen::Matrix<real_t, kInputSize, 1> reference_input;
      reference_input << accz, omg.x(), omg.y(), omg.z();
      
      reference_inputs_.col(i) = reference_input;
      t += t_step;
    }
    // std::cout << "reference_input: " << std::endl << reference_inputs_.transpose() << std::endl;
    // std::cout << "reference_state: " << std::endl << reference_states_.transpose() << std::endl;
  }

  void MpcController::preparationThread()
  {
    const clock_t start = clock();

    mpc_wrapper_.prepare();

    // Timing
    const clock_t end = clock();
    timing_preparation_ = timing_preparation_*0.9 + 0.1* double(end - start) / CLOCKS_PER_SEC;
  }

  double MpcController::angle_limit(double ang)
  {
    while (ang > M_PI)
    {
      ang -= 2.0 * M_PI;
    }
    while (ang <= -M_PI)
    {
      ang += 2.0 * M_PI;
    }
    return ang;
  }
  double MpcController::angle_diff(double a, double b) // 计算两个角度之间的差异，保证结果在[-π，π]之间
  {
    double d1, d2;
    d1 = a-b;
    d2 = 2*M_PI - fabs(d1);
    if(d1 > 0)
      d2 *= -1.0;
    if(fabs(d1) < fabs(d2))
      return(d1);
    else
      return(d2);
  }
  void MpcController::calculate_yaw(Eigen::Vector3d &vel, const double dt, double &yaw, double &yawdot)
  {
    const double YAW_DOT_MAX_PER_SEC = params_.max_bodyrate_z_;

    double yaw_temp;
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * dt;

    // tangent line
    if ((fabs(vel(1)) + fabs(vel(0))) < 0.1) // 避免除零错误
    {
      yaw_temp = last_yaw_;
    }
    else
    {
      yaw_temp = atan2(vel(1), vel(0));     
    }
    double yaw_diff = angle_diff(yaw_temp, last_yaw_);
    
    if (yaw_diff > max_yaw_change )
    {
      yaw_diff = max_yaw_change;
      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else if(yaw_diff < -max_yaw_change)
    {
      yaw_diff = -max_yaw_change;
      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yawdot = yaw_diff / dt;
    }
    
    yaw = last_yaw_ + yaw_diff;

    // std::cout<<"last_yaw: "<< last_yaw_ << "yaw: " << yaw << " yawdot: " << yawdot << std::endl;
    
    // std::cout<< "dt: " << dt << " max_yaw_change: " << max_yaw_change << " vel: " << vel << " last_yaw_: " << last_yaw_ << " yaw: " << yaw << " yawdot: " << yawdot << std::endl;

    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;
  }

  
}
