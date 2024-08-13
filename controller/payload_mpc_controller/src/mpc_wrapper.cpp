/**
 * @file mpc_wrapper.cpp
 * @author Haojia Li (hlied@connect.ust.hk)
 * @brief MPC interface. Wrapper for ACADO.
 * Thanks for rpg_mpc
 * @version 1.0
 * @date 2022-07-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "mpc_wrapper.h"

namespace PayloadMPC
{
  ACADOvariables acadoVariables;
  ACADOworkspace acadoWorkspace;

  MpcWrapper::MpcWrapper()
  {
    ;
    // Please use the initialize function to initialize the MPC
    // const Eigen::Matrix<real_t, kStateSize, 1> hover_state =
    //   (Eigen::Matrix<real_t, kStateSize, 1>() << 0.0, 0.0, 0.0,
    //                                         1.0, 0.0, 0.0, 0.0,
    //                                         0.0, 0.0, 0.0).finished();
  }

  void MpcWrapper::initialize(
      const Eigen::Ref<const Eigen::Matrix<real_t, kCostSize, kCostSize>> &Q,
      const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, kInputSize>> &R,
      const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, 1>> &initial_state,
      const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, 1>> &initial_input,
      real_t state_cost_scaling,
      real_t input_cost_scaling)
  // Clear solver memory.
  {
    memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
    memset(&acadoVariables, 0, sizeof(acadoVariables));

    setCosts(Q, R, state_cost_scaling, input_cost_scaling);
    // setDynamicParams(mass_q_, mass_l_, l_length_);

    // Initialize the solver.
    acado_initializeSolver();

    // Initialize the states and controls.
    kHoverInput_ = initial_input;
    // for(int i = 0; i < kInputSize; i++)
    //   ROS_ERROR("row %d: %f", i, kHoverInput_(i));

    // Initialize states x and xN and input u.
    acado_initial_state_ = initial_state;

    acado_states_ = initial_state.replicate(1, kSamples + 1);

    acado_inputs_ = kHoverInput_.replicate(1, kSamples);

    // Initialize references y and yN.
    acado_reference_states_.block(0, 0, kStateSize, kSamples) =
        initial_state.replicate(1, kSamples);

    acado_reference_states_.block(kStateSize, 0, kCostSize - kStateSize, kSamples) =
        Eigen::Matrix<real_t, kCostSize - kStateSize, kSamples>::Zero();

    acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
        kHoverInput_.replicate(1, kSamples);

    acado_reference_end_state_.segment(0, kStateSize) =
        initial_state;

    acado_reference_end_state_.segment(kStateSize, kCostSize - kStateSize) =
        Eigen::Matrix<real_t, kCostSize - kStateSize, 1>::Zero();

    // Initialize Cost matrix W and WN.
    if (!(acado_W_.trace() > 0.0))
    {
      acado_W_ = W_.replicate(1, kSamples);
      acado_W_end_ = WN_;
    }

    // Initialize solver.
    acado_initializeNodesByForwardSimulation();
    acado_preparationStep();
    acado_is_prepared_ = true;
  }

  // Set cost matrices with optional scaling.
  bool MpcWrapper::setCosts(
      const Eigen::Ref<const Eigen::Matrix<real_t, kCostSize, kCostSize>> &Q,
      const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, kInputSize>> &R,
      real_t state_cost_scaling, real_t input_cost_scaling)
  {
    if (state_cost_scaling < 0.0 || input_cost_scaling < 0.0)
    {
      ROS_ERROR("MPC: Cost scaling is wrong, must be non-negative!");
      return false;
    }

    W_.block(0, 0, kCostSize, kCostSize) = Q;
    W_.block(kCostSize, kCostSize, kInputSize, kInputSize) = R;
    WN_ = W_.block(0, 0, kCostSize, kCostSize);

    // ROS_INFO_STREAM("weight_matrix: " << std::endl << W_);

    real_t state_scale{1.0};
    real_t input_scale{1.0};
    for (int i = 0; i < kSamples; i++)
    {
      state_scale = exp(-real_t(i) / real_t(kSamples) * real_t(state_cost_scaling));
      input_scale = exp(-real_t(i) / real_t(kSamples) * real_t(input_cost_scaling));
      acado_W_.block(0, i * kRefSize, kCostSize, kCostSize) =
          W_.block(0, 0, kCostSize, kCostSize) * state_scale;
      acado_W_.block(kCostSize, i * kRefSize + kCostSize, kInputSize, kInputSize) =
          W_.block(kCostSize, kCostSize, kInputSize, kInputSize) * input_scale;
    }
    acado_W_end_ = WN_ * state_scale;

    return true;
  }

  // Set the input limits.
  // bool MpcWrapper::setLimits(real_t max_accz, real_t max_rollpitchrate, real_t max_yawrate)
  // {
  //   if (max_accz <= 0.0)
  //   {
  //     ROS_ERROR("MPC: Maximal acceleration is not set properly, not changed.");
  //     return false;
  //   }

  //   if (max_rollpitchrate <= 0.0)
  //   {
  //     ROS_ERROR("MPC: Maximal xy-rate is not set properly, not changed.");
  //     return false;
  //   }

  //   if (max_yawrate <= 0.0)
  //   {
  //     ROS_ERROR("MPC: Maximal yaw-rate is not set properly, not changed.");
  //     return false;
  //   }

  //   // Set input boundaries.
  //   Eigen::Matrix<real_t, 4, 1> lower_bounds = Eigen::Matrix<real_t, 4, 1>::Zero();
  //   Eigen::Matrix<real_t, 4, 1> upper_bounds = Eigen::Matrix<real_t, 4, 1>::Zero();
  //   lower_bounds << -max_accz,
  //       -max_rollpitchrate, -max_rollpitchrate, -max_yawrate;
  //   upper_bounds << max_accz,
  //       max_rollpitchrate, max_rollpitchrate, max_yawrate;
  //   Eigen::Matrix<real_t, 1, 1> lower_affine_bounds;
  //   lower_affine_bounds << -1.1;
  //   Eigen::Matrix<real_t, 1, 1> upper_affine_bounds;
  //   upper_affine_bounds << -0.1;

  //   acado_lower_bounds_ =
  //       lower_bounds.replicate(1, kSamples);

  //   acado_upper_bounds_ =
  //       upper_bounds.replicate(1, kSamples);
  //   return true;
  // }

  bool MpcWrapper::setLimits(real_t max_accz, real_t max_rollpitchrate, real_t max_yawrate, real_t max_vel)
  {
    if (max_accz <= 0.0)
    {
      ROS_ERROR("MPC: Maximal acceleration is not set properly, not changed.");
      return false;
    }

    if (max_rollpitchrate <= 0.0)
    {
      ROS_ERROR("MPC: Maximal xy-rate is not set properly, not changed.");
      return false;
    }

    if (max_yawrate <= 0.0)
    {
      ROS_ERROR("MPC: Maximal yaw-rate is not set properly, not changed.");
      return false;
    }

    if (max_vel <= 0.0)
    {
      ROS_ERROR("MPC: Maximal velocity is not set properly, not changed.");
      return false;
    }

    // Set input boundaries.
    Eigen::Matrix<real_t, 4, 1> lower_bounds = Eigen::Matrix<real_t, 4, 1>::Zero();
    Eigen::Matrix<real_t, 4, 1> upper_bounds = Eigen::Matrix<real_t, 4, 1>::Zero();
    lower_bounds << 0.0, // 加速度不能反向！
                  -max_rollpitchrate, -max_rollpitchrate, -max_yawrate;
    upper_bounds << max_accz,
                  max_rollpitchrate, max_rollpitchrate, max_yawrate;

    // Eigen::Matrix<real_t, 3, 1> lower_affine_bounds;
    // lower_affine_bounds << -max_vel, -max_vel, -max_vel;
    // Eigen::Matrix<real_t, 3, 1> upper_affine_bounds;
    // upper_affine_bounds << max_vel, max_vel, max_vel;

    acado_lower_bounds_ =
        lower_bounds.replicate(1, kSamples);

    acado_upper_bounds_ =
        upper_bounds.replicate(1, kSamples);
    
    // acado_lower_affine_bounds_ = 
    //     lower_affine_bounds.replicate(1, kSamples);

    // acado_upper_affine_bounds_ =
    //     upper_affine_bounds.replicate(1, kSamples);

    return true;
  }
  
  // Set a reference pose.
  bool MpcWrapper::setReferencePose(
      const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, 1>> reference_state)
  {
    acado_reference_states_.block(0, 0, kStateSize, kSamples) =
        reference_state.replicate(1, kSamples);

    acado_reference_states_.block(kStateSize, 0, kCostSize - kStateSize, kSamples) =
        Eigen::Matrix<real_t, kCostSize - kStateSize, kSamples>::Zero();

    acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
        kHoverInput_.replicate(1, kSamples);

    acado_reference_end_state_.segment(0, kStateSize) =
        reference_state;

    acado_reference_end_state_.segment(kStateSize, kCostSize - kStateSize) =
        Eigen::Matrix<real_t, kCostSize - kStateSize, 1>::Zero();

    acado_initializeNodesByForwardSimulation();
    return true;
  }

  // Set a reference trajectory.
  bool MpcWrapper::setTrajectory(
      const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, kSamples + 1>> reference_states,
      const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, kSamples + 1>> inputs)
  {

    acado_reference_states_.block(0, 0, kStateSize, kSamples) =
        reference_states.block(0, 0, kStateSize, kSamples);

    acado_reference_states_.block(kStateSize, 0, kCostSize - kStateSize, kSamples) =
        Eigen::Matrix<real_t, kCostSize - kStateSize, kSamples>::Zero();

    acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
        inputs.block(0, 0, kInputSize, kSamples);

    acado_reference_end_state_.segment(0, kStateSize) =
        reference_states.col(kSamples);
    acado_reference_end_state_.segment(kStateSize, kCostSize - kStateSize) =
        Eigen::Matrix<real_t, kCostSize - kStateSize, 1>::Zero();

    return true;
  }

  // Reset states and inputs and calculate new solution.
  // 每次停下（结束上一段轨迹），重置 states 和 inputs 
  bool MpcWrapper::solve(
      const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, 1>> state)
  {
    acado_states_ = state.replicate(1, kSamples + 1);

    acado_inputs_ = kHoverInput_.replicate(1, kSamples); 

    return update(state);
  }

  // Calculate new solution from last known solution.
  bool MpcWrapper::update(
      const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, 1>> state,
      bool do_preparation)
  {
    if (!acado_is_prepared_)
    {
      ROS_WARN("MPC: Solver was triggered without preparation, abort!");
      return false;
    }

    // Check if estimated and reference quaternion live in the same hemisphere.
    acado_initial_state_ = state; // 将当前state设置为初始状态
    if (acado_initial_state_.segment(3, 4).dot(
            Eigen::Matrix<real_t, 4, 1>(acado_reference_states_.block(3, 0, 4, 1))) < (real_t)0.0)
    {
      acado_initial_state_.segment(3, 4) = -acado_initial_state_.segment(3, 4);
    }

    // Perform feedback step and reset preparation check.
    int ret = acado_feedbackStep();
    acado_is_prepared_ = false;

    // Prepare if the solver if wanted
    if (do_preparation)
    {
      prepare();
    }
    if (ret != 0)
    {
      ROS_ERROR("MPC: Feedback step failed,  %d", ret);
      ROS_ERROR("MPC ERROR: %s", acado_getErrorString(ret));
      return false;
    }

    return true;
  }

  // Prepare the solver.
  // Must be triggered between iterations if not done in the update function.
  // template <typename T>

  bool MpcWrapper::prepare()
  {
    int ret = acado_preparationStep();
    if (ret != 0)
    {
      ROS_ERROR("MPC: Preparation step failed,  %d", ret);
      ROS_ERROR("MPC ERROR: %s", acado_getErrorString(ret));
      return false;
    }
    acado_is_prepared_ = true;
    return true;
  }

  // Get all states.
  void MpcWrapper::getStates(
      Eigen::Ref<Eigen::Matrix<real_t, kStateSize, kSamples + 1>> return_states)
  {
    std::cout << "acado_states_" << std::endl << acado_states_.transpose() << std::endl;
    // if
    return_states = acado_states_;
  }

  // Get all inputs.
  void MpcWrapper::getInputs(
      Eigen::Ref<Eigen::Matrix<real_t, kInputSize, kSamples>> return_inputs)
  {
    std::cout << "acado_inputs_" << std::endl << acado_inputs_.transpose() << std::endl;
    return_inputs = acado_inputs_;
  }

} // namespace rpg_mpc
