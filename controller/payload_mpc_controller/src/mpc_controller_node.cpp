#include "mpc_wrapper.h"
#include "mpc_fsm.h"
#include "mpc_controller.h"
#include "airsim_ros/AngleRateThrottle.h"
#include <ros/ros.h>

std::unique_ptr<PayloadMPC::MPCFSM> fsm_ptr; // 状态机指针
void MPC_controller_main(const ros::TimerEvent &)
{
    fsm_ptr->process();
}

void system_state_update_main(const ros::TimerEvent &)
{
    fsm_ptr->addNewForceObseverState();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MPCctrl");
    ros::NodeHandle nh("~");

    PayloadMPC::MpcParams param;
    param.config_from_ros_handle(nh);

    PayloadMPC::MpcController controller(param);
    fsm_ptr.reset(new PayloadMPC::MPCFSM(nh, param, controller));
    PayloadMPC::MPCFSM &fsm = *fsm_ptr;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",
                                                                 10,
                                                                 boost::bind(&State_Data_t::feed, &fsm.state_data, _1));

    ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",
                                                                                  10,
                                                                                  boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));
    
    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber start_trig_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("start_trigger",
                                                 10,
                                                 boost::bind(&Start_Trigger_Data_t::feed, &fsm.start_trigger_data, _1));

    ros::Subscriber cmd_trig_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("cmd_trigger",
                                                 10,
                                                 boost::bind(&Cmd_Trigger_Data_t::feed, &fsm.cmd_trigger_data, _1));

    ros::Subscriber mpc_traj_sub =
        nh.subscribe<quadrotor_msgs::PolynomialTraj>("traj",
                                                     100,
                                                     boost::bind(&Trajectory_Data_t::feed, &fsm.trajectory_data, _1),
                                                     ros::VoidConstPtr(),
                                                     ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("drone_imu/data",
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    fsm.imu_data.set_filter_params(param.filter_param_.sample_freq_quad_acc, param.filter_param_.cutoff_freq_quad_acc,
                                   param.filter_param_.sample_freq_quad_omg, param.filter_param_.cutoff_freq_quad_omg);

    ros::Subscriber rc_sub;
    {
        rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
                                                 10,
                                                 boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
    }

    ros::Subscriber bat_sub =
        nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                                                100,
                                                boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
    // ros::Subscriber esc_sub =
    //     nh.subscribe<mavros_msgs::ESCTelemetry>("/mavros/esc_telemetry",
    //                                             100,
    //                                             boost::bind(&Rpm_Data_t::feed, &fsm.rpm_data, _1),
    //                                             ros::VoidConstPtr(),
    //                                             ros::TransportHints().tcpNoDelay());


    fsm.planning_stop_pub_ = nh.advertise<std_msgs::Empty>("/planning_stop_trigger", 10);
    fsm.planning_restart_pub_ = nh.advertise<std_msgs::Empty>("/planning_restart_trigger", 10);

    bool use_simulation;
    nh.param<bool>("use_simulation", use_simulation, false);
    
    if(use_simulation){
        fsm.ctrl_FCU_pub = nh.advertise<airsim_ros::AngleRateThrottle>("/airsim_node/drone_1/angle_rate_throttle_frame", 10);
    }else{
        fsm.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    }

    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);
    fsm.des_yaw_pub = nh.advertise<nav_msgs::Odometry>("/des_yaw_pub", 10);

    fsm.set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    fsm.arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    fsm.reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    ros::Duration(0.5).sleep();
    if (!param.use_simulation_)
    {
        ROS_INFO("[MPCCTRL] Waiting for RC");
        while (ros::ok())
        {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now()))
            {
                ROS_INFO("[MPCCTRL] RC received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
        int trials = 0;
        while (ros::ok() && !fsm.state_data.current_state.connected)
        {
            ros::spinOnce();
            ros::Duration(1.0).sleep();
            if (trials++ > 5)
                ROS_ERROR("Unable to connnect to PX4!!!");
        }
    }
    else
    {
        ROS_WARN("[MPCCTRL] Remote controller disabled, be careful!");
    }

    // Create a ROS timer for controller
    ros::Timer MPC_controller_main_timer = nh.createTimer(ros::Duration(1.0 / param.ctrl_freq_max_), MPC_controller_main);
    // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.

    ros::spin();

    return 0;
}
