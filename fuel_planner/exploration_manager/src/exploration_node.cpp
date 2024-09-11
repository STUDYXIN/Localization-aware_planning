#include <ros/ros.h>
// #include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/perception_aware_exploration_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");

  auto expl_fsm = std::make_shared<PAExplorationFSM>();  // shared_from_this()必须以智能指针的形式初始化对象
  expl_fsm->init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
