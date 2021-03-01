#include <memory>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include "ugv_sdk/scout/scout_base.hpp"
#include "scout_base/scout_messenger.hpp"

using namespace westonrobot;

std::shared_ptr<ScoutBase> robot;

void DetachRobot(int signal) {
  robot->Disconnect();
  robot->Terminate();
}

int main(int argc, char **argv) {
  // setup ROS node
  ros::init(argc, argv, "scout_odom");
  ros::NodeHandle node(""), private_node("~");

  std::signal(SIGINT, DetachRobot);

  // check wether controlling scout mini
  bool is_scout_mini = false;
  private_node.param<bool>("is_scout_mini", is_scout_mini, false);
  std::cout << "Working as scout mini: " << is_scout_mini << std::endl;

  // instantiate a robot object
  robot = std::make_shared<ScoutBase>(is_scout_mini);
  ScoutROSMessenger messenger(robot.get(), &node);

  // fetch parameters before connecting to robot
  std::string port_name;
  private_node.param<std::string>("port_name", port_name, std::string("can0"));
  private_node.param<std::string>("odom_frame", messenger.odom_frame_,
                                  std::string("odom"));
  private_node.param<std::string>("base_frame", messenger.base_frame_,
                                  std::string("base_link"));
  private_node.param<bool>("simulated_robot", messenger.simulated_robot_,
                           false);
  private_node.param<int>("control_rate", messenger.sim_control_rate_, 50);
  private_node.param<std::string>("odom_topic_name", messenger.odom_topic_name_,
                                  std::string("odom"));

  if (!messenger.simulated_robot_) {
    // connect to robot and setup ROS subscription
    if (port_name.find("can") != std::string::npos) {
      robot->Connect(port_name);
      ROS_INFO("Using CAN bus to talk with the robot");
    } else {
      robot->Connect(port_name, 115200);
      ROS_INFO("Using UART to talk with the robot");
    }
  }
  messenger.SetupSubscription();

  // publish robot state at 50Hz while listening to twist commands
  ros::Rate rate(50);
  while (true) {
    if (!messenger.simulated_robot_) {
      messenger.PublishStateToROS();
    } else {
      double linear, angular;
      messenger.GetCurrentMotionCmdForSim(linear, angular);
      messenger.PublishSimStateToROS(linear, angular);
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}