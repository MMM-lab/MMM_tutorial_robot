#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "../include/torque_calculater.hpp"

#include <string>

Robot_Wheel_Rate robot_wheel_rate;

void subscribeState(const sensor_msgs::JointState robotState)
{
  robot_wheel_rate.left = robotState.velocity[0];
  robot_wheel_rate.right = robotState.velocity[1];
  ROS_INFO("joint1 velocity %f", robot_wheel_rate.left);
  ROS_INFO("joint2 velocity %f", robot_wheel_rate.right);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mmm_robot_interface");

  Torque_Control torque_control;
  torque_control.set_param( 0.2, 0.1, std::stof(argv[3]), std::stof(argv[4]), std::stof(argv[5]));

  ros::NodeHandle n;

  ros::Publisher joint1_pub = n.advertise<std_msgs::Float64>("/mmm_robot/left_wheel_joint/command", 1000);
  ros::Publisher joint2_pub = n.advertise<std_msgs::Float64>("/mmm_robot/right_wheel_joint/command", 1000);

  ros::Subscriber sub = n.subscribe("/mmm_robot/joint_states", 1000, subscribeState);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    Robot_Input robot_input;
    Robot_State target_robot_state;

    target_robot_state.yow_rate = std::stof(argv[1]);
    target_robot_state.velocity = std::stof(argv[2]);


    torque_control.get_torques(&robot_input, &robot_wheel_rate, &target_robot_state);

    std_msgs::Float64 joint1_torque;
    std_msgs::Float64 joint2_torque;

    joint1_torque.data = robot_input.left;
    joint2_torque.data = robot_input.right;

    ROS_INFO("joint1_torque %f", joint1_torque.data);
    ROS_INFO("joint2_torque %f", joint2_torque.data);

    joint1_pub.publish(joint1_torque);
    joint2_pub.publish(joint2_torque);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
