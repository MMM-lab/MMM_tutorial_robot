#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "../include/AngleController.hpp"

#include <string>

Robot_p3d robot_p3d;
Robot_Wheel_Rate robot_wheel_rate;

void subscribeState(const sensor_msgs::JointState robotState)
{
  robot_wheel_rate.left = robotState.velocity[0];
  robot_wheel_rate.right = robotState.velocity[1];
  // ROS_INFO("joint1 velocity %f", robot_wheel_rate.left);
  // ROS_INFO("joint2 velocity %f", robot_wheel_rate.right);
}

void subscribePosState(const nav_msgs::Odometry posState)
{
  robot_p3d.posX = posState.pose.pose.position.x;
  robot_p3d.posY = posState.pose.pose.position.y;
  robot_p3d.posZ = posState.pose.pose.position.z;
  robot_p3d.angleX = posState.twist.twist.angular.x;
  robot_p3d.angleY = posState.twist.twist.angular.y;
  robot_p3d.angleZ = posState.twist.twist.angular.z;

  ROS_INFO("robot_p3d.posX: %f", robot_p3d.posX);
  ROS_INFO("robot_p3d.posY: %f", robot_p3d.posY);
  ROS_INFO("robot_p3d.posZ: %f", robot_p3d.posZ);
  ROS_INFO("robot_p3d.angleX: %f", robot_p3d.angleX);
  ROS_INFO("robot_p3d.angleY: %f", robot_p3d.angleY);
  ROS_INFO("robot_p3d.angleZ: %f", robot_p3d.angleZ);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mmm_robot_interface");

  AngleController angleController;
  angleController.anglePIDController.setGain(std::stof(argv[3]), std::stof(argv[4]), std::stof(argv[5]));
  ros::NodeHandle n;

  ros::Publisher joint1_pub = n.advertise<std_msgs::Float64>("/mmm_robot/left_wheel_joint/command", 1000);
  ros::Publisher joint2_pub = n.advertise<std_msgs::Float64>("/mmm_robot/right_wheel_joint/command", 1000);

  ros::Subscriber sub = n.subscribe("/mmm_robot/joint_states", 1000, subscribeState);
  ros::Subscriber posStateSub = n.subscribe("/base_link_pos", 1000, subscribePosState);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    Robot_Input robot_input;

    float targetAngle = std::stof(argv[1]);
    float accel = std::stof(argv[2]);
    angleController.getTorques(&robot_input, targetAngle, robot_p3d.angleZ, accel);

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
