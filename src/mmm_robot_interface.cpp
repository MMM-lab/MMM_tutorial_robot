#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include "../include/torque_calculater.hpp"
#include "../include/extended_kalman_filter.hpp"

#include <string>
#include "cstdio"

Extended_Kalman_Filter EKF;

void Inputcallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  float velocity        = cmd_vel->linear.x;
  float anglar_velocity = cmd_vel->angular.z;

  //ROS_INFO("velocity %f", cmd_vel->linear.x);
  //ROS_INFO("Angular velocity %f", cmd_vel->angular.z);

  EKF.setInput(velocity, anglar_velocity);
}

void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg)
{ 
  //msg->pose.pose.position, msg->pose.pose.orientation, 
  float x = msg->pose.pose.position.x;
  float y = msg->pose.pose.position.y;
  float theta = msg->pose.pose.orientation.z;
  /*ROS_INFO("Position: X = %f \n", x);
  ROS_INFO("Position: Y = %f \n", y);
  ROS_INFO("Position: Theta = %f \n", theta);*/

  EKF.setPosition(x, y, theta);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mmm_robot_interface");

  ros::NodeHandle n;
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/velocity_robot/diff_drive_controller/cmd_vel", 100);

  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = 1.0;
  cmd_vel.linear.y  = 0.0;
  cmd_vel.linear.z  = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = 0.2;

  ros::Subscriber cmd_vel_sub = n.subscribe("/velocity_robot/diff_drive_controller/cmd_vel", 100, Inputcallback);
  ros::Subscriber position    = n.subscribe<nav_msgs::Odometry>("/velocity_robot/diff_drive_controller/odom", 100, groundTruthCallback);

  ros::Rate loop_rate(10);

  // save data
  FILE *fp;
  if ((fp = fopen("data.txt", "w")) == NULL) {
      printf("Error\n");
      exit(1);
  }
  
  while (ros::ok())
  {
    cmd_vel_pub.publish(cmd_vel);

    /*nav_msgs::Odometry odom;

    float velocity        = cmd_vel.linear.x;
    float anglar_velocity = cmd_vel.angular.z;
    float x               = odom.pose.pose.position.x;
    float y               = odom.pose.pose.position.y;
    float theta           = odom.pose.pose.orientation.z;*/
    EKF.simulation();
    fprintf(fp, "%lf\t%lf\t%lf\t%lf\n", EKF.xTrue(0, 0), EKF.xTrue(1, 0), EKF.xEst(0, 0), EKF.xEst(1, 0));

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
