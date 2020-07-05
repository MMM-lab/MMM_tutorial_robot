#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

#include <string>

void subscribeState(const sensor_msgs::JointState rrobotState)
{
  ROS_INFO("joint1 position %f", rrobotState.position[0]);
  ROS_INFO("joint2 position %f", rrobotState.position[1]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mmm_robot_interface");

  ros::NodeHandle n;

  ros::Publisher joint1_pub = n.advertise<std_msgs::Float64>("/mmm_robot/left_wheel_joint/command", 1000);
  ros::Publisher joint2_pub = n.advertise<std_msgs::Float64>("/mmm_robot/right_wheel_joint/command", 1000);

  ros::Subscriber sub = n.subscribe("/mmm_robot/joint_states", 1000, subscribeState);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::Float64 joint1_torque;
    std_msgs::Float64 joint2_torque;

    joint1_torque.data = std::stof(argv[1]);
    joint2_torque.data = std::stof(argv[2]);

    // ROS_INFO("%f", joint1_torque.data);
    // ROS_INFO("%f", joint2_torque.data);

    joint1_pub.publish(joint1_torque);
    joint2_pub.publish(joint2_torque);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
