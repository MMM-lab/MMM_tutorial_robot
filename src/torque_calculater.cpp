#include "../include/torque_calculater.hpp"
#include "cstdio"

void Torque_Control::set_param( float tread, float diameter, float gain_P, float gain_I, float gain_D)
{
    param.diameter = diameter;
    param.tread = tread;
    P = gain_P;
    I = gain_I;
    D = gain_D;
}

void Torque_Control::get_torques(Robot_Input *robot_input, Robot_Wheel_Rate *robot_wheel_rate, Robot_State *target_robot_state)
{
    float target_wheel_left_velocity = ( 2 * target_robot_state->velocity / param.diameter + 2 * PI * param.tread * target_robot_state->yow_rate / param.diameter ) / 2;
    float target_wheel_right_velocity = ( 2 * target_robot_state->velocity / param.diameter - 2 * PI * param.tread * target_robot_state->yow_rate / param.diameter ) / 2;

    printf("\n robot_wheel_rate->left %f \n", robot_wheel_rate->left);
    printf("\n target_wheel_left_velocity %f \n", target_wheel_left_velocity);
    printf("\n robot_wheel_rate->right %f \n", robot_wheel_rate->right);
    printf("\n target_wheel_right_velocity %f \n", target_wheel_right_velocity);
    float Pg_right = target_wheel_right_velocity - robot_wheel_rate->right;
    Ig_right=Ig_right+Pg_right*dt;
    float Dg_right=(robot_input->right-late_param.right)/dt;
    printf("\n Pg_right %f \n", Pg_right);
    printf("\n Ig_right %f \n", Ig_right);
    robot_input->right=P*Pg_right + I*Ig_right + D*Dg_right;
    
    float Pg_left = target_wheel_left_velocity - robot_wheel_rate->left;
    Ig_left=Ig_left+Pg_left*dt;
    float Dg_left=(robot_input->left-late_param.left)/dt;
    printf("\n Pg_left %f \n", Pg_left);
    printf("\n Ig_left %f \n", Ig_left);
    robot_input->left=P*Pg_left + I*Ig_left + D*Dg_left;

    late_param.left = robot_input->left;
    late_param.right = robot_input->right;
};