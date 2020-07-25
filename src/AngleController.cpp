#include "../include/AngleController.hpp"
#include "cstdio"

void AngleController::getTorques(Robot_Input *robot_input, float targetAngle, float angle, float accel)
{
    float torqueDiff = anglePIDController.update(angle, targetAngle, 0.1);
    printf("torqueDiff: %f \n", torqueDiff);
    robot_input -> left = accel + torqueDiff;
    robot_input -> right = accel - torqueDiff;
}

AngleController::AngleController()
{
}

AngleController::~AngleController()
{
}