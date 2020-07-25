#include "../include/PIDController.hpp"
#include "cstdio"

void PIDController::setGain(float setP, float setI, float setD)
{
    P = setP;
    I = setI;
    D = setD;
}

float PIDController::update(float input, float target, float dt)
{
    float diff = target - input;
    integral = diff * dt;
    previousValue = input;

    printf("input: %f, target: %f, dt: %f \n", input, target, dt);
    printf("P: %f, I: %f, D: %f \n", P, I, D);

    return P * diff + I * integral + D * (input - previousValue)/dt;
}

PIDController::PIDController()
{
}

PIDController::~PIDController()
{
}
