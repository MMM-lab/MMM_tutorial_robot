#include <iostream>
#include <eigen3/Eigen/Dense>
#include "cstdio"
#include <cmath>
#include <random>
#include "../include/extended_kalman_filter.hpp"

#define PI 3.14159265359

using namespace std;
using namespace Eigen;

Extended_Kalman_Filter::Extended_Kalman_Filter() 
{
    // Time 
    dt = 0.1; 
    // Covariance Matrix for motion
    Q << pow(0.1, 2.0), 0, 0,
         0, pow(0.1, 2.0), 0,
         0, 0, pow(toRadian(1), 2.0);
    // Covariance Matrix for observation 
    R << pow(0.3, 2.0), 0, 0,
         0, pow(0.3, 2.0), 0,
         0, 0, pow(toRadian(3), 2.0);
    // Simulation parameter
    Qsigma << pow(0.3, 2.0), 0,
              0, pow(toRadian(3), 2.0);
    Rsigma << pow(0.3, 2.0), 0, 0,
              0, pow(0.3, 2.0), 0,
              0, 0, pow(toRadian(3), 2.0);
    PEst << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;   
}

Extended_Kalman_Filter::~Extended_Kalman_Filter()
{
    cout << "Finish" << endl;
}

double Extended_Kalman_Filter::toRadian(double degree)
{
    // degree to radian
    double radian = degree / 180.0 * PI;
    return radian;
}

MatrixXd Extended_Kalman_Filter::noise_distribution(MatrixXd u) 
{
    // gaussian distribution
    random_device rd{};
    mt19937 gen{rd()};
    normal_distribution<> gaussian_d{0, 1};

    u(0, 0) = u(0, 0) + gaussian_d(gen) * Qsigma(0, 0);
    u(1, 0) = u(1, 0) + gaussian_d(gen) * Qsigma(1, 1);

    return u;
}

MatrixXd Extended_Kalman_Filter::Observation(MatrixXd x) 
{
    // gaussian distribution
    random_device rd{};
    mt19937 gen{rd()};
    normal_distribution<> gaussian_d{0, 1};

    z(0, 0) = x(0, 0) + gaussian_d(gen) * Rsigma(0, 0);
    z(1, 0) = x(1, 0) + gaussian_d(gen) * Rsigma(1, 1);
    z(2, 0) = x(2, 0) + gaussian_d(gen) * Rsigma(2, 2);

    return z;
}

MatrixXd Extended_Kalman_Filter::model(MatrixXd x, MatrixXd u)
{
    
    A << 1.0, 0, 0,
         0, 1.0, 0,
         0, 0, 1.0;
    
    B << dt * cos(x(2, 0)), 0,
         dt * sin(x(2, 0)), 0,
         0, dt;
    
    return x = A * x + B * u;
}

MatrixXd Extended_Kalman_Filter::jacobF(MatrixXd x, MatrixXd u)
{
    jF << 1, 0, -dt * u(0, 0) * sin(x(2, 0)),
          0, 1, dt * u(0, 0) * cos(x(2, 0)),
          0, 0, 1;
    
    return jF;
}

MatrixXd Extended_Kalman_Filter::measuement(MatrixXd x) 
{
    C << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    return H * x;
}

MatrixXd Extended_Kalman_Filter::jacobH()
{
    jF << 1, 0, 0,
          0, 1, 0,
          0, 0, 1;
    
    return jF;
}

void Extended_Kalman_Filter::simulation()
{
        xTrue = model(xTrue, u);
        // noise distribution
        u     = noise_distribution(u);
        z     = Observation(xTrue);
        // ------Extended Kalman Filter --------
        // Prediction step
        xPred = model(xEst, u);   
        F     = jacobF(xPred, u);  
        PPred = F * PEst * F.transpose() + Q;
        // Filtering step
        H     = jacobH(); 
        K     = PPred * H.transpose() * (H * PPred * H.transpose() + R).inverse();
        xEst  = xPred + K * (z - measuement(xPred)); 
        PEst  = (Matrix3d::Identity() - K * H) * PPred;

        cout << "EstimatePosition: X = " << xEst(0, 0) << endl;
        cout << "EstimatePosition: Y = " << xEst(1, 0) << endl;
        cout << "EstimatePosition: Theta = " << xEst(2, 0) << endl;
}

void Extended_Kalman_Filter::setPosition(float x, float y, float anglar)
{
    xTrue(0, 0) = x;
    xTrue(1, 0) = y;
    xTrue(2, 0) = anglar;
    cout << "TruePosition: X = " << xTrue(0, 0) << endl;
    cout << "TruePosition: Y = " << xTrue(1, 0) << endl;
    cout << "TruePosition: Theta = " << xTrue(2, 0) << endl;
}

void Extended_Kalman_Filter::setInput(float velocity, float angular_velocity)
{
    u(0, 0) = velocity;
    u(1, 0) = angular_velocity;
}