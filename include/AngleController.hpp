#include "PIDController.hpp"

typedef struct _Robot_p3d
{
    float posX;
    float posY;
    float posZ;

    float angleX;
    float angleY;
    float angleZ;
} Robot_p3d;

typedef struct _Robot_Wheel_Rate
{
    float right, left;
} Robot_Wheel_Rate;

typedef struct _Robot_Input
{
    float right, left;
} Robot_Input;

class AngleController
{
private:

public:
    PIDController anglePIDController;
    AngleController(/* args */);
    ~AngleController();
    void getTorques(Robot_Input *robot_input, float targetAngle, float angle, float accel);
};

