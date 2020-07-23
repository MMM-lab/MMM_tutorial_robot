#ifndef TORQUE_CONTROL
#define TORQUE_CONTROL

typedef struct _Robot_State {
    float yow_rate;
    float velocity;
} Robot_State;

typedef struct _Robot_Input {
    float right;
    float left;
} Robot_Input;

typedef struct _Robot_Wheel_Rate {
    float right;
    float left;
} Robot_Wheel_Rate;

typedef struct _Robot_Param {
    float mass;
    float tread;
    float inertia;
    float diameter;
    float g_positon;
} Robot_Param;

class Torque_Control
{
private:
    Robot_Input late_param;
    Robot_Input next_param;
    Robot_Param param;
    float P;
    float I;
    float D;
    float dt;
    float Ig_left;
    float Ig_right;

public:
    Torque_Control();
    ~Torque_Control();
    void set_param(float tread, float diameter, float P, float I, float D);
    void get_torques(Robot_Input *robot_input, Robot_Wheel_Rate *robot_wheel_rate, Robot_State *target_robot_state);
};

#endif //TORQUE_CONTROL 