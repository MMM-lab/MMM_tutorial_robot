typedef struct _Robot_State
{
    float yow_rate, velocity;
} Robot_State;

typedef struct _Robot_Input
{
    float right, left;
} Robot_Input;

typedef struct _Robot_Wheel_Rate
{
    float right, left;
} Robot_Wheel_Rate;

typedef struct _Robot_Param
{
    float mass, inertia, tread, diameter, g_positon;
} Robot_Param;

class Torque_Control
{
    private:
        Robot_Input late_param;
        Robot_Input next_param;
        Robot_Param param;
        float PI=3.1415;
        float P=1,I=0,D=0;
        float dt=0.1;
        float Ig_left, Ig_right;

    public:
        void set_param(float tread, float diameter, float P, float I, float D);
        void get_torques(Robot_Input *robot_input, Robot_Wheel_Rate *robot_wheel_rate, Robot_State *target_robot_state);

};