class PIDController
{
private:
    float P;
    float I;
    float D;

    float integral;
    float previousValue;

public:
    PIDController();
    ~PIDController();

    void setGain(float P, float I, float D);
    float update(float input, float target, float dt);
};
