#ifndef EXTENDED_KALMAN_FILTER
#define EXTENDED_KALMAN_FILTER

#include <eigen3/Eigen/Dense>
using namespace Eigen;

class Extended_Kalman_Filter 
{
    private:
        // Time
        double dt;
    private:
        // system matrix
        Matrix<double, 3, 1> z;
        Matrix<double, 2, 1> u;
        Matrix<double, 3, 3> Q;
        Matrix<double, 3, 3> R;
        Matrix<double, 3, 3> A;
        Matrix<double, 3, 2> B;
        Matrix<double, 3, 3> C;
        Matrix<double, 3, 3> jF;
        Matrix<double, 3, 3> jH;
        Matrix<double, 3, 3> F;
        Matrix<double, 3, 3> H;
        Matrix<double, 3, 3> K;
        //Matrix<double, 3, 1> xEst;
        Matrix<double, 3, 1> xPred;
        //Matrix<double, 3, 1> xTrue;
        Matrix<double, 3, 3> PEst;
        Matrix<double, 3, 3> PPred;
        Matrix<double, 2, 2> Qsigma;
        Matrix<double, 3, 3> Rsigma;
    private:
        double toRadian(double degree);
        MatrixXd model(MatrixXd x, MatrixXd u);
        MatrixXd jacobF(MatrixXd x, MatrixXd u);
        MatrixXd measurement(MatrixXd x);
        MatrixXd jacobH(MatrixXd x);
        MatrixXd jacobH();
        MatrixXd noise_distribution(MatrixXd u);
        MatrixXd Observation(MatrixXd x);
        MatrixXd measuement(MatrixXd x);
    public:
        Matrix<double, 3, 1> xEst;
        Matrix<double, 3, 1> xTrue;
    public:
        Extended_Kalman_Filter();
        ~Extended_Kalman_Filter();
        void simulation();
        void setPosition(float x, float y, float anglar);
        void setInput(float velocity, float angular_velocity);
};

#endif // EXTENDED_KALMAN_FILTER
