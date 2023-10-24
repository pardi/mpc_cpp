#include <eigen3/Eigen/Dense>
#include <LinearSys.h>
#include <mpc.h>
#include <iostream>

int main (){

    const double m1 = 2.0;
    const double m2 = 2.0;
    const double k1 = 100.0;
    const double k2 = 200.0;
    const double d1 = 1.0;
    const double d2 = 5.0;

    std::array<double, 16> A = { 0.0, 1.0, 0.0, 0.0, 
                                - (k1 + k2) / m1, - (d1 + d2) / m1, k2 / m1, d2 / m1,
                                0.0, 0.0, 0.0, 1.0, 
                                k2 / m2,  d2 / m2, - k2 / m2, - d2 / m2};
    std::array<double, 4> b = {0.0, 0.0, 0.0, 1.0 / m2};
    std::array<double, 4> c = {1.0, 0.0, 0.0, 0.0};
    std::array<double, 4> x0 = {0.0, 0.0, 0.0, 0.0};

    double dt = 0.05;
    size_t predHorizon = 6;
    size_t ctrlHorizon = 5;

    mpc mpc_ctrl(A, b, c, x0, dt, predHorizon, ctrlHorizon);

    Eigen::VectorXd desiredTrajectory(1000);

    for (auto idx = 0; idx < 1000; idx++){
        desiredTrajectory(idx) = 3 * sin( 2.0 * M_PI * 0.03 * idx / (predHorizon + 1));
    }

    for (auto idx = 0; idx < 800; ++idx){

        auto inputVector = mpc_ctrl.computeControl(desiredTrajectory.segment(idx, predHorizon + 1));
        auto ouputValue = mpc_ctrl.sysStep(inputVector(0));
        std::cout << ouputValue << std::endl;
    }

    return 0;
}
