#include <eigen3/Eigen/Dense>
#include <LinearSys.h>
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
    LinearSys ls(A, b, c, x0, dt);

    std::cout << ls.step(1) << std::endl;
    

    // std::array<double, 3> avec = {1, 2, 3};
    // const Eigen::Map<Eigen::VectorXd> a(avec.data(), 3, 1);
    // Eigen::VectorXd at = a.transpose();

    // // Eigen::VectorXd a(3);
    // // a <<  1, 2, 3;
    // // Eigen::VectorXd a2 = a.transpose();
    
    // std::cout << at << std::endl;
    // std::cout << a.dot(at) << std::endl;
    return 0;
}
