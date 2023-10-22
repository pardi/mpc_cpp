#include <eigen3/Eigen/Dense>
#include <LinearSys.h>

int main (){

    const double m1 = 20.0;
    const double m2 = 20.0;
    const double k1 = 1000.0;
    const double k2 = 2000.0;
    const double d1 = 1.0;
    const double d2 = 5.0;

    std::array<double, 16> A = { 0.0, 1.0, 0.0, 0.0, 
                                - (k1 + k2) / m1, - (d1 + d2) / m1, k2 / m1, d1 / m1,
                                0.0, 0.0, 0.0, 1.0, 
                                k2 / m2,  d2 / m2, - k2 / m2, - d2 / m2};
    std::array<double, 4> b = {0.0, 0.0, 0.0, 1.0 / m2};
    std::array<double, 4> c = {1.0, 0.0, 0.0, 0.0};
    std::array<double, 4> x0 = {0.0, 0.0, 0.0, 0.0};

    double dt = 1e-3;
    LinearSys ls(A, b, c, x0, dt);

    return 0;
}
