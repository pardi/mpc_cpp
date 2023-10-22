#include <LinearSys.h>

double LinearSys::step(const double u){

    const Eigen::Map<Eigen::MatrixXd> A(A_.data(), 4, 4);
    const Eigen::Map<Eigen::MatrixXd> b(b_.data(), 4, 1);
    const Eigen::Map<Eigen::MatrixXd> c(c_.data(), 1, 4);
    Eigen::Map<Eigen::MatrixXd> x(x_.data(), 1, 4);

    double y = 0.0;

    x = A_ * x + b * u;
    y = c_ * x;

    return y;
}

void LinearSys::reset(){
    x_ = x0_;
}