#include <LinearSys.h>

std::array<double, 8> LinearSys::step(std::array<double, 8> u){

    const Eigen::Map<Eigen::MatrixXd> u(u.data(), 8, 1);
    const Eigen::Map<Eigen::MatrixXd> A(A_.data(), 4, 4);
    const Eigen::Map<Eigen::MatrixXd> b(b_.data(), 8, 1);
    const Eigen::Map<Eigen::MatrixXd> c(c_.data(), 1, 8);
    Eigen::Map<Eigen::MatrixXd> x(x_.data(), 1, 8);

    std::array<double, 8> yVec;
    
    Eigen::Map<Eigen::MatrixXd> y(yVec.data(), 8, 1);
    
    x = A_ * x + b * u;
    y = c_ * x;

    return yVec;
}

void LinearSys::reset(){
    x_ = x0_;
}