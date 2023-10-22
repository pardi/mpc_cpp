#include <LinearSys.h>

double LinearSys::step(const double u){

    const Eigen::Map<Eigen::MatrixXd> A(A_.data(), 4, 4);
    const Eigen::Map<Eigen::VectorXd> b(b_.data(), 4, 1);
    const Eigen::Map<Eigen::VectorXd> c(c_.data(), 1, 4);
    Eigen::Map<Eigen::VectorXd> x(x_.data(), 1, 4);

    double y = 0.0;

    x = A * x + b * u;
    y = c.dot(x);

    return y;
}

void LinearSys::reset(){
    x_ = x0_;
}