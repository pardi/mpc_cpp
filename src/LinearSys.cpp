#include <LinearSys.h>

double LinearSys::step(const double u){

    const Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > A(A_.data());
    const Eigen::Map<Eigen::VectorXd> b(b_.data(), 4, 1);
    const Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor> > c(c_.data());
    Eigen::Map<Eigen::VectorXd> x(x_.data(), 4, 1);

    double y = 0.0;

    x = A * x + b * u;
    y = (c * x)[0];

    return y;
}

void LinearSys::reset(){
    x_ = x0_;
}