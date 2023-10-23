#include <LinearSys.h>


LinearSys::LinearSys(const std::array<double, 16>& A, const std::array<double, 4>& b, const std::array<double, 4>& c, const std::array<double, 4>& x0, double dt): 
Ac_{A}, bc_{b}, cc_{c}, x0_{x0}, x_{x0}, dt_{dt}{
    discretiseSys();
}

void LinearSys::discretiseSys(){
    // Backward Euler
    const Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > Ac(Ac_.data());
    const Eigen::Map<Eigen::Matrix<double, 4, 1> > bc(bc_.data(), 4, 1);
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > A(A_.data());
    Eigen::Map<Eigen::Matrix<double, 4, 1>> b(b_.data());
    

    A = (Eigen::Matrix<double, 4, 4>::Identity() - dt_ * Ac).inverse();
    b = dt_ * A * bc;

    c_ = cc_;
}

double LinearSys::step(const double u){

    const Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > A(A_.data());
    const Eigen::Map<Eigen::Matrix<double, 4, 1> > b(b_.data());
    const Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor> > c(c_.data());
    Eigen::Map<Eigen::Matrix<double, 4, 1> > x(x_.data());

    // System
    x = A * x + b * u;
    auto y = (c * x)[0];

    return y;
}

void LinearSys::reset(){
    x_ = x0_;
}