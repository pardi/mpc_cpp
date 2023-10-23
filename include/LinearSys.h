#ifndef LINEARSYS_HPP
#define LINEARSYS_HPP

#include <eigen3/Eigen/Dense>
#include <array>
#include <iostream>

class LinearSys{

    public: 
        LinearSys(const std::array<double, 16>& A, const std::array<double, 4>& b, const std::array<double, 4>& c, const std::array<double, 4>& x0): A_{A}, b_{b}, c_{c}, x0_{x0}, x_{x0}{}
        ~LinearSys() = default;
        double step(double u);
        void reset();
    private:
        std::array<double, 16> A_;
        std::array<double, 4> b_;
        std::array<double, 4> c_;
        std::array<double, 4> x0_;
        std::array<double, 4> x_;

        
};

// // Include template implementations
// #include <LinearSys.tpp>

#endif