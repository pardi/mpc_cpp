#ifndef LINEARSYS_HPP
#define LINEARSYS_HPP

#include <eigen3/Eigen/Dense>

class LinearSys{

    public: 
        LinearSys(const std::array<double, 8> A, const std::array<double, 8> b, const std::array<double, 8> c, const std::array<double, 8> x0, double dt): A_{A}, b_{b}, c_{c}, x_{x0}, dt_{dt}{}
        ~LinearSys() = default;
        template<std::size_t N>
        std::array<double, 8> solve(const std::array<double, N> u);
    private:
        double dt_;
        std::array<double, 8> A_;
        std::array<double, 8> b_;
        std::array<double, 8> c_;
        std::array<double, 8> x_;
        
};

// Include template implementations
#include <LinearSys.tpp>

#endif