#ifndef LINEARSYS_HPP
#define LINEARSYS_HPP

#include <eigen3/Eigen/Dense>
#include <array>

class LinearSys{

    public: 
        LinearSys(const std::array<double, 8>& A, const std::array<double, 8>& b, const std::array<double, 8>& c, const std::array<double, 8>& x0, double dt): A_{A}, b_{b}, c_{c}, x_{x0}, x0_{x0}, dt_{dt}{}
        ~LinearSys() = default;
        std::array<std::array<double, 8>, 100> step();
        std::array<double, 8> step(std::array<double, 8> u);
        void reset();
    private:
        double dt_;
        std::array<double, 8> A_;
        std::array<double, 8> b_;
        std::array<double, 8> c_;
        std::array<double, 8> x_;
        std::array<double, 8> x0_;
        
};

// // Include template implementations
// #include <LinearSys.tpp>

#endif