#ifndef LINEARSYS_HPP
#define LINEARSYS_HPP

#include <eigen3/Eigen/Dense>
#include <array>
#include <iostream>

class LinearSys{

    public: 
        LinearSys(const std::array<double, 16>& A, const std::array<double, 4>& b, const std::array<double, 4>& c, const std::array<double, 4>& x0, double dt = 1e-3);
        ~LinearSys() {std::cout << "mpc linearsys" << std::endl;};
        double step(double u);
        void reset();

        inline Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > getStateMatrix() const {
            return Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > (A_.data());;
        }

        inline Eigen::Map<const Eigen::Matrix<double, 4, 1> > getInputMatrix() const {
            return Eigen::Map<const Eigen::Matrix<double, 4, 1> > (b_.data());;
        }

        inline Eigen::Map<const Eigen::Matrix<double, 1, 4> > getOutputMatrix() const {
            return Eigen::Map<const Eigen::Matrix<double, 1, 4> > (c_.data());;
        }

        inline Eigen::Map<const Eigen::Matrix<double, 4, 1> > getState() const {
            return Eigen::Map<const Eigen::Matrix<double, 4, 1> > (x_.data());;
        }
    
    private:

        void discretiseSys();

        // Continuos Matrices
        std::array<double, 16> Ac_;
        std::array<double, 4> bc_;
        std::array<double, 4> cc_;
        // Discretised Matrices
        std::array<double, 16> A_;
        std::array<double, 4> b_;
        std::array<double, 4> c_;
        // State space
        std::array<double, 4> x0_;
        std::array<double, 4> x_;
        
        double dt_;
        
};

// // Include template implementations
// #include <LinearSys.tpp>

#endif