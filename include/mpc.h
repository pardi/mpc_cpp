#ifndef MPC_HPP
#define MPC_HPP

#include <eigen3/Eigen/Dense>
#include <array>
#include <iostream>
#include <algorithm>

#include <LinearSys.h>


class mpc{

    public: 
    
        mpc( const std::array<double, 16>& A, 
            const std::array<double, 4>& b, 
            const std::array<double, 4>& c, 
            const std::array<double, 4>& x0, 
            double dt = 1e-3, 
            size_t pred_horizon = 5, 
            size_t ctrl_horizon = 1);
            
        ~mpc() {std::cout << "mpc dest" << std::endl;};
        
    private:
        void computeControl();
        void computeMPCMatrices();
        template<typename Derived>
        Eigen::MatrixXd matPow(const Eigen::MatrixBase<Derived>& matrix, size_t power);

        LinearSys sys_;
        size_t pred_horizon_;
        size_t ctrl_horizon_;


        
};

// Include template implementations
#include <impl/mpc.tpp>

#endif