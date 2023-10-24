#ifndef MPC_HPP
#define MPC_HPP

#include <eigen3/Eigen/Dense>
#include <array>
#include <iostream>
#include <algorithm>

#include <LinearSys.h>


class mpc{

    public: 
    
        mpc(    const std::array<double, 16>& stateMatrixContinuous, 
                const std::array<double, 4>& inputMatrixContinuous, 
                const std::array<double, 4>& outputMatrixContinuous, 
                const std::array<double, 4>& initialState, 
                double sampleTime = 1e-3, 
                size_t predHorizon = 5, 
                size_t ctrlHorizon = 1);
            
        ~mpc() = default;
        
    private:
        void computeControl();
        void computeMPCMatrices();
        void composeMPCStateMatrix();
        void composeMPCInputMatrix();
        void composeMPCWeights();


        template<typename Derived>
        Eigen::MatrixXd matPow(const Eigen::MatrixBase<Derived>& matrix, size_t power);

        LinearSys sys_;
        size_t predHorizon_;
        size_t ctrlHorizon_;

        Eigen::MatrixXd stateMatrix_;
        Eigen::MatrixXd inputMatrix_;
        Eigen::MatrixXd weightMatrix1_;
        Eigen::MatrixXd weightMatrix2_;
        Eigen::MatrixXd weightMatrix3_;
        Eigen::MatrixXd weightMatrix4_;

        
};

// Include template implementations
#include <impl/mpc.tpp>

#endif