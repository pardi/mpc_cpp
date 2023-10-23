#include <mpc.h>

template<typename Derived>
Eigen::MatrixXd mpc::matPow(const Eigen::MatrixBase<Derived>& matrix, size_t power){
    
    Eigen::Matrix<double, 4, 4> powMatrix = Eigen::MatrixBase<Derived>::Identity();

    for (size_t idxPow = 0; idxPow < power; ++idxPow){
        powMatrix = powMatrix * matrix;
    }

    return powMatrix;
}
    