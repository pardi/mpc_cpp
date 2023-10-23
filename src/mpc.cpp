#include <mpc.h>

mpc::mpc(   const std::array<double, 16>& A, 
            const std::array<double, 4>& b, 
            const std::array<double, 4>& c, 
            const std::array<double, 4>& x0, 
            double dt, 
            size_t pred_horizon, 
            size_t ctrl_horizon): 
            sys_{A, b, c, x0, dt}, pred_horizon_{pred_horizon}, ctrl_horizon_{ctrl_horizon}{

    computeMPCMatrices();
}

void mpc::computeMPCMatrices(){

    // z_hat = O x[k] + M u_hat
    std::vector<std::vector<double>> O;
    auto ASys = sys_.A();
    auto bSys = sys_.b();
    auto cSys = sys_.c();
    const Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > A(ASys.data());
    const Eigen::Map<Eigen::Matrix<double, 4, 1> > b(bSys.data());
    const Eigen::Map<Eigen::Matrix<double, 1, 4> > c(cSys.data());

    /*
    // STATE MATRIX - O
    // 
    // size[pred_horizon, 4]
    // O = [CA
    //      CA^2
    //      CA^3
    //      ...
    //      CA^f]
    // f -> size of the prediction horizon
    */

    std::vector<double> ORow = {0.0, 0.0, 0.0, 0.0};
    Eigen::Map<Eigen::Matrix<double, 1, 4>> oRow(ORow.data());
    oRow = c * A;

    O.push_back(ORow);

    for (size_t idx_pred = 1; idx_pred < pred_horizon_; ++idx_pred){
        oRow = oRow * A;
        O.push_back(ORow);
    }


    /*
    // CONTROL MATRIX - M
    // size[pred_horizon, ctrl_horizon]
    // 
    // M = [CB  0   0   0   ...
    //      CAB CB  0   0   ...
    //      CA^2B   CAB CB  0   ...
    //      ...
    //      CA^(v-1)B   CA^(v-2)B ... CB
    //      CA^vB   CA^(v-1)B   ...     CA_(1,v)B]
    //      ...
    //      CA^fB   CA^(f-1)B   ...     CA_(f,v)B]

    */

    std::vector<std::vector<double>> M; 

    std::vector<double> MRow(ctrl_horizon_);
    std::fill(MRow.begin(), MRow.end(), 0);

    // Define shift function for vectors

    auto shift_right = [](std::vector<double>& vec, size_t distance) {
        if(distance >= vec.size()) {
            std::fill(vec.begin(), vec.end(), 0);
            return;
        }
        
        std::move_backward(vec.begin(), vec.end() - distance, vec.end());
        std::fill(vec.begin(), vec.begin() + distance, 0);
    };  


    // Within ctrl horizon

    for (size_t idx_ctrl = 0; idx_ctrl < ctrl_horizon_; ++idx_ctrl){
        shift_right(MRow, 1);    
   
        MRow[0] = (c * matPow(A, idx_ctrl) * b)[0];

        M.push_back(MRow);

    }

    // After ctrl horizon

    Eigen::Matrix<double, 4, 4> Ahat = Eigen::Matrix<double, 4, 4>::Identity();

    for (size_t idx_pred = ctrl_horizon_; idx_pred <= pred_horizon_; ++idx_pred){
        shift_right(MRow, 1);  

        MRow[0] = (c * matPow(A, idx_pred) * b)[0];

        // A_(i,v) = A^(i-v) + A^(i-v-1)+ ... + A + I

        auto powAhat = matPow(A, idx_pred - ctrl_horizon_);

        for (size_t idx_pow = 0; idx_pow < idx_pred; ++idx_pow){
            Ahat = Ahat + powAhat;
        }
         
        *(MRow.end() - 1) = (c * Ahat * b)[0];

        M.push_back(MRow);
    }

    /*
    // WEIGHTS
    // 
    // W1 = [I  0   0   0   ...
    //      -I  I   0   0   ...
    //      .   .   .
    //      0  -I   I   0]
    // W2 = [Q0  0   0   0   ...
    //      0  Q1   0   0   ...
    //      .   .   .
    //      0  0   0   ... Qv-1]
    // W3 = W1^T * W2 * W1
    */

    std::vector<std::vector<double>> W1(ctrl_horizon_);
    std::vector<double> W1row(ctrl_horizon_);
    std::fill(W1row.begin(), W1row.end(), 0);
    W1row[0] = 1.0;
    W1.push_back(W1row);

    W1row[0] = -1.0;
    W1row[1] = 1.0;
    W1.push_back(W1row);






}

void mpc::computeControl(){
    // u = (M^T * W4 * M + w3)^-1 * M^T * W4 * s



}