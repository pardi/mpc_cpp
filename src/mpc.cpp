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
    */

    std::vector<std::vector<double>> M; 

    std::vector<double> MRow(ctrl_horizon_);
    std::fill(MRow.begin(), MRow.end(), 0);

    // Define shift function for vectors

    auto shift_right = [](std::vector<double>::iterator start, std::vector<double>::iterator end, size_t distance){
        
        auto ptr = end - distance;

        for (auto vecPtr = end; vecPtr > start; --vecPtr){
            if (ptr < start){
                std::fill(start, vecPtr + 1, 0);
                return;
            }
            *vecPtr = *ptr;
            ptr--;
        }
    };


    // Within ctrl horizon

    for (size_t idx_ctrl = 0; idx_ctrl < ctrl_horizon_; ++idx_ctrl){
        shift_right(MRow.begin(), MRow.end(), 1);    
        Eigen::Matrix<double, 4, 4> powA = Eigen::Matrix<double, 4, 4>::Identity();
        
        for (size_t idx_pow = 0; idx_pow < idx_ctrl; ++idx_pow){
            powA = powA * A;
        }

        MRow[0] = (c * powA * b)[0];

        M.push_back(MRow);

    }

    // After ctrl horizon

    Eigen::Matrix<double, 4, 4> Ahat = Eigen::Matrix<double, 4, 4>::Identity();

    for (size_t idx_pred = ctrl_horizon_; idx_pred < pred_horizon_; ++idx_pred){
        shift_right(MRow.begin(), MRow.end(), 1);  

        Eigen::Matrix<double, 4, 4> powA = Eigen::Matrix<double, 4, 4>::Identity();
        
        for (size_t idx_pow = 0; idx_pow < idx_pred; ++idx_pow){
            powA = powA * A;
        }

        MRow[0] = (c * powA * b)[0];


        Eigen::Matrix<double, 4, 4> powAhat = Eigen::Matrix<double, 4, 4>::Identity();
        
        for (size_t idx_pow = 0; idx_pow < idx_pred - ctrl_horizon_; ++idx_pow){
            powAhat = powAhat * A;
        }

        for (size_t idx_pow = 0; idx_pow < idx_pred; ++idx_pow){
            Ahat = Ahat + powAhat;
        }

        *MRow.end() = (c * Ahat * b)[0];

        M.push_back(MRow);
        


    }

    


    // M = [CB  0   0   0   ...
    //      CAB CB  0   0   ...
    //      CA^2B   CAB CB  0   ...
    //      ...
    //      CA^(v-1)B   CA^(v-2)B ... CB
    //      CA^vB   CA^(v-1)B   ...     CA_(1,v)B]
    //      ...
    //      CA^fB   CA^(f-1)B   ...     CA_(f,v)B]

}


void mpc::computeControl(){
    // u = (M^T * W4 * M + w3)^-1 * M^T * W4 * s


    // A_(i,v) = A^(i-v) + A^(i-v-1)+ ... + A + I

    // W3 = W1^T * W2 * W1
    // W1 = [I  0   0   0   ...
    //      -I  I   0   0   ...
    //      .   .   .
    //      0  -I   I   0]
    // W2 = [Q0  0   0   0   ...
    //      0  Q1   0   0   ...
    //      .   .   .
    //      0  0   0   ... Qv-1]
}