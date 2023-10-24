#include <mpc.h>

mpc::mpc(   const std::array<double, 16>& stateMatrixContinuous, 
            const std::array<double, 4>& inputMatrixContinuous, 
            const std::array<double, 4>& outputMatrixContinuous, 
            const std::array<double, 4>& initialState, 
            double sampleTime, 
            size_t predHorizon, 
            size_t ctrlHorizon):
            sys_{stateMatrixContinuous, inputMatrixContinuous, outputMatrixContinuous, initialState, sampleTime}, predHorizon_{predHorizon}, ctrlHorizon_{ctrlHorizon}{

    computeMPCMatrices();
}

void mpc::composeMPCStateMatrix(){
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

    auto ASys = sys_.A();
    auto bSys = sys_.b();
    auto cSys = sys_.c();
    const Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > A(ASys.data());
    const Eigen::Map<Eigen::Matrix<double, 4, 1> > b(bSys.data());
    const Eigen::Map<Eigen::Matrix<double, 1, 4> > c(cSys.data());

    stateMatrix_.resize(predHorizon_, 4);
    stateMatrix_.setZero();

    stateMatrix_.block<1, 4>(0, 0) = c * A;

    for (size_t idx_pred = 1; idx_pred < predHorizon_; ++idx_pred){
        stateMatrix_.block<1, 4>(idx_pred, 0) = stateMatrix_.block<1, 4>(idx_pred - 1, 0) * A;
    }    
}


void mpc::composeMPCInputMatrix(){
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

    auto ASys = sys_.A();
    auto bSys = sys_.b();
    auto cSys = sys_.c();
    const Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > A(ASys.data());
    const Eigen::Map<Eigen::Matrix<double, 4, 1> > b(bSys.data());
    const Eigen::Map<Eigen::Matrix<double, 1, 4> > c(cSys.data());

    inputMatrix_.resize(predHorizon_ + 1, ctrlHorizon_);
    inputMatrix_.setZero();

    // Within ctrl horizon
    // First row
    inputMatrix_.block(0, 0, 1, 1) = c * b;
    
    for (size_t idxCtrl = 1; idxCtrl < ctrlHorizon_; ++idxCtrl){
        
        // Shift right
        inputMatrix_.block(idxCtrl, 1, 1, ctrlHorizon_ - 1) = inputMatrix_.block(idxCtrl - 1, 0, 1, ctrlHorizon_ - 1);
        inputMatrix_.block(idxCtrl, 0, 1, 1).setZero(); // TODO: it can be removed for speed

        // Fill first element
        inputMatrix_.block(idxCtrl, 0, 1, 1) = c * matPow(A, idxCtrl) * b;
       
    }

    // After ctrl horizon
    Eigen::Matrix<double, 4, 4> Ahat = Eigen::Matrix<double, 4, 4>::Identity();

    for (size_t idxPred = ctrlHorizon_; idxPred <= predHorizon_; ++idxPred){
        // Shift right
        inputMatrix_.block(idxPred, 1, 1, ctrlHorizon_ - 1) = inputMatrix_.block(idxPred - 1, 0, 1, ctrlHorizon_ - 1);
        inputMatrix_.block(idxPred, 0, 1, 1).setZero(); // TODO: it can be removed for speed

        // Fill first element
        inputMatrix_.block(idxPred, 0, 1, 1) = c * matPow(A, idxPred) * b;

        // A_(i,v) = A^(i-v) + A^(i-v-1)+ ... + A + I
        auto powAhat = matPow(A, idxPred - ctrlHorizon_);

        for (size_t idxPow = 0; idxPow < idxPred; ++idxPow){
            Ahat = Ahat + powAhat;
        }

        inputMatrix_.block(idxPred, ctrlHorizon_ - 1, 1, 1) = c * Ahat * b;
    }
}

void mpc::composeMPCWeights() {

    /*
    ////////////////////// 
    // WEIGHTS
    ////////////////////// 
    */

    /*
    // W1 = [I  0   0   0   ...
    //      -I  I   0   0   ...
    //      .   .   .
    //      0  -I   I   0]
    */

    weightMatrix1_.resize(ctrlHorizon_, ctrlHorizon_);
    weightMatrix1_.setZero();

    // First row
    weightMatrix1_(0, 0) = 1.0;

    // Second row
    weightMatrix1_(1, 0) = -1.0;
    weightMatrix1_(1, 1) = 1.0;

    for(size_t idxCtrl = 2; idxCtrl < ctrlHorizon_; ++idxCtrl){
        // Shift right
        weightMatrix1_.block(idxCtrl, 1, 1, ctrlHorizon_ - 1) = weightMatrix1_.block(idxCtrl - 1, 0, 1, ctrlHorizon_ - 1);
        weightMatrix1_.block(idxCtrl, 0, 1, 1).setZero();
    }

    /*
    // W2 = [Q0  0   0   0   ...
    //      0  Q1   0   0   ...
    //      .   .   .
    //      0  0   0   ... Qv-1]
    */
    
    weightMatrix2_.resize(ctrlHorizon_, ctrlHorizon_);
    weightMatrix2_.setIdentity();

    /*
    // W3 = W1^T * W2 * W1
    */

    weightMatrix3_ = weightMatrix1_.transpose() * weightMatrix2_ * weightMatrix1_;

    /*
    // W4 = [P0  0   0   0   ...
    //      0  P1   0   0   ...
    //      .   .   .
    //      0  0   0   ... Pf]
    */

    weightMatrix4_.resize(predHorizon_, predHorizon_);
    weightMatrix4_.setIdentity();

}

void mpc::computeMPCMatrices(){

    // z_hat = O x[k] + M u_hat

    composeMPCStateMatrix();

    composeMPCInputMatrix();

    composeMPCWeights();

}

void mpc::computeControl(){

    // // u = (M^T * W4 * M + w3)^-1 * M^T * W4 * s
    // // s = zd - Oxk

    // auto s = zd - O * x;
    // auto control_input = (M.tranpose() * W4 * M + W3).inverse() * M.tranpose() * W5 * s;

    // return control_input;
}