#include <mpc.h>

mpc::mpc(   const std::array<double, 16>& stateMatrixContinuous, 
            const std::array<double, 4>& inputMatrixContinuous, 
            const std::array<double, 4>& outputMatrixContinuous, 
            const std::array<double, 4>& initialState, 
            double sampleTime, 
            size_t predHorizon, 
            size_t ctrlHorizon):
            sys_{stateMatrixContinuous, inputMatrixContinuous, outputMatrixContinuous, initialState, sampleTime}, predHorizon_{predHorizon}, ctrlHorizon_{ctrlHorizon}{

    initMPCMatrices();

}

mpc::mpc(   const LinearSys& sys,
            size_t predHorizon, 
            size_t ctrlHorizon):
            sys_{sys}, predHorizon_{predHorizon}, ctrlHorizon_{ctrlHorizon}{

    initMPCMatrices();

}


void mpc::initMPCMatrices(){

    initMPCStateMatrix();

    initMPCInputMatrix();

    initMPCWeights();

}

void mpc::initMPCStateMatrix(){
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

    auto sysStateMatrix = sys_.getStateMatrix();
    auto sysOutputMatrix = sys_.getOutputMatrix();

    stateMatrix_.resize(predHorizon_ + 1, 4);
    stateMatrix_.setZero();

    stateMatrix_.block<1, 4>(0, 0) = sysOutputMatrix * sysStateMatrix;

    for (size_t idx_pred = 1; idx_pred <= predHorizon_; ++idx_pred){
        stateMatrix_.block<1, 4>(idx_pred, 0) = stateMatrix_.block<1, 4>(idx_pred - 1, 0) * sysStateMatrix;
    }    
}


void mpc::initMPCInputMatrix(){
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

    auto sysStateMatrix = sys_.getStateMatrix();
    auto sysInputMatrix = sys_.getInputMatrix();
    auto sysOutputMatrix = sys_.getOutputMatrix();

    inputMatrix_.resize(predHorizon_ + 1, ctrlHorizon_);
    inputMatrix_.setZero();

    // Within ctrl horizon
    // First row
    inputMatrix_.block(0, 0, 1, 1) = sysOutputMatrix * sysInputMatrix;
    
    for (size_t idxCtrl = 1; idxCtrl < ctrlHorizon_; ++idxCtrl){
        
        // Shift right
        inputMatrix_.block(idxCtrl, 1, 1, ctrlHorizon_ - 1) = inputMatrix_.block(idxCtrl - 1, 0, 1, ctrlHorizon_ - 1);
        inputMatrix_.block(idxCtrl, 0, 1, 1).setZero(); // TODO: it can be removed for speed

        // Fill first element
        inputMatrix_.block(idxCtrl, 0, 1, 1) = sysOutputMatrix * matPow(sysStateMatrix, idxCtrl) * sysInputMatrix;
       
    }

    // After ctrl horizon
    Eigen::Matrix<double, 4, 4> sysStateMatrixHat = Eigen::Matrix<double, 4, 4>::Identity();

    for (size_t idxPred = ctrlHorizon_; idxPred <= predHorizon_; ++idxPred){
        // Shift right
        inputMatrix_.block(idxPred, 1, 1, ctrlHorizon_ - 1) = inputMatrix_.block(idxPred - 1, 0, 1, ctrlHorizon_ - 1);
        inputMatrix_.block(idxPred, 0, 1, 1).setZero(); // TODO: it can be removed for speed

        // Fill first element
        inputMatrix_.block(idxPred, 0, 1, 1) = sysOutputMatrix * matPow(sysStateMatrix, idxPred) * sysInputMatrix;

        // A_(i,v) = A^(i-v) + A^(i-v-1)+ ... + A + I
        auto powAhat = matPow(sysStateMatrix, idxPred - ctrlHorizon_);

        for (size_t idxPow = 0; idxPow < idxPred; ++idxPow){
            sysStateMatrixHat = sysStateMatrixHat + powAhat;
        }

        inputMatrix_.block(idxPred, ctrlHorizon_ - 1, 1, 1) = sysOutputMatrix * sysStateMatrixHat * sysInputMatrix;
    }
}

void mpc::initMPCWeights() {

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
    // Penalise the inputs
    // W2 = [Q0  0   0   0   ...
    //      0  Q1   0   0   ...
    //      .   .   .
    //      0  0   0   ... Qv-1]
    */
    
    weightMatrix2_.resize(ctrlHorizon_, ctrlHorizon_);
    weightMatrix2_.setIdentity();
    setInputPenalties(inputPenaltiesWeightStd_);

    /*
    // W3 = W1^T * W2 * W1
    */

    weightMatrix3_ = weightMatrix1_.transpose() * weightMatrix2_ * weightMatrix1_;

    /*
    // Penalise the Error
    // W4 = [P0  0   0   0   ...
    //      0  P1   0   0   ...
    //      .   .   .
    //      0  0   0   ... Pf]
    */

    weightMatrix4_.resize(predHorizon_ + 1, predHorizon_ + 1);
    weightMatrix4_.setIdentity();
    setErrorPenalties(errorPenaltiesWeightStd_);

}

void mpc::setErrorPenalties(double weight){
    weightMatrix4_ *= weight;
}

void mpc::setInputPenalties(double weight){
    weightMatrix2_ *= weight;
}

Eigen::MatrixXd mpc::computeControl(const Eigen::VectorXd& desiredTrajectory){

    desiredTrajectory_ = desiredTrajectory;

    auto currentState = sys_.getState();

    // s = zd - Oxk
    auto stateError = desiredTrajectory_ - stateMatrix_ * currentState;


    // u = (M^T * W4 * M + W3)^-1 * M^T * W4 * s
    auto controlInput = (inputMatrix_.transpose() * weightMatrix4_ * inputMatrix_ + weightMatrix3_).inverse() * inputMatrix_.transpose() * weightMatrix4_ * stateError;
    
    return controlInput;
}

double mpc::sysStep(double ctrlValue){
    return sys_.step(ctrlValue);
}