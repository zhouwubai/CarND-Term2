#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    if (estimations.size() != ground_truth.size()){
        cout << "CalculationRMSE error: dismatch size" <<endl;
        return rmse;
    } else if (estimations.size() == 0){
        cout << "CalculationRMSE error: size 0" <<endl;
        return rmse;
    }
    
    //accumulate squared residuals
    for(int i = 0; i < estimations.size(); ++i){
        VectorXd residual = estimations[i] - ground_truth[i];
        rmse += VectorXd(residual.array() * residual.array());
    }
    
    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();
    
    return rmse;
}
