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

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3, 4);
    // initialize
    Hj << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;
    
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    //check division here
    if (px ==0 && py == 0){
        cout << "error: divided by zero" <<endl;
        return Hj;
    }
    
    //compute the Jacobian matrix
    float dist = sqrt(px * px + py * py);
    Hj << px/dist, py/dist, 0, 0,
          -py/pow(dist, 2), px/pow(dist, 2), 0, 0,
          py*(vx*py-vy*px)/pow(dist, 3), px*(vy*px-vx*py)/pow(dist, 3), px/dist, py/dist;
    
    return Hj;
}

