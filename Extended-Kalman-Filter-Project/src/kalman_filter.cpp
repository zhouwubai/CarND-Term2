#include "kalman_filter.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd H_t = H_.transpose();
    MatrixXd S = H_ * P_ * H_t + R_;
    MatrixXd S_inv = S.inverse();
    MatrixXd K = P_ * H_t * S_inv;

    //new estimate
    x_ = x_ + K * y;
    P_ = P_ - K * H_ * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
    * it is exactly the same as Update, only z, H_, R_ is different (size,)
  */

  // to calculate z, we will use the non-linear h(x) function directly
  VectorXd z_pred(3);
  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  if (rho < 0.0001){
    rho = 0.0001;
  }
  float phi = atan2(x_(1), x_(0)); // normalize phi
  float rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
  z_pred << rho, phi, rho_dot;

  VectorXd y = z - z_pred;
  y(1) = atan2(sin(y(1)), cos(y(1))); // normalize phi after substraction

  MatrixXd H_t = H_.transpose();
  MatrixXd S = H_ * P_ * H_t + R_;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = P_ * H_t * S_inv;

  //new estimate
  x_ = x_ + K * y;
  P_ = P_ - K * H_ * P_;
}
