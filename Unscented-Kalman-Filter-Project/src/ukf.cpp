#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}


/**
* Generate Sigma Points
* @param {MatrixXd*} Xsig_out
*/
void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out){
    int n_x = 5;
    int lambda = 3 - n_x;
    
    MatrixXd Xsig = MatrixXd(n_x, 2*n_x + 1);
    //square root of (lambda + nx) * P
    MatrixXd A = P_.llt().matrixL();
    A = A * sqrt(lambda + n_x);
    
    Xsig.col(0) = x_;
    for (int i = 0; i < n_x; i++){
        Xsig.col(i+1) = x_ + A.col(i);
        Xsig.col(i+n_x+1) = x_ - A.col(i);
    }
    *Xsig_out = Xsig;
}

/**
* Generate Augmented Sigma Points
* Given noise covariance on acceleration and yaw rate acceleration
* @Param {MatrixXd*} Xsig_out
*/
void UKF::AugmentedSigmaPoints(MatrixXd *Xsig_out){
    int n_x = 5;
    int n_aug = 7;
    double lambda = 3 - n_aug;
    // process noise standard deviation longitudial acceleration in (m/s)^2
    double std_a = 0.2;
    // process noise standard deviation yaw rate acceleration in (rad/s)^2
    double std_yawdd = 0.2;
    
    MatrixXd x_aug = VectorXd(n_aug);
    x_aug.head(n_x) = x_;
    
    MatrixXd P_aug = MatrixXd(n_aug, n_aug);
    P_aug.topLeftCorner(n_x, n_x) = P_;
    P_aug(n_x+1, n_x+1) = std_a;
    P_aug(n_x+2, n_x+2) = std_yawdd;
    
    MatrixXd A = P_aug.llt().matrixL();
    MatrixXd sqrt_A = A * sqrt(lambda +  n_aug);
    
    MatrixXd Xsig_aug = MatrixXd(n_aug, 2*n_aug+1);
    Xsig_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug; i++){
        Xsig_aug.col(i+1) = x_aug + sqrt_A.col(i);
        Xsig_aug.col(i+n_aug+1) = x_aug - sqrt_A.col(i);
    }
    
    *Xsig_out = Xsig_aug;
}

