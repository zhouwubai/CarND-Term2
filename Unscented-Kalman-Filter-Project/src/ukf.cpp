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
    P_aug(n_x+1, n_x+1) = std_a*std_a;
    P_aug(n_x+2, n_x+2) = std_yawdd*std_yawdd;
    
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


/**
* Predict augmented sigma points to state sigma points
* @Param {MatrixXd*} Xsig_out
*/
void UKF::SigmaPointPrediction(MatrixXd* Xsig_out){
  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  double delta_t = 0.1; //time diff in sec
  double delta_t2 = delta_t * delta_t;

  //predict sigma points, map augmented state of length 7 to length 5
  //avoid division by zero
  //write predicted sigma points into right column
  for (int i = 0; i < 2*n_aug+1; i++){
    VectorXd aug_x = Xsig_aug.col(i);
    float v = aug_x(2);
    float rho = aug_x(3);
    float rho_dot = aug_x(4);
    float v_dot = aug_x(5);
    float rho_dd = aug_x(6);
    
    float c_rho = cos(rho);
    float s_rho = sin(rho);
    
    VectorXd delta_x(n_x);
    VectorXd noise_x(n_x);
    // if yaw rate is 0, car is going straight
    if (fabs(rho_dot) < 0.001){
        delta_x << v * c_rho * delta_t, v * s_rho * delta_t, 0, 0, 0;
    } else {
        float vr_dot = v / rho_dot;
        float max_rho = rho + rho_dot * delta_t;
        delta_x << vr_dot * (sin(max_rho) - s_rho), vr_dot * (-cos(max_rho) + c_rho), 0, rho_dot * delta_t, 0;
    }
    noise_x << delta_t2 * c_rho * v_dot/2, delta_t2 * s_rho * v_dot/2, delta_t * v_dot, delta_t2 * rho_dd/2, delta_t * rho_dd;
    Xsig_pred.col(i) = aug_x.head(n_x) + delta_x + noise_x;
  }

  *Xsig_out = Xsig_pred;
}

/**
* Given predicted state sigma points, estimate its mean and covariances
* @Param {VectorXd*} x_pred
* @Param {Matrix*} P_pred
*/
void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out){
  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  //create vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
  
  //create vector for predicted state
  VectorXd x = VectorXd(n_x);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x, n_x);

  //set weights
  //predict state mean
  //predict state covariance matrix
  weights(0) = lambda / (lambda + n_aug);
  for (int i = 0; i < 2 * n_aug + 1; i++){
    weights(i) = 0.5 / (lambda + n_aug);
  }
  
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++){
    x += weights(i) * Xsig_pred.col(i);
  }
  
  P.fill(0.0);
  for(int i = 0; i < 2 * n_aug + 1; i++){
    VectorXd residual = Xsig_pred.col(i) - x;
    P += weights(i) * (residual * residual.transpose());
  }
  
  //write result
  *x_out = x;
  *P_out = P;
}







