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
        //angle normalization
    while (residual(3) > M_PI) residual(3) -= 2.*M_PI;
    while (residual(3) < -M_PI) residual(3) += 2.*M_PI;
    P += weights(i) * (residual * residual.transpose());
  }
  
  //write result
  *x_out = x;
  *P_out = P;
}


/**
* Map from augmented sigma points to predict measurements
* @Param {VectorXd*} z_out
* @Param {MatrixXd*} S_out
*/
void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out) {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
   double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }

  //radar measurement noise standard deviation radius in m
  double std_radr = 0.3;

  //radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;

  //radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;

  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; i++){
  
    float px = Xsig_pred(0, i);
    if (px == 0) px = 0.0001;  // avoid division by zero
    
    float py = Xsig_pred(1, i);
    float v = Xsig_pred(2, i);
    float rho = Xsig_pred(3, i);
    float dist = sqrt(px * px + py * py);  // this wont be zero
    
    VectorXd z_sig(n_z);
    z_sig << dist, atan2(py, px), (px*v*cos(rho) + py*v*sin(rho))/dist;
    Zsig.col(i) = z_sig;
  }
  
  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++){
    z_pred += weights(i) * Zsig.col(i);
  }
  
  //calculate innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++){
    VectorXd residual = Zsig.col(i) - z_pred;
    while (residual(1) > M_PI) residual(1) -= 2.*M_PI;
    while (residual(1) < -M_PI) residual(1) += 2.*M_PI;
    S += weights(i) * (residual * residual.transpose());
  }
  
 // Calculate R matrix
 MatrixXd R(n_z, n_z);
 R.fill(0.0);
 R(0, 0) = std_radr * std_radr;
 R(1, 1) = std_radphi * std_radphi;
 R(2, 2) = std_radrd * std_radrd;
 S += R;
 
  //write result
  *z_out = z_pred;
  *S_out = S;
}

/**
* Update the state for time K+1
* @Param {VectorXd*} x_out
* @Param {MatrixXd*} P_out
*/
void UKF::UpdateState(VectorXd* x_out, MatrixXd* P_out) {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
   double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }

  //create example matrix with predicted sigma points in state space
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  //create example vector for predicted state mean
  VectorXd x = VectorXd(n_x);
  x <<
     5.93637,
     1.49035,
     2.20528,
    0.536853,
    0.353577;

  //create example matrix for predicted state covariance
  MatrixXd P = MatrixXd(n_x,n_x);
  P <<
  0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
  -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
  0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
 -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
 -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;

  //create example matrix with sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
  Zsig <<
      6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057,
     0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239,
      2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295;

  //create example vector for mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred <<
      6.12155,
     0.245993,
      2.10313;

  //create example matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z,n_z);
  S <<
      0.0946171, -0.000139448,   0.00407016,
   -0.000139448,  0.000617548, -0.000770652,
     0.00407016, -0.000770652,    0.0180917;

  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z <<
      5.9214,   //rho in m
      0.2187,   //phi in rad
      2.0062;   //rho_dot in m/s

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug+1; i++){
    Tc += weights(i) * (Xsig_pred.col(i) - x) * (Zsig.col(i) - z_pred).transpose();
  }
  
  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  //update state mean and covariance matrix
  x += K * (z - z_pred); //angle normalize might need here
  P -= K * S * K.transpose();

  //write result
  *x_out = x;
  *P_out = P;
}






