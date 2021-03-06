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

  n_x_ = 5;
  // initial state vector and covariance matrix
  x_ = VectorXd(n_x_);
  P_ = MatrixXd(n_x_, n_x_);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.7;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  n_z_laser_ = 2; //px, py
  R_laser_ = MatrixXd(n_z_laser_, n_z_laser_);
  R_laser_ << std_laspx_ * std_laspx_, 0,
              0, std_laspy_ * std_laspy_;
  H_laser_ = MatrixXd(n_z_laser_, n_x_);
  H_laser_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;

  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  n_z_radar_ = 3; // rho, phi, rho_dot
  R_radar_ = MatrixXd(n_z_radar_, n_z_radar_);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
              0, std_radphi_ * std_radphi_, 0,
              0, 0, std_radrd_ * std_radrd_;

  /**
  TODO:
  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;

  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++){
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }

  NIS_laser_ = 0.0;
  NIS_radar_ = 0.0;
  Xsig_pred_ = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);
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
  if (!is_initialized_) {
    cout << "UKF: " << endl;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
        float rho = meas_package.raw_measurements_(0);
        float yaw = meas_package.raw_measurements_(1);
        float rho_dot = meas_package.raw_measurements_(2);
        float px = rho*cos(yaw);
        float py = rho*sin(yaw);
        // px, py, v, yaw, yawr, not sure about yawrd
        x_ << px, py, rho_dot, atan2(py, px), 0;
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER){
        x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0, 0, 0;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }// END_IF (!is_initialized)


    // Gateway for data, check use_laser_, use_radar or both
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && !use_radar_){
        return;
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && !use_laser_){
        return;
    }

   /*****************************************************************************
   *  Prediction
   ****************************************************************************/
   /**
    [x] calculate time delta
    [x] predict
   */

    float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
    cout << delta_t <<endl;
    time_us_ = meas_package.timestamp_;

    // small granualarity on prediciton
    float interval = 0.05;
    while(delta_t > interval){
        Prediction(interval);
        delta_t -= interval;
    }

    Prediction(delta_t);

   /*****************************************************************************
   *  Update
   ****************************************************************************/
  /**
  [x] check sensor type
  [x] update: almost same as EKF, just use sampling instead jacobian matrix
  */
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
    UpdateRadar(meas_package);
    cout << "NIS Radar:\n";
    cout << NIS_radar_ << endl;

  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
    cout << "NIS laser:\n";
    cout << NIS_laser_ << endl;
  }
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
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  AugmentedSigmaPoints(&Xsig_aug);
  SigmaPointPrediction(&Xsig_pred_, Xsig_aug, delta_t);
  PredictMeanAndCovariance(&x_, &P_, Xsig_pred_);
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
  UpdateLidarDirect(meas_package);
  //UpdateLidarApprox(meas_package);
}

void UKF::UpdateLidarDirect(MeasurementPackage meas_package) {
  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_pred = H_laser_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd H_t = H_laser_.transpose();
  MatrixXd S = H_laser_ * P_ * H_t + R_laser_;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = P_ * H_t * S_inv;

  //new estimate and NIS
  x_ += K * y;
  P_ -= K * H_laser_ * P_;
  NIS_laser_ = y.transpose() * S_inv * y;
}

void UKF::UpdateLidarApprox(MeasurementPackage meas_package){
  VectorXd z = meas_package.raw_measurements_;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd::Zero(n_z_laser_, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++){

    Zsig(0, i) = Xsig_pred_(0, i);
    Zsig(1, i) = Xsig_pred_(1, i);
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z_laser_);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z_laser_,n_z_laser_);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z_laser_);

  //calculate mean predicted measurement
  for (int i = 0; i < 2 * n_aug_ + 1; i++){
    z_pred += weights_(i) * Zsig.col(i);
  }

  //calculate innovation covariance matrix S
  for (int i = 0; i < 2 * n_aug_ + 1; i++){
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_(i) * (z_diff * z_diff.transpose());

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

 // Add measurement covariance R_radar_ matrix
 S += R_laser_;

 //calculate Kalman gain K;
 MatrixXd K = Tc * S.inverse();

 VectorXd y = z - z_pred;
 //update state mean and covariance matrix
 x_ += K * y; //angle normalize might need here
 P_ -= K * S * K.transpose();

 NIS_laser_ = y.transpose() * S.inverse() * y;
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
  VectorXd z = meas_package.raw_measurements_;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd::Zero(n_z_radar_, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++){

    float px = Xsig_pred_(0, i);
    float py = Xsig_pred_(1, i);
    float v = Xsig_pred_(2, i);
    float rho = Xsig_pred_(3, i);

    // avoid division by zero
    if(fabs(px) <= 0.0001){
        px = 0.0001;
    }
    if (fabs(py) <= 0.0001){
        py = 0.0001;
    }

    float dist = sqrt(px * px + py * py);  // this wont be zero

    VectorXd z_sig(n_z_radar_);
    z_sig << dist, atan2(py, px), (px*v*cos(rho) + py*v*sin(rho))/dist;
    Zsig.col(i) = z_sig;
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd::Zero(n_z_radar_);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z_radar_,n_z_radar_);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z_radar_);

  //calculate mean predicted measurement
  for (int i = 0; i < 2 * n_aug_ + 1; i++){
    z_pred += weights_(i) * Zsig.col(i);
  }

  VectorXd y = z - z_pred;
  y(1) = NormalizeAngle(y(1));

  //calculate innovation covariance matrix S
  for (int i = 0; i < 2 * n_aug_ + 1; i++){
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = NormalizeAngle(z_diff(1));

    S += weights_(i) * (z_diff * z_diff.transpose());

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(1) = NormalizeAngle(x_diff(1));

    Tc += weights_(i) * x_diff * z_diff.transpose();

  }

 // Add measurement covariance R_radar_ matrix
 S += R_radar_;

 //calculate Kalman gain K;
 MatrixXd K = Tc * S.inverse();

 //update state mean and covariance matrix
 x_ += K * y; //angle normalize might need here
 P_ -= K * S * K.transpose();

 NIS_radar_ = y.transpose() * S.inverse() * y;
}

/**
* Generate Augmented Sigma Points
* Given noise covariance on acceleration and yaw rate acceleration
* @Param {MatrixXd*} X_sample: 2 * n_aug_ + 1 samples
*/
void UKF::AugmentedSigmaPoints(MatrixXd *X_sample){

    VectorXd x_aug = VectorXd::Zero(n_aug_);
    x_aug.head(n_x_) = x_;

    MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(n_x_, n_x_) = std_a_ * std_a_; // index start from 0
    P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

    MatrixXd A = P_aug.llt().matrixL();
    MatrixXd sqrt_A = A * sqrt(lambda_ +  n_aug_);

    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    Xsig_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug_; i++){
        Xsig_aug.col(i+1) = x_aug + sqrt_A.col(i);
        Xsig_aug.col(i+n_aug_+1) = x_aug - sqrt_A.col(i);
    }

    *X_sample = Xsig_aug;
}


/**
* Predict augmented sigma points to state sigma points after time delta
* @Param {MatrixXd*} X_pred: 2 * n_aug_ + 1 state sigma points
* @Param {MatrixXd&} X_sample: 2 * n_aug_ + 1 sampled augmented sigma points
*/
void UKF::SigmaPointPrediction(MatrixXd* X_pred, MatrixXd& X_sample, double delta_t){

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);
  double delta_t2 = delta_t * delta_t;

  //predict sigma points, map augmented state of length 7 to length 5
  //avoid division by zero
  //write predicted sigma points into right column
  for (int i = 0; i < 2 * n_aug_ + 1; i++){
    VectorXd aug_x = X_sample.col(i);
    float v = aug_x(2);
    float rho = aug_x(3);
    float rho_dot = aug_x(4);
    float v_dot = aug_x(5);
    float rho_dd = aug_x(6);

    float c_rho = cos(rho);
    float s_rho = sin(rho);

    VectorXd delta_x(n_x_);
    VectorXd noise_x(n_x_);
    // if yaw rate is 0, car is going straight
    if (fabs(rho_dot) < 0.001){
        delta_x << v * c_rho * delta_t, v * s_rho * delta_t, 0, 0, 0;
    } else {
        float vr_dot = v / rho_dot;
        float max_rho = rho + rho_dot * delta_t;
        delta_x << vr_dot * (sin(max_rho) - s_rho), vr_dot * (-cos(max_rho) + c_rho), 0, rho_dot * delta_t, 0;
    }
    noise_x << delta_t2 * c_rho * v_dot/2, delta_t2 * s_rho * v_dot/2, delta_t * v_dot, delta_t2 * rho_dd/2, delta_t * rho_dd;
    Xsig_pred.col(i) = aug_x.head(n_x_) + delta_x + noise_x;
  }

  *X_pred = Xsig_pred;
}

/**
* Given predicted state sigma points, estimate its mean and covariances
* @Param {VectorXd*} x_pred: estimated state mean from sigma points
* @Param {Matrix*} P_pred: estimated state covariance
* @Param {Matrix&} X_pred: predicted states derived from sampled augmented sigma points
*/
void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out, MatrixXd& X_pred){

  //create vector for predicted state
  VectorXd x = VectorXd::Zero(n_x_);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd::Zero(n_x_, n_x_);

  //predict state mean
  //predict state covariance matrix

  for (int i = 0; i < 2 * n_aug_ + 1; i++){
    x += weights_(i) * X_pred.col(i);
  }

  for(int i = 0; i < 2 * n_aug_ + 1; i++){
    VectorXd x_diff = X_pred.col(i) - x;
    x_diff(3) = NormalizeAngle(x_diff(3));

    P += weights_(i) * (x_diff * x_diff.transpose());
  }

  //write result
  *x_out = x;
  *P_out = P;
}


/**
* normalize angle to interval [-PI, PI]
* @Param {float} angle
*/
float UKF::NormalizeAngle(float angle){
    while (angle > M_PI) angle -= 2.*M_PI;
    while (angle < -M_PI) angle += 2.*M_PI;
    return angle;
}
