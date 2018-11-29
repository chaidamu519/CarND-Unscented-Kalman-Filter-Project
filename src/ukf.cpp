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
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  
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
  //set state dimension
  n_x_ = 5;

  //set augmented dimension
  n_aug_ = 7;
   
  //define spreading parameter
  lambda_ = 3 - n_aug_;
  
  //create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);  
 
  //create vector for weights
  weights_ = VectorXd(2*n_aug_+1);
  double c1 = lambda_ + n_aug_;
  weights_.fill(0.5 / c1);
  weights_(0) = lambda_ / c1;
  
  // calculate NIS
  NIS_lidar_ = 0.0;
  NIS_radar_ = 0.0;
  
  // time
  time_us_ = 0;
  
  // Lidar measurement noise matrix
  R_lidar_ = MatrixXd(2,2);
  R_lidar_.fill(0.0);
  R_lidar_(0,0) =  std_laspx_ *  std_laspx_;
  R_lidar_(1,1) =  std_laspy_ *  std_laspy_;
  
  // Radar measurement noise matrix
  R_radar_ = MatrixXd(3,3);
  R_radar_.fill(0.0);
  R_radar_(0,0) = std_radr_ * std_radr_;
  R_radar_(1,1) = std_radphi_ * std_radphi_;
  R_radar_(2,2) = std_radrd_ * std_radrd_;
  
  // Lidar measurement dimension
  n_z_lidar_ = 2;
  
  // Radar measurement dimension
  n_z_radar_ = 3;
  
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
  
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // initial state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    x_ << 0.0, 0.0, 0.0, 0.0, 0.0;
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar data to the state vector x_.
      */
	  //set the state with the initial location and velocity

      x_(0) = meas_package.raw_measurements_(0) * cos(meas_package.raw_measurements_(1));
      x_(1) = meas_package.raw_measurements_(0) * sin(meas_package.raw_measurements_(1));
     // x_(2) = meas_package.raw_measurements_(2);
      
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state for LIDER measurement
      */
		//set the state with the initial location 
		x_(0) = meas_package.raw_measurements_(0);
		x_(1) = meas_package.raw_measurements_(1);
    }
    
    time_us_ = meas_package.timestamp_;
    // done initializing
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //compute the time elapsed between the current and previous measurements
  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds / 1000000.0
  time_us_ = meas_package.timestamp_;
	
			   
  // Call the UKF predict() function
  Prediction(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    
     // Radar updates
     UpdateRadar(meas_package);
     // print the output
     cout << "NIS_radar = " << NIS_radar_<< endl;
     
  } else {
	  
     // Laser updates
     UpdateLidar(meas_package);
     // print the output
     cout << "NIS_lidar = " << NIS_lidar_ << endl;
  }

}



/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  /*****************************************************************************
   *  Generate augmented sigma points
   ****************************************************************************/

  //create augmented mean vector
  VectorXd x_aug_ = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
 
  //create augmented mean state
  x_aug_.head(n_x_) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;
  
  //create augmented covariance matrix
  MatrixXd Q_ = MatrixXd(2, 2);
  Q_ << std_a_*std_a_, 0,
       0, std_yawdd_*std_yawdd_;
  P_aug_.fill(0);
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_.bottomRightCorner(2,2) = Q_;
  MatrixXd A_ = P_aug_.llt().matrixL();
  
  //create square root matrix
  double coeff_ = n_aug_ + lambda_;
  MatrixXd expansion_ = sqrt(coeff_) * A_;
 
  // first column: mean
  Xsig_aug_.col(0) = x_aug_;
  
  // calculate the spread points
  Xsig_aug_.block<7, 7>(0,1) = expansion_.colwise() + x_aug_;
  Xsig_aug_.block<7, 7>(0, n_aug_ + 1) = (-expansion_).colwise() + x_aug_;
  

/*****************************************************************************
   *  Sigma Points Prediction
   ****************************************************************************/

  
  for (int i=0; i< (2*n_aug_ +1);i++){
      
    VectorXd aa_ = Xsig_aug_.col(i);
    
    // predefine some parameters
    double c1 = cos(aa_(3));
    double c2 = sin(aa_(3));
    double dd = 0.5 * delta_t * delta_t;
    
    // noise vector
    VectorXd vt_ = VectorXd(5);
    vt_ << dd * c1 * aa_(5),
           dd * c2 * aa_(5),
          delta_t * aa_(5),
          dd * aa_(6),
          delta_t * aa_(6);
          
    // process vector
    VectorXd h_ = VectorXd(5);
    h_.fill(0.0);
    
    // check division by zero
    if (fabs(aa_(4)) < 0.001){
        h_(0) = aa_(2) * c1 * delta_t;
        h_(1) = aa_(2) * c2 * delta_t;
    }
    else{
        double c3 = aa_(3) + delta_t * aa_(4);
        h_(0) = aa_(2) / aa_(4) * (sin(c3) - c2);
        h_(1) = aa_(2) / aa_(4) * (-cos(c3) + c1);
        h_(3) = aa_(4) * delta_t;
    }
    
    // calculate by CTRV process
    Xsig_pred_.col(i) = aa_.head(5) + h_ + vt_;
  }


/*****************************************************************************
   *  Predicted mean and covariance
   ****************************************************************************/
   
  // Temporal  state vector and covariance matrix
  VectorXd x_temp = VectorXd(n_x_);
  x_temp.fill(0.0);
  MatrixXd P_temp = MatrixXd(n_x_, n_x_);
  P_temp.fill(0.0);
  
  
  for (int i=0;i<(2 * n_aug_ + 1);i++){
      
      //predict state mean
      x_temp += Xsig_pred_.col(i) * weights_(i);
      
  }
  
   for (int i=0;i<(2 * n_aug_ + 1);i++){
      
      //predict state covariance matrix
      VectorXd cc = Xsig_pred_.col(i) - x_temp;
      while (cc(3)> M_PI) cc(3)-=2.*M_PI;
      while (cc(3)<-M_PI) cc(3)+=2.*M_PI;
      P_temp += weights_(i) * cc * cc.transpose();
      
  }
  
  // write prediction results
  x_ = x_temp;
  P_ = P_temp;
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
  
  /*****************************************************************************
   * Predict lidar measurement
   ****************************************************************************/
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig_ = MatrixXd(n_z_lidar_, 2 * n_aug_ + 1);
  Zsig_.fill(0.0);

  //mean predicted measurement
  VectorXd z_pred_ = VectorXd(n_z_lidar_);
  z_pred_.fill(0.0);
  
  //measurement covariance matrix S
  MatrixXd S_ = MatrixXd(n_z_lidar_,n_z_lidar_);
  S_.fill(0.0);

  //transform sigma points into measurement space
  Zsig_.row(0) = Xsig_pred_.row(0);
  Zsig_.row(1) = Xsig_pred_.row(1);
 
  
  
  //calculate mean predicted measurement
  for (int i=0;i<(2 * n_aug_ + 1);i++){
      z_pred_ += weights_(i) *  Zsig_.col(i);
  }
  
  //calculate innovation covariance matrix S
  for (int i=0;i<(2 * n_aug_ + 1);i++){
      
      VectorXd cc = Zsig_.col(i) - z_pred_;
      S_ += weights_(i) * cc * cc.transpose();
  }
  

  S_ = S_ + R_lidar_;

  
  /*****************************************************************************
   *  Update lidar measurement
   ****************************************************************************/
  
  //create example vector for incoming radar measurement
  VectorXd z_ = VectorXd(n_z_lidar_);
  z_ <<
      meas_package.raw_measurements_(0),   //px in m
      meas_package.raw_measurements_(1);   //py in m

  //create matrix for cross correlation Tc
  MatrixXd Tc_ = MatrixXd(n_x_, n_z_lidar_);
  Tc_.fill(0.0);


  //calculate cross correlation matrix
  for (int i=0;i<(2 * n_aug_ + 1);i++){
    
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    //angle normalization
    //while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    //while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    
    VectorXd z_diff = Zsig_.col(i) - z_pred_;
    
    //angle normalization
    //while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    //while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

      
    Tc_ += weights_(i) * x_diff * z_diff.transpose();
  }
  
  //calculate Kalman gain K;
  MatrixXd K_ = Tc_ * S_.inverse();
  
  // residul for angle normalization 
  VectorXd z_diff_ = z_ - z_pred_;
  while (z_diff_(1)> M_PI) z_diff_(1)-=2.*M_PI;
  while (z_diff_(1)<-M_PI) z_diff_(1)+=2.*M_PI;
  
  //update state mean and covariance matrix
  x_ = x_ + K_ * z_diff_;
  P_ = P_ - K_ * S_ * K_.transpose();
  
  // calculate NIS for lidar
  NIS_lidar_ = z_diff_.transpose() * S_.inverse() * z_diff_;
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
  
  /*****************************************************************************
   * Predict radar measurement
   ****************************************************************************/

  //create matrix for sigma points in measurement space
  MatrixXd Zsig_ = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);
  Zsig_.fill(0.0);

  //mean predicted measurement
  VectorXd z_pred_ = VectorXd(n_z_radar_);
  z_pred_.fill(0.0);
  
  //measurement covariance matrix S
  MatrixXd S_ = MatrixXd(n_z_radar_,n_z_radar_);
  S_.fill(0.0);

  //transform sigma points into measurement space
  for (int i=0;i<(2 * n_aug_ + 1);i++){
      
      VectorXd State = Xsig_pred_.col(i);
      
      Zsig_(0,i) = sqrt(State(0)*State(0)+State(1)*State(1));
      Zsig_(1,i) = atan2(State(1), State(0));
      if (Zsig_(0,i) < 0.0001){
          Zsig_(2,i) = 0.0;}
      else{
          Zsig_(2,i) = (State(0) * cos(State(3)) * State(2) + State(1) * sin(State(3)) * State(2)) / Zsig_(0,i); }
  }
  
  //calculate mean predicted measurement
  for (int i=0;i<(2 * n_aug_ + 1);i++){
      z_pred_ += weights_(i) *  Zsig_.col(i);
  }
  
  //calculate innovation covariance matrix S
  for (int i=0;i<(2 * n_aug_ + 1);i++){
      
      VectorXd cc = Zsig_.col(i) - z_pred_;
      
      while (cc(1)> M_PI) cc(1) -= 2.*M_PI;
      while (cc(1)<-M_PI) cc(1) += 2.*M_PI;
      
      S_ += weights_(i) * cc * cc.transpose();
  }
  
 
  S_ = S_ + R_radar_;

  
  /*****************************************************************************
   *  Update radar measurement
   ****************************************************************************/
  
  //create example vector for incoming radar measurement
  VectorXd z_ = VectorXd(n_z_radar_);
  z_ <<
      meas_package.raw_measurements_(0),   //rho in m
      meas_package.raw_measurements_(1),   //phi in rad
      meas_package.raw_measurements_(2);   //rho_dot in m/s

  //create matrix for cross correlation Tc
  MatrixXd Tc_ = MatrixXd(n_x_, n_z_radar_);
  Tc_.fill(0.0);


  //calculate cross correlation matrix
  for (int i=0;i<(2 * n_aug_ + 1);i++){
    
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    
    VectorXd z_diff = Zsig_.col(i) - z_pred_;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

      
    Tc_ += weights_(i) * x_diff * z_diff.transpose();
  }
  
  //calculate Kalman gain K;
  MatrixXd K_ = Tc_ * S_.inverse();
  
  // residul for angle normalization 
  VectorXd z_diff_ = z_ - z_pred_;
  while (z_diff_(1)> M_PI) z_diff_(1)-=2.*M_PI;
  while (z_diff_(1)<-M_PI) z_diff_(1)+=2.*M_PI;
  
  //update state mean and covariance matrix
  x_ = x_ + K_ * z_diff_;
  P_ = P_ - K_ * S_ * K_.transpose();
  
  // calculate NIS Radar
  NIS_radar_ = z_diff_.transpose() * S_.inverse() * z_diff_;
}
