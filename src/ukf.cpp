#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
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

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  //set state dimension
  n_x_ = 5;

  //set augmented dimension
  n_aug_ = 7;

  //set measurement dimension, radar can measure r, phi, and r_dot
  //n_z_ = 3;

  //define spreading parameter
  lambda_ = 3.0 - n_aug_;

  //Re set the process noise std values
  //Since the moving vehicle is bi-cycle,
  //the longitudinal acceleration is going to be much less than 30 m/s^2
  std_a_ = 1.0;
  std_yawdd_ = 0.5;

  //NIS values
  NIS_Lidar_ = 0.0;
  NIS_Radar_ = 0.0;

  //set vector for weights
  weights_ = VectorXd(2*n_aug_+1);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  cout << "weights" << weights_ << endl;

  //Initialize the P covariance vector
  //Never do the sort cuts. Using Identity matrix caused 3 days of debugging
  //Because it inits with int values  :(
  //P_ = MatrixXd::Identity(n_x_, n_x_);
  //Update with differnt values at (0,0) and (1,1) as mentioned in Lesson 7.32
  P_ << 0.15, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.15, 0.0, 0.0, 0.0,
        0.0, 0.0, 100, 0.0, 0.0,
        0.0, 0.0, 0.0, 100, 0.0,
        0.0, 0.0, 0.0, 0.0,1;
  cout << P_;

  //measurement covariance matrix R (2x2)
  R_Laser_ = MatrixXd(2, 2);
  R_Laser_.fill(0.0);

  R_Laser_(0, 0) = std_laspx_ * std_laspx_;
  R_Laser_(1, 1) = std_laspy_ * std_laspy_;

  //measurement covariance matrix R (3x3)
  R_Radar_ = MatrixXd(3, 3);
  R_Radar_.fill(0.0);
  R_Radar_(0, 0) = std_radr_ * std_radr_;
  R_Radar_(1, 1) = std_radphi_ * std_radphi_;
  R_Radar_(2, 2) = std_radrd_ * std_radrd_;


  //Initialize input vector x_
  x_ = VectorXd(n_x_);

  //Init sigma point matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

//Open the files for writing
  fRadar_.open("radar_nis.csv");
  fLaser_.open("laser_nis.csv");
}

UKF::~UKF() {
  //Close the files
  fRadar_.close();
  fLaser_.close();
}

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
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "UKF: " << endl;
    x_ << 1, 1, 0, 0, 0;
		float pX, pY, vX, vY, v, psi, psi_dot;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
			//Get ro (range) from the measurement data
			float rho = meas_package.raw_measurements_[0];
			//Get theta (bearing) from the measurement data
			float phi = meas_package.raw_measurements_[1];
			//Get ro_dot (radial velocity) from the measurement data
			float rho_dot = meas_package.raw_measurements_[2];

			//Get the pX and pY values by converting from polar to cartesian space
			pX = rho * cos(phi);
			pY = rho * sin(phi);
			//Get the vX and vY values by converting from polar to cartesian space
			vX = rho_dot * cos(phi);
			vY = rho_dot * sin(phi);

      v = sqrt(vX*vX+vY*vY);

			//Set the velocity components to be 0, as deriving velocy from
			//the first measurement may not be perfect
			psi = psi_dot = 0.0;


    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
		  /**
		  Initialize state.
		  */
			//set the state with the initial location and zero velocity
			pX = meas_package.raw_measurements_[0];
			pY = meas_package.raw_measurements_[1];

      float VSV = 0.001;
      if (fabs(pX) < VSV) pX = VSV;
      if (fabs(pY) < VSV) pY = VSV;

			v = psi = psi_dot = 0.0;
			//ekf_.x_ << , measurement_pack.raw_measurements_[1], 0, 0;

    }

		// if (pX < leastValue)
		//pX = tools.SetMinValues(pX);
		//pY = tools.SetMinValues(pY);
		/*
		if (fabs(pY) < MIN_VALUE){
			pY = LEAST_VALUE;
		}
		*/

		//set the state with the initial location and  velocity
		x_ << pX, pY, v, psi, psi_dot;

		//cout << "LOG: x_: " << ekf_.x_ << endl;
		//Set the time time stamp for finding the time difference between samples
		time_us_ = meas_package.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**

   */
	//compute the time elapsed between the current and previous measurements
	float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
	time_us_ = meas_package.timestamp_;

  cout << "Before of Predict" << endl;
  Prediction(dt);

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && (use_radar_)) {
    // Radar updates
    UpdateRadar(meas_package);

  } else if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && (use_laser_)) {
    // Laser updates
    UpdateLidar(meas_package);
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
  //cout << "begining of update for Predict" << endl;

  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  //calculate sigma points ...
  //set the mean at col(0)
  /*
  Xsig.col(0) = x_;
  //set sigma points as columns of matrix Xsig
  for (int nColIndex = 0; nColIndex < n_x_; nColIndex++){
      Xsig.col(nColIndex+1) = x_ + sqrt(lambda_+n_x_) * A.col(nColIndex);
      Xsig.col(n_x_+nColIndex+1) = x_ - sqrt(lambda_+n_x_) * A.col(nColIndex);
  }
  */

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.fill(0.0);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.head(n_x_) = x_;

  //create augmented covariance matrix
  //Set the P to the top left corner of 5x5 size
  P_aug.topLeftCorner(n_x_, n_x_) = P_;

  //Set the bottom corner of (2x2) with the variance for longitudinal and yaw acceleration
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  //A = MatrixXd(n_aug_, n_aug_);
  MatrixXd A = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  //set sigma points as columns of matrix Xsig

  for (int nColIndex = 0; nColIndex < n_aug_; nColIndex++){
      Xsig_aug.col(nColIndex+1) = x_aug + sqrt(lambda_+n_aug_) * A.col(nColIndex);
      Xsig_aug.col(n_aug_+nColIndex+1) = x_aug - sqrt(lambda_+n_aug_) * A.col(nColIndex);
  }

  //Now predict the sigma points
  //VectorXd x_prev = VectorXd(n_aug_);
  //VectorXd x_kplus = VectorXd(n_x_);
  //predict sigma points
  //avoid division by zero
  //write predicted sigma points into right column
  for (int nColIndex = 0; nColIndex < n_aug_*2+1; nColIndex++){
      //Get each column from the augmented sigma points
      //VectorXd x_prev = Xsig_aug.col(nColIndex);
      //VectorXd x_kplus =  x_prev.head(n_x_);
      //Take out the components from the Xsig_aug
      float pX = Xsig_aug(0, nColIndex);
      float pY = Xsig_aug(1, nColIndex);
      float vel = Xsig_aug(2, nColIndex);
      float yaw = Xsig_aug(3, nColIndex);
      float yaw_dot = Xsig_aug(4, nColIndex);
      float new_la = Xsig_aug(5, nColIndex);
      float new_yd = Xsig_aug(6, nColIndex);

      //
      //if Xsig_aug[4] != 0  (i.e yawdot)
      if (yaw_dot > VSV){
         pX = pX + (vel/yaw_dot)*((sin(yaw+yaw_dot*delta_t))-sin(yaw)) ;
         pY = pY + (vel/yaw_dot)*((-cos(yaw+yaw_dot*delta_t))+cos(yaw)) ;

      }else{
         pX = pX + vel*cos(yaw)*delta_t;
         pY = pY + vel*sin(yaw)*delta_t;
      }

      //Check for the second vector i.e varianve components
      pX = pX + 0.5*delta_t*delta_t*cos(yaw)*new_la;
      pY = pY + 0.5*delta_t*delta_t*sin(yaw)*new_la;
      vel = vel + delta_t*new_la;
      yaw = yaw + 0.5*delta_t*delta_t*new_yd + yaw_dot*delta_t;
      yaw_dot = yaw_dot + delta_t*new_yd;

      //Now set the x_kplus into Xsig_pred vector
      Xsig_pred_(0, nColIndex) = pX;
      Xsig_pred_(1, nColIndex) = pY;
      Xsig_pred_(2, nColIndex) = vel;
      Xsig_pred_(3, nColIndex) = yaw;
      Xsig_pred_(4, nColIndex) = yaw_dot;
    }

    //Predict x mean and P covariance matrix
    x_.fill(0.0);
    for (int nIndex = 0; nIndex < 2*n_aug_+1; nIndex++){
      //traverse thru each column in Xsig_pred and multiply with the weights vector
      x_ = x_ + weights_(nIndex)*Xsig_pred_.col(nIndex);

    }
    //std::cout << "x" << x << std::endl;
    //predict state mean

    //predict state covariance matrix
    P_.fill(0.0);
    //VectorXd x_slice = VectorXd(n_x_);
    for (int nIndex = 0; nIndex < 2*n_aug_+1; nIndex++){
      //traverse thru each column in Xsig_pred and multiply with the weights vector
      VectorXd x_slice = Xsig_pred_.col(nIndex) - x_;

      //cout << "     before angle norm in predict" << Xsig_pred_.col(nIndex) << endl;
      //angle normalization;
      //MAke sure the phi is within -PI to PI

      while (x_slice(3) > M_PI) x_slice(3) -=2.*M_PI;
      while (x_slice(3) <-M_PI) x_slice(3) +=2.*M_PI;

      //cout << "     after angle norm in predict" << x_slice << endl;

      P_ = P_ + weights_(nIndex)*x_slice*x_slice.transpose();

    }

    cout << "end of update for Predict" << endl;
    cout << "P_" << P_ << endl;


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
  cout << "begining of update for Lidar" << endl;

  int n_z = 2;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  //transform sigma points into measurement space
  //VectorXd x_sigma_pred_slice = VectorXd(n_x_);
  Zsig.fill(0.0);
  z_pred.fill(0.0);
  for (int nSigmaIndex = 0; nSigmaIndex <  2 * n_aug_ + 1; nSigmaIndex++){
      VectorXd x_sigma_pred_slice = Xsig_pred_.col(nSigmaIndex);
      float p_x = x_sigma_pred_slice[0];
      float p_y = x_sigma_pred_slice[1];

      Zsig(0, nSigmaIndex) = p_x;
      Zsig(1, nSigmaIndex) = p_y;

      z_pred = z_pred+weights_(nSigmaIndex)*Zsig.col(nSigmaIndex);

  }
  //calculate mean predicted measurement
  //calculate measurement covariance matrix S


  //Traverse thru predicted data in measurement space and find
  //the diff between the measured and mean to find co-variance matrix
  S.fill(0.0);
  //VectorXd z_sigma_pred_slice = VectorXd(n_z);
  for (int nSigmaIndex = 0; nSigmaIndex <  2 * n_aug_ + 1; nSigmaIndex++){
      VectorXd z_sigma_pred_slice = Zsig.col(nSigmaIndex) - z_pred;

      S = S + weights_(nSigmaIndex)*z_sigma_pred_slice*z_sigma_pred_slice.transpose();
  }
  S = S + R_Laser_;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  VectorXd z = meas_package.raw_measurements_;

  Tc.fill(0.0);
  //calculate cross correlation matrix
  VectorXd x_slice; // = VectorXd(n_x);
  VectorXd z_slice; // = VectorXd(n_z);
  for (int nSigmaIndex = 0; nSigmaIndex <  2 * n_aug_ + 1; nSigmaIndex++){
      VectorXd x_slice = Xsig_pred_.col(nSigmaIndex) - x_;
      VectorXd z_slice = Zsig.col(nSigmaIndex) - z_pred;

	  Tc = Tc + weights_(nSigmaIndex)*x_slice*z_slice.transpose();

  }
  //calculate Kalman gain K;
  MatrixXd K = Tc*S.inverse();

  cout << "z = " << z.rows()  << " "  << z.cols() << "other" << z_pred.rows() << " " << z_pred.cols() << endl;
  //update state mean and covariance matrix
  VectorXd z_diff = z-z_pred;

  //Update the state (x)
  x_ = x_ + K*z_diff;

  //Update Covariance martix P
  P_ = P_ - K*S*K.transpose();

  //Update NIS value
  NIS_Lidar_ = z_diff.transpose()*S.inverse()*z_diff;

  //Dump NIS values to a files
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    if (fLaser_.is_open()) {
      fLaser_ << NIS_Lidar_ << "\n";
      fLaser_.flush();
    }
  } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    if (fRadar_.is_open()) {
      fRadar_ << NIS_Radar_ << "\n";
      fRadar_.flush();
    }
  }

  cout << "end of update for Lidar" << P_ << endl;
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
  cout << "begining of update for Radar" << endl;
  int n_z = 3;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  //transform sigma points into measurement space
  VectorXd x_sigma_pred_slice = VectorXd(n_x_);
  Zsig.fill(0.0);
  z_pred.fill(0.0);
  for (int nSigmaIndex = 0; nSigmaIndex <  2 * n_aug_ + 1; nSigmaIndex++){
      x_sigma_pred_slice = Xsig_pred_.col(nSigmaIndex);
      float p_x = x_sigma_pred_slice[0];
      float p_y = x_sigma_pred_slice[1];
      float vel = x_sigma_pred_slice[2];
      float psi = x_sigma_pred_slice[3];
      float psi_dot = x_sigma_pred_slice[4];

      //If px, py is less that 0.01 (min value), set it to 0.01
      //p_x = tools.SetMinValues(p_x);
  		//p_y = tools.SetMinValues(p_y);
      float VSV = 0.001;
      if (fabs(p_x) < VSV) p_x = VSV;
      if (fabs(p_y) < VSV) p_y = VSV;

      float rho = sqrt(p_x*p_x+p_y*p_y);
      float phi = atan2(p_y, p_x);
      float rho_dot = (vel*(p_x*cos(psi)+p_y*sin(psi)))/rho;

      Zsig(0, nSigmaIndex) = rho;
      Zsig(1, nSigmaIndex) = phi;
      Zsig(2, nSigmaIndex) = rho_dot;

      z_pred = z_pred+weights_(nSigmaIndex)*Zsig.col(nSigmaIndex);

  }
  //calculate mean predicted measurement
  //calculate measurement covariance matrix S

  //Traverse thru predicted data in measurement space and find
  //the diff between the measured and mean to find co-variance matrix
  S.fill(0.0);
  //VectorXd z_sigma_pred_slice = VectorXd(n_z);
  for (int nSigmaIndex = 0; nSigmaIndex <  2 * n_aug_ + 1; nSigmaIndex++){
    VectorXd z_sigma_pred_slice = Zsig.col(nSigmaIndex) - z_pred;

    //angle normalization;
    //MAke sure the phi is within -PI to PI
    while (z_sigma_pred_slice(1)> M_PI) z_sigma_pred_slice(1)-=2.*M_PI;
    while (z_sigma_pred_slice(1)<-M_PI) z_sigma_pred_slice(1)+=2.*M_PI;

    S = S + weights_(nSigmaIndex)*z_sigma_pred_slice*z_sigma_pred_slice.transpose();
  }
  S = S + R_Radar_;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  VectorXd z = meas_package.raw_measurements_;

  Tc.fill(0.0);
  //calculate cross correlation matrix
  VectorXd x_slice; // = VectorXd(n_x);
  VectorXd z_slice; // = VectorXd(n_z);
  for (int nSigmaIndex = 0; nSigmaIndex <  2 * n_aug_ + 1; nSigmaIndex++){
      VectorXd x_slice = Xsig_pred_.col(nSigmaIndex) - x_;
      VectorXd z_slice = Zsig.col(nSigmaIndex) - z_pred;

      //MAke sure the phi is within -PI to PI
      while (z_slice(1)> M_PI) z_slice(1)-=2.*M_PI;
      while (z_slice(1)<-M_PI) z_slice(1)+=2.*M_PI;

  	  //MAke sure the phi is within -PI to PI
      while (x_slice(3)> M_PI) x_slice(3)-=2.*M_PI;
      while (x_slice(3)<-M_PI) x_slice(3)+=2.*M_PI;

	  Tc = Tc + weights_(nSigmaIndex)*x_slice*z_slice.transpose();

  }
  //calculate Kalman gain K;
  MatrixXd K = Tc*S.inverse();
  //update state mean and covariance matrix
  VectorXd z_diff = z-z_pred;
  //Make sure the phi is within -PI to PI

  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //Update the state (x)
  x_ = x_ + K*z_diff;

  //Update Covariance martix P
  P_ = P_ - K*S*K.transpose();

  //Update NIS value
  NIS_Radar_ = z_diff.transpose()*S.inverse()*z_diff;

  //Dump NIS values to a files
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    //if (fLaser_.is_open()) {

      fLaser_ << NIS_Lidar_ << "\n";
      fLaser_.flush();
    //}
  } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    //if (fRadar_.is_open()) {
      cout << "Inside NIS Radar" << endl;
      fRadar_ << NIS_Radar_ << "\n";
      fRadar_.flush();
    //}
  }

  cout << "end of update for Radar" << P_ << endl;
}
