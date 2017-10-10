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
  lambda_ = 3 - n_aug_;

  //Re set the process noise std values
  //Since the moving vehicle is bi-cycle,
  //the longitudinal acceleration is going to be much less than 30 m/s^2
  std_a_ = 6.0;
  std_yawdd_ = 1.0;

  //set vector for weights
  weights_ = VectorXd(2*n_aug_+1);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  //Initialize the P covariance vector
  P_ = MatrixXd::Identity(n_x_, n_x_);

  //Initialize input vector x_
  x_ = VectorXd(n_x_);

  //Init sigma point matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
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
			psi = psi_dot = 0;


    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
		  /**
		  Initialize state.
		  */
			//set the state with the initial location and zero velocity
			pX = meas_package.raw_measurements_[0];
			pY = meas_package.raw_measurements_[1];
			v = psi = psi_dot = 0;
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

  Prediction(dt);

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates


  } else {
    // Laser updates

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
  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();
  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  //calculate sigma points ...
  //set the mean at col(0)
  Xsig.col(0) = x_;
  //set sigma points as columns of matrix Xsig
  for (int nColIndex = 0; nColIndex < n_x_; nColIndex++){
      Xsig.col(nColIndex+1) = x_ + sqrt(lambda_+n_x_) * A.col(nColIndex);
      Xsig.col(n_x_+nColIndex+1) = x_ - sqrt(lambda_+n_x_) * A.col(nColIndex);
  }

  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug[n_aug_-2] = 0;
  x_aug[n_aug_-1] = 0;
  //create augmented covariance matrix
  //Set the P to the top left corner of 5x5 size
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  //Set the bottom corner of (2x2) with the variance for longitudinal and yaw acceleration
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;
  //create square root matrix
  A = MatrixXd(n_aug_, n_aug_);
  A = P_aug.llt().matrixL();
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  //set sigma points as columns of matrix Xsig

  for (int nColIndex = 0; nColIndex < n_aug_; nColIndex++){
      Xsig_aug.col(nColIndex+1) = x_aug + sqrt(lambda_+n_aug_) * A.col(nColIndex);
      Xsig_aug.col(n_aug_+nColIndex+1) = x_aug - sqrt(lambda_+n_aug_) * A.col(nColIndex);
  }

  //Now predict the sigma points
  VectorXd x_prev = VectorXd(n_aug_);
  VectorXd x_kplus = VectorXd(n_x_);
  //predict sigma points
  //avoid division by zero
  //write predicted sigma points into right column
  for (int nColIndex = 0; nColIndex < n_aug_*2+1; nColIndex++){
      //Get each column from the augmented sigma points
      x_prev = Xsig_aug.col(nColIndex);
      x_kplus =  x_prev.head(n_x_);
      //Take out the components from the vector
      float vel = x_prev[2];
      float yaw = x_prev[3];
      float yaw_dot = x_prev[4];
      float new_la = x_prev[5];
      float new_yd = x_prev[6];
      //Check for the second vector i.e varianve components
      x_kplus[0] = x_kplus[0]+ 0.5*delta_t*delta_t*cos(yaw)*new_la;
      x_kplus[1] = x_kplus[1]+ 0.5*delta_t*delta_t*sin(yaw)*new_la;
      x_kplus[2] = x_kplus[2]+ delta_t*new_la;
      x_kplus[3] = x_kplus[3] + 0.5*delta_t*delta_t*new_yd;
      x_kplus[4] = x_kplus[4] + delta_t*new_yd;

      //
      //if Xsig_aug[4] != 0  (i.e yawdot)
      if (x_prev(4) != 0){
         x_kplus[0] = x_kplus[0] + (vel/yaw_dot)*((sin(yaw+yaw_dot*delta_t))-sin(yaw)) ;
         x_kplus[1] = x_kplus[1] + (vel/yaw_dot)*((-cos(yaw+yaw_dot*delta_t))+cos(yaw)) ;

      }else{
         x_kplus[0] = x_kplus[0] + vel*cos(yaw)*delta_t;
         x_kplus[1] = x_kplus[1] + vel*sin(yaw)*delta_t;
      }

      x_kplus[2] = x_kplus[2] + 0;
      x_kplus[3] = x_kplus[3] + yaw_dot*delta_t;
      x_kplus[4] = x_kplus[4] + 0;

      //Now set the x_kplus into Xsig_pred vector
      Xsig_pred_.col(nColIndex) = x_kplus;
    }

    //Predict x mean and P covariance matrix
    x_(0.0);
    for (int nIndex = 0; nIndex < 2*n_aug_+1; nIndex++){
      //traverse thru each column in Xsig_pred and multiply with the weights vector
      x_ = x_ + weights_(nIndex)*Xsig_pred_.col(nIndex);

    }
    //std::cout << "x" << x << std::endl;
    //predict state mean

    //predict state covariance matrix
    P_(0.0);
    VectorXd x_slice = VectorXd(n_x_);
    for (int nIndex = 0; nIndex < 2*n_aug_+1; nIndex++){
      //traverse thru each column in Xsig_pred and multiply with the weights vector
      x_slice = Xsig_pred_.col(nIndex) - x_;
      P_ = P_ + weights_(nIndex)*x_slice*x_slice.transpose();

    }

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
  int n_z = 3;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  //transform sigma points into measurement space
  VectorXd x_sigma_pred_slice = VectorXd(n_x_);
  Zsig(0.0);
  for (int nSigmaIndex = 0; nSigmaIndex <  2 * n_aug_ + 1; nSigmaIndex++){
      x_sigma_pred_slice = Xsig_pred_.col(nSigmaIndex);
      float p_x = x_sigma_pred_slice[0];
      float p_y = x_sigma_pred_slice[1];
      float vel = x_sigma_pred_slice[2];
      float psi = x_sigma_pred_slice[3];
      float psi_dot = x_sigma_pred_slice[4];

      float rho = sqrt(p_x*p_x+p_y*p_y);
      float phi = atan(p_y/p_x);
      float rho_dot = vel*(p_x*cos(psi)+p_y*sin(psi))/rho;

      Zsig(0, nSigmaIndex) = rho;
      Zsig(1, nSigmaIndex) = phi;
      Zsig(2, nSigmaIndex) = rho_dot;

      z_pred = z_pred+weights_(nSigmaIndex)*Zsig.col(nSigmaIndex);

  }
  //calculate mean predicted measurement
  //calculate measurement covariance matrix S
  //measurement covariance matrix R
  MatrixXd R = MatrixXd(n_z, n_z);
  R(0.0);
  R(0, 0) = std_radr_ * std_radr_;
  R(1, 1) = std_radphi_ * std_radphi_;
  R(2, 2) = std_radrd_ * std_radrd_;

  //Traverse thru predicted data in measurement space and find
  //the diff between the measured and mean to find co-variance matrix
  S(0.0);
  VectorXd z_sigma_pred_slice = VectorXd(n_z);
  for (int nSigmaIndex = 0; nSigmaIndex <  2 * n_aug_ + 1; nSigmaIndex++){
      z_sigma_pred_slice = Zsig.col(nSigmaIndex) - z_pred;

      //angle normalization;
      //MAke sure the phi is within -PI to PI
      while((z_sigma_pred_slice(1) > M_PI) || (z_sigma_pred_slice(1) < -1*M_PI))
	  {
		if (z_sigma_pred_slice(1) > M_PI){
			z_sigma_pred_slice(1) -= 2*M_PI;
		} else if (z_sigma_pred_slice(1) < -1*M_PI){
			z_sigma_pred_slice(1) += 2*M_PI;
		}
	  }
      //while (z_sigma_pred_slice(1)> M_PI) z_sigma_pred_slice(1)-=2.*M_PI;
      //while (z_sigma_pred_slice(1)<-M_PI) z_sigma_pred_slice(1)+=2.*M_PI;

      S = S + weights_(nSigmaIndex)*z_sigma_pred_slice*z_sigma_pred_slice.transpose();
  }
  S = S + R;

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
      while((z_slice(1) > M_PI) || (z_slice(1) < -1.0*M_PI))
	  {
		if (z_slice(1) > M_PI){
			z_slice(1) -= 2.0*M_PI;
		} else if (z_slice(1) < -1*M_PI){
			z_slice(1) += 2.0*M_PI;
		}
	  }

	  //MAke sure the phi is within -PI to PI
      while((x_slice(3) > M_PI) || (x_slice(3) < -1.0*M_PI))
	  {
		if (x_slice(3) > M_PI){
			x_slice(3) -= 2.0*M_PI;
		} else if (x_slice(3) < -1.0*M_PI){
			x_slice(3) += 2.0*M_PI;
		}
	  }

	  Tc = Tc + weights_(nSigmaIndex)*x_slice*z_slice.transpose();

  }
  //calculate Kalman gain K;
  MatrixXd K = Tc*S.inverse();
  //update state mean and covariance matrix
  VectorXd z_diff = z-z_pred;
  //Make sure the phi is within -PI to PI

  while((z_diff(1) > M_PI) || (z_diff(1) < -1.0*M_PI))
  {
	if (z_diff(1) > M_PI){
		z_diff(1) -= 2.0*M_PI;
	} else if (z_diff(1) < -1.0*M_PI){
		z_diff(1) += 2.0*M_PI;
	}
  }

  //Update the state (x)
  x_ = x_ + K*z_diff;

  //Update Covariance martix P
  P_ = P_ - K*S*K.transpose();
}
