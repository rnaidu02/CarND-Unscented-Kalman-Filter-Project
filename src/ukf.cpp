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
		previous_timestamp_ = meas_package.timestamp_;
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
	float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = meas_package.timestamp_;
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
