#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
	VectorXd x = VectorXd(4);
	x << 0,0,0,0;
	MatrixXd P = MatrixXd(4,4);
	P << 1, 0, 0, 0,
	     0, 1, 0, 0,
	     0, 0, 1000, 0,
	     0, 0, 0, 1000;
	MatrixXd F = MatrixXd(4,4);
	F << 1, 0, 1, 0,
	     0, 1, 0, 1,
	     0, 0, 1, 0,
	     0, 0, 0, 1;
	H_laser_ <<
		1, 0, 0, 0,
		0, 1, 0, 0;
	MatrixXd Q = MatrixXd::Zero(4,4);
	ekf_.Init(x,P,F,H_laser_,R_laser_,Q);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  VectorXd z = measurement_pack.raw_measurements_;
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
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
	float px, py, vx, vy, cz = cos(z[1]), sz = sin(z[1]);
	px = cz * z[0];
	py = sz * z[0];
	vx = cz * z[2];
	vy = sz * z[2];
	ekf_.x_ << px,py,vx,vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
	ekf_.x_ << z[0],z[1],0,0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.;
  previous_timestamp_ = measurement_pack.timestamp_;
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  ekf_.Q_ <<
	pow(dt,4)*9./4.,	0,		pow(dt,3)*9./2.,	0,
	0,			pow(dt,4)*9./4.,0,			pow(dt,3)*9./2.,
	pow(dt,3)*9./2,		0,		pow(dt,2)*9.,		0,
	0,			pow(dt,3)*9./2, 0,			pow(dt,2)*9.;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    float px = ekf_.x_[0], py = ekf_.x_[1], vx=ekf_.x_[2], vy=ekf_.x_[3];
    float d = pow(px,2) + pow(py,2), d12 = sqrt(d), d32 = pow(d,1.5);
    if (d < 0.0001) {
        std:cout << "div by zero" << std::endl;
        return;
    }
    Hj_ <<
        px/d12, py/d12, 0, 0,
        -py/d, px/d, 0, 0,
        py*(vx*py-vy*px)/d32, px*(vy*px-vx*py)/d32, px/d12, py/d12;
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(z);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
