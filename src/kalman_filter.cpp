#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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

void KalmanFilter::update_common(VectorXd &y) {
	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	MatrixXd k = P_ * H_.transpose() * S.inverse();
	x_ = x_ + k * y;
	P_ = (MatrixXd::Identity(4,4) - k * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd y = z - H_ * x_;
	update_common(y);
}

static VectorXd h(VectorXd x) {
	MatrixXd ret = MatrixXd(3,1);
	float rho = sqrt(pow(x[0],2) + pow(x[1],2));
	ret <<
		rho,
		atan2(x[1],x[0]),
		(x[0] * x[2] + x[1] * x[3]) / rho;
	return ret;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	VectorXd y = z - h(x_);
	y[1] = atan2(sin(y[1]),cos(y[1]));
	update_common(y);
}
