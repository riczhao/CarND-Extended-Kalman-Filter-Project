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
std::cout << "F:\n" << F_ << "\nx:\n" << x_ << std::endl;
	x_ = F_ * x_;
std::cout << "F2:\n" << F_ << "\nx2:\n" << x_ << std::endl;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update_common(VectorXd &y) {
	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	MatrixXd k = P_ * H_.transpose() * S.inverse();
std::cout << "update_common:\n"<<"\ny:\n"<<y<<"\nk:\n" << k << "\nH:\n" << H_ << "\nx:\n" << x_ <<std::endl; 
	x_ = x_ + k * y;
std::cout << "\nx_out:\n" << x_ <<std::endl; 
	P_ = (MatrixXd::Identity(4,4) - k * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	printf("%s:%d\n", __func__,__LINE__);
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
	std::cout << "h:\n"<<ret<<std::endl;
	return ret;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	VectorXd y = z - h(x_);
	y[1] = atan2(sin(y[1]),cos(y[1]));
	std::cout << "z:\n"<<z<<std::endl;
	update_common(y);
std::cout << "h_out:\n" << h(x_) <<std::endl; 
}
