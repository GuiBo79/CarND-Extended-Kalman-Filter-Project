#include "kalman_filter.h"
#include <iostream>
#include "tools.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

double PI = 3.14159265359;



// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	int x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {


	double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
	double phi= 0;
	double vel= 0;

	if (fabs(x_(0)) < 0.0001){
		std::cout<<"Error: division by ZERO"<<std::endl;
	} else {
		phi = atan2(x_(1), x_(0));

	}


	if (rho < 0.0001){
			std::cout<<"Error: division by ZERO"<<std::endl;
	} else {
		vel = (x_(0)*x_(2) + x_(1)*x_(3))/rho;

	}

	cout << "Radar EKF x_ = " << x_ << endl;

	cout << "RHO:" << rho << endl;
	cout << "PHI:" << phi << endl;
	cout << "VEL:" << vel << endl;



	VectorXd hx(3);
	hx << rho, phi, vel;
	VectorXd y = z - hx;

	while (y(1) < -PI){
				y(1) += 2 * PI;
	}

	while (y(1) > PI){
		        y(1) -= 2 * PI;
	}

	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	int x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;



}
