#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  	VectorXd rmse(4);
	rmse << 0.0,0.0,0.0,0.0;


	if (estimations.size() != ground_truth.size() || estimations.size() ==0){
	    cout << "Invalid Data";
	    return rmse;
	}

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
		VectorXd residual = estimations[i]-ground_truth[i];
		residual = residual.array()*residual.array();
		rmse += residual;

	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	cout << "RMSE:" << rmse << endl;


	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {


	MatrixXd Hj(3,4);
	Hj << 0,0,0,0,
		  0,0,0,0,
		  0,0,0,0;
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	float px2py2 = px*px + py*py;
	float px3py2 = sqrt(px2py2)*px2py2;

	if(fabs(px2py2) < 0.0001){
			cout << "CalculateJacobian () - Error - Division by Zero" << endl;
			return Hj;
	}


	Hj << px/(sqrt(px2py2)),py/(sqrt(px2py2)),0,0,
	     -py/(px2py2),px/(px2py2),0,0,
	      py*(vx*py-vy*px)/(px3py2),px*(vy*px-vx*py)/(px3py2),px/(sqrt(px2py2)),py/(sqrt(px2py2));


	return Hj;


}
