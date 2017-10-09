#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

const float MIN_VALUE = 0.001;
const float LEAST_VALUE = 0.1;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size

  if (estimations.size() == 0)
    return rmse;

  if (estimations.size() != ground_truth.size() )
    return rmse;

  VectorXd diff_vec;
  VectorXd rmse_vec;
  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    //take the diff of the prediction vs. ground truth
    diff_vec = estimations[i] - ground_truth[i];
    //Take the sum of square values of the difference
    rmse_vec = diff_vec.array()*diff_vec.array();
    rmse = rmse + rmse_vec;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

// Helper funxtion to set min values
float Tools::SetMinValues(float in) {
	// if (pX < leastValue)
	if (fabs(in) < MIN_VALUE){
		std::cout << "LOG: min value " << in << std::endl;

		in = LEAST_VALUE;
	}
	return in;
}
