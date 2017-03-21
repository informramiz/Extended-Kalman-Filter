#include <iostream>
#include "tools.h"
#include "DivisionByZeroException.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    std::cout << "calculateRMSE() - Error - Invalid Estimations vector size" << std::endl;
    return rmse;
  }

  //accumulate squared residuals
  VectorXd sum(4);
  sum << 0, 0, 0, 0;

  for(int i = 0; i < estimations.size(); ++i) {
    VectorXd estimated = estimations[i];
    VectorXd actual = ground_truth[i];

    VectorXd diff = (estimated - actual);
    //coefficient-wise multiplication
    diff = diff.array() * diff.array();
    sum = sum + diff;
  }

  //calculate the mean
  Eigen::VectorXd mean = sum / estimations.size();

  //calculate the squared root
  rmse = mean.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  Eigen::MatrixXd Hj = MatrixXd::Zero(3, 4);

  //state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //divsion by zero check
  if(px == 0 || py == 0 ) {
    throw DivisionByZeroException();
  }

  // calculating common values used in Jacobian
  float px2 = px * px;
  float py2 = py * py;
  float px2_py2_sum = px2 + py2;
  float px2_py2_sqrt = sqrt(px2_py2_sum);
  float px2_py2_sum_3_by_2 = pow((px2_py2_sum), 3/2.0);

  if(fabs(px2_py2_sum) < 0.0001) {
    throw DivisionByZeroException();
  }

  // calculating and inserting jacobian values
  Hj << (px / px2_py2_sqrt), (py / px2_py2_sqrt), 0, 0,
      (-py / px2_py2_sum), (px / px2_py2_sum), 0, 0,
      ((py * (vx * py - vy * px)) / px2_py2_sum_3_by_2), ((px * (vy * px - vx * py)) / px2_py2_sum_3_by_2), (px / px2_py2_sqrt), (py / px2_py2_sqrt);

  return Hj;
}
