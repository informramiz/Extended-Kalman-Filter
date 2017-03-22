#include <iostream>
#include <cmath>
#include "kalman_filter.h"

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
  // state prediction
  x_ = F_ * x_;
  // covariance/uncertainty prediction
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  //predict what next measurement z should be
  VectorXd z_prediction = H_ * x_;
  //calculate the difference between
  // predicted and actual measurement
  VectorXd y = z - z_prediction;

  // Calculate Kalman Game
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // update the state based on measurement
  x_ = x_ + K * y;
  // update the state covariance/uncertainty based on measurement
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //predict what next measurement z should be
  VectorXd z_prediction = MapToPolar(x_);

  //calculate the difference between
  //predicted and actual measurement
  VectorXd y = z - z_prediction;

  // Calculate Kalman Game
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // update the state based on measurement
  x_ = x_ + K * y;
  // update the state covariance/uncertainty based on measurement
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}

Eigen::VectorXd KalmanFilter::MapToPolar(const Eigen::VectorXd& x) {
  Eigen::VectorXd z_predicted(3);

  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);

  float px2_py2_sum = px * px + py * py;
  float px2_py2_sum_sqrt = sqrt(px2_py2_sum);

  z_predicted << px2_py2_sum_sqrt, atan2(py, px), ((px * vx + py * vy) / px2_py2_sum_sqrt);
  return z_predicted;
}
