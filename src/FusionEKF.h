#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Calculates jacobian matrix Hj, given input @param x_state = (px, py, vx, vy)
   * and output z = (range=rho, angle=phi, range_rate=rho_dot).
   * It is used for linear approximation of non-linear function h(x)
   */
  Eigen::MatrixXd calculateJacobian(const Eigen::VectorXd& x_state);

  /**
   * Non-linear function h(x) that maps cartesian coordinates (px, py, vx, vy)
   * to polar coordinates (range=rho, angle=phi, range_rate=rho_dot)
   */
  Eigen::VectorXd MapToCartesian(const Eigen::VectorXd& z);

  void UpdatePredictionMatrices(long timestamp);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
  float noise_ax_;
  float noise_ay_;
};

#endif /* FusionEKF_H_ */