#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <vector>
#include <string>
#include <fstream>
#include "Eigen/Dense"
#include "measurement_package.h"
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
  void ProcessMeasurement(const MeasurementPackage& measurement_pack);

  Eigen::VectorXd GetEstimations() const;

private:
  /**
   * Given the @param timestamp of new measurement it calculates
   * time differnece delta_t in seconds. It then uses that time difference to
   * update state transition matrix F and process covariance matrix Q
   */
  void UpdatePredictionMatrices(long timestamp);

  /**
   * Non-linear function that maps cartesian coordinates @param z =(px, py, vx, vy)
   * to polar coordinates (range=rho, angle=phi, range_rate=rho_dot)
   */
  Eigen::VectorXd MapToCartesian(const Eigen::VectorXd& z);

  // check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long previous_timestamp_;

  /**
   * Kalman Filter update and prediction math lives in here.
   */
  KalmanFilter ekf_;

  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
  float noise_ax_;
  float noise_ay_;
};

#endif /* FusionEKF_H_ */
