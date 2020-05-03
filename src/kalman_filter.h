#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include "tools.h"

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(double dt);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

 private:
  void UpdateXP(Eigen::VectorXd &y, Eigen::MatrixXd &H, Eigen::MatrixXd &R);

  // linear state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement covariance matrix
  Eigen::MatrixXd R_LASER_;

  // measurement matrix
  Eigen::MatrixXd H_LASER_;

  // measurement covariance matrix
  Eigen::MatrixXd R_RADAR_;

  double noise_ax_;
  double noise_ay_;
  Tools tools;
};
#endif // KALMAN_FILTER_H_
