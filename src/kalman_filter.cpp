#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
  // filter state
  x_ = VectorXd(4);
  P_ = MatrixXd(4, 4);
  // reasonable initialization (hopefully)
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  // linear prediction init
  F_ = MatrixXd(4, 4);
  Q_ = MatrixXd(4, 4);
  noise_ax_ = 9.;
  noise_ay_ = 9.;

  // linear update - lidar
  // covariance
  R_LASER_ = MatrixXd(2, 2);
  R_LASER_ << 0.0225, 0,
              0, 0.0225;
  H_LASER_ = MatrixXd(2, 4);
  H_LASER_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // non-linear update - radar
  // covariance
  R_RADAR_ = MatrixXd(3, 3);
  R_RADAR_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
}

KalmanFilter::~KalmanFilter() {}

// Linear prediction function
void KalmanFilter::Predict(double dt) {
  F_ << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;
  double dt_2 = dt * dt; //dt^2
  double dt_3 = dt_2 * dt; //dt^3
  double dt_4 = dt_3 * dt; //dt^4
  double dt_4_4 = dt_4 / 4; //dt^4/4
  double dt_3_2 = dt_3 / 2; //dt^3/2
  Q_ << dt_4_4 * noise_ax_, 0, dt_3_2 * noise_ax_, 0,
        0, dt_4_4 * noise_ay_, 0, dt_3_2 * noise_ay_,
        dt_3_2 * noise_ax_, 0, dt_2 * noise_ax_, 0,
        0, dt_3_2 * noise_ay_, 0, dt_2 * noise_ay_;
  MatrixXd Ft = F_.transpose();
  x_ = F_ * x_;
  P_ = F_ * P_ * Ft + Q_;
}

// Linear sensor update
void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_LASER_ * x_;
  UpdateXP(y, H_LASER_, R_LASER_);
}

// Non-linear sensor update
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];
  // skip this update to avoid division by zero
  if( px == 0. && py == 0. )
    return;
  VectorXd hofx(3);
  double rho = sqrt(px * px + py * py);
  hofx << rho, atan2(py, px), (px* vx + py * vy) / rho;
  VectorXd y = z - hofx;
  if(y[1] > M_PI)
    y[1] -= 2.f * M_PI;
  if(y[1] < -M_PI)
    y[1] += 2.f * M_PI;
  MatrixXd H = tools.CalculateJacobian(x_);
  UpdateXP(y, H, R_RADAR_);
}

void KalmanFilter::UpdateXP(VectorXd &y, MatrixXd &H, MatrixXd &R){
  MatrixXd Ht = H.transpose();
  MatrixXd S = H*P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
  x_ = x_ + (K * y);
  P_ = (Eigen::MatrixXd::Identity(4, 4) - K * H) * P_;
}
