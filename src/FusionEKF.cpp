#include "FusionEKF.h"
#include <iostream>

FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    // first measurement
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
      double rho = measurement_pack.raw_measurements_[0]; // range
      double phi = measurement_pack.raw_measurements_[1]; // bearing
      double rho_dot = measurement_pack.raw_measurements_[2]; // rho'(t)
      double x = rho * cos(phi);
      if ( x < 0.0001 ) {
        x = 0.0001;
      }
      double y = rho * sin(phi);
      if ( y < 0.0001 ) {
        y = 0.0001;
      }
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);
      ekf_.x_ << x, y, vx, vy;
      previous_timestamp_ = measurement_pack.timestamp_;
      is_initialized_ = true;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      previous_timestamp_ = measurement_pack.timestamp_;
      is_initialized_ = true;
    }
    // if  neither of the above conditions are met, the EKF will remain uninitialized
    return;
  }

  /**
   * Prediction
   */
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  // bad synchronization or latency may result in measurement packages with out of order timestamps. A standard Kalman filter doesn't revise past state.
  if (dt < 0.0) {
    std::cerr << "measurement package dropped: " << measurement_pack.sensor_type_ << std::endl;
    return;
  }
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.Predict(dt);
  /**
   * Update
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  std::cout << "updated" << std::endl;
  std::cout << "x_ = " << ekf_.x_ << std::endl;
  std::cout << "P_ = " << ekf_.P_ << std::endl;
}
