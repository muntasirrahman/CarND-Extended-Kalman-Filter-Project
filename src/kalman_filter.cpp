#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

#define MIN_VALUE 0.0001

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
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - (H_ * x_);
  RecalcStateVectorAndCovariance(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  // convert cartesian to polar coordinates
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double rho = sqrt(px * px + py * py);
  if (rho < MIN_VALUE) rho = MIN_VALUE;
  double rho_dot = (px * vx + py * vy) / rho;;
  double phi = atan2(py, px);

  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;

  // adjust polar degree into valid phi range
  double phi_diff = y(1);
  if (phi_diff > M_PI) y(1) = fmod(phi_diff, M_PI);
  if (phi_diff < -M_PI) y(1) = fmod(phi_diff, -M_PI);

  RecalcStateVectorAndCovariance(y);
}

void KalmanFilter::RecalcStateVectorAndCovariance(const VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd kalman_gain = P_ * Ht * S.inverse();

  // prediction
  x_ = x_ + (kalman_gain * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - kalman_gain * H_) * P_;
}
