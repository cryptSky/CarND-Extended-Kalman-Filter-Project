#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

VectorXd h(const VectorXd& x);
double adjust_angle(double phi);

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
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  UpdateY(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd z_pred = h(x_);
  VectorXd y = z - z_pred;
  y(1) = adjust_angle(y(1));

  UpdateY(y);
}

void KalmanFilter::UpdateY(const VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

VectorXd h(const VectorXd& x) {
  auto h0 = sqrt(x(0)*x(0)+x(1)*x(1));
  auto h1 = atan2(x(1), x(0));
  auto h2 = (x(0)*x(2) + x(1)*x(3)) / h0;

  VectorXd res(3);
  res << h0, h1, h2;

  return res;
}

double adjust_angle(double phi) {
  if (phi > M_PI) {
    auto div = phi/(2*M_PI);  
    auto quotient = floor(phi/(2*M_PI));
    auto remainder = phi * (div - quotient);
    phi = remainder;
  } else if (phi < -M_PI) {
    auto div = -phi/(2*M_PI);
    auto quotient = floor(phi/(2*M_PI));
    auto remainder = phi *(div-quotient);
    phi = -remainder;
  }

  return phi;
}
