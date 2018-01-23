#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  x_ = F_ * x_ ;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd y_ = z - H_ * x_;
  MatrixXd S_ = H_ * P_ * H_.transpose() + R_;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();

  x_ = x_ + K_ * y_ ;

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd hx_ = VectorXd(3);
  double c1 = pow(pow(x_[0],2) + pow(x_[1],2),0.5);
  hx_ << c1, atan2(x_[1],x_[0]), (x_[0]*x_[2] + x_[1]*x_[3])/c1;
  MatrixXd y_ = z - hx_;
  MatrixXd S_ = H_ * P_ * H_.transpose() + R_;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();

  x_ = x_ + K_ * y_ ;

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K_ * H_) * P_;

}
