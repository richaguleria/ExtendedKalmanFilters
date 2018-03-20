#include <iostream>
#include "FusionEKF.h"
#include "kalman_filter.h"
using namespace std;
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
  cout << "Predict" << endl;

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  cout << "Update" << endl;

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
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

VectorXd KalmanFilter::CalculatePolarCordinates(const VectorXd &x_state) {
  //recover state parameters
  
  cout << "CalculatePolarCordinates" << endl;

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  float rho = sqrt(px*px+py*py);
  float theta = atan2(py,px); 
  float rho_dot;

  if (fabs(rho) < 0.0001) {
    rho_dot = 0;
  } else {
    rho_dot = (px*vx + py*vy)/rho;
  }

  VectorXd h(3);
  h << rho,theta,rho_dot;

  return h;
  
}
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  cout << "UpdateEKF" << endl;

  VectorXd z_pred = CalculatePolarCordinates(x_);
  VectorXd y = z - z_pred;

  if ( -(PI) > y(1) )
  {
    y(1) += 2 * PI;
  }
  else if (PI < y(1))
  {
    y(1) -= 2 * PI;
  }

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
