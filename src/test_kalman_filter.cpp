#include "kalman_filter.h"

int main()
{
  // state vector
  Eigen::VectorXd x_(4);
  x_ << 1, 1, 1, 1;

  // process covariance matrix
  Eigen::MatrixXd Q_(4, 4);
  Q_ << 0.000225, 0, 0.0045, 0,
        0, 0.000225, 0, 0.0045,
        0.0045, 0, 0.09, 0,
        0, 0.0045, 0, 0.09;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

  // initializing matrices
  Eigen::MatrixXd R_laser_ = MatrixXd(2, 2);
  Eigen::MatrixXd R_radar_ = MatrixXd(3, 3);
  Eigen::MatrixXd H_laser_ = MatrixXd(2, 4);
  Eigen::MatrixXd Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  Hj_ <<  0.7, 0.7, 0.0, 0.0,
         -0.5, 0.5, 0.0, 0.0,
          0.0, 0.0, 0.7, 0.7;

  //LASER

  H_ = H_laser_;
  R_ = R_laser_;

  return 0;
}
