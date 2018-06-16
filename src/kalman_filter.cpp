#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
  //cout << "KalmanFilter()";


  P_ = MatrixXd (4, 4);
  P_ << 1,  0,  0,  0,
         0, 1,  0,  0,
         0,  0, 1000,  0,
         0,  0,  0, 1000;

  I_ = MatrixXd::Identity(4, 4);
  Q_ = MatrixXd::Zero(4, 4);
}

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

void KalmanFilter::UpdateQ(const double dt) {

  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;

  const double q00 = dt4 * noise_ax / 4;
  const double q02 = dt3 * noise_ax / 2;
  const double q11 = dt4 * noise_ay / 4;
  const double q13 = dt3 * noise_ay / 2;
  const double q20 = dt3 * noise_ax / 2;
  const double q22 = dt2 * noise_ax;
  const double q31 = dt3 * noise_ay / 2;
  const double q33 = dt2 * noise_ay;

  Q_      << q00, 0.0, q02, 0.0,
             0.0, q11, 0.0, q13,
             q20, 0.0, q22, 0.0,
             0.0, q31, 0.0, q33;

}

void KalmanFilter::Predict(const double delta_t) {
  /**
  TODO:
    * predict the state
  */
  // update delta t coefficients in state transition matrix
  F_(0, 2) = delta_t;
  F_(1, 3) = delta_t;

  // update process covariance matrix
  UpdateQ(delta_t);

#if DEBUG
  cout << "........PREDICT........................\n";
  cout << "F_=\n" << F_ << endl;
  cout << "Q_=\n" << Q_ << endl;
  cout << ".......................................\n";
#endif

  // approximate state vector
  x_ = F_*x_;

  // calculate approximation covariance matrix
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  // calculate approximation error
  VectorXd y = z - H_*x_;

  // calculate Kalman gain
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd S_inv = S.inverse();
  MatrixXd K = P_*Ht*S_inv;

  // update state vector and covariance matrix
  x_ = x_ + K*y;
  P_ = (I_ - K*H_)*P_;

#if DEBUG
  cout << "--------------UPDATE        -------------\n";
  cout << "x_=\n" << x_ << endl;
  cout << "z=\n" << z << endl;
  cout << "H_=\n" << H_ << endl;
  cout << "y = z - H_*x_;\ny=\n" << y << endl;
  cout << "S = H_*P_*H_.transpose() + R_;\n";
  cout << "P_=\n" << P_ << endl;
  cout << "S =\n" << S << endl;
  cout << "R_ =\n" << R_ << endl;
  cout << "K = P_*H_.transpose()*S.inverse();\n";
  cout << "K=\n" << K << endl;
  cout << "x_ = x_ + K*y;\n";
  cout << "x_=\n" << x_ << endl;
  cout << "P_ = (I_ - K*H_)*P_;\n";
  cout << "P_=\n" << P_ << endl;
  cout << "-----------------UPDATE END---------------------\n";
#endif

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // converts x_k-1 to polar coordinates
  VectorXd h_x_prime = tools.ConvertCartesian2Polar(x_);

  // approximation error
  VectorXd y = z - h_x_prime;

  // converts to range -pi..pi
  y[1] = atan2(sin(y[1]), cos(y[1]));

  // calculate Kalman gain
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd Sinv = S.inverse();
  MatrixXd K = P_*Ht*Sinv;

  // update state vector and covariance matrix
  x_ = x_ + K*y;
  P_ = (I_ - K*H_)*P_;

#if DEBUG
  cout << "--------UPDATE EKF----------\n";
  cout << "x=\n" << x_ << endl;
  cout << "h'(x)=\n" << h_x_prime<< endl;
  cout << "z=\n" << z << endl;
  cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
  cout << "y = z - h'(x)\n" << y << endl;
  cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
  cout << "H_=\n" << H_ << endl;
  cout << "P_=\n" << P_ << endl;
  cout << "R_=\n" << R_ << endl;
  cout << "##########################################\n";
  cout << "S = H_*P*H_.transpose() + R;\n" << S << endl;
  cout << "K = P_*H_.transpose()*S.inverse();\n" << K << endl;
  cout << "x_ = x_ + K*y;\n" << x_ << endl;
  cout << "P_ = (I_ - K*H_)*P_;\n" << P_ << endl;
  cout << "--------------UPDATE EKF END-------------\n";
#endif


}
