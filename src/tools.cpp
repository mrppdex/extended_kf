#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  VectorXd res(4);
  res << 0, 0, 0, 0;

  if(estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    cerr << "Wrong estimations size or length...\n";
    return res;
  }

  for(int i=0; i<estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()  * residual.array();
    res += residual;
  }
  res /= (double)estimations.size();
  res = res.array().sqrt();

  return res;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  MatrixXd Hj = MatrixXd::Zero(3, 4);

  double px = x_state[0];
  double py = x_state[1];
  double vx = x_state[2];
  double vy = x_state[3];

  const double sum_px_py_2 = px*px + py*py;
  const double sum_px_py = sqrt(sum_px_py_2);
  const double sum_px_py_32 = sum_px_py_2 * sum_px_py;

  //check division by zero
	if(fabs(sum_px_py_2) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

  const double a00 = px / sum_px_py;
  const double a01 = py / sum_px_py;
  const double a10 = -py / sum_px_py_2;
  const double a11 = px / sum_px_py_2;
  const double a20 = py * (vx*py - vy*px) / sum_px_py_32;
  const double a21 = px * (vy*px - vx*py) / sum_px_py_32;

  Hj << a00, a01, 0, 0,
        a10, a11, 0, 0,
        a20, a21, a00, a01;
  return Hj;
}

VectorXd Tools::ConvertPolar2Cartesian(const VectorXd &polar_vector)
{
  VectorXd cart_vector(4);
  cart_vector << 0, 0, 0, 0;

  if(polar_vector.size() != 3) {
    std::cerr << "ConvertP2C: wrong polar_vector's shape...\n";
    return cart_vector;
  }

  double rho = polar_vector[0];
  //std::cout << rho << std::endl;
  double phi = polar_vector[1];
  //std::cout << phi << std::endl;
  double rho_dot = polar_vector[2];
  double cos_phi = cos(phi);
  double sin_phi = sin(phi);

  double px = rho*cos_phi;
  double py = rho*sin_phi;
  double vx = rho_dot*cos_phi; // - py*d_phi/1.e6;
  double vy = rho_dot*sin_phi; // + px*d_phi/1.e6;

  cart_vector << px, py, vx, vy;
  return cart_vector;

}

VectorXd Tools::ConvertCartesian2Polar(const VectorXd &cart_vector)
{
  const double THRESH = 0.0001;
  VectorXd polar_vector(3);
  polar_vector << 0, 0, 0;

  if(cart_vector.size() != 4) {
    std::cerr << "ConvertC2P: wrong cart_vector's shape...\n";
    return polar_vector;
  }

  double px = cart_vector[0];
  double py = cart_vector[1];
  double vx = cart_vector[2];
  double vy = cart_vector[3];

  double rho = sqrt(px*px + py*py);
  double phi = atan2(py, px);
  double rho_dot = (rho > THRESH) ? (vx*px + vy*py)/rho: 0.0;

  polar_vector << rho, phi, rho_dot;

  //cout << "ConvertC2P, polar_vector:\n" << polar_vector << endl;

  return polar_vector;
}
