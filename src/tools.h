#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include <math.h>
#include "Eigen/Dense"

#define PI 3.14159265

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
  * A helper method to convert coordinates
  */

  VectorXd ConvertPolar2Cartesian(const VectorXd &polar_vector);
  VectorXd ConvertCartesian2Polar(const VectorXd &cart_vector);

};

#endif /* TOOLS_H_ */