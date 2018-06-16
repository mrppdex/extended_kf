#include <iostream>
#include "tools.h"

#define PI 3.14159265

int main()
{
  Tools tools;

  /*
	 * Compute RMSE
	 */
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	//the input list of estimations
	VectorXd e(4);
	e << 1, 1, 0.2, 0.1;
	estimations.push_back(e);
	e << 2, 2, 0.3, 0.2;
	estimations.push_back(e);
	e << 3, 3, 0.4, 0.3;
	estimations.push_back(e);

	//the corresponding list of ground truth values
	VectorXd g(4);
	g << 1.1, 1.1, 0.3, 0.2;
	ground_truth.push_back(g);
	g << 2.1, 2.1, 0.4, 0.3;
	ground_truth.push_back(g);
	g << 3.1, 3.1, 0.5, 0.4;
	ground_truth.push_back(g);

	//call the CalculateRMSE and print out the result
  std:;cout << "Testing: CalculateRMSE\n";
	std::cout << tools.CalculateRMSE(estimations, ground_truth) << std::endl;

  VectorXd measurements(4);
  measurements << 1.0, 0.0, 2.0, 3.0;
  std::cout << "Testing: CalculateJacobian\n";
  std::cout << tools.CalculateJacobian(measurements) << std::endl;

  vector<VectorXd> vec_pol;

  VectorXd vp(3);
  vp << 1, (PI)/4, 0;
  vec_pol.push_back(vp);
  vp << 1, (3*(PI)/4), 0;
  vec_pol.push_back(vp);
  vp << 1, (-3*(PI)/4), 0;
  vec_pol.push_back(vp);
  vp << 1, -(PI)/4, 0;
  vec_pol.push_back(vp);

  vector<VectorXd>::iterator v_it;
  for(v_it = vec_pol.begin(); v_it != vec_pol.end(); ++v_it) {
    VectorXd vec_cart = tools.ConvertP2C(*v_it, 0.1);
    std::cout << "Testing: Conversion to cartesian: \n";
    std::cout << "Original:\n" << *v_it << "\n";
    std::cout << "Transformed:\n" << vec_cart << "\n";
  }

  /*
  vec_cart << 1, 1, 0, 0;
  vec_polar = tools.ConvertC2P(vec_cart);
  std::cout << "Testing: Conversion to polar: \n" << vec_polar << "\n";
  */

  return 0;
}
