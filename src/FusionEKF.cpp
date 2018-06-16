#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09,   0,    0,
              0, 0.0009,    0,
              0,      0, 0.09;

  //measurement matrix for laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //measurement matrix for radar
  Hj_ <<  0.71, 0.71, 0, 0,
         -0.5, 0.5, 0, 0,
          0, 0, 0.71, 0.71;

  //state transition matrix
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  //object covariance matrix
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 10, 0,
             0, 0, 0, 10;


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  double d_t = (measurement_pack.timestamp_ - previous_timestamp_);

  if (!is_initialized_) {
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      //convert measurement coordinates
      ekf_.x_ = tools.ConvertPolar2Cartesian(measurement_pack.raw_measurements_);
      //set measurement matrix
      ekf_.H_ = Hj_;
      //set measurement covariance matrix
      ekf_.R_ = R_radar_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //initialize first x and y coordinates
      ekf_.x_[0] = measurement_pack.raw_measurements_[0];
      ekf_.x_[1] = measurement_pack.raw_measurements_[1];

      //set measurement matrix
      ekf_.H_ = H_laser_;
      //set measurement covariance matrix
      ekf_.R_ = R_laser_;
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //convert delta t to seconds and predict x' and P'
  ekf_.Predict(d_t/1000000.0);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    VectorXd raw_vec(4);
    //convert coordinates
    raw_vec = tools.ConvertPolar2Cartesian(measurement_pack.raw_measurements_);
    //calculate jacobian for linear approximation
    Hj_ = tools.CalculateJacobian(raw_vec);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    //update x and P
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }
  previous_timestamp_ = measurement_pack.timestamp_;

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
