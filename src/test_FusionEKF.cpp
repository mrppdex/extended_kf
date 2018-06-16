#include "FusionEKF.h"
#include <ctime>

int main()
{
  FusionEKF fekf;
  MeasurementPackage package_laser;
  MeasurementPackage package_radar;

  Eigen::Vector2d vec_laser(1.0, 0.5);

  package_laser.sensor_type_ = MeasurementPackage::LASER;
  package_laser.raw_measurements_ = vec_laser;
  package_laser.timestamp_ = std::time(nullptr);

  fekf.ProcessMeasurement(package_laser);

  return 0;
}
