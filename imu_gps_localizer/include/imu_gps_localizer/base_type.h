#pragma once

#include <memory>
#include <Eigen/Core>

namespace ImuGpsLocalization {

struct ImuData {
    double timestamp;      // In second.

    Eigen::Vector3d acc;   // Acceleration in m/s^2
    Eigen::Vector3d gyro;  // Angular velocity in radian/s.
};
using ImuDataPtr = std::shared_ptr<ImuData>;

struct GpsData {
    double timestamp;     // In second.
 
    Eigen::Vector3d lla;  // Longitude in radian, latitude in radian, and altitude in meter.
    Eigen::Matrix3d cov;  // Covariance in m^2.
};
using GpsDataPtr = std::shared_ptr<GpsData>;

}  // ImuGpsLocalization