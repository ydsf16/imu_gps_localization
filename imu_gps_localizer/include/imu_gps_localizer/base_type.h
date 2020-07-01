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

struct GpsPositionData {
    double timestamp;     // In second.
 
    Eigen::Vector3d lla;  // Latitude in degree, longitude in degree, and altitude in meter.
    Eigen::Matrix3d cov;  // Covariance in m^2.
};
using GpsPositionDataPtr = std::shared_ptr<GpsPositionData>;

struct State {
    double timestamp;
    
    Eigen::Vector3d lla;       // WGS84 position.
    Eigen::Vector3d G_p_I;     // The original point of the IMU frame in the Global frame.
    Eigen::Vector3d G_v_I;     // The velocity original point of the IMU frame in the Global frame.
    Eigen::Matrix3d G_R_I;     // The rotation from the IMU frame to the Global frame.
    Eigen::Vector3d acc_bias;  // The bias of the acceleration sensor.
    Eigen::Vector3d gyro_bias; // The bias of the gyroscope sensor.

    // Covariance.
    Eigen::Matrix<double, 15, 15> cov;

    // The imu data.
    ImuDataPtr imu_data_ptr; 
};

}  // ImuGpsLocalization