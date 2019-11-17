#pragma once

#include <Eigen/Core>

#include "imu_gps_localizer/base_type.h"
#include "imu_gps_localizer/gps_processor.h"
#include "imu_gps_localizer/imu_processor.h"
#include "imu_gps_localizer/initializer.h"

namespace ImuGpsLocalization {

class ImuGpsLocalizer {
public:
    ImuGpsLocalizer(const Eigen::Vector4d& imu_noise, const Eigen::Vector3d& I_p_Gps);

    bool ProcessImuData(const ImuDataPtr imu_data_ptr, State* fused_state);

    bool ProcessGpsData(const GpsDataPtr gps_data_ptr);

private:
    std::unique_ptr<Initializer> initializer_;

    bool initialized_;
    Eigen::Vector3d init_lla_; // The initial reference gps point.
    State state_;
};

}  // namespace ImuGpsLocalization