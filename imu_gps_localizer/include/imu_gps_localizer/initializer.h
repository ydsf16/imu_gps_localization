#pragma once

#include <deque>

#include "imu_gps_localizer/base_type.h"

namespace ImuGpsLocalization {

constexpr int kImuDataBufferLength = 100;
constexpr int kAccStdLimit         = 3.;

class Initializer {
public:
    Initializer(const Eigen::Vector3d& init_I_p_Gps);
    
    void AddImuData(const ImuDataPtr imu_data_ptr);

    bool AddGpsPositionData(const GpsPositionDataPtr gps_data_ptr, State* state);

private:
    bool ComputeG_R_IFromImuData(Eigen::Matrix3d* G_R_I);

    Eigen::Vector3d init_I_p_Gps_;
    std::deque<ImuDataPtr> imu_buffer_;
};

}  // namespace ImuGpsLocalization