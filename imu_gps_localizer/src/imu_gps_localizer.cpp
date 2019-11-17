#include "imu_gps_localizer/imu_gps_localizer.h"

#include <glog/logging.h>

#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

ImuGpsLocalizer::ImuGpsLocalizer(const Eigen::Vector4d& imu_noise, const Eigen::Vector3d& I_p_Gps) 
    : initialized_(false){
    initializer_ = std::make_unique<Initializer>(I_p_Gps);
}

bool ImuGpsLocalizer::ProcessImuData(const ImuDataPtr imu_data_ptr, State* fused_state) {
    if (!initialized_) {
        initializer_->AddImuData(imu_data_ptr);
        return false;
    }

    return true;
}

bool ImuGpsLocalizer::ProcessGpsData(const GpsDataPtr gps_data_ptr) {
    if (!initialized_) {
        if (!initializer_->AddGpsData(gps_data_ptr, &state_)) {
            return false;
        }

        // Initialize the initial gps point used to convert lla to ENU.
        init_lla_ = gps_data_ptr->lla;
        
        initialized_ = true;

        LOG(INFO) << "[ProcessGpsData]: System initialized!";
        return true;
    }

    return true;
}

}  // namespace ImuGpsLocalization