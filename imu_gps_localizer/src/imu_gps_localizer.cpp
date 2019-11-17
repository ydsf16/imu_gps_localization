#include "imu_gps_localizer/imu_gps_localizer.h"

#include <glog/logging.h>

#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

ImuGpsLocalizer::ImuGpsLocalizer(const double acc_noise, const double gyro_noise,
                                 const double acc_bias_noise, const double gyro_bias_noise,
                                 const Eigen::Vector3d& I_p_Gps) 
    : initialized_(false){
    initializer_ = std::make_unique<Initializer>(I_p_Gps);
    imu_processor_ = std::make_unique<ImuProcessor>(acc_noise, gyro_noise, 
                                                    acc_bias_noise, gyro_bias_noise,
                                                    Eigen::Vector3d(0., 0., -9.81007));
}

bool ImuGpsLocalizer::ProcessImuData(const ImuDataPtr imu_data_ptr, State* fused_state) {
    if (!initialized_) {
        initializer_->AddImuData(imu_data_ptr);
        return false;
    }

    imu_processor_->Predict(state_.imu_data_ptr, imu_data_ptr, &state_);

    LOG(INFO) << "G_p_I: " << state_.G_p_I.transpose();
    LOG(INFO) << "G_v_I: " << state_.G_v_I.transpose();
    LOG(INFO) << "G_R_I: \n" << state_.G_R_I;
    LOG(INFO) << "Acc acc_bias: " << state_.acc_bias.transpose();
    LOG(INFO) << "gyro_bias: " << state_.gyro_bias.transpose();
    LOG(INFO) << "I_p_Gps: " << state_.I_p_Gps.transpose();
    LOG(INFO) << "Cov: \n" << state_.cov;
    LOG(INFO) << "\n\n\n";

    // TODO: Convert ENU state to lla.
    
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