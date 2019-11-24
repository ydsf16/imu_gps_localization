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
    gps_processor_ = std::make_unique<GpsProcessor>(I_p_Gps);
}

bool ImuGpsLocalizer::ProcessImuData(const ImuDataPtr imu_data_ptr, State* fused_state) {
    if (!initialized_) {
        initializer_->AddImuData(imu_data_ptr);
        return false;
    }
    
    // Predict.
    imu_processor_->Predict(latest_state_.imu_data_ptr, imu_data_ptr, &latest_state_);

    // Convert ENU state to lla.
    ConvertENUToLLA(init_lla_, latest_state_.G_p_I, &(latest_state_.lla));
    *fused_state = latest_state_;

    // Save state to buffer.
    state_buffer_.push_back(latest_state_);
    if (state_buffer_.size() > kStateBufferLength) {
        state_buffer_.pop_front();
    }

    return true;
}

bool ImuGpsLocalizer::ProcessGpsPositionData(const GpsPositionDataPtr gps_data_ptr) {
    if (!initialized_) {
        if (!initializer_->AddGpsPositionData(gps_data_ptr, &latest_state_)) {
            return false;
        }

        // Initialize the initial gps point used to convert lla to ENU.
        init_lla_ = gps_data_ptr->lla;
        
        initialized_ = true;

        LOG(INFO) << "[ProcessGpsPositionData]: System initialized!";
        return true;
    }

    if (state_buffer_.size() < 2) {
        LOG(WARNING) << "[ProcessGpsPositionData]: No enought states in state buffer.";
        return false;
    }

    gps_data_buffer_.push_back(gps_data_ptr);
    if (gps_data_buffer_.size() > kGpsBufferLength) {
        gps_data_buffer_.pop_front();
    }

    while (!gps_data_buffer_.empty()) {
        const auto cur_gps_data_ptr = gps_data_buffer_.front();
        
        // Check whether the gps_data is valid.
        if (gps_data_ptr->timestamp <= state_buffer_.front().timestamp) {
            LOG(WARNING) << "[ProcessGpsPositionData]: Too old gps data!";
            gps_data_buffer_.pop_front();
            continue;
        }
        
        if (cur_gps_data_ptr->timestamp >= state_buffer_.back().timestamp) {
            LOG(WARNING) << "[ProcessGpsPositionData]: Too new gps data!";
            break;
        }

        // Interpolate an imu with gps time.
        int af_idx = 1;
        while (state_buffer_[af_idx].timestamp < cur_gps_data_ptr->timestamp) {
            ++af_idx;
        }

        const auto& bf_state = state_buffer_[af_idx - 1];
        auto& af_state = state_buffer_[af_idx];
        
        const double bf_time = cur_gps_data_ptr->timestamp - bf_state.timestamp;
        const double af_time = af_state.timestamp - cur_gps_data_ptr->timestamp;
        const double time_interval = af_state.timestamp - bf_state.timestamp;

        const ImuDataPtr imu_inter = std::make_shared<ImuData>();
        imu_inter->timestamp = cur_gps_data_ptr->timestamp;
        imu_inter->acc = (bf_time * af_state.imu_data_ptr->acc + af_time * bf_state.imu_data_ptr->acc) / time_interval;
        imu_inter->gyro = (bf_time * af_state.imu_data_ptr->gyro + af_time * bf_state.imu_data_ptr->gyro) / time_interval;

        // Predict from before state to the interpolate state.
        State state_inter = bf_state;
        imu_processor_->Predict(bf_state.imu_data_ptr, imu_inter, &state_inter);

        // Update by gps measurement.
        gps_processor_->UpdateStateByGpsPosition(init_lla_, cur_gps_data_ptr, &state_inter);

        // Predict from the interpolate state to the after state.
        const auto af_imu = af_state.imu_data_ptr;
        af_state = state_inter;
        imu_processor_->Predict(af_state.imu_data_ptr, af_imu, &af_state);

        // Predict from the after state to the latest state.
        for (size_t i = af_idx + 1; i < state_buffer_.size(); ++i) {
            auto& cur_state = state_buffer_[i];
            const auto& last_state = state_buffer_[i - 1];
            const auto cur_imu = cur_state.imu_data_ptr;
            cur_state = last_state;
            imu_processor_->Predict(cur_state.imu_data_ptr, cur_imu, &cur_state);
        } 
        
        // Update the latest state.
        latest_state_ = state_buffer_.back();
        gps_data_buffer_.pop_front();
    }

    return true;
}

}  // namespace ImuGpsLocalization