#include "imu_gps_localizer/initializer.h"

#include <Eigen/Dense>
#include <glog/logging.h>

#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

Initializer::Initializer(const Eigen::Vector3d& init_I_p_Gps) 
    : init_I_p_Gps_(init_I_p_Gps), latest_gps_vel_data_(nullptr) { }

void Initializer::AddImuData(const ImuDataPtr imu_data_ptr) {
    imu_buffer_.push_back(imu_data_ptr);

    if (imu_buffer_.size() > kImuDataBufferLength) {
        imu_buffer_.pop_front();
    }
}

bool Initializer::AddGpsPositionData(const GpsPositionDataPtr gps_data_ptr, State* state) {
    if (imu_buffer_.size() < kImuDataBufferLength) {
        LOG(WARNING) << "[AddGpsPositionData]: No enought imu data!";
        return false;
    }

    if (latest_gps_vel_data_ == nullptr) {
        LOG(WARNING) << "[AddGpsPositionData]: No gps velocity data!";
        return false;
    }

    const ImuDataPtr last_imu_ptr = imu_buffer_.back();
    // TODO: synchronize all sensors.
    if (std::abs(gps_data_ptr->timestamp - last_imu_ptr->timestamp) > 0.5 ||
        std::abs(gps_data_ptr->timestamp - latest_gps_vel_data_->timestamp) > 0.5) {
        LOG(ERROR) << "[AddGpsPositionData]: Gps and imu timestamps are not synchronized!";
        return false;
    }

    // Set timestamp and imu date.
    state->timestamp = last_imu_ptr->timestamp;
    state->imu_data_ptr = last_imu_ptr;

    // Set initial mean.
    state->G_p_I.setZero();
    // We have no information to set initial velocity. 
    // So, just set it to zero and given big covariance.
    state->G_v_I.setZero();
    // We can use the direction of gravity to set roll and pitch. 
    // But, we cannot set the yaw. 
    // So, we set yaw to zero and give it a big covariance.
    if (!ComputeG_R_IFromImuData(&state->G_R_I)) {
        LOG(WARNING) << "[AddGpsPositionData]: Failed to compute G_R_I!";
        return false;
    }

    const double yaw = std::atan2(latest_gps_vel_data_->vel(1), latest_gps_vel_data_->vel(0));
    state->G_R_I = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * state->G_R_I.eval();

    // Set bias to zero.
    state->acc_bias.setZero();
    state->gyro_bias.setZero();

    // Set covariance.
    state->cov.setZero();
    state->cov.block<3, 3>(0, 0) = 100. * Eigen::Matrix3d::Identity(); // position std: 10 m
    state->cov.block<3, 3>(3, 3) = 100. * Eigen::Matrix3d::Identity(); // velocity std: 10 m/s
    // roll pitch std 10 degree.
    state->cov.block<2, 2>(6, 6) = 10. * kDegreeToRadian * 10. * kDegreeToRadian * Eigen::Matrix2d::Identity();
    state->cov(8, 8)             = 100. * kDegreeToRadian * 100. * kDegreeToRadian; // yaw std: 100 degree.
    // Acc bias.
    state->cov.block<3, 3>(9, 9) = 0.0004 * Eigen::Matrix3d::Identity();
    // Gyro bias.
    state->cov.block<3, 3>(12, 12) = 0.0004 * Eigen::Matrix3d::Identity();

    return true;
}

void Initializer::AddGpsVelocityData(const GpsVelocityDataPtr gps_vel_ptr) {
    latest_gps_vel_data_ = gps_vel_ptr;
}

bool Initializer::ComputeG_R_IFromImuData(Eigen::Matrix3d* G_R_I) {
    // Compute mean and std of the imu buffer.
    Eigen::Vector3d sum_acc(0., 0., 0.);
    for (const auto imu_data : imu_buffer_) {
        sum_acc += imu_data->acc;
    }
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer_.size();

    Eigen::Vector3d sum_err2(0., 0., 0.);
    for (const auto imu_data : imu_buffer_) {
        sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
    }
    const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buffer_.size()).cwiseSqrt();

    if (std_acc.maxCoeff() > kAccStdLimit) {
        LOG(WARNING) << "[ComputeG_R_IFromImuData]: Too big acc std: " << std_acc.transpose();
        return false;
    }

    // Compute rotation.
    const Eigen::Vector3d acc_norm = mean_acc.normalized();  
    const double rot_y = std::atan2(acc_norm(0), acc_norm(2));
    double rot_x = std::atan2(std::abs(acc_norm(1)), std::sqrt(acc_norm(0) * acc_norm(0) + acc_norm(2) * acc_norm(2)));
    if (-std::sin(rot_x) * acc_norm(1) < 0.) {
        rot_x = -rot_x;
    }

    Eigen::AngleAxisd RotX(rot_x, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd RotY(rot_y, Eigen::Vector3d::UnitY());

    *G_R_I = (RotY * RotX).toRotationMatrix().transpose();

    return true;
}

}  // namespace ImuGpsLocalization