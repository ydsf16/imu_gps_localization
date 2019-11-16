#include "localization_wrapper.h"

#include <glog/logging.h>

#include "imu_gps_localizer/base_type.h"

LocalizationWrapper::LocalizationWrapper(ros::NodeHandle& nh) {
    // Initialization imu gps localizer.
    imu_gps_localizer_ptr_ = std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>();

    // Subscribe topics.
    imu_sub_ = nh.subscribe("/imu/data", 10,  &LocalizationWrapper::ImuCallback, this);
    gps_sub_ = nh.subscribe("/fix", 10,  &LocalizationWrapper::GpsCallBack, this);
}

void LocalizationWrapper::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    ImuGpsLocalization::ImuDataPtr imu_data_ptr = std::make_shared<ImuGpsLocalization::ImuData>();
    imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x, 
                         imu_msg_ptr->linear_acceleration.y,
                         imu_msg_ptr->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
                          imu_msg_ptr->angular_velocity.y,
                          imu_msg_ptr->angular_velocity.z;
    
    LOG(INFO) << "[ImuCallback]: Imu data: " << std::fixed << imu_data_ptr->timestamp;
}

void LocalizationWrapper::GpsCallBack(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr) {
    ImuGpsLocalization::GpsDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsData>();
    gps_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
    gps_data_ptr->lla << gps_msg_ptr->longitude,
                         gps_msg_ptr->latitude,
                         gps_msg_ptr->altitude;
    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());

    LOG(INFO) << "[GpsCallBack]: Gps data: " << std::fixed << gps_data_ptr->timestamp;
}
