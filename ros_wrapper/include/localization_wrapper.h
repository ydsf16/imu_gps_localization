#pragma once

#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include "imu_gps_localizer/imu_gps_localizer.h"

class LocalizationWrapper {
public:
    LocalizationWrapper(ros::NodeHandle& nh);
    
    void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    void GpsCallBack(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr);

private:
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_sub_;
    ros::Publisher state_pub_;

    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_localizer_ptr_;
};