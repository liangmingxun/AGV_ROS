#ifndef chassis_device_CONFIG_H
#define chassis_device_CONFIG_H

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

struct ChassisDeviceSensorData
{
    Eigen::Vector3f acc_;
    Eigen::Vector3f gyro_;
    Eigen::Quaternionf orientation_; 
    std::vector<float> wheelmotor_speed_;       // mm/s
    float stepmotor_speed_;                     // RPM
    float battery_voltage_;                     // V
    uint32_t packet_seq_ = 0;
    uint32_t mess_delay_ = 0;
    std::string uid = "";
    bool empty_ = true;
};

#endif 
