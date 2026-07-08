#pragma once

#include <iostream>
#include <Eigen/Eigen>

#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0)
// #define rad2deg (180.0 / M_PI)
// #define deg2rad (M_PI / 180.0)
#define rpm2radps (2.0 * M_PI / 60.0)
#define radps2rpm (60.0 / (2.0 * M_PI))

inline Eigen::Vector3f quaternion_to_rpy(const Eigen::Quaternionf &q) {
    const float &qw = q.w();
    const float &qx = q.x();
    const float &qy = q.y();
    const float &qz = q.z();

    Eigen::Vector3f rpy;
    /* roll  */ rpy.x() = std::atan2(2. * (qw*qx + qy*qz), 1. - 2. * (qx*qx + qy*qy));
    float sin_pitch = 2. * (qw*qy - qz*qx);
    // sin_pitch = sin_pitch >  1.0 ?  1.0 : sin_pitch;
    // sin_pitch = sin_pitch < -1.0 ? -1.0 : sin_pitch;
    /* pitch */ rpy.y() = std::asin(sin_pitch);
    /* yaw   */ rpy.z() = std::atan2(2. * (qw*qz + qx*qy), 1. - 2. * (qy*qy + qz*qz));
    return rpy;
}

inline Eigen::Matrix<float, 3, 3> quaternion_to_matrix(const Eigen::Quaternionf &q) {
    auto &x = q.x();
    auto &y = q.y();
    auto &z = q.z();
    auto &w = q.w();
    Eigen::Matrix<float, 3, 3> m;
    m << 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
        2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
        2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y);
    return m;
}


inline Eigen::Quaternionf rpy_to_quaternion(Eigen::Vector3f rpy) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    float cy = cos(rpy[2] * 0.5);
    float sy = sin(rpy[2] * 0.5);
    float cp = cos(rpy[1] * 0.5);
    float sp = sin(rpy[1] * 0.5);
    float cr = cos(rpy[0] * 0.5);
    float sr = sin(rpy[0] * 0.5);
 
    Eigen::Quaternionf q;
    q.w() = cy * cp * cr + sy * sp * sr;
    q.x() = cy * cp * sr - sy * sp * cr;
    q.y() = sy * cp * sr + cy * sp * cr;
    q.z() = sy * cp * cr - cy * sp * sr;
 
    return q;
}