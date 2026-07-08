#ifndef chassis_device_CONFIG_H
#define chassis_device_CONFIG_H

#include <iostream>
#include <Eigen/Dense>

struct joystickMsg
{
    float joy_right = 0.0f;
    float joy_forward = 0.0f;
    float joy_rotation = 0.0f;
    // bool enable_leso = false;
    // bool enable_v_control = false;
    // bool enable_stab_control = false;
};

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

struct IMUData{
    Eigen::Vector3f acc_raw_;
    Eigen::Vector3f gyro_raw_;
    Eigen::Vector3f mag_raw_;
    Eigen::Vector3f body_rpy_;
    Eigen::Vector3f rpy_dot_;
    Eigen::Vector3f rpy_dot_filtered_;
};

struct ControlCommand{
    volatile float linear_x;
    volatile float linear_y;
    volatile float angular_z;
    enum ControlMode{
        Keyboard_Ctrl = 0,      // keyboard control
        Algorithm_Ctrl = 1,     // algorithm control
    }CtrlMode = Keyboard_Ctrl;
    std::string robot_ip = "";
    std::string robot_id = "";
    std::string sender_ip = ""; // ip address of sender  
};

struct RobotState{
    IMUData imu_;
    ControlCommand control_cmd_;
    Eigen::Vector3f control_speeds_;
    Eigen::Vector3f motor_speeds_;
    float battery_voltage_;
};

#endif 
