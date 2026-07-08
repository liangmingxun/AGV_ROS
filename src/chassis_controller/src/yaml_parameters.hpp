#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <string>
#include "eigen_rosparam_utils.hpp"
#include "shared_data_manager.hpp"

struct PIDConfig {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float k_ff = 0.0f;
    float max_error = 0.0f;
    float max_integral = 0.0f;
    float max_output = 0.0f;
};

struct AllParams {
    Eigen::Vector3f acc_bias;
    Eigen::Vector3f acc_scale;
    Eigen::Vector3f gyro_bias;
    Eigen::Vector3f mag_bias;
    Eigen::Vector3f mag_scale;
    std::string chassisdev_path;

    PIDConfig pitch_rate_pid;
    PIDConfig roll_rate_pid;
    PIDConfig yaw_rate_pid;
    PIDConfig pitch_pid;
    PIDConfig roll_pid;
    PIDConfig yaw_pid;

    float gyro_x_lpf_tau;
    float gyro_y_lpf_tau;
    float gyro_z_lpf_tau;
    float dxy_lpf_tau;
    float yaw_ref_filter_tau;

    std::string robot_ip;
    std::string robot_id;
};

class YamlParametersServer {
public:
    // 构造函数：传入设备UID
    YamlParametersServer(ros::NodeHandle& nh, const std::string& device_uid) 
        : nh_(nh), device_uid_(device_uid) {
        
        // 加载该设备的所有参数
        loadDeviceParameters();
    }

    // 获取参数
    AllParams getParams() const {
        return params_;
    }

    // 检查是否成功加载
    bool isLoaded() const {
        return loaded_;
    }

    // 获取设备UID
    std::string getDeviceUID() const {
        return device_uid_;
    }

private:
    // 加载设备参数
    void loadDeviceParameters() {
        
        std::string base_path = "/device_" + device_uid_ + "/";
        
        // 尝试从参数服务器读取设备参数
        if (!loadFromParamServer(base_path)) {
            ROS_ERROR("Failed to load parameters for device UID: %s", device_uid_.c_str());
            loaded_ = false;
        } else {
            ROS_INFO("Successfully loaded parameters for device UID: %s", device_uid_.c_str());
            loaded_ = true;
        }
    }
    
    // 从参数服务器加载参数
    bool loadFromParamServer(const std::string& base_path) {
        try {
            using eigen_param_utils::getParam;
            
            // 加载向量参数
            params_.acc_bias = getParam<Eigen::Vector3f>(nh_, base_path + "acc_bias", Eigen::Vector3f::Zero());
            params_.acc_scale = getParam<Eigen::Vector3f>(nh_, base_path + "acc_scale", Eigen::Vector3f::Ones());
            params_.gyro_bias = getParam<Eigen::Vector3f>(nh_, base_path + "gyro_bias", Eigen::Vector3f::Zero());
            params_.mag_bias = getParam<Eigen::Vector3f>(nh_, base_path + "mag_bias", Eigen::Vector3f::Zero());
            params_.mag_scale = getParam<Eigen::Vector3f>(nh_, base_path + "mag_scale", Eigen::Vector3f::Ones());

            /*加载robot ip and id*/
            if (!nh_.getParam(base_path + "robot_ip", params_.robot_ip)) {
                ROS_WARN("No robot_ip found for device %s, using empty string", device_uid_.c_str());
                params_.robot_ip = "";
            }
            
            // 从参数服务器获取 robot_id
            if (!nh_.getParam(base_path + "robot_id", params_.robot_id)) {
                // 如果没有显式指定 robot_id，可以尝试从 IP 推导或留空
                ROS_INFO("No robot_id found for device %s, will derive from IP if possible", device_uid_.c_str());
                params_.robot_id = "";
            }

            // 加载字符串参数
            if (!nh_.getParam(base_path + "chassisdev_path", params_.chassisdev_path)) {
                ROS_WARN("No chassisdev_path found for device %s, using default", device_uid_.c_str());
                params_.chassisdev_path = "";
            }
            
            // 加载PID参数（这些可能不在YAML中，从默认位置读取）
            // 注意：这里使用默认路径，因为PID参数可能不是设备特定的
            loadPID("/pitch_rate_pid", params_.pitch_rate_pid);
            loadPID("/roll_rate_pid", params_.roll_rate_pid);
            loadPID("/yaw_rate_pid", params_.yaw_rate_pid);
            loadPID("/pitch_pid", params_.pitch_pid);
            loadPID("/roll_pid", params_.roll_pid);
            loadPID("/yaw_pid", params_.yaw_pid);
            
            // 加载其他参数
            nh_.param<float>("/gyro_x_lpf_tau", params_.gyro_x_lpf_tau, 1e-6);
            nh_.param<float>("/gyro_y_lpf_tau", params_.gyro_y_lpf_tau, 1e-6);
            nh_.param<float>("/gyro_z_lpf_tau", params_.gyro_z_lpf_tau, 1e-6);
            nh_.param<float>("/dxy_lpf_tau", params_.dxy_lpf_tau, 1e-6);
            nh_.param<float>("/yaw_ref_filter_tau", params_.yaw_ref_filter_tau, 1e-6);
            
            return true;
            
        } catch (const std::exception& e) {
            ROS_ERROR("Exception while loading parameters: %s", e.what());
            return false;
        }
    }
    
    void loadPID(const std::string& ns, PIDConfig& pid) {
        nh_.param<float>(ns + "/kp", pid.kp, 0.0);
        nh_.param<float>(ns + "/ki", pid.ki, 0.0);
        nh_.param<float>(ns + "/kd", pid.kd, 0.0);
        nh_.param<float>(ns + "/k_ff", pid.k_ff, 0.0);
        nh_.param<float>(ns + "/max_error", pid.max_error, 0.0);
        nh_.param<float>(ns + "/max_integral", pid.max_integral, 0.0);
        nh_.param<float>(ns + "/max_output", pid.max_output, 0.0);
    }

private:
    ros::NodeHandle nh_;
    std::string device_uid_;
    AllParams params_;
    bool loaded_ = false;
};