#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "chassis_driver/chassis_device.hpp" 
#include "yaml_parameters.hpp"
#include "FirstOrderLPF.h"
#include "QEKF.h"
#include "Parameters.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "Quaternion.h"
#include "vofa.hpp"
#include <functional>
#include "chassis_controller/RobotState.h"
#include "chassis_controller/ControlCommand.h"
#include <map>
#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;

/*定义本车机器人结构体*/
RobotState this_agv_state;

/*全局变量用于接收线程*/
std::atomic<bool> udp_running{true};
ReceivedData last_received_data;
std::mutex data_mutex;

// 通信算法线程共享数据
struct SwarmData {
    chassis_controller::RobotState self_state; // 本机状态快照
    std::map<std::string, chassis_controller::RobotState> other_states; // 其他机器人状态
    std::mutex mutex;
    std::atomic<bool> new_self_state{false};
};
SwarmData swarm_data;

// 算法计算出的指令
struct AlgorithmCommand {
    std::atomic<float> linear_x{0.0f};
    std::atomic<float> angular_z{0.0f};
    std::atomic<bool> valid{false};
};
AlgorithmCommand algorithm_cmd;

// 通信线程控制
std::atomic<bool> swarm_thread_running{true};

/*驱动轮轮距*/
const float wheelspacing = 0.114f;
/*rad/s转rpm系数*/
const float rad_s_to_rpm = 60.0f/(2.0f*3.1415926f);

/*UDP接收回调函数*/
void udpReceiveCallback(const std::vector<uint8_t>& data, const std::string& from_ip) {
    /*加锁保护接收数据*/
    std::lock_guard<std::mutex> lock(data_mutex);
    /*写入数据*/
    last_received_data.data = data;
    last_received_data.sender_ip = from_ip;
    last_received_data.has_new_data = true;
}

/*UDP接收线程*/
void udpReceiverThread() {
    VofaFrame receiver;
    
    // 初始化接收器（只接收，不发送）
    if (receiver.initialize("0.0.0.0", 1348, "192.168.0.52", 1347)) {
        ROS_INFO("UDP receiver started on port 1348");
        
        // 启动接收
        if (receiver.startReceiving(udpReceiveCallback)) {
            ROS_INFO("UDP reception started");
            
            // 保持线程运行
            while (udp_running.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            receiver.stopReceiving();
        }
    } else {
        ROS_ERROR("Failed to initialize UDP receiver");
    }
}

void swarmCommsAlgorithmThread(ros::NodeHandle* nh, std::string robot_id, ChassisDevice* chassisdev_ptr) {
    // 创建本线程的ROS节点句柄
    ros::NodeHandle nh_local(*nh);
    
    // 发布本机状态
    std::string pub_topic = "/" + robot_id + "/state";
    ros::Publisher state_pub = nh_local.advertise<chassis_controller::RobotState>(pub_topic, 1); // 队列设为1保证最新
    
    // 订阅其他机器人状态（假设三台：agv1, agv2, agv3）
    std::vector<std::string> other_robot_ids = {"agv1", "agv2", "agv3"};
    std::map<std::string, ros::Subscriber> subs;
    for (const auto& id : other_robot_ids) {
        if (id != robot_id) {
            std::string sub_topic = "/" + id + "/state";
            subs[id] = nh_local.subscribe<chassis_controller::RobotState>(
                sub_topic, 10,
                [id](const chassis_controller::RobotState::ConstPtr& msg) {
                    std::lock_guard<std::mutex> lock(swarm_data.mutex);
                    swarm_data.other_states[id] = *msg;
                }
            );
        }
    }
    
    ros::Rate rate(100); // 目标频率：100Hz
    while (ros::ok() && swarm_thread_running.load()) {
        // 1. 发布本机状态 (100Hz)
        {
            std::lock_guard<std::mutex> lock(swarm_data.mutex);
            if (swarm_data.new_self_state) {
                state_pub.publish(swarm_data.self_state);
                swarm_data.new_self_state = false;
            }
        }
        
        // 2. 运行协同算法 (100Hz)
        // {
        //     std::lock_guard<std::mutex> lock(swarm_data.mutex);
        //     if (!swarm_data.other_states.empty()) {
        //         // 假设 robot1 为领航者，其他机器人跟随
        //         if (robot_id != "agv1" && swarm_data.other_states.count("agv1") > 0) {
        //             const auto& leader_state = swarm_data.other_states["agv1"];
                    
        //             // 获取领航者姿态
        //             float leader_yaw = leader_state.imu.body_rpy[2];
        //             // 获取领航者速度（这里假设控制指令中的线速度就是领航者的速度）
        //             float leader_vx = leader_state.ctrl_cmd.linear_x;
                    
        //             // 简单的跟随算法：保持一定距离，并调整角度对准领航者
        //             float desired_distance = 0.5; // 期望跟随距离
        //             float kp_distance = 0.5;      // 距离控制比例系数
        //             float kp_angle = 0.8;         // 角度控制比例系数
                    
        //             // 计算控制指令（这里仅为示例，你需要根据实际情况设计算法）
        //             float cmd_vx = leader_vx; // 基本跟随领航者速度
        //             float cmd_wz = kp_angle * leader_yaw; // 调整角度
                    
        //             // 存储算法指令
        //             algorithm_cmd.linear_x.store(cmd_vx);
        //             algorithm_cmd.angular_z.store(cmd_wz);
        //             algorithm_cmd.valid.store(true);
        //         }
        //     } else {
        //         // 如果没有收到其他机器人状态，算法指令无效
        //         algorithm_cmd.valid.store(false);
        //     }
        // }
        
        ros::spinOnce(); // 处理订阅回调
        rate.sleep();
    }
}

/*将机体角速度转换为欧拉角速率*/
bool ConvertBodyAngularVelocityToRPYDot(const Quaternionf& body_quat,
    const Vector3f& omega_body,
    Vector3f& rpy_dot) {
    Vector3f rpy = quaternion_to_rpy(body_quat);
    float phi = rpy(0);   // roll
    float theta = rpy(1); // pitch

    float cos_theta = std::cos(theta);
    float sin_theta = std::sin(theta);
    float tan_theta = std::tan(theta);
    float sin_phi = std::sin(phi);
    float cos_phi = std::cos(phi);

    // 检查 pitch 奇异区
    if (std::abs(cos_theta) < 1e-4) {
    rpy_dot.setZero();
    return false;
    }

    Matrix3f T_inv;
    T_inv << 
    1, sin_phi * tan_theta,  cos_phi * tan_theta,
    0, cos_phi,             -sin_phi,
    0, sin_phi / cos_theta,  cos_phi / cos_theta;

    rpy_dot = T_inv * omega_body;
    return true;
}

/*运动学逆解*/
vector<float>convertToM_Speed(const geometry_msgs::Twist::ConstPtr& msg){
    vector<float> motorspeeds(3,0.0f);
    /*存储控制量*/
    this_agv_state.control_cmd_.linear_x = msg->linear.x;
    this_agv_state.control_cmd_.angular_z = msg->angular.z;
    /*逆解解析*/
    float wheelmotorL_Speed = (this_agv_state.control_cmd_.linear_x - this_agv_state.control_cmd_.angular_z*wheelspacing/2.0f);
    float wheelmotorR_Speed = (this_agv_state.control_cmd_.linear_x + this_agv_state.control_cmd_.angular_z*wheelspacing/2.0f);
    motorspeeds[0] = wheelmotorL_Speed*1000.0f;                                  //左轮速度     单位: mm/s
    motorspeeds[1] = wheelmotorR_Speed*1000.0f;                                  //右轮速度     单位: mm/s
    motorspeeds[2] = this_agv_state.control_cmd_.angular_z*rad_s_to_rpm;           //步进电机转速  单位: rpm
    for(int i=0;i<2;i++){
        motorspeeds[i] = clamp(motorspeeds[i], -900.0f, 900.0f);
    }
    motorspeeds[2] = clamp(motorspeeds[2], -2.0f, 2.0f);
    // ROS_INFO("motorspeeds: %f, %f, %f", motorspeeds[0], motorspeeds[1], motorspeeds[2]);
    return motorspeeds;
}


/*接受速度回调函数*/
void cmdVelCallBack(const geometry_msgs::Twist::ConstPtr& msg, ChassisDevice *chassisdev){
    /*将车速转换为轮速*/
    vector<float> motor_speeds = convertToM_Speed(msg);
    for(int i=0;i<3;i++){
        this_agv_state.control_speeds_[i] = motor_speeds[i];
    }
    /*发送速度指令给下位机*/
    chassisdev->sendMotorSpeed(motor_speeds, chassisdev->getSensorData().packet_seq_);
}


int main(int argc, char **argv){
    /*区分不同主机的节点名称，防止多机通信节点名冲突*/
    char hostname[128];
    gethostname(hostname, sizeof(hostname));
    std::string node_name = "chassis_controller_" + std::string(hostname);
    ros::init(argc, argv, "node_name");
    ros::NodeHandle nh("~");

    /*显视定义机器人控制状态*/
    this_agv_state.control_cmd_.CtrlMode = ControlCommand::Keyboard_Ctrl;
    
    /*初始化串口设备*/
    string chassisdev_path = nh.param<string>("chassisdev_path", "/dev/ttyACM0");
    ChassisDevice chassisdev(chassisdev_path);
        
    ROS_INFO("Chassis Controller is running...");
    ROS_INFO("chassisdev_path: %s", chassisdev_path.c_str());

    std::string device_uid = chassisdev.getDeviceUID();
    if (device_uid.empty()) {
        ROS_ERROR("Failed to get device UID");
        return -1;
    }
    
    /*获取参数服务器参数*/
    YamlParametersServer yaml_params_server(nh, device_uid);
    if (!yaml_params_server.isLoaded()) {
        ROS_ERROR("Failed to load parameters for device UID: %s", device_uid.c_str());
        return -1;
    }
    sleep(1);
    AllParams yaml_params = yaml_params_server.getParams();
    /*初始化UDP发送器*/
    VofaFrame vofa_sender;
    this_agv_state.control_cmd_.sender_ip = nh.param<std::string>("vofa_remote_ip", "192.168.0.52");
    int remote_port = nh.param<int>("vofa_remote_port", 1347);
    int local_port = nh.param<int>("vofa_local_port", 1349);
    
    if (vofa_sender.initialize("0.0.0.0", local_port, this_agv_state.control_cmd_.sender_ip, remote_port)) {
        ROS_INFO("UDP sender initialized: %s:%d", this_agv_state.control_cmd_.sender_ip.c_str(), remote_port);
    } else {
        ROS_ERROR("Failed to initialize UDP sender");
    }

    /*多机通信初始化*/
    if (yaml_params.robot_ip.empty()) {
        ROS_ERROR("FATAL: 'robot_ip' field not found or empty in YAML configuration for device UID: %s", device_uid.c_str());
        ROS_ERROR("Please ensure your YAML config contains 'robot_ip' for this device.");
        return -1; // 直接退出程序
    }
    this_agv_state.control_cmd_.robot_ip = yaml_params.robot_ip;
    ROS_INFO("Loaded robot_ip from YAML: %s", this_agv_state.control_cmd_.robot_ip.c_str());

    size_t last_dot = this_agv_state.control_cmd_.robot_ip.find_last_of('.');
    if (last_dot == std::string::npos) {
        ROS_ERROR("FATAL: Invalid IP address format in YAML: %s", this_agv_state.control_cmd_.robot_ip.c_str());
        return -1;
    }
    // 提取最后一段并验证
    std::string last_octet = this_agv_state.control_cmd_.robot_ip.substr(last_dot + 1);

    // 检查是否是有效的数字
    for (char c : last_octet) {
        if (!std::isdigit(c)) {
            ROS_ERROR("FATAL: Invalid octet in IP address: %s (contains non-digit)", this_agv_state.control_cmd_.robot_ip.c_str());
            return -1;
        }
    }

    // 严格限制只允许预设的IP
    if (last_octet == "50") {
        this_agv_state.control_cmd_.robot_id = "agv1";
    } else if (last_octet == "54") {
        this_agv_state.control_cmd_.robot_id = "agv2";
    } else if (last_octet == "55") {
        this_agv_state.control_cmd_.robot_id = "agv3";
    } else {
        ROS_ERROR("FATAL: IP address %s is not in the allowed range for multi-robot communication.", this_agv_state.control_cmd_.robot_ip.c_str());
        ROS_ERROR("Allowed IPs: 192.168.0.50 (robot1), .51 (robot2), .52 (robot3)");
        ROS_ERROR("Please check your YAML configuration for device UID: %s", device_uid.c_str());
        return -1;
    }

    ROS_INFO("Assigned robot_id: %s (from IP: %s)", this_agv_state.control_cmd_.robot_id.c_str(), this_agv_state.control_cmd_.robot_ip.c_str());

    /*启动UDP接收线程*/
    std::thread udp_thread(udpReceiverThread);

    /*启动算法-通信线程*/
    std::thread swarm_thread(swarmCommsAlgorithmThread, &nh, this_agv_state.control_cmd_.robot_id, &chassisdev);
    swarm_thread.detach(); // 分离线程，让它独立运行

    Parameters params;
    QEKF qekf(params);
    bool qekf_initialized = false;
    
    // 里程计相关变量
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    vector<FirstOrderLPF> gyro_lpf = {FirstOrderLPF(1.0f / params.controller.SampleRate, yaml_params.gyro_x_lpf_tau), FirstOrderLPF(1.0f / params.controller.SampleRate, yaml_params.gyro_y_lpf_tau), FirstOrderLPF(1.0f / params.controller.SampleRate, yaml_params.gyro_z_lpf_tau)};
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/chassis/imu/data_raw", 10);
    ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("/chassis/imu/mag", 10);

    /*加载IMU纠偏参数*/
    Vector3f acc_bias = yaml_params.acc_bias;
    Vector3f gyro_bias = yaml_params.gyro_bias;
    Vector3f mag_bias = yaml_params.mag_bias;
    Vector3f acc_scale = yaml_params.acc_scale;
    Vector3f mag_scale = yaml_params.mag_scale;
    ROS_INFO("acc_bias: [%.6f, %.6f, %.6f]", 
        acc_bias[0], acc_bias[1], acc_bias[2]);
    ROS_INFO("acc_scale: [%.6f, %.6f, %.6f]",
        acc_scale[0], acc_scale[1], acc_scale[2]);
    ROS_INFO("gyro_bias: [%.6f, %.6f, %.6f]",
        gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    ROS_INFO("mag_bias: [%.6f, %.6f, %.6f]",
        mag_bias[0], mag_bias[1], mag_bias[2]);
    ROS_INFO("mag_scale: [%.6f, %.6f, %.6f]",
        mag_scale[0], mag_scale[1], mag_scale[2]);

    /*订阅速度 遥控使用*/
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(
        "/agv_chassis/cmd_vel", 10, 
        std::bind(cmdVelCallBack, std::placeholders::_1, &chassisdev));
    
    auto past_time = chrono::high_resolution_clock::now();
    int loop_cnt = 0;

    float base_height = 0.09276;
    float lidi = 0.01331;
    float base_link_z = base_height/2 + lidi;

    ros::Rate rate(250);   
    while(ros::ok()){
        /*计算循环时间*/
        auto current_time = chrono::high_resolution_clock::now();
        float dt = chrono::duration_cast<chrono::duration<float>>(current_time - past_time).count();
        past_time = current_time;

        /*EKF*/
        auto sensor_data = chassisdev.getSensorData();       

        /*获取IMU原始数据*/
        this_agv_state.imu_.acc_raw_ = sensor_data.acc_;
        this_agv_state.imu_.gyro_raw_ = sensor_data.gyro_;
        this_agv_state.imu_.mag_raw_ = Vector3f::Zero();
        /*获取电机速度数据*/
        this_agv_state.motor_speeds_[0] = sensor_data.wheelmotor_speed_[0];
        this_agv_state.motor_speeds_[1] = sensor_data.wheelmotor_speed_[1];
        this_agv_state.motor_speeds_[2] = sensor_data.stepmotor_speed_;
        /*获取电池电压*/
        this_agv_state.battery_voltage_ = sensor_data.battery_voltage_;
        /*IMU数据纠偏*/
        Vector3f acc_corrected = this_agv_state.imu_.acc_raw_ - acc_bias;
        acc_corrected = acc_corrected.cwiseQuotient(acc_scale);
        Vector3f gyro_corrected = this_agv_state.imu_.gyro_raw_ - gyro_bias;
        Vector3f mag_corrected = this_agv_state.imu_.mag_raw_ - mag_bias;
        mag_corrected = mag_corrected.cwiseQuotient(mag_scale);
        /*陀螺仪滤波*/
        Vector3f gyro_filtered(gyro_lpf[0].Filter(gyro_corrected[0]), gyro_lpf[1].Filter(gyro_corrected[1]), gyro_lpf[2].Filter(gyro_corrected[2]));
        
        if (!qekf_initialized) {
            qekf.Reset(acc_corrected);
        }

        Quaternionf past_body_quat;
        qekf.GetQuaternion(past_body_quat);

        Vector3f past_body_rpy = quaternion_to_rpy(past_body_quat);
        past_body_rpy[2] = 0;
        Quaternionf q_rp = rpy_to_quaternion(past_body_rpy).inverse();
        
        // 步骤2：将磁力计数据从机体系变换到世界系
        Eigen::Vector3f mag_world = q_rp * mag_corrected;  // q 是 Eigen::Quaterniond，自动处理旋转

        // 步骤3：计算yaw（单位：弧度）
        float mag_yaw = -std::atan2(mag_world.y(), mag_world.x());

        if (!qekf_initialized) {
            qekf_initialized = true;
        }
        qekf.Step(acc_corrected, gyro_corrected, params.estimator.EstimateBias);
        Quaternionf body_quat;
        qekf.GetQuaternion(body_quat);
        Vector4f body_quat_dot;
        qekf.GetQuaternionDerivative(body_quat_dot);
        Matrix<float, 4, 4> cov_q;
        qekf.GetQuaternionCovariance(cov_q);
        this_agv_state.imu_.body_rpy_ = quaternion_to_rpy(body_quat);
        Vector3f omega_body, tmp_v3;
        Quaternion_GetAngularVelocity_Body(body_quat, body_quat_dot, omega_body);
        ConvertBodyAngularVelocityToRPYDot(body_quat, omega_body, tmp_v3);
        omega_body = tmp_v3;
        ConvertBodyAngularVelocityToRPYDot(body_quat, gyro_corrected, this_agv_state.imu_.rpy_dot_);
        // Vector3f rpy_dot_filtered;
        ConvertBodyAngularVelocityToRPYDot(body_quat, gyro_filtered, this_agv_state.imu_.rpy_dot_filtered_);

        /*更新共享状态给通信线程*/
        {
            std::lock_guard<std::mutex> lock(swarm_data.mutex);
            // 将this_agv_state转换为chassis_controller::RobotState并存入swarm_data.self_state
            chassis_controller::RobotState& state_msg = swarm_data.self_state;
            state_msg.header.stamp = ros::Time::now();
            state_msg.header.frame_id = this_agv_state.control_cmd_.robot_id + "/base_link";
            static uint32_t seq = 0;
            state_msg.header.seq = seq++;
            
            // 填充IMU数据
            for (int i = 0; i < 3; i++) {
                state_msg.imu.acc_raw[i] = this_agv_state.imu_.acc_raw_[i];
                state_msg.imu.gyro_raw[i] = this_agv_state.imu_.gyro_raw_[i];
                state_msg.imu.mag_raw[i] = this_agv_state.imu_.mag_raw_[i];
                state_msg.imu.body_rpy[i] = this_agv_state.imu_.body_rpy_[i];
                state_msg.imu.rpy_dot[i] = this_agv_state.imu_.rpy_dot_[i];
                state_msg.imu.rpy_dot_filtered[i] = this_agv_state.imu_.rpy_dot_filtered_[i];
            }
            
            // 控制指令
            state_msg.ctrl_cmd.linear_x = this_agv_state.control_cmd_.linear_x;
            state_msg.ctrl_cmd.linear_y = 0.0; 
            state_msg.ctrl_cmd.angular_z = this_agv_state.control_cmd_.angular_z;
            if (this_agv_state.control_cmd_.CtrlMode == ControlCommand::Keyboard_Ctrl) {
                state_msg.ctrl_cmd.control_mode = chassis_controller::ControlCommand::CTRL_MODE_KEYBOARD;
            } else {
                state_msg.ctrl_cmd.control_mode = chassis_controller::ControlCommand::CTRL_MODE_ALGORITHM;
            }
            state_msg.ctrl_cmd.robot_ip = this_agv_state.control_cmd_.robot_ip;
            state_msg.ctrl_cmd.robot_id = this_agv_state.control_cmd_.robot_id;
            state_msg.ctrl_cmd.sender_ip = this_agv_state.control_cmd_.sender_ip;
            
            // 电机速度
            for (int i = 0; i < 3; i++) {
                state_msg.motor_speeds[i] = this_agv_state.motor_speeds_[i];
            }
            
            swarm_data.new_self_state = true; // 通知线程发布
        }
        
        /*应用算法指令（如果算法指令有效且当前为算法模式）*/
        if (algorithm_cmd.valid && this_agv_state.control_cmd_.CtrlMode == ControlCommand::Algorithm_Ctrl) {
            this_agv_state.control_cmd_.linear_x = algorithm_cmd.linear_x.load();
            this_agv_state.control_cmd_.angular_z = algorithm_cmd.angular_z.load();
        }

        /* 里程计积分与发布 */
        // 线速度：左右轮均值（单位转换 mm/s -> m/s）
        float v_left = this_agv_state.motor_speeds_[0] / 1000.0f;
        float v_right = this_agv_state.motor_speeds_[1] / 1000.0f;
        float v = (v_left + v_right) / 2.0f;

        // 角速度：使用 IMU 滤波后的 yaw 角速度（单位：rad/s）
        float w = this_agv_state.imu_.rpy_dot_filtered_[2];

        // 积分（使用 dt）
        static double odom_x = 0.0, odom_y = 0.0, odom_yaw = 0.0;
        odom_yaw += w * dt;
        double delta_x = v * dt * cos(odom_yaw);
        double delta_y = v * dt * sin(odom_yaw);
        odom_x += delta_x;
        odom_y += delta_y;

        /*VOFA数据发送*/
        vofa_sender.reset();
        
        /*发送IMU数据*/
        vofa_sender.push(this_agv_state.imu_.body_rpy_[0], this_agv_state.imu_.body_rpy_[1], this_agv_state.imu_.body_rpy_[2]);
        vofa_sender.push(this_agv_state.imu_.rpy_dot_[0], this_agv_state.imu_.rpy_dot_[1], this_agv_state.imu_.rpy_dot_[2]);
        vofa_sender.push(this_agv_state.imu_.rpy_dot_filtered_[0], this_agv_state.imu_.rpy_dot_filtered_[1], this_agv_state.imu_.rpy_dot_filtered_[2]);
        /*发送电机实时速度*/
        vofa_sender.push(this_agv_state.motor_speeds_[0], this_agv_state.motor_speeds_[1], this_agv_state.motor_speeds_[2]);
        /*发送控制量*/
        vofa_sender.push(this_agv_state.control_cmd_.linear_x, this_agv_state.control_cmd_.angular_z);
        /*发送电机期望期望值*/
        vofa_sender.push(this_agv_state.control_speeds_[0], this_agv_state.control_speeds_[1], this_agv_state.control_speeds_[2]);
        /*发送电池电压*/
        vofa_sender.push(this_agv_state.battery_voltage_);
        /*发送数据*/
        vofa_sender.send();
        

        // ROS_INFO("r: %.2f, p: %.2f, y: %.2f, "
        //     "r_dot: %.2f, p_dot: %.2f, y_dot: %.2f, "
        //     "r_dot_filtered: %.2f, p_dot_filtered: %.2f, y_dot_filtered: %.2f",
        //     body_rpy[0], body_rpy[1], body_rpy[2],
        //     rpy_dot[0], rpy_dot[1], rpy_dot[2],
        //     rpy_dot_filtered[0], rpy_dot_filtered[1], rpy_dot_filtered[2]);

        // ROS_INFO("r: %.2f, p: %.2f, y: %.2f, ",
        //     this_agv_state.imu_.body_rpy_[0], this_agv_state.imu_.body_rpy_[1], this_agv_state.imu_.body_rpy_[2]);

        // === 发送 ROS IMU 消息 ===
        sensor_msgs::Imu imu_msg;
        sensor_msgs::MagneticField mag_msg;
        ros::Time now = ros::Time::now();
        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = "imu_link";
        imu_msg.linear_acceleration.x = this_agv_state.imu_.acc_raw_[0];
        imu_msg.linear_acceleration.y = this_agv_state.imu_.acc_raw_[1];
        imu_msg.linear_acceleration.z = this_agv_state.imu_.acc_raw_[2];
        imu_msg.angular_velocity.x = this_agv_state.imu_.gyro_raw_[0];
        imu_msg.angular_velocity.y = this_agv_state.imu_.gyro_raw_[1];
        imu_msg.angular_velocity.z = this_agv_state.imu_.gyro_raw_[2];
        imu_msg.linear_acceleration_covariance[0] = -1;
        imu_msg.angular_velocity_covariance[0] = -1;
        imu_pub.publish(imu_msg);

        mag_msg.header.stamp = now;
        mag_msg.header.frame_id = "imu_link";
        mag_msg.magnetic_field.x = this_agv_state.imu_.mag_raw_[0];
        mag_msg.magnetic_field.y = this_agv_state.imu_.mag_raw_[1];
        mag_msg.magnetic_field.z = this_agv_state.imu_.mag_raw_[2];
        mag_msg.magnetic_field_covariance[0] = -1;
        mag_pub.publish(mag_msg);
        
        // === 发送 ROS odometry 消息 ===
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = odom_x;
        odom_msg.pose.pose.position.y = odom_y;
        odom_msg.pose.pose.position.z = base_link_z;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_yaw);
        odom_msg.pose.pose.orientation = odom_quat;

        // 速度（在机器人坐标系中表达）
        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.angular.z = w;

        // 简单设置协方差（可根据需要调整）
        for (int i = 0; i < 36; i++) odom_msg.pose.covariance[i] = 0.0;
        for (int i = 0; i < 36; i++) odom_msg.twist.covariance[i] = 0.0;

        odom_pub.publish(odom_msg);

        // 广播 TF: odom -> base_link
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = now;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = odom_x;
        odom_trans.transform.translation.y = odom_y;
        odom_trans.transform.translation.z = base_link_z;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        /*检查电池电量*/
        if(loop_cnt % 5 == 0){
            if(this_agv_state.battery_voltage_ < 10.0f){
                // ROS_ERROR("Low battery voltage: %.2f V", this_agv_state.battery_voltage_);
            }
        }

        loop_cnt = (loop_cnt + 1) % 50;
        rate.sleep();
        ros::spinOnce();
    }
    swarm_thread_running = false;
    vector<float> motor_speeds = {0.0f, 0.0f, 0.0f};
    chassisdev.sendMotorSpeed(motor_speeds);
    /*清理线程资源*/
    udp_running = false;
    if (udp_thread.joinable()) {
        udp_thread.join();
    }
    return 0;
}