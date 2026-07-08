#pragma once

#include <thread>
#include <deque>
#include <boost/asio.hpp>
#include <mutex>
#include <numeric>
#include <Eigen/Dense>
#include <sys/select.h>
#include "chassis_driver/config.h"
#include "shared_data_manager.hpp"
#include "rotation_math.hpp"

#define CHASSIS_DATA_LEN    (66-6)

class ChassisDevice {
public:
    ChassisDevice(const std::string& port, int baud_rate = 1000000)
        : io(), serial(io, port), running(true) {
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        int fd = serial.native_handle();
        struct termios tio;
        tcgetattr(fd, &tio);
        cfmakeraw(&tio);              // 禁用 canonical 模式、回显等
        tio.c_cc[VMIN]  = 1;
        tio.c_cc[VTIME] = 0;          // 最多等待 100ms，设为 1 表示 0.1s 超时
        tcsetattr(fd, TCSANOW, &tio);
        read_thread = std::thread(&ChassisDevice::readLoop, this);
        
        std::string uid = "";
        while (running) {
            bool connected = false;
            sensor_data_.multiaction_with_unique_lock([&connected](ChassisDeviceSensorData& data) {
                connected = !data.empty_;
            ROS_WARN("USB Disconnected");
            });
            if (connected) {
                ROS_INFO("USB Connected");
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    ~ChassisDevice() {
        running = false;
        if (read_thread.joinable())
            read_thread.join();
    }

    std::string getDeviceUID() {
        std::string uid;
        sensor_data_.multiaction_with_shared_lock([&uid](const ChassisDeviceSensorData& data) {
            uid = data.uid;
        });
        return uid;
    }

    void sendMotorSpeed(const std::vector<float>& speeds, uint32_t seq = 0) {
        if (speeds.size() != 3) {
            printf("Invalid currents size: %lu", speeds.size());
            return;
        }

        std::vector<uint8_t> frame;
        for (auto ch : frame_start_)
            frame.push_back(ch);

        frame.push_back(static_cast<uint8_t>((seq >> 24) & 0xFF));
        frame.push_back(static_cast<uint8_t>((seq >> 16) & 0xFF));
        frame.push_back(static_cast<uint8_t>((seq >> 8) & 0xFF));
        frame.push_back(static_cast<uint8_t>(seq & 0xFF));

        for (auto speed : speeds) {
            // if (MOTOR_DIR)
            // speed = -std::clamp(speeds, -MAX_SPEED, MAX_SPEED);
            // else
            speed = std::clamp(speed, -MAX_SPEED, MAX_SPEED);
            uint8_t temp[4];
            std::memcpy(temp, &speed, 4);
            frame.push_back(temp[0]);
            frame.push_back(temp[1]);
            frame.push_back(temp[2]);
            frame.push_back(temp[3]);
        }

        for (auto ch : frame_end_)
            frame.push_back(ch);

        std::lock_guard<std::mutex> lock(write_mutex);
        // boost::asio::write(serial, boost::asio::buffer(frame));
        try {
            std::size_t bytes_written = boost::asio::write(serial, boost::asio::buffer(frame));
            if (bytes_written != frame.size()) {
                std::cerr << "[WARN] Partial write: expected " << frame.size()
                        << ", got " << bytes_written << " bytes\n";
            }
        } catch (const boost::system::system_error& e) {
            std::cerr << "[ERROR] Serial write failed: " << e.what() << "\n";
            exit(0);
        }
    }

    void getSensorData(ChassisDeviceSensorData& data) {
        sensor_data_.multiaction_with_unique_lock([&data](ChassisDeviceSensorData& sensor_data) {
            data = sensor_data;
        });
    }

    const ChassisDeviceSensorData getSensorData() {
        ChassisDeviceSensorData data;
        getSensorData(data);
        return data;
    }

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
    std::thread read_thread;
    std::atomic<bool> running;
    std::mutex write_mutex;
    std::deque<uint8_t> buffer;

    const float MAX_SPEED = 900.0f;    //最大轮速0.9m/s

    const std::vector<uint8_t> frame_start_ = {'#', '$', '#'};
    const std::vector<uint8_t> frame_end_ = {'!', '@', '!'};

    SharedDataManager<ChassisDeviceSensorData> sensor_data_;
    std::chrono::_V2::high_resolution_clock::time_point last_time_;

    void readLoop() {
        while (running) {
            boost::system::error_code ec;
            std::array<uint8_t, 256> buf;
            int fd = serial.native_handle();
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(fd, &read_fds);

            struct timeval timeout;
            timeout.tv_sec = 0;
            timeout.tv_usec = 10000;  // 10ms 检查一次

            int ret = select(fd + 1, &read_fds, nullptr, nullptr, &timeout);
            if (ret > 0 && FD_ISSET(fd, &read_fds)) {
                boost::system::error_code ec;
                size_t n = serial.read_some(boost::asio::buffer(buf), ec);
                if (!ec) {
                    buffer.insert(buffer.end(), buf.begin(), buf.begin() + n);
                }
            } else if (ret == 0) {
                // timeout，继续 loop
            } else {
                // select error
            }
            

            // for (auto ch : buffer) {
            //     printf("%02X ", ch);
            // }

            // 检查是否为完整帧 #$# ... #$#
            while (buffer.size() >= 4) {
                auto start = std::search(buffer.begin(), buffer.end(), frame_start_.begin(), frame_start_.end());
                if (start == buffer.end()) {
                    buffer.clear();
                    break;
                }

                auto end = std::search(start + 3, buffer.end(), frame_end_.begin(), frame_end_.end());
                if (end == buffer.end()) break;

                size_t dataStart = std::distance(buffer.begin(), start) + 3;
                size_t dataEnd = std::distance(buffer.begin(), end);

                if (dataEnd > dataStart) {
                    std::vector<uint8_t> payload(buffer.begin() + dataStart, buffer.begin() + dataEnd);
                    parseFrame(payload);
                }

                buffer.erase(buffer.begin(), end + 3);
            }
        }
    }

    void parseFrame(const std::vector<uint8_t>& data) {
        if (data.size() != CHASSIS_DATA_LEN) {
            ROS_ERROR("Invalid data size: %lu", data.size());
            return;
        }

        // 计算帧间隔
        auto now = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_time_).count();
        last_time_ = now;

        static std::deque<float> rx_dt;
        rx_dt.push_back(dt);
        if (rx_dt.size() > 100)
            rx_dt.pop_front();
        double avg_dt = std::accumulate(rx_dt.begin(), rx_dt.end(), 0.0) / rx_dt.size();

        size_t idx = 0;

        auto readInt8 = [&](size_t& i) -> int8_t {
            int8_t val = data[i];
            i += 1;
            return val;
        };

        auto readInt16 = [&](size_t& i) -> int16_t {
            int16_t val = (data[i] << 8) | data[i + 1];
            i += 2;
            return val;
        };

        auto readInt32 = [&](size_t& i) -> int32_t {
            int32_t val = (data[i] << 24) | (data[i + 1] << 16) | (data[i + 2] << 8) | data[i + 3];
            i += 4;
            return val;
        };
    
        auto readFloat = [&](size_t& i) -> float {
            float val;
            uint8_t temp[4] = {data[i + 0], data[i + 1], data[i + 2], data[i + 3]};
            std::memcpy(&val, temp, 4);
            i += 4;
            return val;
        };

        ChassisDeviceSensorData sensor_data_tmp;
        // seq
        sensor_data_tmp.packet_seq_ = readInt32(idx);
        // 6轴 IMU 数据（float）
        sensor_data_tmp.gyro_ << readFloat(idx), readFloat(idx), readFloat(idx);
        sensor_data_tmp.acc_ << readFloat(idx), readFloat(idx), readFloat(idx);
        // 驱动轮转速 mm/s
        sensor_data_tmp.wheelmotor_speed_ = {readFloat(idx), readFloat(idx)};
        // 转盘转速  RPM
        sensor_data_tmp.stepmotor_speed_ = readFloat(idx);
        // 电池电压 V
        sensor_data_tmp.battery_voltage_ = readFloat(idx);
        // uint32_t rxtx_interval = readInt32(idx);

        sensor_data_tmp.uid.clear();
        for(int i=0; i<12; i++){
            uint8_t ch = readInt8(idx);
            char buf[4];
            snprintf(buf, 4, "%02X", ch);
            sensor_data_tmp.uid += buf;
        }
        
        sensor_data_tmp.mess_delay_ = readInt32(idx);

        sensor_data_tmp.empty_ = false;

        sensor_data_.multiaction_with_unique_lock([&](ChassisDeviceSensorData& sensor_data) {
            sensor_data = sensor_data_tmp;
        });
        // ROS_INFO("seq: %d, gyro_X: %.6f, gyro_Y: %.6f, gyro_Z: %.6f, acc_X: %.6f, acc_Y: %.6f, acc_Z: %.6f, wheelmotor_speedL: %.2f, wheelmotor_speedR: %.2f, stepmotor: %.2f, messdelay: %d", 
        //     seq,  // 第一个 %d 对应 seq
        //     sensor_data_tmp.gyro_[0],  // 第一个 %.2f
        //     sensor_data_tmp.gyro_[1],  // 第一个 %.2f
        //     sensor_data_tmp.gyro_[2],  // 第一个 %.2f
        //     sensor_data_tmp.acc_[0],   // 第二个 %.2f
        //     sensor_data_tmp.acc_[1],   // 第二个 %.2f
        //     sensor_data_tmp.acc_[2],   // 第二个 %.2f
        //     sensor_data_tmp.wheelmotor_speed_[0],  // 第三个 %.2f
        //     sensor_data_tmp.wheelmotor_speed_[1],  // 第四个 %.2f
        //     sensor_data_tmp.stepmotor_speed_,      // 第五个 %.2f
        //     rxtx_interval  // 最后一个 %d
        // );
        // ROS_INFO("seq: %d, delay: %d,", sensor_data_tmp.packet_seq_, sensor_data_tmp.mess_delay_);
    }
};
