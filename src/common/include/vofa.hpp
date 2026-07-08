#pragma once

#include <vector>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <functional>
#include <atomic>
#include <iostream>
#include <Eigen/Dense>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

class VofaFrame {
public:
    using ReceiveCallback = std::function<void(const std::vector<uint8_t>& data, const std::string& from_ip)>;

    VofaFrame() : socket_fd_(-1), is_initialized_(false) {
        buffer_.reserve(1024);
    }

    ~VofaFrame() {
        cleanup();
    }

    // 初始化UDP Socket
    bool initialize(const std::string& local_ip = "0.0.0.0", uint16_t local_port = 0, 
                   const std::string& remote_ip = "127.0.0.1", uint16_t remote_port = 1347) {
        cleanup();

        // 创建socket
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            perror("socket creation failed");
            return false;
        }

        // 设置地址重用
        int reuse = 1;
        if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
            perror("setsockopt failed");
            closesocket();
            return false;
        }

        // 设置本地地址
        sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_port = htons(local_port);
        
        if (local_ip == "0.0.0.0") {
            local_addr.sin_addr.s_addr = INADDR_ANY;
        } else {
            if (inet_pton(AF_INET, local_ip.c_str(), &local_addr.sin_addr) <= 0) {
                perror("invalid local IP address");
                closesocket();
                return false;
            }
        }

        // 绑定本地地址
        if (bind(socket_fd_, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            perror("bind failed");
            closesocket();
            return false;
        }

        // 设置远程目标地址
        memset(&remote_addr_, 0, sizeof(remote_addr_));
        remote_addr_.sin_family = AF_INET;
        remote_addr_.sin_port = htons(remote_port);
        if (inet_pton(AF_INET, remote_ip.c_str(), &remote_addr_.sin_addr) <= 0) {
            perror("invalid remote IP address");
            closesocket();
            return false;
        }

        is_initialized_ = true;
        return true;
    }

    // 开始接收数据
    bool startReceiving(ReceiveCallback callback = nullptr) {
        if (!is_initialized_) {
            std::cerr << "Socket not initialized" << std::endl;
            return false;
        }
        
        if (!callback) {
            std::cerr << "Receive callback is null" << std::endl;
            return false;
        }

        receive_callback_ = callback;
        receive_thread_running_.store(true);
        
        receive_thread_ = std::thread([this]() {
            receiveLoop();
        });

        return true;
    }

    // 停止接收数据
    void stopReceiving() {
        receive_thread_running_.store(false);
        
        // 发送一个空数据包来唤醒阻塞的recvfrom
        if (is_initialized_) {
            sendto(socket_fd_, "", 0, 0, 
                  (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
        }
        
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }

    // 数据打包方法
    void push(float value) {
        uint8_t bytes[4];
        std::memcpy(bytes, &value, 4);
        buffer_.insert(buffer_.end(), bytes, bytes + 4);
    }

    void push(float v1, float v2) {
        push(v1);
        push(v2);
    }

    void push(float v1, float v2, float v3) {
        push(v1);
        push(v2);
        push(v3);
    }

    void push(const Eigen::Vector3f& vec) {
        for (int i = 0; i < 3; ++i) {
            push(vec[i]);
        }
    }

    void push(const Eigen::Vector3d& vec) {
        for (int i = 0; i < 3; ++i) {
            push(static_cast<float>(vec[i]));
        }
    }

    // 发送数据到预设的远程地址
    bool send() {
        if (!is_initialized_) {
            std::cerr << "Socket not initialized" << std::endl;
            return false;
        }
        
        if (buffer_.empty()) {
            std::cerr << "No data to send" << std::endl;
            return false;
        }
        
        auto frame = getFrame();
        ssize_t result = sendto(socket_fd_, 
                               frame.data(), 
                               frame.size(), 
                               0,
                               (struct sockaddr*)&remote_addr_, 
                               sizeof(remote_addr_));
        
        if (result < 0) {
            perror("send failed");
        }
        
        reset();
        return result >= 0;
    }

    // 发送数据到指定地址
    bool sendTo(const std::string& ip, uint16_t port) {
        if (!is_initialized_) {
            std::cerr << "Socket not initialized" << std::endl;
            return false;
        }
        
        if (buffer_.empty()) {
            std::cerr << "No data to send" << std::endl;
            return false;
        }

        sockaddr_in target_addr;
        memset(&target_addr, 0, sizeof(target_addr));
        target_addr.sin_family = AF_INET;
        target_addr.sin_port = htons(port);
        if (inet_pton(AF_INET, ip.c_str(), &target_addr.sin_addr) <= 0) {
            std::cerr << "Invalid target IP address: " << ip << std::endl;
            return false;
        }

        auto frame = getFrame();
        ssize_t result = sendto(socket_fd_, 
                               frame.data(), 
                               frame.size(), 
                               0,
                               (struct sockaddr*)&target_addr, 
                               sizeof(target_addr));
        
        if (result < 0) {
            perror("sendTo failed");
        }
        
        reset();
        return result >= 0;
    }

    // 直接发送原始数据
    bool sendRaw(const std::vector<uint8_t>& data, const std::string& ip = "", uint16_t port = 0) {
        if (!is_initialized_) {
            std::cerr << "Socket not initialized" << std::endl;
            return false;
        }
        
        if (data.empty()) {
            std::cerr << "No data to send" << std::endl;
            return false;
        }

        sockaddr_in target_addr = remote_addr_;
        if (!ip.empty() && port != 0) {
            memset(&target_addr, 0, sizeof(target_addr));
            target_addr.sin_family = AF_INET;
            target_addr.sin_port = htons(port);
            if (inet_pton(AF_INET, ip.c_str(), &target_addr.sin_addr) <= 0) {
                std::cerr << "Invalid target IP address: " << ip << std::endl;
                return false;
            }
        }

        ssize_t result = sendto(socket_fd_, 
                               data.data(), 
                               data.size(), 
                               0,
                               (struct sockaddr*)&target_addr, 
                               sizeof(target_addr));
        
        if (result < 0) {
            perror("sendRaw failed");
        }
        
        return result >= 0;
    }

    std::vector<uint8_t> getFrame() const {
        std::vector<uint8_t> frame = buffer_;
        const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
        frame.insert(frame.end(), tail, tail + 4);
        return frame;
    }

    void reset() {
        buffer_.clear();
    }

    bool isInitialized() const { return is_initialized_; }
    int getSocket() const { return socket_fd_; }

private:
    void receiveLoop() {
        std::vector<uint8_t> recv_buffer(4096);
        sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);

        while (receive_thread_running_.load()) {
            ssize_t recv_len = recvfrom(socket_fd_, 
                                       recv_buffer.data(), 
                                       recv_buffer.size(), 
                                       0,
                                       (struct sockaddr*)&from_addr, 
                                       &from_len);

            if (recv_len > 0) {
                char from_ip[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &from_addr.sin_addr, from_ip, INET_ADDRSTRLEN);

                std::vector<uint8_t> received_data(recv_buffer.begin(), 
                                                  recv_buffer.begin() + recv_len);
                
                if (receive_callback_) {
                    receive_callback_(received_data, std::string(from_ip));
                }
            } else if (recv_len < 0 && receive_thread_running_.load()) {
                perror("recvfrom failed");
            }
        }
    }

    void closesocket() {
        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
        }
    }

    void cleanup() {
        stopReceiving();
        closesocket();
        is_initialized_ = false;
    }

private:
    int socket_fd_;
    sockaddr_in remote_addr_;
    std::vector<uint8_t> buffer_;
    std::atomic<bool> is_initialized_;
    std::atomic<bool> receive_thread_running_{false};
    std::thread receive_thread_;
    ReceiveCallback receive_callback_;
};

/*接收到的数据结构*/
struct ReceivedData {
    std::vector<uint8_t> data;
    std::string sender_ip;
    bool has_new_data = false;
};