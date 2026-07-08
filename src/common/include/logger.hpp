#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <variant>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <Eigen/Dense>

#define VAR_PAIR(x) std::make_pair(#x, x)

using namespace Eigen;

// Timestamp as seconds + nanoseconds from epoch
struct Timestamp {
    uint32_t seconds;
    uint32_t nanoseconds;
};

inline Timestamp getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto sec = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto ns  = std::chrono::duration_cast<std::chrono::nanoseconds>(now - sec).count();

    Timestamp ts;
    ts.seconds = static_cast<uint32_t>(sec.time_since_epoch().count());
    ts.nanoseconds = static_cast<uint32_t>(ns);
    return ts;
}

class Logger {
public:
    using DataVariant = std::variant<float, double, bool, int, uint32_t, Vector2f, Vector2d, Vector3f, Vector3d, std::array<float, 2>, std::array<double, 2>, std::array<float, 3>, std::array<double, 3>>;

    Logger(const std::string& filename) : filename_(filename), initialized_(false) {
        ofs_.open(filename, std::ios::binary);
        if (!ofs_.is_open()) {
            throw std::runtime_error("Cannot open log file");
        }
    }

    ~Logger() {
        if (ofs_.is_open()) {
            ofs_.close();
        }
    }

    void push(const std::pair<std::string, DataVariant>& item) {
        const auto& name = item.first;
        const auto& val = item.second;

        if (!initialized_) {
            if (data_map_.count(name) == 0) data_order_.push_back(name);
            data_map_[name] = val;
        } else {
            if (data_map_.count(name)) {
                data_map_[name] = val;
            } else {
                std::cerr << "[Warning] Attempt to push unknown variable: " << name << std::endl;
            }
        }
    }

    void write_frame() {
        Timestamp ts = getCurrentTimestamp();

        if (!initialized_) {
            initialized_ = true;
            build_flat_order();
            N_ = static_cast<int>(flat_order_.size()) + 2; // include seconds + nanoseconds
            write_header();
        }

        ofs_.write(reinterpret_cast<const char*>(&ts.seconds), sizeof(uint32_t));
        ofs_.write(reinterpret_cast<const char*>(&ts.nanoseconds), sizeof(uint32_t));
        for (const auto& [var_name, index] : flat_order_) {
            write_data(data_map_[var_name], index);
        }

        ofs_.flush();
    }

private:
    std::ofstream ofs_;
    std::string filename_;
    bool initialized_;
    int N_ = 0;
    std::unordered_map<std::string, DataVariant> data_map_;
    std::vector<std::string> data_order_; 
    std::vector<std::pair<std::string, int>> flat_order_; // (var_name, index), index = -1 for scalar

    void build_flat_order() {
        for (const auto& name : data_order_) {
            const auto& val = data_map_[name];
            std::visit([&](auto&& v) {
                using T = std::decay_t<decltype(v)>;
                if constexpr (std::is_same_v<T, Vector2f> || std::is_same_v<T, Vector2d>) {
                    for (int i = 0; i < 2; ++i) flat_order_.emplace_back(name, i);
                } else if constexpr (std::is_same_v<T, Vector3f> || std::is_same_v<T, Vector3d>) {
                    for (int i = 0; i < 3; ++i) flat_order_.emplace_back(name, i);
                } else if constexpr (
                    std::is_same_v<T, std::array<float, 2>> || std::is_same_v<T, std::array<double, 2>> ||
                    std::is_same_v<T, std::array<float, 3>> || std::is_same_v<T, std::array<double, 3>>
                ) {
                    for (int i = 0; i < static_cast<int>(v.size()); ++i) flat_order_.emplace_back(name, i);
                } else {
                    flat_order_.emplace_back(name, -1);
                }
            }, val);
        }
    }

    void write_header() {
        ofs_.write(reinterpret_cast<const char*>(&N_), sizeof(int));
        std::string sec_str = "seconds_from_epoch";
        uint32_t len = sec_str.length();
        ofs_.write(reinterpret_cast<const char*>(&len), sizeof(uint32_t));
        ofs_.write(sec_str.data(), len);
        std::string nsec_str = "nanoseconds";
        len = nsec_str.length();
        ofs_.write(reinterpret_cast<const char*>(&len), sizeof(uint32_t));
        ofs_.write(nsec_str.data(), len);
        for (const auto& [name, index] : flat_order_) {
            std::string label = (index == -1) ? name : (name + "_" + std::to_string(index));
            uint32_t len = label.length();
            ofs_.write(reinterpret_cast<const char*>(&len), sizeof(uint32_t));
            ofs_.write(label.data(), len);
        }
    }

    void write_timestamp(const Timestamp& ts) {
        ofs_.write(reinterpret_cast<const char*>(&ts), sizeof(Timestamp));
    }

    void write_data(const DataVariant& data, int index) {
        std::visit([&](auto&& val) {
            using T = std::decay_t<decltype(val)>;
            if constexpr (std::is_same_v<T, Vector2f> || std::is_same_v<T, Vector2d> ||
                          std::is_same_v<T, Vector3f> || std::is_same_v<T, Vector3d>) {
                write_scalar(static_cast<float>(val[index]));
            } else if constexpr (
                std::is_same_v<T, std::array<float, 2>> || std::is_same_v<T, std::array<double, 2>> ||
                std::is_same_v<T, std::array<float, 3>> || std::is_same_v<T, std::array<double, 3>>
            ) {
                write_scalar(static_cast<float>(val[index]));
            } else {
                if (index == -1) write_scalar(static_cast<float>(val));
            }
        }, data);
    }

    template <typename T>
    void write_scalar(const T& value) {
        ofs_.write(reinterpret_cast<const char*>(&value), sizeof(T));
    }
};