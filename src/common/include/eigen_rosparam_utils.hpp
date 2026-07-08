#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <type_traits>

namespace eigen_param_utils {

    // === 一维向量：Vector2/3/4f/d/X ===
    template <typename Derived>
    typename std::enable_if<
        std::is_base_of<Eigen::MatrixBase<Derived>, Derived>::value &&
        Derived::ColsAtCompileTime == 1,
        Derived>::type
    getParam(ros::NodeHandle& nh, const std::string& name, const Derived& default_val) {
        using Scalar = typename Derived::Scalar;
        std::vector<Scalar> tmp;
    
        const int expected_dim = Derived::RowsAtCompileTime;
    
        if (nh.getParam(name, tmp)) {
            if (expected_dim != Eigen::Dynamic && static_cast<int>(tmp.size()) != expected_dim) {
                ROS_ERROR_STREAM("Param '" << name << "' size mismatch. Expected "
                                           << expected_dim << ", got " << tmp.size());
                return default_val;
            }
    
            Eigen::Matrix<Scalar, Eigen::Dynamic, 1> vec(tmp.size());
            for (size_t i = 0; i < tmp.size(); ++i) vec[i] = tmp[i];
            return vec;
        } else {
            ROS_WARN_STREAM("Param '" << name << "' not found. Using default.");
            return default_val;
        }
    }

    // === 矩阵类型：MatrixXf / MatrixXd / Matrix3f 等 ===
    template <typename Derived>
    typename std::enable_if<
        std::is_base_of<Eigen::MatrixBase<Derived>, Derived>::value &&
        Derived::ColsAtCompileTime != 1,
        Derived>::type
    getParam(ros::NodeHandle& nh, const std::string& name, int rows, int cols, const Derived& default_val) {
        std::vector<typename Derived::Scalar> tmp;
        if (nh.getParam(name, tmp)) {
            if ((int)tmp.size() != rows * cols) {
                ROS_ERROR_STREAM("Matrix param '" << name << "' size mismatch: expected "
                                                  << rows * cols << ", got " << tmp.size()
                                                  << ". Using default.");
                return default_val;
            }

            Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> mat =
                Eigen::Map<const Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic>>(
                    tmp.data(), rows, cols);
            return mat;
        } else {
            ROS_WARN_STREAM("Matrix param '" << name << "' not found. Using default.");
            return default_val;
        }
    }

}  // namespace eigen_param_utils