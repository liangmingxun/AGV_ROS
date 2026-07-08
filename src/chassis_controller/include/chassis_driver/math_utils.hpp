#pragma once

#include <Eigen/Dense>

namespace math_utils {

/**
 * @brief 对数组中每个元素按照scale进行缩放。
 *
 * @param src 源数组。
 * @param scale 缩放比例。
 * @param dst 目标数组。
 * @param size 数组长度。
 * @return true 执行成功。
 * @return false 执行失败。如空指针或参数错误。
 */
inline bool scale_f32(const float* src, const float scale, float* dst, const int size) noexcept {
    if (src == nullptr || dst == nullptr || size <= 0) {
        return false;
    }

    Eigen::Map<const Eigen::VectorXf> src_vec{src, size};
    Eigen::Map<Eigen::VectorXf> dst_vec{dst, size};
    dst_vec = src_vec * scale;

    return true;
}

/**
 * @brief 对数组中每个元素取相反数。
 *
 * @param src 源数组。
 * @param dst 目标数组。
 * @param size 数组长度。
 * @return true 执行成功。
 * @return false 执行失败。如空指针或参数错误。
 */
inline bool negate_f32(const float* src, float* dst, const int size) noexcept {
    return scale_f32(src, -1.0f, dst, size);
}

/**
 * @brief 计算两个数组的逐元素和。
 *
 * @param src_a 源数组。
 * @param src_b 源数组。
 * @param dst 目标数组。
 * @param size 数组长度。
 * @return true 执行成功。
 * @return false 执行失败。如空指针或参数错误。
 */
inline bool add_f32(const float* src_a, const float* src_b, float* dst, const int size) noexcept {
    if (src_a == nullptr || src_b == nullptr || dst == nullptr || size <= 0) {
        return false;
    }

    Eigen::Map<const Eigen::VectorXf> src_a_vec{src_a, size};
    Eigen::Map<const Eigen::VectorXf> src_b_vec{src_b, size};
    Eigen::Map<Eigen::VectorXf> dst_vec{dst, size};
    dst_vec = src_a_vec + src_b_vec;

    return true;
}

/**
 * @brief 计算两个数组的逐元素差。
 *
 * @param src_a 被减数组。
 * @param src_b 减数组。
 * @param dst 目标数组。
 * @param size 数组长度。
 * @return true 执行成功。
 * @return false 执行失败。如空指针或参数错误。
 */
inline bool sub_f32(const float* src_a, const float* src_b, float* dst, const int size) noexcept {
    if (src_a == nullptr || src_b == nullptr || dst == nullptr || size <= 0) {
        return false;
    }

    Eigen::Map<const Eigen::VectorXf> src_a_vec{src_a, size};
    Eigen::Map<const Eigen::VectorXf> src_b_vec{src_b, size};
    Eigen::Map<Eigen::VectorXf> dst_vec{dst, size};
    dst_vec = src_a_vec - src_b_vec;

    return true;
}

}  // namespace math_utils