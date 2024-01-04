#pragma once

#include <eigen3/Eigen/Dense>

constexpr double kDegree2Radian = M_PI / 180;

/**
 * @brief 前右地-右前天,因为gps转换的是右前天,将imu的数据转换到右前天
 */
inline void TransformCoordinate(Eigen::Vector3d& vec) {
    // T_w_b
    Eigen::Quaterniond Q_b_w = Eigen::AngleAxisd(90 * kDegree2Radian, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(180 * kDegree2Radian, Eigen::Vector3d::UnitX());
    // 世界坐标系转到body坐标系
    vec = Q_b_w.inverse() * vec;
}

// 反对称矩阵
inline Eigen::Matrix3d skew(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d matrix;
    // clang-format off
    matrix << 0.0, -vec[2], vec[1], 
            vec[2], 0.0, -vec[0],
            -vec[1], vec[0], 0.0;
    // clang-format on
    return matrix;
}