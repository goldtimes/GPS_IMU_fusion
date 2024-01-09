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

inline Eigen::Matrix4d Vector2Matrix(const Eigen::Vector3d& vec) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 1>(0, 3) = vec;
    return matrix;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> BuildSkewSymmetricMatrix(const Eigen::MatrixBase<Derived>& vec) {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> matrix;
    matrix << static_cast<typename Derived::Scalar>(0.0), -vec[2], vec[1], vec[2], static_cast<typename Derived::Scalar>(0.0), -vec[0], -vec[1],
        vec[0], static_cast<typename Derived::Scalar>(0.0);

    return matrix;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SO3Exp(const Eigen::MatrixBase<Derived>& v) {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
    typename Derived::Scalar theta = v.norm();
    Eigen::Matrix<typename Derived::Scalar, 3, 1> v_normalized = v.normalized();
    R = std::cos(theta) * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() +
        (typename Derived::Scalar(1.0) - std::cos(theta)) * v_normalized * v_normalized.transpose() +
        std::sin(theta) * BuildSkewSymmetricMatrix(v_normalized);

    return R;
}

inline Eigen::Vector3d LLA2ENU(const Eigen::Vector3d& lla) {}