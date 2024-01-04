#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class IMUData {
   public:
    IMUData() = default;

    double time = 0.0;
    Eigen::Vector3d linear_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();

    Eigen::Vector3d true_linear_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d true_angular_vel = Eigen::Vector3d::Zero();
};

class GPSData {
   public:
    GPSData() = default;
    double time = 0.0;
    Eigen::Vector3d lla = Eigen::Vector3d::Zero();           // lla 坐标
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();      // 速度
    Eigen::Vector3d position_enu = Eigen::Vector3d::Zero();  // 位置

    Eigen::Vector3d true_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d true_lla = Eigen::Vector3d::Zero();
};