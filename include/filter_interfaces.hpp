#pragma once

#include "sensor.hpp"

/**
 * @brief ekf, eskf的抽象接口
 */
class FilterInterface {
   public:
    virtual ~FilterInterface() = default;

    /**
     * @brief 滤波器的初始化
     */
    virtual bool Init(const GPSData& curr_gps_data, const IMUData& curr_imu_data) = 0;
    /**
     * @brief 滤波器系统预测
     */
    virtual bool Predict(const IMUData& imu_data) = 0;
    /**
     * @brief 利用观测更新
     */
    virtual bool Correct(const GPSData& gps_data) = 0;

    virtual Eigen::Matrix4d GetPose() const = 0;
    virtual Eigen::Vector3d GetVelocity() const = 0;
};