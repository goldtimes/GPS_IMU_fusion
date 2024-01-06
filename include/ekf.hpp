#pragma once

#include "sensor.hpp"
#include "filter_interfaces.hpp"
#include <deque>
#include <yaml-cpp/yaml.h>
#include "eigen_type.hpp"

class EKF : public FilterInterface
{
    public:
        EKF(const YAML::Node & node);
        virtual ~EKF() = default;

        virtual bool Init(const GPSData& gps_data, const IMUData& imu_data) override;
        virtual bool Predict(const IMUData& imu_data) override;
        virtual bool Correct(const GPSData& gps_data) override;
        virtual Eigen::Matrix4d GetPose() const override;
        virtual Eigen::Vector3d GetVelocity() const override;
    private:
        void SetConvarianceQ(double gyro_noise_cov, double acc_noise_cov);
        void SetConvarianceW(double gyro_w_noise, double acc_w_noise);
        void SetConvarianceR(double position_noise_cov);
        void SetConvarianceP(double position_noise, double vel_noise, double ore_noise, double gyro_noise, double acc_noise);

        bool UpdateOdomEstimation();
        bool UpdateEKFState(const double t, const Eigen::Vector3d& acc, const Eigen::Vector3d& curr_angle_velocity, const Eigen::Quaterniond& curr_qua);
        void UpdateState();

    private:
        // state
        Vec16d X_;
        // measurement
        Vec3d Y_;
        // state_F jacobian
        Matrix16 F_;
        // state_B jacobian
        MatrixB B_;
        // W
        Vec6d W_; // 加速度和陀螺仪的噪声
        // Q 
        Matrix6 Q_;
        // P 误差协方差 16 * 16
        Matrix16 P_;
        // K 
        MatrixK K_;
        // C_
        Matrix3 C_;
        // G_, 观测量对状态量的jacobian 3*16
        MatrixG G_; 
        // R_;
        Matrix3 R_;

        Vec16d gt_;

        Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
        Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();

        Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();

        Eigen::Vector3d g_; // 重力加速度

        GPSData gps_data_;
        std::deque<IMUData> imu_data_queue_; // 只保存两帧的imu数据
};