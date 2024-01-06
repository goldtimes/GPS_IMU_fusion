#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


// 状态量维度 p,v,q,ba,bw 
constexpr unsigned int DIM_STATE = 16; 
// 陀螺仪和加速度计的噪声维度
constexpr unsigned int DIM_STATE_NOISE = 6;
// 观测量维度
constexpr unsigned int DIM_MEASUREMENT = 3;
// 观测噪声维度
constexpr unsigned int DIM_MEASUREMENT_NOISE = 3;
// 对状态量的下标访问
constexpr unsigned int INDEX_STATE_POSI = 0;
constexpr unsigned int INDEX_STATE_VEL = 3;
constexpr unsigned int INDEX_STATE_ORI = 6;
constexpr unsigned int INDEX_STATE_GYRO_BIAS = 10;
constexpr unsigned int INDEX_STATE_ACC_BIAS = 13;
// 对观测量的下标访问
constexpr unsigned int INDEX_MEASUREMENT_POSI = 0;

using Vec16d = Eigen::Matrix<double, DIM_STATE, 1>;
using Vec5d = Eigen::Matrix<double, 15, 1>;
using Vec3d = Eigen::Matrix<double, DIM_MEASUREMENT,1>;
using Vec6d = Eigen::Matrix<double, DIM_MEASUREMENT_NOISE, 1>;

using Matrix16 = Eigen::Matrix<double, DIM_STATE, DIM_STATE>;
using Matrix15 = Eigen::Matrix<double, 15, 15>;
using MatrixB = Eigen::Matrix<double, DIM_STATE, DIM_STATE_NOISE>;
using Matrix6 = Eigen::Matrix<double, DIM_MEASUREMENT_NOISE, DIM_MEASUREMENT_NOISE>;
using Matrix3 = Eigen::Matrix<double, 3, 3>;
// 3X16
using MatrixG = Eigen::Matrix<double, DIM_MEASUREMENT_NOISE, DIM_STATE>;
// 卡尔曼增益 16 * 3
using MatrixK = Eigen::Matrix<double, DIM_STATE, DIM_MEASUREMENT>;
// x = x+Fx+Bw





