#include "eskf.hpp"
#include "data_read.hpp"
#include "utils.hpp"

ESKF::ESKF(const ConfigParameters &params) : config_parameters_(params) {
    earth_rotation_speed_ = config_parameters_.earth_rotation_speed_;
    g_ = Eigen::Vector3d(0.0, 0.0, -config_parameters_.earth_gravity_);
    // 设置先验的协方差
    SetCovarianceP(config_parameters_.position_error_prior_std_, config_parameters_.velocity_error_prior_std_,
                   config_parameters_.rotation_error_prior_std_, config_parameters_.gyro_bias_error_prior_std_,
                   config_parameters_.accelerometer_noise_std_);
    // 观测的噪声
    SetCovarianceR(config_parameters_.gps_position_x_std_, config_parameters_.gps_position_y_std_, config_parameters_.gps_position_z_std_);
    SetCovarianceQ(config_parameters_.gyro_noise_std_, config_parameters_.accelerometer_noise_std_);

    // 状态量
    X_.setZero();
    // 系统的jacobian矩阵
    F_.setZero();
    C_.Identity();  // 这个矩阵可有可无,update阶段计算增益的时候用
    // gps的观测jacobian 对误差状态量为单位矩阵
    G_.block<3, 3>(INDEX_MEASUREMENT_POSI, INDEX_MEASUREMENT_POSI) = Eigen::Matrix3d::Identity();
}
// 系统噪声
void ESKF::SetCovarianceQ(double gyro_nosise, double accel_noise) {
    Q_.setZero();
    Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * gyro_nosise * gyro_nosise;
    Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * accel_noise * accel_noise;
}

void ESKF::SetCovarianceP(double posi_noise, double vel_noise, double ori_noise, double gyro_noise, double acc_noise) {
    P_.setZero();
    P_.block<3, 3>(INDEX_STATE_POSI, INDEX_STATE_POSI) = Eigen::Matrix3d::Identity() * posi_noise * posi_noise;
    P_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity() * vel_noise * vel_noise;
    P_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = Eigen::Matrix3d::Identity() * ori_noise * ori_noise;
    P_.block<3, 3>(INDEX_STATE_GYRO_BIAS, INDEX_STATE_GYRO_BIAS) = Eigen::Matrix3d::Identity() * gyro_noise * gyro_noise;
    P_.block<3, 3>(INDEX_STATE_ACC_BIAS, INDEX_STATE_ACC_BIAS) = Eigen::Matrix3d::Identity() * acc_noise * acc_noise;
}

// 观测噪声
void ESKF::SetCovarianceR(double position_x_std, double position_y_std, double position_z_std) {
    R_.Zero();
    R_(0, 0) = position_x_std * position_x_std;
    R_(1, 1) = position_x_std * position_x_std;
    R_(2, 2) = position_x_std * position_x_std;
}

bool ESKF::Init(const GPSData &gps_data, const IMUData &imu_data) {
    velocity_ = gps_data.true_velocity;
    Eigen::Quaterniond Q_init = Eigen::AngleAxisd(0 * kDegree2Radian, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(0 * kDegree2Radian, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(0 * kDegree2Radian, Eigen::Vector3d::UnitX());

    pose_.block<3, 3>(0, 0) = Q_init.toRotationMatrix();
    pose_.block<3, 1>(0, 3) = gps_data.position_enu;

    imu_data_buff_.clear();
    imu_data_buff_.push_back(imu_data);

    curr_gps_data_ = gps_data;
    return true;
}

void ESKF::GetFGY(TypeMatrixF &F, TypeMatrixG &G, TypeVectorY &Y) {
    F = Ft_;  // 系统jacobian
    G = G_;   // 观测jacobian
    Y = Y_;   // 观测数据
}

bool ESKF::Predict(const IMUData &imu_data) {
    imu_data_buff_.push_back(imu_data);
    Eigen::Vector3d w_in = Eigen::Vector3d::Zero();
    if (config_parameters_.use_earth_model_) {
        w_in = ComputeNavigationFrameAngularVelocity();  // 时刻 m-1 -> m 地球转动引起的导航系转动角速度
    }
    UpdateOdomEstimation(w_in);
    double delta_t = imu_data.time - imu_data_buff_.front().time;

    Eigen::Vector3d curr_accel = pose_.block<3, 3>(0, 0) * imu_data.linear_acc;

    UpdateErrorState(delta_t, curr_accel, w_in);

    imu_data_buff_.pop_front();

    return true;
}

void ESKF::UpdateOdomEstimation(const Eigen::Vector3d &w_in) {
    const auto &last_imu_data = imu_data_buff_.at(0);
    const auto &curr_imu_data = imu_data_buff_.at(1);
    const double delta_t = curr_imu_data.time - last_imu_data.time;

    Eigen::Vector3d delta_rotation = ComputeDeltaRotation(last_imu_data, curr_imu_data);

    const Eigen::Vector3d phi_in = w_in * delta_t;
    const Eigen::AngleAxisd angle_axisd(phi_in.norm(), phi_in.normalized());
    const Eigen::Matrix3d R_nm_nm_1 = angle_axisd.toRotationMatrix().transpose();

    Eigen::Matrix3d curr_R;  // R_n_m m时刻的旋转
    Eigen::Matrix3d last_R;  // C_n_m_1 m-1时刻的旋转
    ComputeOrientation(delta_rotation, R_nm_nm_1, curr_R, last_R);

    Eigen::Vector3d curr_vel;  // 当前时刻导航系下的速度
    Eigen::Vector3d last_vel;  // 上一时刻导航系下的速度
    ComputeVelocity(last_R, curr_R, last_imu_data, curr_imu_data, last_vel, curr_vel);

    ComputePosition(last_vel, curr_vel, last_imu_data, curr_imu_data);
}

Eigen::Vector3d ESKF::ComputeDeltaRotation(const IMUData &imu_data_0, const IMUData &imu_data_1) {
    const double delta_t = imu_data_1.time - imu_data_0.time;
    const Eigen::Vector3d &unbias_gyro_0 = ComputeUnbiasGyro(imu_data_0.angular_vel);
    const Eigen::Vector3d &unbias_gyro_1 = ComputeUnbiasGyro(imu_data_1.angular_vel);

    Eigen::Vector3d delta_theta = 0.5 * (unbias_gyro_0 + unbias_gyro_1) * delta_t;
    return delta_theta;
}

void ESKF::ComputeOrientation(const Eigen::Vector3d &angular_delta, const Eigen::Matrix3d &R_nm_nm_1, Eigen::Matrix3d &curr_R,
                              Eigen::Matrix3d &last_R) {
    Eigen::AngleAxisd angle_axisd(angular_delta.norm(), angular_delta.normalized());

    last_R = pose_.block<3, 3>(0, 0);
    curr_R = R_nm_nm_1.transpose() * pose_.block<3, 3>(0, 0) * angle_axisd.toRotationMatrix();
    pose_.block<3, 3>(0, 0) = curr_R;
}

void ESKF::ComputeVelocity(const Eigen::Matrix3d &R_0, const Eigen::Matrix3d &R_1, const IMUData &imu_data_0, const IMUData &imu_data_1,
                           Eigen::Vector3d &last_vel, Eigen::Vector3d &curr_vel) {
    double delta_t = imu_data_1.time - imu_data_0.time;

    CHECK_GT(delta_t, 0.0) << "IMU timestamp error";

    Eigen::Vector3d unbias_accel_0 = R_0 * ComputeUnbiasAccel(imu_data_0.linear_acc) - g_;
    Eigen::Vector3d unbias_accel_1 = R_1 * ComputeUnbiasAccel(imu_data_1.linear_acc) - g_;

    last_vel = velocity_;

    // 中值积分
    velocity_ += delta_t * 0.5 * (unbias_accel_0 + unbias_accel_1);

    curr_vel = velocity_;
}

/*!
 * 通过imu计算当前位移
 * @param curr_vel
 * @param last_vel
 * @return
 */
void ESKF::ComputePosition(const Eigen::Vector3d &last_vel, const Eigen::Vector3d &curr_vel, const IMUData &imu_data_0, const IMUData &imu_data_1) {
    double delta_t = imu_data_1.time - imu_data_0.time;

    pose_.block<3, 1>(0, 3) += 0.5 * delta_t * (curr_vel + last_vel) + 0.25 * (imu_data_0.linear_acc + imu_data_1.linear_acc) * delta_t * delta_t;
}

/**
 * 消费级的imu可以忽略此项
 */

Eigen::Vector3d ESKF::ComputeUnbiasAccel(const Eigen::Vector3d &accel) { return accel - accel_bias_; }

Eigen::Vector3d ESKF::ComputeUnbiasGyro(const Eigen::Vector3d &gyro) { return gyro - gyro_bias_; }

void ESKF::UpdateErrorState(double dt, const Eigen::Vector3d &accel, const Eigen::Vector3d &w_in) {
    Eigen::Matrix3d F_23 = BuildSkewSymmetricMatrix(accel);
    Eigen::Matrix3d F_33 = -BuildSkewSymmetricMatrix(w_in);

    F_.block<3, 3>(INDEX_STATE_POSI, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity();
    F_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_ORI) = F_23;
    F_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = F_33;
    F_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_ACC_BIAS) = pose_.block<3, 3>(0, 0);
    F_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_GYRO_BIAS) = -pose_.block<3, 3>(0, 0);
    B_.block<3, 3>(INDEX_STATE_VEL, 3) = pose_.block<3, 3>(0, 0);
    B_.block<3, 3>(INDEX_STATE_ORI, 0) = -pose_.block<3, 3>(0, 0);

    TypeMatrixF Fk = TypeMatrixF::Identity() + F_ * dt;
    TypeMatrixB Bk = B_ * dt;

    // 用于可观测性分析
    Ft_ = F_ * dt;
    X_ = Fk * X_;
    P_ = Fk * P_ * Fk.transpose() + Bk * Q_ * Bk.transpose();
}

Eigen::Vector3d ESKF::ComputeNavigationFrameAngularVelocity() {
    const double latitude = curr_gps_data_.lla.y() * kDegree2Radian;
    const double height = curr_gps_data_.lla.z();

    constexpr double f = 1.0 / 298.257223563;  // 椭球扁率

    constexpr double Re = 6378137.0;                     // 椭圆长半轴
    constexpr double Rp = (1.0 - f) * Re;                // 椭圆短半轴
    const double e = std::sqrt(Re * Re - Rp * Rp) / Re;  // 椭圆的偏心率

    const double Rn = Re / std::sqrt(1.0 - e * e * std::sin(latitude) * std::sin(latitude));                            // 子午圈主曲率半径
    const double Rm = Re * (1.0 - e * e) / std::pow(1.0 - e * e * std::sin(latitude) * std::sin(latitude), 3.0 / 2.0);  // 卯酉圈主曲率半径

    // 由于载体在地球表面运动造成的导航系姿态变化。在导航系下表示
    Eigen::Vector3d w_en_n;
    w_en_n << velocity_[1] / (Rm + height), -velocity_[0] / (Rn + height), -velocity_[1] / (Rn + height) * std::tan(latitude);

    Eigen::Vector3d w_ie_n;
    w_ie_n << earth_rotation_speed_ * std::cos(latitude), 0.0, -earth_rotation_speed_ * std::sin(latitude);

    Eigen::Vector3d w_in_n = w_en_n + w_ie_n;

    return w_in_n;
}

/*!
 * 每次矫正之后，需要重置状态变量X
 */
void ESKF::ResetState() { X_.setZero(); }

bool ESKF::Correct(const GPSData &gps_data) {
    curr_gps_data_ = gps_data;

    Y_ = gps_data.position_enu - pose_.block<3, 1>(0, 3);

    K_ = P_ * G_.transpose() * (G_ * P_ * G_.transpose() + C_ * R_ * C_.transpose()).inverse();

    P_ = (TypeMatrixP::Identity() - K_ * G_) * P_;
    X_ = X_ + K_ * (Y_ - G_ * X_);

    EliminateError();

    ResetState();

    return true;
}

// 利用误差量来修正
void ESKF::EliminateError() {
    pose_.block<3, 1>(0, 3) = pose_.block<3, 1>(0, 3) + X_.block<3, 1>(INDEX_STATE_POSI, 0);

    velocity_ = velocity_ + X_.block<3, 1>(INDEX_STATE_VEL, 0);
    Eigen::Matrix3d C_nn = SO3Exp(-X_.block<3, 1>(INDEX_STATE_ORI, 0));
    pose_.block<3, 3>(0, 0) = C_nn * pose_.block<3, 3>(0, 0);

    gyro_bias_ = gyro_bias_ + X_.block<3, 1>(INDEX_STATE_GYRO_BIAS, 0);
    accel_bias_ = accel_bias_ + X_.block<3, 1>(INDEX_STATE_ACC_BIAS, 0);
}
