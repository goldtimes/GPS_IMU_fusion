#include "ekf.hpp"
#include "3rd/sophus/se3.hpp"


constexpr double kDegree2Raidian = M_PI / 180;

EKF::EKF(const YAML::Node& node)
{
    std::string data_path = node["data_path"].as<std::string>();
    std::string cov_node_string;
    if (data_path == "/data/raw_data")
    {
        cov_node_string = "covariance";
    }
    else if (data_path == "/data/raw_data1")
    {
        cov_node_string = "covariance1";
    }
    else 
    {
        exit(0);
    }
    double gravity = node["earth"]["gravity"].as<double>();

    double cov_prior_position = node["EKF"][cov_node_string]["prior"]["posi"].as<double>();
    double cov_prior_vel = node["EKF"][cov_node_string]["prior"]["vel"].as<double>();
    double cov_prior_ori = node["EKF"][cov_node_string]["prior"]["ori"].as<double>();
    double cov_prior_gyro_delta = node["EKF"][cov_node_string]["prior"]["gyro_delta"].as<double>();
    double cov_prior_acc_delta = node["EKF"][cov_node_string]["prior"]["accel_delta"].as<double>();

    double cov_measurement_pos = node["EKF"][cov_node_string]["measurement"]["posi"].as<double>();

    // 系统的过程噪声
    double cov_process_gyro = node["EKF"][cov_node_string]["process"]["gyro_delta"].as<double>();
    double cov_process_acc = node["EKF"][cov_node_string]["process"]["accel_delta"].as<double>();
    // imu 噪声
    double cov_w_gyro = node["EKF"][cov_node_string]["IMU_noise"]["gyro_delta"].as<double>();
    double cov_w_acc = node["EKF"][cov_node_string]["IMU_noise"]["accel_delta"].as<double>();

    // 设置ekf的先验协方差
    SetConvarianceP(cov_prior_position, cov_prior_vel, cov_prior_ori, cov_prior_gyro_delta, cov_prior_acc_delta);
    //  设置观测噪声R
    SetConvarianceR(cov_measurement_pos);
    // 设置系统的过程噪声Q
    SetConvarianceQ(cov_process_gyro, cov_process_acc);
    // 设置零偏
    SetConvarianceW(cov_w_gyro, cov_w_acc);
    // 重力加速度    
    g_ = Eigen::Vector3d(0.0, 0.0, -gravity);
}
// 设置误差误差协方差
void EKF::SetConvarianceP(double position_noise, double vel_noise, double ore_noise, double gyro_noise, double acc_noise)
{
    P_.setZero();
    P_.block<3,3>(INDEX_STATE_POSI, INDEX_MEASUREMENT_POSI) = Eigen::Matrix3d::Identity() * position_noise;
    P_.block<3,3>(INDEX_STATE_VEL, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity() * vel_noise;
    P_.block<3,3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = Eigen::Matrix4d::Identity() * ore_noise;
    P_.block<3,3>(INDEX_STATE_GYRO_BIAS, INDEX_STATE_GYRO_BIAS) = Eigen::Matrix3d::Identity() * gyro_noise;
    P_.block<3,3>(INDEX_STATE_ACC_BIAS, INDEX_STATE_ACC_BIAS) = Eigen::Matrix3d::Identity() * acc_noise;
    std::cout << "P_: \n" << P_ << std::endl;
}

// 设置观测噪声3*3
void EKF::SetConvarianceR(double position_noise_cov)
{
    R_.setZero();
    R_ = Eigen::Matrix3d::Identity() * position_noise_cov * position_noise_cov;
    std::cout << "R_: \n" << R_ << std::endl;
}

// Q 过程噪声 6*6
void EKF::SetConvarianceQ(double gyro_noise_cov, double acc_noise_cov)
{
    Q_.setZero();
    Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * gyro_noise_cov * gyro_noise_cov;
    Q_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * acc_noise_cov * acc_noise_cov;
    std::cout << "Q_: \n" << Q_ << std::endl;
}

void EKF::SetConvarianceW(double gyro_w_noise, double acc_w_noise)
{
    W_.setZero();
    W_.block<3,1>(0,0) = Eigen::Vector3d(gyro_w_noise, gyro_w_noise, gyro_w_noise);
    W_.block<3,1>(3,0) = Eigen::Vector3d(acc_w_noise, acc_w_noise, acc_w_noise);
    std::cout << "W_: \n" << W_ << std::endl; 
}

bool EKF::Init(const GPSData& gps_data, const IMUData& imu_data)
{
    imu_data_queue_.clear();
    imu_data_queue_.push_back(imu_data);
    this->gps_data_ = gps_data;
    X_.setZero();
    X_.block<3,1>(INDEX_STATE_POSI, 0) = gps_data_.position_enu;
    X_.block<3,1>(INDEX_STATE_VEL, 0) = gps_data_.true_velocity;
    Eigen::Quaterniond qua = Eigen::AngleAxisd(90 * kDegree2Raidian, Eigen::Vector3d::UnitZ())*
                            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) * 
                            Eigen::AngleAxisd(180 * kDegree2Raidian, Eigen::Vector3d::UnitY());
    X_(INDEX_STATE_ORI, 0) = qua.w(); 
    X_(INDEX_STATE_ORI + 1, 0) = qua.x(); 
    X_(INDEX_STATE_ORI + 2, 0) = qua.y(); 
    X_(INDEX_STATE_ORI + 3, 0) = qua.z(); 

    return true;
}

bool EKF::Predict(const IMUData& imu_data)
{
    imu_data_queue_.push_back(imu_data);
    double delta_time = imu_data.time - imu_data_queue_.front().time;

    Eigen::Vector3d curr_acc = imu_data.linear_acc; // imu坐标系下的加速度
    Eigen::Vector3d curr_angular_vel = imu_data.angular_vel; 
    Eigen::Quaterniond curr_orien = Eigen::Quaterniond(pose_.block<3,3>(0,0));
    UpdateEKFState(delta_time, curr_acc, curr_angular_vel, curr_orien);
    UpdateState(); 
    imu_data_queue_.pop_front();

    return true;
}

// 非线性化时候的jacobian矩阵
bool EKF::UpdateEKFState(const double t, const Eigen::Vector3d& acc, const Eigen::Vector3d& curr_angle_velocity, const Eigen::Quaterniond& curr_ori)
{
    F_.setZero();
    // 微分方程对p的求导
    F_.block<3,3>(INDEX_STATE_POSI, INDEX_STATE_POSI) = Eigen::Matrix3d::Identity();

    double q0 = curr_ori.w();
    double q1 = curr_ori.x();
    double q2 = curr_ori.y();
    double q3 = curr_ori.z();
    double FVq0 = 2 * Eigen::Vector3d(q0,-q3,q2).transpose() * acc;
    double FVq1 = 2 * Eigen::Vector3d(q1,q2,q3).transpose() * acc;
    double FVq2 = 2 * Eigen::Vector3d(-q2,q1,q0).transpose() * acc;
    double FVq3 = 2 * Eigen::Vector3d(-q3,-q0,q1).transpose() * acc;
    Eigen::Matrix<double,3,4> FVq = (Eigen::Matrix<double,3,4>()<<  FVq0,FVq1,FVq2,FVq3,
                                                                    -FVq3,-FVq2,FVq1,FVq0,
                                                                    FVq2,-FVq3,-FVq0,FVq1).finished();

    F_.block<3,4>(INDEX_STATE_VEL, INDEX_STATE_ORI) = FVq;
    F_.block<3,3>(INDEX_STATE_VEL, INDEX_STATE_ACC_BIAS) = pose_.block<3,3>(0,0);

    Eigen::Vector3d w = curr_angle_velocity;
    Eigen::Matrix<double,4,4> Fqq = 0.5* (Eigen::Matrix<double,4,4>()<<0,-w.x(),-w.y(),-w.z(),
                                                                        w.x(),0,w.z(),-w.y(),
                                                                        w.y(),-w.z(),0,w.x(),
                                                                        w.z(),w.y(),-w.x(),0).finished();
    F_.block<4,4>(INDEX_STATE_ORI,INDEX_STATE_ORI) = Fqq;

    Eigen::Matrix<double,4,3> Fqkesi  = 0.5 * (Eigen::Matrix<double,4,3>()<<-q1,-q2,-q3,
                                                                        q0,-q3,q2,
                                                                        q3,q0,-q1,
                                                                        -q2,q1,q0).finished();
    F_.block<4,3>(INDEX_STATE_ORI,INDEX_STATE_GYRO_BIAS) = Fqkesi;
    // 状态对噪声的求导 16 * 6
    B_.setZero();
    B_.block<3,3>(INDEX_STATE_VEL, 3) = pose_.block<3,3>(0, 0);
    B_.block<4,3>(INDEX_STATE_ORI, 0) = Fqkesi;

    Matrix16 F_k = Matrix16::Identity() + F_ * t;
    MatrixB B_k = B_ * t;

    X_ = F_k * X_ + B_k * W_ + gt_ * t;
    P_ = F_k * P_ + F_k.transpose() + B_k * Q_ * B_k.transpose();

    return true;

}

void EKF::UpdateState() {
    pose_.block<3,1>(0,3) =  X_.block<3,1>(INDEX_STATE_POSI, 0);

    Eigen::Quaterniond q;
    q.w() = X_(INDEX_STATE_ORI + 0, 0);
    q.x() = X_(INDEX_STATE_ORI + 1, 0);
    q.y() = X_(INDEX_STATE_ORI + 2, 0);
    q.z() = X_(INDEX_STATE_ORI + 3, 0);
    q.normalize();

    // 修改旋转矩阵
    pose_.block<3,3>(0,0) = q.toRotationMatrix();
}

bool EKF::Correct(const GPSData& gps_data)
{
    this->gps_data_ = gps_data;
    C_.setIdentity();
    G_.setZero();
    G_.block<3,3>(INDEX_MEASUREMENT_POSI, INDEX_MEASUREMENT_POSI) = Eigen::Matrix3d::Identity();
    Y_ = gps_data_.position_enu;
    K_ = P_ * G_.transpose() * ((G_ * P_ * G_.transpose()) + C_ * R_ * C_.transpose()).inverse();
    P_ = (Matrix16::Identity() - K_ * G_) * P_;
    X_ = X_ + K_ * (Y_ - G_ * X_);
    UpdateState();

    return true;
}

