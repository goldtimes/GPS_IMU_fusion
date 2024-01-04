#include "data_flow.hpp"
#include <fstream>
#include <iostream>
#include "utils.hpp"
void GPSFlow::LLA2ENU(GPSData& gps_data) {
    geo_converter_.Forward(gps_data.lla.x(), gps_data.lla.y(), gps_data.lla.z(), gps_data.position_enu.x(), gps_data.position_enu.y(),
                           gps_data.position_enu.z());
}

bool GPSFlow::ReadGPSData(const std::string& file_path, std::deque<GPSData>& gps_datas, const int skil_rows) {
    bool first_data = true;
    std::string gps_file_path = file_path + "/gps-0.csv";
    std::string ref_gsp_file_path = file_path + "/ref_gps.csv";
    std::string time_file_path = file_path + "/gps_tiem.csv";

    std::ifstream gps_file(gps_file_path, std::ios_base::in);
    std::ifstream ref_gps_file(ref_gsp_file_path, std::ios_base::in);
    std::ifstream gps_time_file(time_file_path, std::ios_base::in);
    //  确认文件是否打开
    if (!gps_file.is_open() || !ref_gps_file.is_open() || !gps_time_file.is_open()) {
        std::cerr << "failure to open gps file" << std::endl;
    }

    GPSData gps_data;
    gps_datas.clear();

    std::string gps_data_line;
    std::string ref_gps_data_line;
    std::string gps_time_line;
    std::string temp;
    // 跳过第一行
    for (int i = 0; i < skil_rows; ++i) {
        std::getline(gps_file, temp);
        std::getline(ref_gps_file, temp);
        std::getline(gps_time_file, temp);
    }

    while (std::getline(gps_file, gps_data_line) && std::getline(ref_gps_file, ref_gps_data_line) && std::getline(gps_time_file, gps_time_line)) {
        gps_data.time = std::stod(gps_time_line);
        // 利用stringstream 分割字符串
        std::stringstream ss1;
        std::stringstream ss2;

        ss1 << gps_data_line;
        ss2 << ref_gps_data_line;
        // x
        std::getline(ss1, temp, ',');
        gps_data.lla.x() = std::stod(temp);
        // y
        std::getline(ss1, temp, ',');
        gps_data.lla.y() = std::stod(temp);
        // z
        std::getline(ss1, temp, ',');
        gps_data.lla.z() = std::stod(temp);
        // v_x
        std::getline(ss1, temp, ',');
        gps_data.velocity.x() = std::stod(temp);
        // v_y
        std::getline(ss1, temp, ',');
        gps_data.velocity.y() = std::stod(temp);
        // v_z
        std::getline(ss1, temp, ',');
        gps_data.velocity.z() = std::stod(temp);

        // 将速度转换到世界坐标系
        TransformCoordinate(gps_data.velocity);
        std::getline(ss2, temp, ',');
        gps_data.true_lla.x() = std::stod(temp);

        std::getline(ss2, temp, ',');
        gps_data.true_lla.y() = std::stod(temp);

        std::getline(ss2, temp, ',');
        gps_data.true_lla.z() = std::stod(temp);

        std::getline(ss2, temp, ',');
        gps_data.true_velocity.x() = std::stod(temp);

        std::getline(ss2, temp, ',');
        gps_data.true_velocity.y() = std::stod(temp);

        std::getline(ss2, temp, ',');
        gps_data.true_velocity.z() = std::stod(temp);

        // 将速度转换到世界坐标系
        TransformCoordinate(gps_data.true_velocity);

        if (first_data) {
            first_data = false;
            geo_converter_.Reset(gps_data.lla.x(), gps_data.lla.y(), gps_data.lla.z());
        }
        LLA2ENU(gps_data);
        gps_datas.emplace_back(gps_data);
    }

    return true;
}

bool IMUFlow::ReadIMUData(const std::string& file_path, std::deque<IMUData>& imu_datas, int skip_rows) {
    std::string acc_file_path = file_path + "/accel-0.csv";  // 加速度
    std::string ref_acc_file_path = file_path + "/ref_accel.csv";
    std::string gyro_file_path = file_path + "/gyro-0.csv";
    std::string ref_gyro_file_path = file_path + "/ref_gyro.csv";
    std::string time_file_path = file_path + "/time.csv";

    std::ifstream acc_file(acc_file_path, std::ios_base::in);
    std::ifstream ref_acc_file(ref_acc_file_path, std::ios_base::in);
    std::ifstream gyro_file(gyro_file_path, std::ios_base::in);
    std::ifstream ref_gyro_file(ref_gyro_file_path, std::ios_base::in);
    std::ifstream time_file(time_file_path, std::ios_base::in);

    if (!acc_file.is_open() || !ref_acc_file.is_open() || !gyro_file.is_open() || !ref_gyro_file.is_open() || !time_file.is_open()) {
        std::cerr << "failure to open imu file" << std::endl;
    } else {
        std::cout << "open imu file success" << std::endl;
    }
    IMUData imu_data;
    imu_datas.clear();

    std::string accel_line;
    std::string ref_accel_line;
    std::string gyro_line;
    std::string ref_gyro_line;
    std::string time_line;
    std::string temp;

    for (int i = 0; i < skip_rows; ++i) {
        std::getline(acc_file, temp);
        std::getline(ref_acc_file, temp);
        std::getline(gyro_file, temp);
        std::getline(ref_gyro_file, temp);
        std::getline(time_file, temp);
    }

    while (std::getline(acc_file, accel_line) && std::getline(ref_acc_file, ref_accel_line) && std::getline(gyro_file, gyro_line) &&
           std::getline(ref_gyro_file, ref_gyro_line) && std::getline(time_file, time_line)) {
        imu_data.time = std::stod(time_line);

        std::stringstream ss;
        ss << accel_line;

        std::getline(ss, temp, ',');
        imu_data.linear_acc.x() = std::stod(temp);
        std::getline(ss, temp, ',');
        imu_data.linear_acc.y() = std::stod(temp);
        std::getline(ss, temp, ',');
        imu_data.linear_acc.z() = std::stod(temp);

        TransformCoordinate(imu_data.linear_acc);

        ss.clear();
        ss << ref_accel_line;
        std::getline(ss, temp, ',');
        imu_data.true_linear_acc.x() = std::stod(temp);
        std::getline(ss, temp, ',');
        imu_data.true_linear_acc.y() = std::stod(temp);
        std::getline(ss, temp, ',');
        imu_data.true_linear_acc.z() = std::stod(temp);

        TransformCoordinate(imu_data.true_linear_acc);

        ss.clear();
        ss << gyro_line;
        std::getline(ss, temp, ',');
        imu_data.angular_vel.x() = std::stod(temp) * kDegree2Radian;
        std::getline(ss, temp, ',');
        imu_data.angular_vel.y() = std::stod(temp) * kDegree2Radian;
        std::getline(ss, temp, ',');
        imu_data.angular_vel.z() = std::stod(temp) * kDegree2Radian;

        TransformCoordinate(imu_data.angular_vel);

        ss.clear();
        ss << ref_gyro_line;
        std::getline(ss, temp, ',');
        imu_data.true_angular_vel.x() = std::stod(temp) * kDegree2Radian;
        std::getline(ss, temp, ',');
        imu_data.true_angular_vel.y() = std::stod(temp) * kDegree2Radian;
        std::getline(ss, temp, ',');
        imu_data.true_angular_vel.z() = std::stod(temp) * kDegree2Radian;

        TransformCoordinate(imu_data.true_angular_vel);

        imu_datas.emplace_back(imu_data);
    }

    return true;
}