#pragma once

#include <deque>
#include <iostream>
#include <string>
#include "3rd/GeographicLib/include/Geocentric/LocalCartesian.hpp"
#include "sensor.hpp"

/**
 * @brief 读取gps数据
 */
class GPSFlow {
   public:
    GPSFlow() = default;

    Eigen::Vector3d LLA2ENU(const Eigen::Vector3d& lla) {
        Eigen::Vector3d enu;
        geo_converter_.Forward(lla[0], lla[1], lla[2], enu[0], enu[1], enu[2]);
        return enu;
    }

    void LLA2ENU(GPSData& gpd_data);

    Eigen::Vector3d LLA2ENU(const Eigen::Vector3d& lla);

    bool ReadGPSData(const std::string& file_path, std::deque<GPSData>& gps_datas, const int skip_rows = 1);

   private:
    GeographicLib::LocalCartesian geo_converter_{30.0, 120.0, 0.0};
};

/**
 * @brief 读取imu数据
 */
class IMUFlow {
   public:
    IMUFlow() = default;

    bool ReadIMUData(const std::string& file_path, std::deque<IMUData>& imu_datas, const int skip_rows = 1);
};