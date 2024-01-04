#pragma once

#include <memory>
#include "data_read.hpp"
#include "filter_interfaces.hpp"

class DataFlow {
   public:
    DataFlow() = default;
    DataFlow(const std::string& work_dir);

    void Run();
    void ReadData();
    void SavePose(std::ofstream& of, const Eigen::Matrix4d& pose);

   private:
    std::shared_ptr<FilterInterface> filter_ptr_;  // 滤波器
    std::shared_ptr<IMUFlow> imu_flow_ptr_;
    std::shared_ptr<GPSFlow> gps_flow_ptr_;

    std::deque<GPSData> gps_data_buff_;
    std::deque<IMUData> imu_data_buff_;

    // 时间同步的imu和gps
    IMUData imu_data;
    GPSData gps_data;

    const std::string work_dir_path_;  // const 变量需要在构造函数前初始化
    std::string data_path_;
};