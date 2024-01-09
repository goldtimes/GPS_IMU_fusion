#include "data_flow.hpp"
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <string>
#include "config_parameters.hpp"
#include "ekf.hpp"
#include "eskf.hpp"
#include "filter_interfaces.hpp"
#include "utils.hpp"

DataFlow::DataFlow(const std::string& work_dir) : work_dir_path_(work_dir) {
    std::string config_file_path = work_dir + "/config/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    data_path_ = config_node["data_path"].as<std::string>();
    std::string filter_name = config_node["filter_name"].as<std::string>();
    if (filter_name == "EKF") {
        filter_ptr_ = std::make_shared<EKF>(config_node);
    } else if (filter_name == "ESKF") {
        std::string eskf_config = work_dir + "/config/eskf.yaml";
        ConfigParameters config_params;
        config_params.LoadParameters(eskf_config);
        filter_ptr_ = std::make_shared<ESKF>(config_params);
    }
    // 初始化滤波器
    std::cout << "data_path_:" << data_path_ << "filter_method:" << filter_name << std::endl;
}

bool DataFlow::ReadData() {
    const std::string data_path = work_dir_path_ + data_path_;
    if (boost::filesystem::exists(data_path)) {
        if (imu_flow_ptr_->ReadIMUData(data_path, imu_data_buff_) && gps_flow_ptr_->ReadGPSData(data_path, gps_data_buff_)) {
            return true;
        } else {
            return false;
        }
    }
    return false;
}

bool DataFlow::sync_data() {
    sync_imu_data = imu_data_buff_.front();
    sync_gps_data = gps_data_buff_.front();

    double delta_time = sync_imu_data.time - sync_gps_data.time;
    // imu晚于gps 0.01
    if (delta_time > 0.01) {
        gps_data_buff_.pop_front();
        return false;
    }
    // gps晚于imu 0.01
    if (delta_time < -0.01) {
        imu_data_buff_.pop_front();
        return false;
    }
    imu_data_buff_.pop_front();
    gps_data_buff_.pop_front();
    return true;
}

bool DataFlow::Run() {
    ReadData();
    std::cout << "data read successfully, imu size: %ld" << imu_data_buff_.size() << ",gps size: %ld" << gps_data_buff_.size() << std::endl;
    while (!imu_data_buff_.empty() && !gps_data_buff_.empty()) {
        if (!sync_data()) {  //确保初始的gps和imu时间不相差10ms
            continue;
        } else {
            filter_ptr_->Init(sync_gps_data, sync_imu_data);
            break;
        }
    }

    std::ofstream gt_file(work_dir_path_ + "/data/gt.txt", std::ios_base::trunc);
    std::ofstream fused_file(work_dir_path_ + "/data/fused.txt", std::ios_base::trunc);
    std::ofstream measured_file(work_dir_path_ + "/data/measured.txt", std::ios_base::trunc);
    // 循环处理数据
    while (!imu_data_buff_.empty() && !gps_data_buff_.empty()) {
        sync_imu_data = imu_data_buff_.front();
        sync_gps_data = gps_data_buff_.front();
        if (sync_imu_data.time < sync_gps_data.time) {
            // imu的数据比gps数据早,然后就利用imu数据预测
            filter_ptr_->Predict(sync_imu_data);
            imu_data_buff_.pop_front();
        } else {
            // 利用gps数据修正滤波器
            filter_ptr_->Correct(sync_gps_data);
            SavePose(fused_file, filter_ptr_->GetPose());
            SavePose(measured_file, Vector2Matrix(sync_gps_data.position_enu));
            SavePose(gt_file, Vector2Matrix(gps_flow_ptr_->LLA2ENU(sync_gps_data.true_lla)));
            gps_data_buff_.pop_front();
        }
    }

    return true;
}