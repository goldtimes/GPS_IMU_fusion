#include "data_flow.hpp"

int main(int argc, char** argv) {
    std::string work_dir_path = "/home/GPS_IMU_fusion";
    DataFlow data_flow(work_dir_path);

    data_flow.Run();
    return 0;
}