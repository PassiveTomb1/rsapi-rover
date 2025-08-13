#include "LidarController.hpp"
#include <iostream>
#include <chrono>

// Static function implementation
uint64_t LidarController::GetTimestampFunctional() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

LidarController::LidarController() : _lidarDriver(nullptr), _isRunning(false) {
    // Instantiate the driver and register the timestamp function
    _lidarDriver = new ldlidar::LDLidarDriver();
    _lidarDriver->RegisterGetTimestampFunctional(GetTimestampFunctional);
    
    // Attempt to start the lidar
    if (!_lidarDriver->Start(ldlidar::LDType::LD_19, "/dev/ttyUSB0", 230400, ldlidar::COMM_SERIAL_MODE)) {
        std::cerr << "Error: Failed to start the lidar." << std::endl;
        delete _lidarDriver;
        _lidarDriver = nullptr;
        _isRunning = false;
        return;
    }

    std::cout << "Lidar started successfully. Waiting for lidar to warm up..." << std::endl;
    sleep(3); // Add a warm-up delay

    _isRunning = true;
}

LidarController::~LidarController() {
    if (_lidarDriver) {
        _lidarDriver->Stop();
        delete _lidarDriver;
        _lidarDriver = nullptr;
    }
}

bool LidarController::IsRunning() const {
    return _isRunning;
}

std::vector<ldlidar::PointData> LidarController::GetLidarScan() {
    std::vector<ldlidar::PointData> raw_scan;
    std::vector<ldlidar::PointData> filtered_scan;

    if (_lidarDriver->GetLaserScanData(raw_scan, 1500) == ldlidar::LidarStatus::NORMAL) {
        // Filter the raw scan data for angles from 0 to 180 degrees
        for (const auto& point : raw_scan) {
            filtered_scan.push_back(point);
        }
    }
    
    return filtered_scan;
}