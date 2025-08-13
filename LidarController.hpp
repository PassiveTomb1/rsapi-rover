#pragma once

#include "core/ldlidar_driver.h"
#include <vector>

class LidarController {
public:
    // Constructor initializes and starts the lidar
    LidarController();
    
    // Destructor stops the lidar and cleans up
    ~LidarController();

    // Gets a single scan, filters for the front half, and returns the data
    std::vector<ldlidar::PointData> GetLidarScan();

    // Checks if the lidar is currently running and active
    bool IsRunning() const;

private:
    // The lidar driver instance
    ldlidar::LDLidarDriver* _lidarDriver;
    bool _isRunning;

    // A static timestamp function required by the lidar SDK
    static uint64_t GetTimestampFunctional();
};