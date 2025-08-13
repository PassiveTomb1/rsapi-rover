cmake_minimum_required(VERSION 3.10)
project(self_driving_car CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add standard build flags
add_compile_options(-Wall -Wextra -Werror -pedantic)

# Find OpenCV
find_package(OpenCV REQUIRED)

# The Lidar SDK directory
set(LD_LIDAR_SDK_DIR "/home/arjun/ldlidar_ws/ldlidar_stl_sdk")

# Include the Lidar SDK headers
include_directories(
    .
    ${LD_LIDAR_SDK_DIR}/ldlidar_driver/include
    ${LD_LIDAR_SDK_DIR}/ldlidar_driver/include/core
    ${LD_LIDAR_SDK_DIR}/ldlidar_driver/include/dataprocess
    ${LD_LIDAR_SDK_DIR}/ldlidar_driver/include/filter
    ${LD_LIDAR_SDK_DIR}/ldlidar_driver/include/logger
    ${LD_LIDAR_SDK_DIR}/ldlidar_driver/include/networkcom
    ${LD_LIDAR_SDK_DIR}/ldlidar_driver/include/serialcom
)

# Link to the Lidar SDK library directory
link_directories(${LD_LIDAR_SDK_DIR}/build)

# Add your executable and all source files
add_executable(my_car_app
    main.cpp
    LidarController.cpp
    car.cpp
    coordinate.cpp
    CameraController.cpp
)

# Link all the necessary libraries to the executable
target_link_libraries(my_car_app
    ldlidar_driver
    ${OpenCV_LIBS}
    gpiod
)