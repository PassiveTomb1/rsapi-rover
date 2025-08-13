// CameraController.hpp
#pragma once

#include <opencv2/opencv.hpp>

class CameraController {
public:
    CameraController();
    ~CameraController();
    bool IsOpen();
    cv::Mat GetFrame();

private:
    cv::VideoCapture cap;
    bool isOpen;
};