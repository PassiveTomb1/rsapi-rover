#include "CameraController.hpp"

CameraController::CameraController() : isOpen(false) {
    // 0 is typically the default camera index
    cap.open(0);
    if (cap.isOpened()) {
        isOpen = true;
    } else {
        std::cerr << "Error: Could not open the webcam." << std::endl;
    }
}

CameraController::~CameraController() {
    if (cap.isOpened()) {
        cap.release();
    }
}

bool CameraController::IsOpen() {
    return isOpen;
}

cv::Mat CameraController::GetFrame() {
    cv::Mat frame;
    cap >> frame; // Capture a new frame
    return frame;
}