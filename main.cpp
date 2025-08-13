#include "LidarController.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "coordinate.hpp"
#include "CameraController.hpp"
#include "car.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>
#include <thread>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// #include <pair>

#ifdef _WIN32
#else
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#endif


using namespace std;




// path executing
double calculate_angle(double x1, double y1, double x2, double y2) {
    double dot = x1*x2 + y1*y2;
    double magA = sqrt(pow((y1 - 0), 2)) + pow((x1 - 0), 2);
    double magB = sqrt(pow((y2 - 0), 2)) + pow((x2 - 0), 2);
    double det = magA * magB;
    return acos(dot / det); 
}

double calculate_turn_time(double angle) {
    const double time_to_90 = .466;
    return angle*time_to_90/90;
}

double calculate_time(double distance) {
    const double in_per_sec = 25.25;
    return distance/in_per_sec;
}

void drive_path (vector<int> path, vector<Coordinate> verticies, int file, vector<ldlidar::PointData> &scan, cv::Mat& frame) {
    pair<int, int> prevHeading = {0, 1};
    pair<int, int> currentHeading;
    for (size_t i = 0; i < path.size() - 1; i++) {
        int startNode = path[i];
        int nextNode = path[i + 1];

        currentHeading = {verticies[nextNode].x - verticies[startNode].x, verticies[nextNode].y - verticies[startNode].y};
        
        /*
        double distance = coordDistance(startNode, nextNode, verticies);
        double timeToMove = calculate_time(distance);
 
        double angle = calculate_angle(prevHeading.first, prevHeading.second, currentHeading.first, currentHeading.second) * 180 / 3.14;
        double timeToAngle = calculate_turn_time(angle);

        if (angle < 180) {
            moveForDuration(file, -950, -950, 1000, 1000, timeToAngle, "Turning right");
        }
        else if (angle >= 180) {
            moveForDuration(file, 950, 950, -1000, -1000, timeToAngle, "Turning left");
        }
        sleep(1);
        */

        setMotorModel(file, -1000, -1000, -1000, -1000);
        for (auto point : scan) {
            double distance_in = point.distance / 25.4;
            if (distance_in > 1 && distance_in < 10 && point.angle > 225 && point.angle < 315) {
                setMotorModel(file, 0, 0, 0, 0);
                cout << "Something in the way" << endl;
                cout << "dist: " << distance_in << ", angle: " << point.angle << endl;
                return;
            }
        }

        sleep(1);

        prevHeading = currentHeading;

    }
    if(!frame.empty()) {
        cout << "Not empty frame" << endl;
    }
    cout << scan.size() << endl;
}

void ultrasonic_dist (double &dist) {
    while (true) {
        dist = getDistance();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        cout << dist << endl;
    }
}

void full_lidar_scan (LidarController& lidar, vector<ldlidar::PointData> &scan) {  
    while (true){   
        scan = lidar.GetLidarScan();
        
        if (!scan.empty()) {
            std::cout << "Received " << scan.size() << " points." << std::endl;
        /*
            for (const auto& point : scan) {
                if (point.distance != 0) {
                    double distance_in = point.distance / 25.4;
                    double x = distance_in * std::cos(point.angle * M_PI / 180);
                    double y = distance_in * std::sin(point.angle * M_PI / 180);

                    std::cout << "Angle: " << point.angle << "°, Distance: " << distance_in << "in, (x, y): (" << x << ", " << y << ")\n";
                }
            }
        */
        }
        
        usleep(100000); 
    }
}

void full_camera_scan(CameraController& camera, cv::Mat& frame) {
    while (true) {
        frame = camera.GetFrame();
        if (frame.empty()) {
            std::cerr << "Error: Captured frame is empty." << std::endl;
            break;
        }
    }
}

// int argc, char *argv[]
int main()
{
    cout << fixed << showpoint;
    cout << setprecision(2);

    LidarController lidar;
    if (!lidar.IsRunning()) {
        std::cerr << "Lidar controller failed to initialize. Exiting." << std::endl;
        return -1;
    }
    std::cout << "Lidar initialized!" << std::endl;


    CameraController camera;
    if (!camera.IsOpen()) {
        return -1;
    }
    std::cout << "Camera initialized!" << std::endl;
    
    
    int file = openI2C(0x40); 
    if (file < 0) return 1;

    initPCA9685(file);

    vector<Coordinate> verticies = {
        Coordinate(0, 0),      // lamp
        Coordinate(0, 80),     // top of bed intersection
        Coordinate(-43, 80),   // chair
        Coordinate(80, 80),    // outside
        Coordinate(80, 0)     // ritvik room
    };
    
    // Hardcoded adjacency matrix for testing
    vector<vector<int>> adj_matrix = {
        {1},        // vertex 0
        {0, 2, 3},     // vertex 1
        {1},        // vertex 2
        {1, 4},
        {3}

    };
    
    int numCoordinates = 5;

    vector<vector<double>> dijkstra_matrix;
    dijkstra_matrix.push_back(vector<double>{1, 0, -1});
    for (int i = 1; i < numCoordinates; i++) {
        dijkstra_matrix.push_back(vector<double>{0, std::numeric_limits<double>::infinity(), -1});
    }

    int start = 0;
    int end = 2;

    vector<int> path = dijkstra(adj_matrix, verticies, dijkstra_matrix, start, end);

    cout << "Path: ";
    for (size_t i = 0; i < path.size(); i++) {
        if (i != path.size() - 1) {
            cout << path[i] << " -> ";
        }
        else {
            cout << path[i] << endl;
        }
    }

    /*

    pair<int, int> prevHeading = {0, 1};
    pair<int, int> currentHeading;
    for (size_t i = 0; i < path.size() - 1; i++) {
        int startNode = path[i];
        int nextNode = path[i + 1];

        currentHeading = {verticies[nextNode].x - verticies[startNode].x, verticies[nextNode].y - verticies[startNode].y};
        
        double distance = coordDistance(startNode, nextNode, verticies);
        double timeToMove = calculate_time(distance);
 
        double angle = calculate_angle(prevHeading.first, prevHeading.second, currentHeading.first, currentHeading.second) * 180 / 3.14;
        cout << prevHeading.first << " " << prevHeading.second << " " << currentHeading.first << " " <<currentHeading.second << "\n";
        cout << angle << "\n";
        double timeToAngle = calculate_turn_time(angle);

        if (angle < 180) {
            moveForDuration(file, -950, -950, 1000, 1000, timeToAngle, "Turning right");
        }
        else if (angle >= 180) {
            moveForDuration(file, 950, 950, -1000, -1000, timeToAngle, "Turning left");
        }
        sleep(1);

        moveForDuration(file, 1050, 1050, 1000, 1000, timeToMove, "Moving forward");

        sleep(1);

        prevHeading = currentHeading;

    }

    */
    
    vector<ldlidar::PointData> scan;
    cv::Mat frame;


    std::thread drive (drive_path, path, verticies, file, std::ref(scan), std::ref(frame));
    std::thread sense (full_lidar_scan, std::ref(lidar), std::ref(scan));
    std::thread picture (full_camera_scan, std::ref(camera), std::ref(frame));

    drive.join();
    sense.join();

    


    /*
    while(true) {
        int gap_to_keep = 400;
        int current_distance = 0;
        int speed = 1000;

        // read in distance

        if (current_distance > gap_to_keep) {
            speed+=100;
        }
        else if (current_distance < gap_to_keep) {
            speed-=100;
        }

        moveForDuration(file, speed, speed, speed, speed, 0.5, "Moving forward");
    }
    */
    return 0;
}