#ifndef COORDINATE_HPP
#define COORDINATE_HPP

#include <vector>

class Coordinate {
public:
    double x;
    double y;
    Coordinate(double x, double y) : x(x), y(y) { }
};

// Function prototypes
double coordDistance(int index1, int index2, std::vector<Coordinate> &coordinates);
std::vector<int> dijkstra(std::vector<std::vector<int>>& adj_matrix, std::vector<Coordinate>& verticies, std::vector<std::vector<double>>& dijkstra_matrix, int a, int b);

#endif