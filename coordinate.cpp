#include "coordinate.hpp"
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>

double coordDistance(int index1, int index2, std::vector<Coordinate> &coordinates) {
    double xDist = (coordinates[static_cast<size_t>(index1)].x - coordinates[static_cast<size_t>(index2)].x);
    double yDist = (coordinates[static_cast<size_t>(index1)].y - coordinates[static_cast<size_t>(index2)].y);
    return std::sqrt(xDist * xDist + yDist * yDist);
}

std::vector<int> dijkstra(std::vector<std::vector<int>>& adj_matrix, std::vector<Coordinate>& verticies, std::vector<std::vector<double>>& dijkstra_matrix, int a, int b) {
    std::vector<int> result;
    int current = a;

    int VISITIED = 0;
    int DISTANCE = 1;
    int PARENT = 2;

    while (current != b) {
        dijkstra_matrix[static_cast<size_t>(current)][static_cast<size_t>(VISITIED)] = 1.0;
        
        // Update distances for all neighbors
        for (size_t i = 0; i < adj_matrix[static_cast<size_t>(current)].size(); i++) {
            int observed_point = adj_matrix[static_cast<size_t>(current)][static_cast<size_t>(i)];
            double distance = dijkstra_matrix[static_cast<size_t>(current)][static_cast<size_t>(DISTANCE)] + 
                             coordDistance(current, observed_point, verticies);
            
            if (distance < dijkstra_matrix[static_cast<size_t>(observed_point)][static_cast<size_t>(DISTANCE)]) {
                dijkstra_matrix[static_cast<size_t>(observed_point)][static_cast<size_t>(DISTANCE)] = distance;
                dijkstra_matrix[static_cast<size_t>(observed_point)][static_cast<size_t>(PARENT)] = static_cast<double>(current);
            }
        }
        
        // Find next unvisited node with minimum distance
        double minDist = std::numeric_limits<double>::infinity();
        int nextNode = -1;
        
        for (size_t i = 0; i < dijkstra_matrix.size(); i++) {
            if (dijkstra_matrix[i][VISITIED] != 1.0 && dijkstra_matrix[i][DISTANCE] < minDist) {
                minDist = dijkstra_matrix[i][DISTANCE];
                nextNode = static_cast<int>(i);
            }
        }
        
        // Check if we found a valid next node
        if (nextNode == -1) {
            std::cout << "No path found!" << std::endl;
            break;
        }
        
        current = nextNode;
    }

    // Reconstruct path
    while (current != a) {
        result.push_back(current);
        current = static_cast<int>(dijkstra_matrix[static_cast<size_t>(current)][static_cast<size_t>(PARENT)]);
    }
    result.push_back(a);
    std::reverse(result.begin(), result.end());

    return result;
}