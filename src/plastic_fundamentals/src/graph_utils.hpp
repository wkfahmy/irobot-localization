#ifndef GRAPH_UTILS_HPP
#define GRAPH_UTILS_HPP

#include <map>
#include <vector>
#include <utility>

enum Direction {
    RIGHT = 0,
    UP = 1,
    LEFT = 2,
    DOWN = 3
};


// Grid dimensions
extern const int ROWS;
extern const int COLS;

// Graph creation
std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> createGraph();

// Pathfinding
std::vector<std::pair<int, int>> findShortestPath(
    const std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> &graph,
    std::pair<int, int> start,
    std::pair<int, int> goal);

// Convert path to directions
std::vector<int> pathToDirections(const std::vector<std::pair<int, int>>& path);

#endif
