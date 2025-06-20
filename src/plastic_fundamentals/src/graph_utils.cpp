#include "graph_utils.hpp"
#include <queue>
#include <set>
#include <cmath>
#include <algorithm>

const int ROWS = 6;
const int COLS = 6;

const std::vector<std::vector<std::vector<Direction>>> walls = {
    {{UP, LEFT, RIGHT}, {UP, LEFT}, {UP, RIGHT}, {UP, LEFT}, {UP, DOWN}, {UP, RIGHT, DOWN}},
    {{LEFT, RIGHT}, {LEFT, DOWN, RIGHT}, {LEFT, RIGHT}, {LEFT, DOWN}, {UP}, {UP, RIGHT, DOWN}},
    {{LEFT, DOWN}, {UP}, {}, {UP, RIGHT}, {LEFT}, {UP, RIGHT}},
    {{LEFT, UP, DOWN}, {RIGHT, DOWN}, {LEFT}, {}, {DOWN}, {RIGHT, DOWN}},
    {{UP, LEFT}, {RIGHT, UP}, {LEFT, RIGHT}, {LEFT}, {UP, RIGHT}, {UP, LEFT, RIGHT, DOWN}},
    {{RIGHT, LEFT, DOWN}, {LEFT, DOWN}, {RIGHT, DOWN}, {LEFT, DOWN}, {DOWN}, {UP, RIGHT, DOWN}}
};


// Create a grid-based graph
std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> createGraph() {
    std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> graph;

    for (int r = 0; r < ROWS; ++r) {
        for (int c = 0; c < COLS; ++c) {
            std::pair<int, int> current = {r, c};
            std::vector<std::pair<int, int>> neighbors;

            const auto& cell_walls = walls[r][c];

            // RIGHT
            if (c + 1 < COLS && std::find(cell_walls.begin(), cell_walls.end(), RIGHT) == cell_walls.end()) {
                const auto& neighbor_walls = walls[r][c + 1];
                if (std::find(neighbor_walls.begin(), neighbor_walls.end(), LEFT) == neighbor_walls.end()) {
                    neighbors.emplace_back(r, c + 1);
                }
            }

            // UP
            if (r - 1 >= 0 && std::find(cell_walls.begin(), cell_walls.end(), UP) == cell_walls.end()) {
                const auto& neighbor_walls = walls[r - 1][c];
                if (std::find(neighbor_walls.begin(), neighbor_walls.end(), DOWN) == neighbor_walls.end()) {
                    neighbors.emplace_back(r - 1, c);
                }
            }

            // LEFT
            if (c - 1 >= 0 && std::find(cell_walls.begin(), cell_walls.end(), LEFT) == cell_walls.end()) {
                const auto& neighbor_walls = walls[r][c - 1];
                if (std::find(neighbor_walls.begin(), neighbor_walls.end(), RIGHT) == neighbor_walls.end()) {
                    neighbors.emplace_back(r, c - 1);
                }
            }

            // DOWN
            if (r + 1 < ROWS && std::find(cell_walls.begin(), cell_walls.end(), DOWN) == cell_walls.end()) {
                const auto& neighbor_walls = walls[r + 1][c];
                if (std::find(neighbor_walls.begin(), neighbor_walls.end(), UP) == neighbor_walls.end()) {
                    neighbors.emplace_back(r + 1, c);
                }
            }

            graph[current] = neighbors;
        }
    }

    return graph;
}


// Breadth-First Search for shortest path
std::vector<std::pair<int, int>> findShortestPath(
    const std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> &graph,
    std::pair<int, int> start,
    std::pair<int, int> goal
) {
    std::map<std::pair<int, int>, std::pair<int, int>> came_from;
    std::queue<std::pair<int, int>> frontier;
    std::set<std::pair<int, int>> visited;

    frontier.push(start);
    visited.insert(start);

    while (!frontier.empty()) {
        auto current = frontier.front();
        frontier.pop();

        if (current == goal) break;

        for (const auto& neighbor : graph.at(current)) {
            if (visited.count(neighbor) == 0) {
                visited.insert(neighbor);
                frontier.push(neighbor);
                came_from[neighbor] = current;
            }
        }
    }

    std::vector<std::pair<int, int>> path;
    if (came_from.find(goal) == came_from.end()) return path; // No path

    for (auto at = goal; at != start; at = came_from[at]) {
        path.push_back(at);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

// Convert path to direction enums
std::vector<int> pathToDirections(const std::vector<std::pair<int, int>>& path) {
    std::vector<int> directions;
    for (size_t i = 1; i < path.size(); ++i) {
        auto [r1, c1] = path[i - 1];
        auto [r2, c2] = path[i];

        if (r2 == r1 && c2 == c1 + 1) {
            directions.push_back(Direction::RIGHT);
        } else if (r2 == r1 - 1 && c2 == c1) {
            directions.push_back(Direction::UP);
        } else if (r2 == r1 && c2 == c1 - 1) {
            directions.push_back(Direction::LEFT);
        } else if (r2 == r1 + 1 && c2 == c1) {
            directions.push_back(Direction::DOWN);
        }
    }
    return directions;
}
