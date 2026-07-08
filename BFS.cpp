
#include "API.h"

#include <queue>
#include <set>
#include <utility>
#include <vector>
#include <iostream>
#include <tuple>

struct Cell {
    int x, y;
    char direction; // 'N', 'E', 'S', 'W'

    bool operator<(const Cell& other) const {
        return std::tie(x, y, direction) < std::tie(other.x, other.y, other.direction);
    }
};

struct MazeSolver {
    int mazeWidth;
    int mazeHeight;
    std::set<std::pair<int, int>> visited;

    MazeSolver() {
        mazeWidth = API::mazeWidth();
        mazeHeight = API::mazeHeight();
    }

    void solve() {
        std::queue<Cell> queue;
        queue.push({0, 0, 'N'});
        visited.insert({0, 0});

        while (!queue.empty()) {
            Cell current = queue.front();
            queue.pop();

            if (isGoal(current.x, current.y)) {
                std::cout << "Goal Reached at (" << current.x << ", " << current.y << ")" << std::endl;
                return;
            }

            explore(current, queue);
        }

        std::cout << "Maze traversal complete, but goal not found." << std::endl;
    }

    bool isGoal(int x, int y) {
        return x == mazeWidth - 1 && y == mazeHeight - 1;
    }

    void explore(const Cell& current, std::queue<Cell>& queue) {
        static const std::vector<std::pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
        static const std::string dirSymbols = "NESW";

        for (int i = 0; i < 4; ++i) {
            int nx = current.x + directions[i].first;
            int ny = current.y + directions[i].second;
            char dir = dirSymbols[i];

            if (isValid(nx, ny) && !visited.count({nx, ny})) {
                if (canMove(current, dir)) {
                    visited.insert({nx, ny});
                    queue.push({nx, ny, dir});
                    move(current.direction, dir);
                    API::moveForward();
                    markWalls(nx, ny);
                    returnToCurrent(current, dir);
                }
            }
        }
    }

    bool isValid(int x, int y) {
        return x >= 0 && x < mazeWidth && y >= 0 && y < mazeHeight;
    }

    bool canMove(const Cell& current, char direction) {
        if (direction == 'N') return !API::wallFront();
        if (direction == 'E') return !API::wallRight();
        if (direction == 'S') return !API::wallFront();
        if (direction == 'W') return !API::wallLeft();
        return false;
    }

    void markWalls(int x, int y) {
        if (API::wallFront()) API::setWall(x, y, 'N');
        if (API::wallRight()) API::setWall(x, y, 'E');
        if (API::wallLeft()) API::setWall(x, y, 'W');
    }

    void move(char currentDir, char targetDir) {
        while (currentDir != targetDir) {
            API::turnRight();
            currentDir = nextDirection(currentDir);
        }
    }

    char nextDirection(char direction) {
        if (direction == 'N') return 'E';
        if (direction == 'E') return 'S';
        if (direction == 'S') return 'W';
        if (direction == 'W') return 'N';
        return 'N';
    }

    void returnToCurrent(const Cell& current, char direction) {
        move(direction, current.direction);
        API::moveForward(-1); // Move back to the previous cell
    }
};

int main() {
    MazeSolver solver;
    solver.solve();
    return 0;
}