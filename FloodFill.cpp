
#include "API.h"

// Mock API class for compilation
namespace API {
    int mazeWidth() { return 16; }
    int mazeHeight() { return 16; }
    bool wallFront() { return false; }
    bool wallRight() { return false; }
    bool wallLeft() { return false; }
    void moveForward(int steps = 1) {}
    void turnRight() {}
    void setWall(int x, int y, char direction) {}
}
#include <queue>
#include <set>
#include <utility>
#include <vector>
#include <iostream>
#include <tuple>

struct Cell {
    int x, y;