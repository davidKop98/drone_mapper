#pragma once

#include <cpp_course/IMap3D.h>
#include <cpp_course/Units.h>
#include <string>
#include <vector>

namespace cpp_course {

// Real-building map loaded from map_input.txt.
// Only the MockLidarSensor accesses this via the IMap3D interface;
// the drone algorithm never sees it.
class InputMap : public IMap3D {
public:
    // Load map from file. Returns false if file cannot be opened or is invalid.
    bool loadFromFile(const std::string& path);

    // IMap3D implementation.
    // Returns 0 for empty or out-of-bounds, 1 for occupied.
    [[nodiscard]] int get(const Position3D& pos) const override;

private:
    // 3D grid indexed [layer z][row y][column x], values 0 or 1.
    std::vector<std::vector<std::vector<int>>> grid_;

    //starting origin of our "world"
    XLength originX_{};
    YLength originY_{};
    ZLength originZ_{};
    //size of our world in "resolution units"
    int sizeX_{0}, sizeY_{0}, sizeZ_{0};

    // Resolution as decimal places: cell_size = 10^(-resolution) cm.
    int resolution_{0};

    // Convert a world position to grid indices (cx, cy, cz).
    // Returns false if the position falls outside the grid.
    // Uses std::floor — grid indices are always non-negative.
    [[nodiscard]] bool worldToGrid(const Position3D& pos,
                                   int& cx, int& cy, int& cz) const;
};

} // namespace cpp_course
