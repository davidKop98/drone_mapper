#pragma once

#include <cpp_course/Units.h>
#include <functional>
#include <vector>

namespace cpp_course {

// Snapped grid coordinate used as key in the sparse output map.
// Plain floats (not strong types) because this is a map key, not a calculated value.
// x and y use xy_resolution; z uses z_resolution.
struct CellKey {
    float x{}, y{}, z{}; //default values are 0

    bool operator==(const CellKey& o) const { 
        return x == o.x && y == o.y && z == o.z;
    }
};

// Hash for CellKey — required for use in unordered_map.
struct CellKeyHash {
    std::size_t operator()(const CellKey& k) const {
        std::size_t hx = std::hash<float>{}(k.x); //hashes for x, y, z. 
        std::size_t hy = std::hash<float>{}(k.y); //hash<float>{} remians the same throughout our execution
        std::size_t hz = std::hash<float>{}(k.z);
        // combine hashes with bit shifts to reduce collisions
        return hx ^ (hy << 16) ^ (hz << 32);
        //return hx ^ (hy << 16) ^ (hz << 8); alternative replacement
    }
};

// The four possible states of any cell in the output map.
enum class CellValue : int {
    Empty       =  0,  // confirmed empty by lidar
    Occupied    =  1,  // confirmed occupied by lidar hit
    Unmapped    = -1,  // drone never learned about this cell (default)
    OutOfBounds = -2   // outside mission boundaries
};

// Shared mutable state written by MockMovementDriver,
// read by MockPositionSensor and MockLidarSensor.
//note, drone state doesnt contain altitude angle, so if future might need to add it if required.
struct DroneState {
    Position3D      position{};  // current drone center in world space
    HorizontalAngle heading{};   // current XY heading (0=east, 90=south)
                                 // altitude of heading is always 0
};

// All actions the drone algorithm can request.
enum class CommandType {
    Rotate,    // rotate in place by angleValue degrees
    Advance,   // move forward along heading by distanceValue cm
    Elevate,   // move up/down by distanceValue cm (negative = down)
    Scan,      // fire lidar in direction scanOrientation
    Finished   // mapping complete, end simulation
};

// Issued by ExplorationAlgorithm, executed by Drone.
// Only the relevant fields are used per command type.
struct Command {
    CommandType type{CommandType::Finished};

    // Used by Rotate
    HorizontalAngle angleValue{};

    // Used by Advance and Elevate
    PhysicalLength distanceValue{};

    // Used by Scan — direction relative to drone's current heading
    Orientation scanOrientation{};
};

} // namespace cpp_course
