#pragma once

#include <cpp_course/Units.h>
#include <cpp_course/LidarSensor.h>

namespace cpp_course {

// Drone physical capabilities, loaded from drone_config.txt.
struct DroneConfig {
    // Minimum gap the drone can pass through (cm)
    PhysicalLength minPassWidth{};
    PhysicalLength minPassLength{};
    PhysicalLength minPassHeight{};

    // Maximum movement per single command
    HorizontalAngle maxRotate{};   // degrees
    PhysicalLength  maxAdvance{};  // cm
    PhysicalLength  maxElevate{};  // cm

    // Lidar capabilities — uses course-provided LidarConfig type
    LidarConfig lidar{};
};

// Mission parameters, loaded from mission_config.txt.
struct MissionConfig {
    // Starting pose
    Position3D      startPosition{};
    HorizontalAngle startHeading{};

    // Mapping boundaries (cm)
    XLength minX{}, maxX{};
    YLength minY{}, maxY{};
    ZLength minZ{}, maxZ{};

    // Output map resolution as decimal places.
    // cell_size = 10^(-resolution) cm.
    int xyResolution{0};  // applies to X and Y axes
    int zResolution{0};   // applies to Z (height) axis
};

} // namespace cpp_course
