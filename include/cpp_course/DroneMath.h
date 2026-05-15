#pragma once

#include <cpp_course/configs.h>
#include <cpp_course/IMap3D.h>
#include <cpp_course/LidarSensor.h>
#include <cpp_course/types.h>
#include <vector>

namespace cpp_course::DroneMath {

// Convert a lidar hit (relative angles + distance) into a world 3D point.
// droneHeading: current drone XY heading in world space.
// hit: contains angle relative to drone heading and distance.
[[nodiscard]] Position3D beamToWorldPoint(
    const Position3D&   dronePos,
    HorizontalAngle     droneHeading,
    const LidarHit&     hit);

// Snap a world position to the output grid CellKey.
// Resolution is decimal places: cell_size = 10^(-resolution) cm.
// Uses floor-toward-zero (positive → floor, negative → ceil).
[[nodiscard]] CellKey snapToGrid(
    const Position3D& worldPos,
    int xyResolution,
    int zResolution);

// Snap a single float value to n decimal places, floor-toward-zero.
// Example: snapValue(2.891, 1) → 2.8  |  snapValue(-1.35, 1) → -1.3
[[nodiscard]] float snapValue(float value, int decimalPlaces);

// Walk a ray from origin and return all output-grid cells it passes through.
// Step size = 0.5 * 10^(-max(xyResolution, zResolution)) cm.
[[nodiscard]] std::vector<CellKey> rayMarch(
    const Position3D&  origin,
    const Orientation& direction,
    PhysicalLength     maxDist,
    int xyResolution,
    int zResolution);

// Compute all beam direction world-space Orientations for one scan cone.
// scanOrientation: world-space direction of cone center.
// Returns one Orientation per beam, circle 0 first.
[[nodiscard]] std::vector<Orientation> computeBeamDirections(
    const Orientation& scanOrientation,
    const LidarConfig& cfg);

// Compute angular step for full-sphere scanning.
// step = 2 * atan(fov_circles * circle_spacing / beam_length_min), in degrees.
// Adjacent cones just touch — guarantees no coverage gaps.
[[nodiscard]] HorizontalAngle computeStepAngle(const LidarConfig& cfg);

// Compute movement step size aligned to output grid and passage constraints.
// step = max(xy_cell_size, minPassWidth / 2), rounded up to nearest xy_cell_size.
[[nodiscard]] PhysicalLength computeMoveStep(
    const DroneConfig&   droneConfig,
    const MissionConfig& missionConfig);

// ---------------------------------------------------------------------------
// Collision-checking helpers — used by MockMovementDriver and the exploration
// algorithm. The map is passed by interface (any IMap3D implementation works);
// mapResolution is the cell size as decimal places (cell_size = 10^-resolution).

// Check one vertical rectangular slice perpendicular to heading.
// halfWidth = half drone width (perpendicular to heading in XY).
// halfHeight = half drone height (Z axis).
[[nodiscard]] bool checkAdvanceSlice(
    const Position3D& center,
    double halfWidth,
    double halfHeight,
    double headingRad,
    const IMap3D& map,
    int mapResolution);

// Check one horizontal rectangular slice parallel to the ground.
// halfWidth  = half drone width (perpendicular to heading).
// halfLength = half drone length (along heading).
[[nodiscard]] bool checkElevateSlice(
    const Position3D& center,
    double halfWidth,
    double halfLength,
    double headingRad,
    const IMap3D& map,
    int mapResolution);

// True if the drone can safely advance from start by distanceCm along headingRad.
// Sweeps advance slices from t=-halfLength (rear face at start) to
// t=distanceCm+halfLength (front face at end).
[[nodiscard]] bool canAdvance(
    const Position3D& start,
    double distanceCm,
    double headingRad,
    double halfWidth,
    double halfLength,
    double halfHeight,
    const IMap3D& map,
    int mapResolution);

// True if the drone can safely elevate from start by distanceCm (negative=down).
// Sweeps elevate slices vertically from t=-halfHeight (bottom face at start)
// to t=|distanceCm|+halfHeight (top face at end).
[[nodiscard]] bool canElevate(
    const Position3D& start,
    double distanceCm,
    double headingRad,
    double halfWidth,
    double halfLength,
    double halfHeight,
    const IMap3D& map,
    int mapResolution);

} // namespace cpp_course::DroneMath
