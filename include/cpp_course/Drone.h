#pragma once

#include <cpp_course/BuildingMap.h>
#include <cpp_course/ExplorationAlgorithm.h>
#include <cpp_course/LidarSensor.h>
#include <cpp_course/MovementDriver.h>
#include <cpp_course/PositionSensor.h>
#include <cpp_course/configs.h>
#include <cpp_course/types.h>

namespace cpp_course {

// Layer 8 — drone façade. Owns no state of its own beyond references to the
// surrounding pieces. Responsibilities:
//   - getNextCommand(): ask the algorithm what to do next
//   - execute(cmd):     dispatch the command to the right interface
//   - processScan():    fold lidar results into BuildingMap
class Drone {
public:
    Drone(const ILidarSensor&    lidar,
          const IPositionSensor& posSensor,
          IMovementDriver&       driver,
          BuildingMap&           buildingMap,
          ExplorationAlgorithm&  algorithm,
          const DroneConfig&     config,
          const MissionConfig&   mission);

    // Returns the next Command from the exploration algorithm.
    [[nodiscard]] Command getNextCommand();

    // Executes the given command via the appropriate interface.
    // Returns false if movement collided (Advance/Elevate driver returned false);
    // all other command types always return true.
    [[nodiscard]] bool execute(const Command& cmd);

private:
    const ILidarSensor&    lidar_;
    const IPositionSensor& posSensor_;
    IMovementDriver&       driver_;
    BuildingMap&           buildingMap_;
    ExplorationAlgorithm&  algorithm_;
    const DroneConfig&     config_;
    const MissionConfig&   mission_;

    // Walk every beam direction, look up the matching hit by angle, and update
    // BuildingMap (path → Empty, hit endpoint → Occupied).
    void processScan(const ScanResults& results,
                     const Orientation& relScanOrientation);

    // Mark a cell Empty unless it has already been confirmed Occupied —
    // Occupied should never be overwritten by later misses. (shouldnt happen anyway in theory)
    void markEmpty(const CellKey& key);
};

} // namespace cpp_course
