#pragma once

#include <cpp_course/BuildingMap.h>
#include <cpp_course/configs.h>
#include <cpp_course/types.h>

#include <unordered_set>
#include <vector>

namespace cpp_course {

// Fixed-direction DFS exploration with per-level inverse-move backtracking.
// From any visited position we consider exactly 10 candidate directions:
//   8 horizontal: 0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°
//   2 vertical:   straight up, straight down
// Move step = DroneMath::computeMoveStep(config, mission).
class ExplorationAlgorithm {
public:
    ExplorationAlgorithm(BuildingMap&         buildingMap,
                         const DroneConfig&   config,
                         const MissionConfig& mission);

    // Called once per Simulator iteration; returns exactly one Command.
    [[nodiscard]] Command decide(const Position3D& currentPos,
                                 HorizontalAngle   currentHeading);

private:
    BuildingMap&         buildingMap_;
    const DroneConfig&   config_;
    const MissionConfig& mission_;

    // Each entry is the inverse command sequence for ONE level (one move from
    // parent to child). Stored in reverse execution order so .back() is the
    // next inverse command to issue during backtracking.
    std::vector<std::vector<Command>> inverseStack_;

    // Snapped CellKeys of every position we have already sphere-scanned.
    std::unordered_set<CellKey, CellKeyHash> visited_;

    enum class Phase {
        Scanning,
        ChoosingNext,
        Moving,
        Backtracking,
        Finished,
    };
    Phase phase_{Phase::Scanning};

    // Sphere-scan progress at the current position.
    HorizontalAngle scanXY_{};
    Altitude        scanEl_{};
    bool            sphereScanDone_{false};

    // Currently chosen neighbor (in world coords).
    Position3D nextTarget_{};
    bool       hasTarget_{false};

    [[nodiscard]] HorizontalAngle computeStepAngle() const;
    [[nodiscard]] PhysicalLength  computeMoveStep()  const;
    [[nodiscard]] double          sphereRadius()    const;
};

} // namespace cpp_course
