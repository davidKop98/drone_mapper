#pragma once

#include <cpp_course/BuildingMap.h>
#include <cpp_course/configs.h>
#include <cpp_course/types.h>

#include <array>
#include <unordered_set>
#include <vector>

namespace cpp_course {

// Fixed-direction DFS exploration with inverse-move backtracking.
// 10 candidate directions per position: 8 horizontal (every 45°) + 2 vertical.
// Move step = DroneMath::computeMoveStep(config, mission).
// Path safety is validated by DroneMath::canAdvance / canElevate using
// BuildingMap (which now implements IMap3D — Occupied cells are walls).
class ExplorationAlgorithm {
public:
    ExplorationAlgorithm(BuildingMap&         buildingMap,
                         const DroneConfig&   config,
                         const MissionConfig& mission);

    // Returns one Command per call. Internal phase state persists across calls.
    [[nodiscard]] Command decide(const Position3D& currentPos,
                                 HorizontalAngle   currentHeading);

private:
    BuildingMap&         buildingMap_;
    const DroneConfig&   config_;
    const MissionConfig& mission_;

    // Flat inverse stack. Each move pushes 1 (Rotate/Elevate) or 3 (Advance)
    // inverse commands in reverse-execution order so .back() pops first.
    std::vector<Command> inverseStack_;

    // Snapped CellKeys of every position where a sphere scan was completed.
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
    HorizontalAngle scanXY_{0.0   * horizontal_angle[deg]};
    Altitude        scanEl_{-90.0 * altitude_angle[deg]};

    // Current move target (set by findNextTarget).
    Position3D nextTarget_{};

    [[nodiscard]] HorizontalAngle computeStepAngle() const;
    [[nodiscard]] PhysicalLength  computeMoveStep()  const;

    [[nodiscard]] double halfWidth()  const;
    [[nodiscard]] double halfLength() const;
    [[nodiscard]] double halfHeight() const;

    // 10 unit-vector candidate directions, fixed priority order.
    static const std::array<std::array<double, 3>, 10> kDirections;

    // First unvisited, reachable, Empty neighbor from pos. Sets nextTarget_.
    [[nodiscard]] bool findNextTarget(const Position3D& pos,
                                      HorizontalAngle   heading);
};

} // namespace cpp_course
