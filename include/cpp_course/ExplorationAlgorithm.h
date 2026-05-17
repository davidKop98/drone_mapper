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

//The logic is as follows: 
//1)We start in scanning phase -> we keep in scanning phase until we do a full sphere ->procceed to ChoosingNext phase
//2)ChoosingNext phase-> we look for a valid target in 10 directions. 
//If we find one, we set it as nextTarget_ and move to Moving phase. If we dont find any, we move to Backtracking phase
//3)backtracking phase -> we pop commands from inverseStack_ and execute them until we pop a LevelMarker (which indicates we fully backtracked to the parent pos of the current target). 
//Once we pop the LevelMarker, we move to ChoosingNext phase to look for a new target (without rescanning since we already scanned in this pos before). 
//4)Moving phase -> we first rotate towards the target if needed, then we advance to it. During this process we
// also push the inverse commands to the inverseStack_ so that we can backtrack later if needed. Once we arrive in the new pos, we move back to Scanning phase to start scanning in the new pos.
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
    Phase phase_{Phase::Scanning}; //defaults to Scanning at start of mission

    // Sphere-scan progress at the current position.
    // these will always reset when we enter scanning phase, and we will persist in scanning phase until 
    // we do a full sphere scan. these two keep track of the current sphere part being scanned.
    //these values are relative to the drone position
    HorizontalAngle scanXY_{0.0   * horizontal_angle[deg]};
    Altitude        scanEl_{-90.0 * altitude_angle[deg]};

    // Current move target (set by findNextTarget).
    Position3D nextTarget_{};

    // Per-decide chunking state. When non-zero, Phase::Moving emits one chunk
    // of this amount (capped by the corresponding max in DroneConfig) and
    // decrements until 0. Only one of the three is non-zero at a time.
    double pendingRotateDeg_{0.0};
    double pendingAdvanceCm_{0.0};
    double pendingElevateCm_{0.0};

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

    // Forward chunk emission. Returns one Command sized at most by the
    // corresponding max in DroneConfig, and decrements `remaining`.
    [[nodiscard]] Command emitRotateChunk (double& remaining) const;
    [[nodiscard]] Command emitAdvanceChunk(double& remaining) const;
    [[nodiscard]] Command emitElevateChunk(double& remaining) const;

    // Inverse-push helpers: push N chunks summing to `total` onto inverseStack_.
    // Each chunk's magnitude ≤ the corresponding max in DroneConfig.
    void pushChunkedRotate (double total);
    void pushChunkedAdvance(double total);
    void pushChunkedElevate(double total);
};

} // namespace cpp_course
