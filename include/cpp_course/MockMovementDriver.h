#pragma once

#include <cpp_course/MovementDriver.h>
#include <cpp_course/InputMap.h>
#include <cpp_course/configs.h>
#include <cpp_course/types.h>

namespace cpp_course {

// Simulated movement driver. Writes the new position to DroneState after each
// command. If the destination cell is occupied, sets simulationFailed and
// returns false without updating state.
class MockMovementDriver final : public IMovementDriver {
private:
    DroneState&        state_;
    const DroneConfig& config_;
    const InputMap&    inputMap_;
    bool&              simulationFailed_;
    [[nodiscard]] bool isOccupied(const Position3D& pos) const;
public:
    MockMovementDriver(DroneState&        state,
                       const DroneConfig& config,
                       const InputMap&    inputMap,
                       bool&              simulationFailed);

    // Rotate in place. Wraps heading to [0, 360).
    [[nodiscard]] bool rotate(HorizontalAngle angle)   override;

    // Move forward along current heading. Fails on collision.
    [[nodiscard]] bool advance(PhysicalLength distance) override;

    // Move along Z axis (positive = up). Fails on collision.
    [[nodiscard]] bool elevate(PhysicalLength distance) override;
};

} // namespace cpp_course
