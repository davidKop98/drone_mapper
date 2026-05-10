#pragma once

#include <cpp_course/PositionSensor.h>
#include <cpp_course/types.h>

namespace cpp_course {

// Read-only view of DroneState for the drone algorithm.
// The drone never sees the concrete type — it only knows IPositionSensor.
class MockPositionSensor final : public IPositionSensor {
private:
    const DroneState& state_;
public:
    // state is shared with MockMovementDriver; this class only reads it.
    explicit MockPositionSensor(const DroneState& state) : state_(state) {}

    [[nodiscard]] Position3D position() const override {
        return state_.position;
    }

    // altitude is always 0 — the drone only tracks XY heading.
    [[nodiscard]] Orientation heading() const override {
        return Orientation{state_.heading, 0.0 * altitude_angle[deg]};
    }
};

} // namespace cpp_course
