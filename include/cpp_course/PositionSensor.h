#pragma once

#include <cpp_course/Units.h>

namespace cpp_course {

// Interface for position sensing — drone uses this, never sees the mock.
class IPositionSensor {
public:
    virtual ~IPositionSensor() = default;

    // Returns the drone's current center position in world space.
    [[nodiscard]] virtual Position3D position() const = 0;

    // Returns the drone's current heading as an Orientation (altitude is always 0).
    [[nodiscard]] virtual Orientation heading() const = 0;
};

} // namespace cpp_course
