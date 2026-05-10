#pragma once

#include <cpp_course/Units.h>

namespace cpp_course {

// Interface for movement — drone uses this, never sees the mock.
// Returns false if the movement failed (collision detected by mock).
class IMovementDriver {
public:
    virtual ~IMovementDriver() = default;

    // Rotate in place. Positive = clockwise (heading increases).
    [[nodiscard]] virtual bool rotate(HorizontalAngle angle)   = 0;

    // Move forward along current heading.
    [[nodiscard]] virtual bool advance(PhysicalLength distance) = 0;

    // Move straight up (positive) or down (negative).
    [[nodiscard]] virtual bool elevate(PhysicalLength distance) = 0;
};

} // namespace cpp_course
