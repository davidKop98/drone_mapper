#include <cpp_course/MockMovementDriver.h>
#include <cpp_course/DroneMath.h>

#include <cmath>

namespace cpp_course {

namespace {

constexpr double PI = 3.14159265358979323846;
double toRad(double d) { return d * PI / 180.0; }

} // namespace


MockMovementDriver::MockMovementDriver(DroneState&        state,
                                       const DroneConfig& config,
                                       const InputMap&    inputMap,
                                       bool&              simulationFailed)
    : state_(state)
    , config_(config)
    , inputMap_(inputMap)
    , simulationFailed_(simulationFailed) {}

//in the future, if the size of the drone will be needed to take into consideration while rotating
// we can add the checks here. we already have drone config + inputmap reference
bool MockMovementDriver::rotate(HorizontalAngle angle) {
    double newDeg = state_.heading.force_numerical_value_in(deg)
                  + angle.force_numerical_value_in(deg);
    // Wrap to [0, 360)
    newDeg = std::fmod(newDeg, 360.0);
    if (newDeg < 0.0) newDeg += 360.0;
    state_.heading = newDeg * horizontal_angle[deg];
    return true;
}

bool MockMovementDriver::advance(PhysicalLength distance) {
    const double h  = toRad(state_.heading.force_numerical_value_in(deg));
    const double d  = distance.force_numerical_value_in(cm);
    // Snap to {-1, 0, 1} for cardinal headings: std::sin(π) returns ~1.22e-16
    // instead of 0. Without snapping, position drifts ~1 ULP per move; over
    // hundreds of moves with an integer-halfWidth drone, the drift carries
    // the body's edge across a cell boundary and into a wall cell.
    const double fx = DroneMath::snapCardinal(std::cos(h));
    const double fy = DroneMath::snapCardinal(std::sin(h));

    const double halfWidth  = config_.minPassWidth.force_numerical_value_in(cm)  / 2.0;
    const double halfLength = config_.minPassLength.force_numerical_value_in(cm) / 2.0;
    const double halfHeight = config_.minPassHeight.force_numerical_value_in(cm) / 2.0;

    if (!DroneMath::canAdvance(state_.position, d, h,
                               halfWidth, halfLength, halfHeight,
                               inputMap_, inputMap_.resolution())) {
        simulationFailed_ = true;
        return false;
    }

    const double startX = state_.position.x.force_numerical_value_in(cm);
    const double startY = state_.position.y.force_numerical_value_in(cm);
    state_.position.x = (startX + fx * d) * x_extent[cm];
    state_.position.y = (startY + fy * d) * y_extent[cm];
    return true;
}

bool MockMovementDriver::elevate(PhysicalLength distance) {
    const double h = toRad(state_.heading.force_numerical_value_in(deg));
    const double d = distance.force_numerical_value_in(cm);

    const double halfWidth  = config_.minPassWidth.force_numerical_value_in(cm)  / 2.0;
    const double halfLength = config_.minPassLength.force_numerical_value_in(cm) / 2.0;
    const double halfHeight = config_.minPassHeight.force_numerical_value_in(cm) / 2.0;

    if (!DroneMath::canElevate(state_.position, d, h,
                               halfWidth, halfLength, halfHeight,
                               inputMap_, inputMap_.resolution())) {
        simulationFailed_ = true;
        return false;
    }

    const double startZ = state_.position.z.force_numerical_value_in(cm);
    state_.position.z = (startZ + d) * z_extent[cm];
    return true;
}

bool MockMovementDriver::isOccupied(const Position3D& pos) const {
    return inputMap_.get(pos) != 0;
}

} // namespace cpp_course
