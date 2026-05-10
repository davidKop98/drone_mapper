#include <cpp_course/MockMovementDriver.h>

#include <cmath>

namespace cpp_course {

namespace {

constexpr double PI = 3.141592;
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

bool MockMovementDriver::checkAdvanceSlice(
    const Position3D& center,double halfWidth,double halfHeight,double headingRad) const
{
    const double cellSize = std::pow(10.0, -inputMap_.resolution());
    //IMPORTANT: vec1 is {0,0,1} since in our project the drone has no latitude angle
    //so i dont calculate it explicitly. but it is "used" (=multiplying by 1) below

    // vec2: horizontal unit vector perpendicular to heading.
    const double vec2X = -std::sin(headingRad);
    const double vec2Y =  std::cos(headingRad);

    //make these steps smaller if i want to ensure we are not missing any cells
    const double stepI = (halfHeight > 0) ? cellSize / halfHeight : 1.0;// prevent division by zero.
    const double stepJ = (halfWidth  > 0) ? cellSize / halfWidth  : 1.0;
    

    const double cx = center.x.force_numerical_value_in(cm);
    const double cy = center.y.force_numerical_value_in(cm);
    const double cz = center.z.force_numerical_value_in(cm);
    //the logic is calculating all cells slice positions: new_pos =start_pos+vec1*i+vec2*j for -1<=i,j<=1
    for (double i = -1.0; i <= 1.0 + stepI * 0.5; i += stepI) {
        const double fi = std::min(i, 1.0); //to ensure we also test for i=1.0
        for (double j = -1.0; j <= 1.0 + stepJ * 0.5; j += stepJ) {
            const double fj = std::min(j, 1.0);
            const Position3D sample{
                (cx + fj * halfWidth  * vec2X) * x_extent[cm],
                (cy + fj * halfWidth  * vec2Y) * y_extent[cm],
                (cz + fi * halfHeight)         * z_extent[cm], //here i would use vector1 to multiple but it's 1 so no point
            };
            if (isOccupied(sample)) return true;
        }
    }
    return false;
}

bool MockMovementDriver::checkElevateSlice(
    const Position3D& center,double halfWidth,double halfLength,double headingRad) const
{
    const double cellSize = std::pow(10.0, -inputMap_.resolution());

    //vector1 direction is same as heading direction 
    //
    const double vec1X =  std::cos(headingRad); 
    const double vec1Y =  std::sin(headingRad);
    const double vec2X = -std::sin(headingRad);
    const double vec2Y =  std::cos(headingRad);

    const double stepI = (halfLength > 0) ? cellSize / halfLength : 1.0;
    const double stepJ = (halfWidth  > 0) ? cellSize / halfWidth  : 1.0;

    const double cx = center.x.force_numerical_value_in(cm);
    const double cy = center.y.force_numerical_value_in(cm);
    const double cz = center.z.force_numerical_value_in(cm);

    for (double i = -1.0; i <= 1.0 + stepI * 0.5; i += stepI) {
        const double fi = std::min(i, 1.0);
        for (double j = -1.0; j <= 1.0 + stepJ * 0.5; j += stepJ) {
            const double fj = std::min(j, 1.0);
            const Position3D sample{
                (cx + fi * halfLength * vec1X + fj * halfWidth * vec2X) * x_extent[cm],
                (cy + fi * halfLength * vec1Y + fj * halfWidth * vec2Y) * y_extent[cm],
                cz* z_extent[cm],
            };
            if (isOccupied(sample)) return true;
        }
    }
    return false;
}

bool MockMovementDriver::advance(PhysicalLength distance) {
    const double h  = toRad(state_.heading.force_numerical_value_in(deg));
    const double d  = distance.force_numerical_value_in(cm);
    const double fx = std::cos(h);
    const double fy = std::sin(h);

    const double halfWidth  = config_.minPassWidth.force_numerical_value_in(cm)  / 2.0;
    const double halfLength = config_.minPassLength.force_numerical_value_in(cm) / 2.0;
    const double halfHeight = config_.minPassHeight.force_numerical_value_in(cm) / 2.0;
    double step_factor = 0.5; //smaller factor= more accurate
    const double step       = step_factor*std::pow(10.0, -inputMap_.resolution());

    const double startX = state_.position.x.force_numerical_value_in(cm);
    const double startY = state_.position.y.force_numerical_value_in(cm);
    const double startZ = state_.position.z.force_numerical_value_in(cm);

    // Step the advance-slice from -halfLength (rear face at start) to
    // d+halfLength (front face at end).
    for (double t = -halfLength; t <= d + halfLength + step * 0.5; t += step) {
        const double tt = std::min(t, d + halfLength);
        const Position3D sliceCenter{
            (startX + fx * tt) * x_extent[cm],
            (startY + fy * tt) * y_extent[cm],
            startZ             * z_extent[cm],
        };
        if (checkAdvanceSlice(sliceCenter, halfWidth, halfHeight, h)) {
            simulationFailed_ = true;
            return false;
        }
    }

    state_.position.x = (startX + fx * d) * x_extent[cm];
    state_.position.y = (startY + fy * d) * y_extent[cm];
    return true;
}

bool MockMovementDriver::elevate(PhysicalLength distance) {
    const double h    = toRad(state_.heading.force_numerical_value_in(deg));
    const double d    = distance.force_numerical_value_in(cm);
    const double absD = std::abs(d);
    const double dirZ = (d >= 0) ? 1.0 : -1.0;

    const double halfWidth  = config_.minPassWidth.force_numerical_value_in(cm)  / 2.0;
    const double halfLength = config_.minPassLength.force_numerical_value_in(cm) / 2.0;
    const double halfHeight = config_.minPassHeight.force_numerical_value_in(cm) / 2.0;
    double step_factor = 0.5;
    const double step       = step_factor*std::pow(10.0, -inputMap_.resolution());

    const double startX = state_.position.x.force_numerical_value_in(cm);
    const double startY = state_.position.y.force_numerical_value_in(cm);
    const double startZ = state_.position.z.force_numerical_value_in(cm);

    // Step the elevate-slice vertically from -halfHeight (bottom face at start)
    // to |d|+halfHeight (top face at end).
    for (double t = -halfHeight; t <= absD + halfHeight + step * 0.5; t += step) {
        const double tt = std::min(t, absD + halfHeight);
        const Position3D sliceCenter{
            startX                   * x_extent[cm],
            startY                   * y_extent[cm],
            (startZ + dirZ * tt)     * z_extent[cm],
        };
        if (checkElevateSlice(sliceCenter, halfWidth, halfLength, h)) {
            simulationFailed_ = true;
            return false;
        }
    }

    state_.position.z = (startZ + d) * z_extent[cm];
    return true;
}

bool MockMovementDriver::isOccupied(const Position3D& pos) const {
    return inputMap_.get(pos) != 0;
}

} // namespace cpp_course
