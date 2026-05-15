#include <cpp_course/ExplorationAlgorithm.h>
#include <cpp_course/DroneMath.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace cpp_course {

namespace {

constexpr double PI      = 3.14159265358979323846;
constexpr double EPS_CM  = 1e-4;
constexpr double EPS_DEG = 1e-3;
constexpr double S       = 0.7071067811865476; // √2/2

// 10 candidate direction unit vectors, in priority order.
const std::array<std::array<double, 3>, 10> kDirections = {{
    {{ 1.0,  0.0,  0.0}}, //   0° east
    {{ S,    S,    0.0}}, //  45°
    {{ 0.0,  1.0,  0.0}}, //  90° north
    {{-S,    S,    0.0}}, // 135°
    {{-1.0,  0.0,  0.0}}, // 180° west
    {{-S,   -S,    0.0}}, // 225°
    {{ 0.0, -1.0,  0.0}}, // 270° south
    {{ S,   -S,    0.0}}, // 315°
    {{ 0.0,  0.0,  1.0}}, // up
    {{ 0.0,  0.0, -1.0}}, // down
}};

double toDeg(double rad) { return rad * 180.0 / PI; }

// Wrap a signed angle to (-180, 180].
double wrapTo180(double d) {
    d = std::fmod(d + 180.0, 360.0);
    if (d < 0.0) d += 360.0;
    return d - 180.0;
}

Command makeRotate(double deg_) {
    Command c{};
    c.type       = CommandType::Rotate;
    c.angleValue = deg_ * horizontal_angle[deg];
    return c;
}

Command makeAdvance(double cm_) {
    Command c{};
    c.type          = CommandType::Advance;
    c.distanceValue = cm_ * cm;
    return c;
}

Command makeElevate(double cm_) {
    Command c{};
    c.type          = CommandType::Elevate;
    c.distanceValue = cm_ * cm;
    return c;
}

Command makeScan(HorizontalAngle xy, Altitude el) {
    Command c{};
    c.type            = CommandType::Scan;
    c.scanOrientation = Orientation{xy, el};
    return c;
}

Command makeFinished() {
    Command c{};
    c.type = CommandType::Finished;
    return c;
}

} // namespace

ExplorationAlgorithm::ExplorationAlgorithm(
    BuildingMap& bm, const DroneConfig& c, const MissionConfig& m)
    : buildingMap_(bm), config_(c), mission_(m)
{
    scanXY_ =   0.0 * horizontal_angle[deg];
    scanEl_ = -90.0 * altitude_angle[deg];
}

HorizontalAngle ExplorationAlgorithm::computeStepAngle() const {
    return DroneMath::computeStepAngle(config_.lidar);
}

PhysicalLength ExplorationAlgorithm::computeMoveStep() const {
    return DroneMath::computeMoveStep(config_, mission_);
}

double ExplorationAlgorithm::sphereRadius() const {
    const double w = config_.minPassWidth.force_numerical_value_in(cm);
    const double l = config_.minPassLength.force_numerical_value_in(cm);
    const double h = config_.minPassHeight.force_numerical_value_in(cm);
    return std::max({w, l, h}) / 2.0;
}

Command ExplorationAlgorithm::decide(
    const Position3D& currentPos, HorizontalAngle currentHeading)
{
    switch (phase_) {

        case Phase::Scanning: {
            if (!sphereScanDone_) {
                Command cmd = makeScan(scanXY_, scanEl_);

                const double stepDeg =
                    computeStepAngle().force_numerical_value_in(deg);
                double xy = scanXY_.force_numerical_value_in(deg) + stepDeg;
                double el = scanEl_.force_numerical_value_in(deg);

                if (xy >= 360.0) {
                    xy = 0.0;
                    el += stepDeg;
                    if (el > 90.0) {
                        sphereScanDone_ = true;
                        const CellKey here = DroneMath::snapToGrid(
                            currentPos, mission_.xyResolution, mission_.zResolution);
                        visited_.insert(here);
                        phase_ = Phase::ChoosingNext;
                    }
                }
                scanXY_ = xy * horizontal_angle[deg];
                scanEl_ = el * altitude_angle[deg];
                return cmd;
            }
            // Defensive — should not reach here in normal flow.
            phase_ = Phase::ChoosingNext;
            return decide(currentPos, currentHeading);
        }

        case Phase::ChoosingNext: {
            const double moveStep = computeMoveStep().force_numerical_value_in(cm);
            const double cx = currentPos.x.force_numerical_value_in(cm);
            const double cy = currentPos.y.force_numerical_value_in(cm);
            const double cz = currentPos.z.force_numerical_value_in(cm);

            for (const auto& dir : kDirections) {
                const double tx = cx + moveStep * dir[0];
                const double ty = cy + moveStep * dir[1];
                const double tz = cz + moveStep * dir[2];
                const Position3D cand{
                    tx * x_extent[cm],
                    ty * y_extent[cm],
                    tz * z_extent[cm],
                };
                const CellKey key = DroneMath::snapToGrid(
                    cand, mission_.xyResolution, mission_.zResolution);
                if (visited_.count(key))                       continue;
                if (buildingMap_.getCell(key) != CellValue::Empty) continue;

                nextTarget_ = cand;
                hasTarget_  = true;
                phase_      = Phase::Moving;
                // Open a new level — Moving will append inverses to inverseStack_.back().
                inverseStack_.emplace_back();
                return decide(currentPos, currentHeading);
            }

            // No unvisited Empty direction.
            if (inverseStack_.empty()) {
                phase_ = Phase::Finished;
                return makeFinished();
            }
            phase_ = Phase::Backtracking;
            return decide(currentPos, currentHeading);
        }

        case Phase::Moving: {
            const double dx = nextTarget_.x.force_numerical_value_in(cm)
                            - currentPos.x.force_numerical_value_in(cm);
            const double dy = nextTarget_.y.force_numerical_value_in(cm)
                            - currentPos.y.force_numerical_value_in(cm);
            const double dz = nextTarget_.z.force_numerical_value_in(cm)
                            - currentPos.z.force_numerical_value_in(cm);

            // Pure vertical move.
            if (std::abs(dz) > EPS_CM) {
                Command cmd = makeElevate(dz);
                // Inverse: Elevate(-dz). Stored in reverse exec order
                // (single command → trivially correct).
                inverseStack_.back().push_back(makeElevate(-dz));
                sphereScanDone_ = false;
                scanXY_         =   0.0 * horizontal_angle[deg];
                scanEl_         = -90.0 * altitude_angle[deg];
                hasTarget_      = false;
                phase_          = Phase::Scanning;
                return cmd;
            }

            const double horiz = std::sqrt(dx * dx + dy * dy);
            if (horiz > EPS_CM) {
                const double requiredDeg = toDeg(std::atan2(dy, dx));
                const double curDeg      = currentHeading.force_numerical_value_in(deg);
                const double delta       = wrapTo180(requiredDeg - curDeg);

                if (std::abs(delta) > EPS_DEG) {
                    Command cmd = makeRotate(delta);
                    // Inverse Rotate(-delta) — append to the current level.
                    // Rotation does not change position; do NOT reset scan state.
                    inverseStack_.back().push_back(makeRotate(-delta));
                    return cmd;
                }

                Command cmd = makeAdvance(horiz);
                // Inverse exec order: Rotate(180), Advance(dist), Rotate(-180).
                // Stored in reverse so .back() pops first → push -180 first.
                inverseStack_.back().push_back(makeRotate(-180.0));
                inverseStack_.back().push_back(makeAdvance(horiz));
                inverseStack_.back().push_back(makeRotate(180.0));
                sphereScanDone_ = false;
                scanXY_         =   0.0 * horizontal_angle[deg];
                scanEl_         = -90.0 * altitude_angle[deg];
                hasTarget_      = false;
                phase_          = Phase::Scanning;
                return cmd;
            }

            // Already at target — abandon the empty level we just opened.
            if (!inverseStack_.empty() && inverseStack_.back().empty()) {
                inverseStack_.pop_back();
            }
            hasTarget_ = false;
            phase_     = Phase::ChoosingNext;
            return decide(currentPos, currentHeading);
        }

        case Phase::Backtracking: {
            if (inverseStack_.empty()) {
                phase_ = Phase::Finished;
                return makeFinished();
            }
            auto& level = inverseStack_.back();
            Command cmd = level.back();
            level.pop_back();
            if (level.empty()) {
                inverseStack_.pop_back();
                // One level fully undone → drone is at the parent. Pick the
                // next unvisited direction from there (no rescan needed).
                phase_ = Phase::ChoosingNext;
            }
            return cmd;
        }

        case Phase::Finished:
        default:
            return makeFinished();
    }
}

} // namespace cpp_course
