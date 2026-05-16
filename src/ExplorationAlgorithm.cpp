#include <cpp_course/ExplorationAlgorithm.h>
#include <cpp_course/DroneMath.h>

#include <cmath>

namespace cpp_course {

namespace {

constexpr double PI      = 3.14159265;
constexpr double EPS_CM  = 1e-4;
constexpr double EPS_DEG = 1e-4;

double toRad(double d) { return d * PI / 180.0; }
double toDeg(double r) { return r * 180.0 / PI; }

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

Command makeLevelMarker() {
    Command c{};
    c.type = CommandType::LevelMarker;
    return c;
}

} // namespace

// 10 fixed candidate directions, clockwise convention (+Y = south).
const std::array<std::array<double, 3>, 10>
ExplorationAlgorithm::kDirections = {{
    { 1.0,           0.0,           0.0},  //   0°  east
    { 0.707106781,   0.707106781,   0.0},  //  45°  southeast
    { 0.0,           1.0,           0.0},  //  90°  south (+Y)
    {-0.707106781,   0.707106781,   0.0},  // 135°  southwest
    {-1.0,           0.0,           0.0},  // 180°  west
    {-0.707106781,  -0.707106781,   0.0},  // 225°  northwest
    { 0.0,          -1.0,           0.0},  // 270°  north (-Y)
    { 0.707106781,  -0.707106781,   0.0},  // 315°  northeast
    { 0.0,           0.0,           1.0},  // up
    { 0.0,           0.0,          -1.0},  // down
}};

ExplorationAlgorithm::ExplorationAlgorithm(
    BuildingMap& bm, const DroneConfig& c, const MissionConfig& m)
    : buildingMap_(bm), config_(c), mission_(m) {}

HorizontalAngle ExplorationAlgorithm::computeStepAngle() const {
    return DroneMath::computeStepAngle(config_.lidar);
}

PhysicalLength ExplorationAlgorithm::computeMoveStep() const {
    return DroneMath::computeMoveStep(config_, mission_);
}

double ExplorationAlgorithm::halfWidth() const {
    return config_.minPassWidth.force_numerical_value_in(cm) / 2.0;
}

double ExplorationAlgorithm::halfLength() const {
    return config_.minPassLength.force_numerical_value_in(cm) / 2.0;
}

double ExplorationAlgorithm::halfHeight() const {
    return config_.minPassHeight.force_numerical_value_in(cm) / 2.0;
}

//given current position (and horizontal heading), return true if a valid target
//has been set as nextTarget_. else return false
bool ExplorationAlgorithm::findNextTarget(const Position3D& pos,
                                          HorizontalAngle   heading) {
    const double step = computeMoveStep().force_numerical_value_in(cm);
    const double hW = halfWidth();
    const double hL = halfLength();
    const double hH = halfHeight();
    const int    mapRes = mission_.xyResolution;

    const double px = pos.x.force_numerical_value_in(cm);
    const double py = pos.y.force_numerical_value_in(cm);
    const double pz = pos.z.force_numerical_value_in(cm);

    for (const auto& dir : kDirections) { //find first valid(=reachable) target in priority order
        const double tx = px + step * dir[0];
        const double ty = py + step * dir[1];
        const double tz = pz + step * dir[2];
        const Position3D cand{ // candidate target position in world coordinates
            tx * x_extent[cm],
            ty * y_extent[cm],
            tz * z_extent[cm],
        };
        const CellKey key = DroneMath::snapToGrid( //find this position's corresponding cell in output map
            cand, mission_.xyResolution, mission_.zResolution);

        if (visited_.count(key))                           continue; //already visited this cell
        if (buildingMap_.getCell(key) != CellValue::Empty) continue;//cell not empty->it cant be target

        if (dir[2] != 0.0) { //  up/down elevate
            const double h = toRad(heading.force_numerical_value_in(deg));
            if (!DroneMath::canElevate(pos, dir[2] * step, h,
                                       hW, hL, hH, buildingMap_, mapRes))
                continue;
        } else { // horizontal advance
            const double reqH = std::atan2(dir[1], dir[0]);
            if (!DroneMath::canAdvance(pos, step, reqH,
                                       hW, hL, hH, buildingMap_, mapRes))
                continue;
        }

        nextTarget_ = cand;
        return true;
    }
    return false; //no target could be found (in 10 directions) -> will need to backtrack
}

//decides the next command to issue based on current phase and state.
// also updates internal state (like phase, scan angles, inverse stack) as needed.
Command ExplorationAlgorithm::decide(const Position3D& currentPos, HorizontalAngle currentHeading)
{
    switch (phase_) {
        case Phase::Scanning: { //we remain in scanning phase until we do full sphere scan at curr pos
            Command cmd = makeScan(scanXY_, scanEl_);

            const double stepDeg = computeStepAngle().force_numerical_value_in(deg);
            double xy = scanXY_.force_numerical_value_in(deg) + stepDeg;
            double el = scanEl_.force_numerical_value_in(deg); 

            if (xy >= 360.0) {
                xy = 0.0;
                el += stepDeg;
                if (el > 90.0) {
                    // Sphere scan complete.
                    const CellKey here = DroneMath::snapToGrid(
                        currentPos, mission_.xyResolution, mission_.zResolution);
                    visited_.insert(here);
                    phase_ = Phase::ChoosingNext;
                }
            }
            scanXY_ = xy * horizontal_angle[deg]; //update next scan angle
            scanEl_ = el * altitude_angle[deg];
            return cmd;
        }

        case Phase::ChoosingNext: {
            if (findNextTarget(currentPos, currentHeading)) {
                // Mark start of this DFS level on the inverse stack so
                // Backtracking knows where to stop unwinding.
                inverseStack_.push_back(makeLevelMarker());
                phase_ = Phase::Moving;
                return decide(currentPos, currentHeading);
            }
            if (inverseStack_.empty()) { //couldnt find next target and no back tracking moves left
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
                inverseStack_.push_back(makeElevate(-dz));
                scanXY_ =   0.0 * horizontal_angle[deg]; //reset scan angles since we are about-
                scanEl_ = -90.0 * altitude_angle[deg];   //-to move and then return to scanning phase
                phase_  = Phase::Scanning;
                return cmd; //send elevate command
            }

            // Horizontal move — rotate first if needed.
            const double horiz = std::sqrt(dx * dx + dy * dy); //horizontal distance to target
            if (horiz > EPS_CM) {
                const double requiredDeg = toDeg(std::atan2(dy, dx)); //absolute needed angle
                const double curDeg      = currentHeading.force_numerical_value_in(deg);
                const double delta       = wrapTo180(requiredDeg - curDeg);

                if (std::abs(delta) > EPS_DEG) { //need to rotate before advance
                    Command cmd = makeRotate(delta);
                    inverseStack_.push_back(makeRotate(-delta));
                    return cmd; // remain in Moving; do NOT reset scan
                }
                //no need to rotate (we already rotated), can advance directly
                Command cmd = makeAdvance(horiz);
                // Push 3 inverses in reverse-execution order;
                // pop order during backtrack: Rot(-180), Adv(d), Rot(180).
                inverseStack_.push_back(makeRotate( 180.0)); // executes 3rd
                inverseStack_.push_back(makeAdvance(horiz)); // executes 2nd
                inverseStack_.push_back(makeRotate(-180.0)); // executes 1st
                scanXY_ =   0.0 * horizontal_angle[deg]; //reset scaning angle since we about to arrive in new pos
                scanEl_ = -90.0 * altitude_angle[deg];
                phase_  = Phase::Scanning;
                return cmd;
            }

            // Already at target (shouldn't normally happen with non-zero step).
            // Pop the level marker we opened in ChoosingNext to keep the stack clean.
            if (!inverseStack_.empty()
                && inverseStack_.back().type == CommandType::LevelMarker) {
                inverseStack_.pop_back();
            }
            //we shouldnt reach this step (only happens when our target was at very short distance from our pos)
            //but if we do end up here, start looking for a new target. 
            phase_ = Phase::ChoosingNext; //(no need to scan since we already scanned in this position if we reached this phase)                  
            return decide(currentPos, currentHeading);
        }

        case Phase::Backtracking: {
            if (inverseStack_.empty()) {
                phase_ = Phase::Finished;
                return makeFinished();
            }
            const Command cmd = inverseStack_.back();
            inverseStack_.pop_back();

            if (cmd.type == CommandType::LevelMarker) {
                // One full DFS level fully undone — drone is back at the parent
                // position (already in visited_). Try another direction without
                // rescanning.
                phase_ = Phase::ChoosingNext;
                return decide(currentPos, currentHeading);
            }
            return cmd;
        }

        case Phase::Finished:
        default:
            return makeFinished();
    }
}

} // namespace cpp_course
