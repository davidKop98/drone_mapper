#include <cpp_course/ExplorationAlgorithm.h>
#include <cpp_course/DroneMath.h>

#include <cmath>

namespace cpp_course {

namespace {

constexpr double PI      = 3.14159265358979323846;
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
// Diagonals use unit vectors (0.707...). At resolution=0 with step=1cm,
// diagonal targets snap back to the current cell (rejected as visited),
// so the algorithm is effectively axis-aligned only. Enabling true
// diagonal moves needs canAdvance to handle a rotated body (its world-
// axis projection grows by √2 at 45°, exceeding `halfWidth`) and
// per-caller FP-precision parity between findNextTarget's reqH and
// MockMovementDriver's toRad(heading). Left as future work.
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

// ---- Chunking helpers ----------------------------------------------------
// Forward emission: return one Command sized at most by the per-command max,
// decrementing `remaining`. Sign of `remaining` is preserved on the chunk.

Command ExplorationAlgorithm::emitRotateChunk(double& remaining) const {
    const double maxRot = config_.maxRotate.force_numerical_value_in(deg);
    double chunk;
    if (maxRot <= 0.0 || std::abs(remaining) <= maxRot) {
        chunk = remaining;
        remaining = 0.0;
    } else {
        chunk = (remaining > 0.0) ? maxRot : -maxRot;
        remaining -= chunk;
    }
    return makeRotate(chunk);
}

Command ExplorationAlgorithm::emitAdvanceChunk(double& remaining) const {
    const double maxAdv = config_.maxAdvance.force_numerical_value_in(cm);
    double chunk;
    if (maxAdv <= 0.0 || std::abs(remaining) <= maxAdv) {
        chunk = remaining;
        remaining = 0.0;
    } else {
        chunk = (remaining > 0.0) ? maxAdv : -maxAdv;
        remaining -= chunk;
    }
    return makeAdvance(chunk);
}

Command ExplorationAlgorithm::emitElevateChunk(double& remaining) const {
    const double maxEle = config_.maxElevate.force_numerical_value_in(cm);
    double chunk;
    if (maxEle <= 0.0 || std::abs(remaining) <= maxEle) {
        chunk = remaining;
        remaining = 0.0;
    } else {
        chunk = (remaining > 0.0) ? maxEle : -maxEle;
        remaining -= chunk;
    }
    return makeElevate(chunk);
}

// Inverse-push: push N Commands summing to `total` onto inverseStack_,
// each chunk's magnitude ≤ the per-command max. Order within one logical
// operation does not matter — all chunks execute consecutively and sum.

void ExplorationAlgorithm::pushChunkedRotate(double total) {
    const double maxRot = config_.maxRotate.force_numerical_value_in(deg);
    if (std::abs(total) <= EPS_DEG) return;
    if (maxRot <= 0.0) { inverseStack_.push_back(makeRotate(total)); return; }
    double remaining = total;
    while (std::abs(remaining) > maxRot) {
        const double chunk = (remaining > 0.0) ? maxRot : -maxRot;
        inverseStack_.push_back(makeRotate(chunk));
        remaining -= chunk;
    }
    if (std::abs(remaining) > EPS_DEG)
        inverseStack_.push_back(makeRotate(remaining));
}

void ExplorationAlgorithm::pushChunkedAdvance(double total) {
    const double maxAdv = config_.maxAdvance.force_numerical_value_in(cm);
    if (std::abs(total) <= EPS_CM) return;
    if (maxAdv <= 0.0) { inverseStack_.push_back(makeAdvance(total)); return; }
    double remaining = total;
    while (std::abs(remaining) > maxAdv) {
        const double chunk = (remaining > 0.0) ? maxAdv : -maxAdv;
        inverseStack_.push_back(makeAdvance(chunk));
        remaining -= chunk;
    }
    if (std::abs(remaining) > EPS_CM)
        inverseStack_.push_back(makeAdvance(remaining));
}

void ExplorationAlgorithm::pushChunkedElevate(double total) {
    const double maxEle = config_.maxElevate.force_numerical_value_in(cm);
    if (std::abs(total) <= EPS_CM) return;
    if (maxEle <= 0.0) { inverseStack_.push_back(makeElevate(total)); return; }
    double remaining = total;
    while (std::abs(remaining) > maxEle) {
        const double chunk = (remaining > 0.0) ? maxEle : -maxEle;
        inverseStack_.push_back(makeElevate(chunk));
        remaining -= chunk;
    }
    if (std::abs(remaining) > EPS_CM)
        inverseStack_.push_back(makeElevate(remaining));
}

//given current position (and horizontal heading), return true if a valid target
//has been set as nextTarget_. else return false
bool ExplorationAlgorithm::findNextTarget(const Position3D& pos,
                                          HorizontalAngle   heading) {
    const double step = computeMoveStep().force_numerical_value_in(cm);
    const double hW = halfWidth();
    const double hL = halfLength();
    const double hH = halfHeight();
    const int xyRes = mission_.xyResolution;
    const int zRes  = mission_.zResolution;

    const double px = pos.x.force_numerical_value_in(cm);
    const double py = pos.y.force_numerical_value_in(cm);
    const double pz = pos.z.force_numerical_value_in(cm);

    for (const auto& dir : kDirections) { //find first valid(=reachable) target in priority order
        // Skip diagonal directions entirely. Unit-vector diagonals (0.707) at
        // step=1cm produce target positions that floor to the current cell for
        // +x/+y diagonals (visited → skipped) but to an ADJACENT cell for
        // -x/-y diagonals (taken!). Taking the asymmetric subset puts the
        // drone at sub-cell positions, and subsequent cardinal moves sweep a
        // rotated body footprint that canAdvance's perpendicular-only
        // sampling doesn't fully cover → collisions. Until canAdvance handles
        // rotated-body geometry (see comment on kDirections), keep moves
        // strictly axis-aligned.
        const bool isDiagonal =
            (std::abs(dir[0]) > 0.1 && std::abs(dir[0]) < 0.9) ||
            (std::abs(dir[1]) > 0.1 && std::abs(dir[1]) < 0.9);
        if (isDiagonal) continue;

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
                                       hW, hL, hH, buildingMap_, zRes)) 
                continue;
        } else { // horizontal advance
            const double reqH = std::atan2(dir[1], dir[0]);
            if (!DroneMath::canAdvance(pos, step, reqH,
                                       hW, hL, hH, buildingMap_, xyRes))//formyself: might need to change xyRes to min(Zres,XY res)
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
            //self note: this is messy because we had to add chunking since we can have limited advance/rotation.
            //the code is cleaner without those constraints.
            // 1. Drain pending chunks first. Only one of the three is ever
            //    non-zero at a time. Rotation completing keeps us in Moving
            //    (the next call recomputes delta — should now be ~0).
            //    Advance/Elevate completing transitions to Scanning.
            if (std::abs(pendingRotateDeg_) > EPS_DEG) {
                return emitRotateChunk(pendingRotateDeg_);
            }
            if (std::abs(pendingAdvanceCm_) > EPS_CM) {
                Command cmd = emitAdvanceChunk(pendingAdvanceCm_);
                if (std::abs(pendingAdvanceCm_) <= EPS_CM) {
                    scanXY_ =   0.0 * horizontal_angle[deg];
                    scanEl_ = -90.0 * altitude_angle[deg];
                    phase_  = Phase::Scanning;
                }
                return cmd;
            }
            if (std::abs(pendingElevateCm_) > EPS_CM) {
                Command cmd = emitElevateChunk(pendingElevateCm_);
                if (std::abs(pendingElevateCm_) <= EPS_CM) {
                    scanXY_ =   0.0 * horizontal_angle[deg];
                    scanEl_ = -90.0 * altitude_angle[deg];
                    phase_  = Phase::Scanning;
                }
                return cmd;
            }

            // 2. No pending — compute deltas to target.
            const double dx = nextTarget_.x.force_numerical_value_in(cm)
                            - currentPos.x.force_numerical_value_in(cm);
            const double dy = nextTarget_.y.force_numerical_value_in(cm)
                            - currentPos.y.force_numerical_value_in(cm);
            const double dz = nextTarget_.z.force_numerical_value_in(cm)
                            - currentPos.z.force_numerical_value_in(cm);

            // Pure vertical move.
            if (std::abs(dz) > EPS_CM) {
                pendingElevateCm_ = dz;
                pushChunkedElevate(-dz);   // inverse: opposite sign
                return decide(currentPos, currentHeading); // emit first chunk
            }

            // Horizontal move — rotate first if needed.
            const double horiz = std::sqrt(dx * dx + dy * dy);
            if (horiz > EPS_CM) {
                const double requiredDeg = toDeg(std::atan2(dy, dx));
                const double curDeg      = currentHeading.force_numerical_value_in(deg);
                const double delta       = wrapTo180(requiredDeg - curDeg);

                if (std::abs(delta) > EPS_DEG) {
                    pendingRotateDeg_ = delta;
                    pushChunkedRotate(-delta);   // inverse rotation
                    return decide(currentPos, currentHeading); // emit first chunk
                }

                // Already facing target — advance.
                pendingAdvanceCm_ = horiz;
                // Push 3-part inverse (each part itself chunked).
                // Execution order during backtrack: Rot(180), Adv(horiz), Rot(-180).
                // Reverse-execution push order: Rot(-180) chunks, Adv chunks, Rot(180) chunks.
                pushChunkedRotate(-180.0);  // executed 3rd (popped last)
                pushChunkedAdvance(horiz);  // executed 2nd
                pushChunkedRotate( 180.0);  // executed 1st (popped first)
                return decide(currentPos, currentHeading); // emit first chunk
            }

            // Already at target (shouldn't normally happen).
            // Pop the level marker we opened in ChoosingNext to keep the stack clean.
            if (!inverseStack_.empty()
                && inverseStack_.back().type == CommandType::LevelMarker) {
                inverseStack_.pop_back();
            }
            phase_ = Phase::ChoosingNext;
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
