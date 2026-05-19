#include <cpp_course/DroneMath.h>
#include <cpp_course/Profiler.h>

#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_set>
#include <vector>

namespace cpp_course::DroneMath {

namespace {

// Full-precision π — closest double to true π. The previous truncated
// value (3.1415926535) was off by ~9e-11, and that drift accumulated
// through atan2 → toDeg → rotate-chunking → toRad → cos/sin, so headings
// like 270° ended up as 269.9999999983°. With a 2x2 drone, that's enough
// for perpendicular samples to leak one cell sideways and trip canAdvance
// against an actual wall — caused a spurious collision in test_big_2floors.
constexpr double PI = 3.14159265358979323846;

double toRad(double d) { return d * PI / 180.0; }
double fromRad(double r) { return r * 180.0 / PI; }

// Raw-double helpers: extract degrees from the two angle types.
double hDeg(HorizontalAngle a) { return a.force_numerical_value_in(deg); }
double aDeg(Altitude a)         { return a.force_numerical_value_in(deg); }

} // namespace

// Snap a sin/cos result to {-1, 0, 1} when within FP tolerance.
// std::sin(π) returns ~1.22e-16 instead of 0, std::cos(3π/2) similar.
// For drones with integer halfWidth (e.g. 2x2), perpendicular samples land
// on exact cell boundaries — any FP residue in vec2X/vec2Y shoves the
// sample one ULP to the wrong side and into an adjacent cell. Snapping
// the trig outputs eliminates this without changing any non-cardinal
// direction (where the values aren't near integers).
// Public (also called from MockMovementDriver to keep position-update
// math in sync with the collision check).
double snapCardinal(double v) {
    constexpr double tol = 1e-10;       // far above FP error (~1e-15), far below cell size
    if (std::abs(v) < tol) return 0.0;
    if (std::abs(v - 1.0) < tol) return 1.0;
    if (std::abs(v + 1.0) < tol) return -1.0;
    return v;
}

// ---------------------------------------------------------------------------

// Reconstruct the world-space point where the lidar beam terminated.
//
// Math MUST match MockLidarSensor::traceBeam bit-for-bit: same mp-units
// si::cos/sin, same operation order. The previous raw-double formula
// (toRad + std::cos/sin, with d pre-factored) diverged by FP epsilon for
// some beam angles, causing the Drone to snap the hit to a cell adjacent
// to the one the lidar actually sampled. Symptom: doorway cells and
// stairwell cells getting wrongly marked Occupied, blocking navigation.
Position3D beamToWorldPoint(const Position3D& dronePos,
                             HorizontalAngle   droneHeading,
                             const LidarHit&   hit) {
    // Absolute beam orientation (drone altitude is always 0 in this exercise).
    const HorizontalAngle world_horizontal = droneHeading + hit.angle.horizontal;
    const Altitude        world_altitude   = hit.angle.altitude;

    const auto cos_altitude = si::cos(world_altitude);
    const auto dx = cos_altitude * si::cos(world_horizontal);
    const auto dy = cos_altitude * si::sin(world_horizontal);
    const auto dz = si::sin(world_altitude);

    return {
        dronePos.x + dx.force_numerical_value_in(mp::one) * hit.distance.force_numerical_value_in(cm) * x_extent[cm],
        dronePos.y + dy.force_numerical_value_in(mp::one) * hit.distance.force_numerical_value_in(cm) * y_extent[cm],
        dronePos.z + dz.force_numerical_value_in(mp::one) * hit.distance.force_numerical_value_in(cm) * z_extent[cm],
    };
}

// ---------------------------------------------------------------------------
// Snap to cell boundary at the given resolution. The float-cast on the input
// is intentional and load-bearing: it absorbs ~1e-7 of FP drift accumulated
// from cos/sin(π) etc. during movement, keeping the drone's stored position
// snapped to the cell it logically occupies. Don't change to double floor
// without also addressing position-drift accumulation.
float snapValue(float value, int decimalPlaces) {
    const float factor = std::pow(10.0f, static_cast<float>(decimalPlaces));
    return std::floor(value * factor) / factor;
}

// ---------------------------------------------------------------------------
//helper function to find the appropriate cell in output map
CellKey snapToGrid(const Position3D& worldPos, int xyResolution, int zResolution) {
    return {
        snapValue(static_cast<float>(worldPos.x.force_numerical_value_in(cm)), xyResolution),
        snapValue(static_cast<float>(worldPos.y.force_numerical_value_in(cm)), xyResolution),
        snapValue(static_cast<float>(worldPos.z.force_numerical_value_in(cm)), zResolution),
    };
}

// ---------------------------------------------------------------------------
//given a position, (an absolute) direction and a max distance, walk in that direction and "save" all
//cells which we pass up until we reach that max distance.
//returns vector of cell keys we passed through
std::vector<CellKey> rayMarch(const Position3D&  origin,
                               const Orientation& direction,
                               PhysicalLength     maxDist,
                               int xyResolution,
                               int zResolution) {
    ++profiler::rayMarch_calls;
    profiler::ScopedTimer _t(profiler::rayMarch_total_ns);
    //smaller factor = smaller steps while "collecting" all cells in the direction of the beam, up to max distance
    double factor = 0.3;
    const int maxRes = std::max(xyResolution, zResolution);
    const double stepCm = factor * std::pow(10.0, -static_cast<double>(maxRes));

    const double h = toRad(hDeg(direction.horizontal));
    const double a = toRad(aDeg(direction.altitude));
    const double cos_a = std::cos(a);
    const double dx = cos_a * std::cos(h);
    const double dy = cos_a * std::sin(h);
    const double dz = std::sin(a);

    const double maxCm = maxDist.force_numerical_value_in(cm);
    const double ox = origin.x.force_numerical_value_in(cm);
    const double oy = origin.y.force_numerical_value_in(cm);
    const double oz = origin.z.force_numerical_value_in(cm);

    std::vector<CellKey> result;
    std::unordered_set<CellKey, CellKeyHash> seen;

    for (double dist = 0.0; dist <= maxCm; dist += stepCm) {
        const Position3D p{
            (ox + dx * dist) * x_extent[cm],
            (oy + dy * dist) * y_extent[cm],
            (oz + dz * dist) * z_extent[cm],
        };
        const CellKey key = snapToGrid(p, xyResolution, zResolution);
        if (seen.insert(key).second) {
            result.push_back(key);
            ++profiler::rayMarch_cells_pushed;
        }
    }
    return result;
}

// ---------------------------------------------------------------------------
//given an ABSOLUTE scan orientaion (=circle 0 angle), computes all ABSOLUTE angles-
//-for ALL the beams in this scan, according to lidar configs (= D, min_dist).
std::vector<Orientation> computeBeamDirections(const Orientation& scanOrientation,
                                                const LidarConfig& cfg) {
    std::vector<Orientation> result;
    if (cfg.fov_circles == 0) return result;

    // Circle 0: center beam is the scan orientation itself.
    result.push_back(scanOrientation);

    const double spacing  = cfg.circle_spacing.force_numerical_value_in(cm);
    const double beam_min = cfg.beam_length_min.force_numerical_value_in(cm);

    // Use the same separable-atan formula as MockLidarSensor so that
    // computeBeamDirections and the lidar agree on beam angles.  The 3-D
    // vector approach diverges from the separable formula when scanEl ≠ 0,
    // causing false "no-hit" treatments and incorrect Empty markings.
    const double scanH = hDeg(scanOrientation.horizontal);
    const double scanA = aDeg(scanOrientation.altitude);

    for (std::size_t circle = 1; circle < cfg.fov_circles; ++circle) {
        const auto beam_count = static_cast<std::size_t>(
            std::round(std::pow(4.0, static_cast<double>(circle))));

        const double radius = static_cast<double>(circle) * spacing;

        for (std::size_t i = 0; i < beam_count; ++i) {
            const double theta = 2.0 * PI * static_cast<double>(i)
                               / static_cast<double>(beam_count);

            // Separable offsets matching MockLidarSensor::scan():
            //   horizontal_offset = radius * cos(theta)
            //   altitude_offset   = radius * sin(theta)
            //   h_delta = atan2(horizontal_offset, beam_min)
            //   a_delta = atan2(altitude_offset,   beam_min)
            const double h_delta = std::atan2(radius * std::cos(theta), beam_min);
            const double a_delta = std::atan2(radius * std::sin(theta), beam_min);

            result.push_back(Orientation{
                (scanH + fromRad(h_delta)) * horizontal_angle[deg],
                (scanA + fromRad(a_delta)) * altitude_angle[deg],
            });
        }
    }
    return result;
}

// ---------------------------------------------------------------------------
// calculates step size for scan step we intend to do in a 360 spherical around the current position
// the smaller the factor the more steps we gonna do = better coverage but less efficient.
HorizontalAngle computeStepAngle(const LidarConfig& cfg) {
    double factor = 1; //may need to adjust this. can heavily influence run time.
    double maxStepAngle = 5.0;
    const double spacing  = cfg.circle_spacing.force_numerical_value_in(cm); //D
    const double beam_min = cfg.beam_length_min.force_numerical_value_in(cm); //Zmin
    const double step_deg = factor * fromRad(atan(spacing / beam_min));
    return std::min(maxStepAngle, step_deg) * horizontal_angle[deg];
}

// ---------------------------------------------------------------------------
//we decided that the minimal step in our program while moving (during BFS) 
// will be the smallest resolution in our output map.
//if i want to make this max efficient, i'd need to calculate the biggest steping distance-
//possible for the given FOV angle, and to ensure we jump the biggest distance, while never hitting a potential blind spot.
PhysicalLength computeMoveStep(const DroneConfig&   droneConfig,
                                const MissionConfig& missionConfig) {
    double xy_cell = std::pow(10.0, -static_cast<double>(missionConfig.xyResolution));
    double z_cell  = std::pow(10.0, -static_cast<double>(missionConfig.zResolution));
    double step    = std::min(xy_cell, z_cell);
    return step * cm;
}

// ---------------------------------------------------------------------------
// Collision-checking helpers (moved from MockMovementDriver; logic unchanged).
//returns TRUE IF WE HIT A WALL
bool checkAdvanceSlice(const Position3D& center,
                       double halfWidth,
                       double halfHeight,
                       double headingRad,
                       const IMap3D& map,
                       int mapResolution) {
    const double cellSize = std::pow(10.0, -mapResolution);

    // vec2: horizontal unit vector perpendicular to heading.
    // Snap cardinals — sin(π) returns ~1e-16, and with integer halfWidth
    // that FP residue shoves perpendicular samples one cell sideways.
    const double vec2X = snapCardinal(-std::sin(headingRad));
    const double vec2Y = snapCardinal( std::cos(headingRad));

    const double stepI = (halfHeight > 0) ? cellSize / halfHeight : 1.0;
    const double stepJ = (halfWidth  > 0) ? cellSize / halfWidth  : 1.0;

    const double cx = center.x.force_numerical_value_in(cm);
    const double cy = center.y.force_numerical_value_in(cm);
    const double cz = center.z.force_numerical_value_in(cm);

    for (double i = -1.0; i <= 1.0 + stepI * 0.5; i += stepI) {
        const double fi = std::min(i, 1.0);
        for (double j = -1.0; j <= 1.0 + stepJ * 0.5; j += stepJ) {
            const double fj = std::min(j, 1.0);
            const Position3D sample{
                (cx + fj * halfWidth  * vec2X) * x_extent[cm],
                (cy + fj * halfWidth  * vec2Y) * y_extent[cm],
                (cz + fi * halfHeight)         * z_extent[cm],
            };
            if (map.get(sample) != 0) {
                std::printf("[HIT-A] center=(%.17g,%.17g,%.17g) hRad=%.17g fi=%.5g fj=%.5g sample=(%.17g,%.17g,%.17g) vec2=(%.17g,%.17g)\n",
                            cx, cy, cz, headingRad, fi, fj,
                            sample.x.force_numerical_value_in(cm),
                            sample.y.force_numerical_value_in(cm),
                            sample.z.force_numerical_value_in(cm),
                            vec2X, vec2Y);
                return true; //hit a wall
            }
        }
    }
    return false; //no wall hit
}

bool checkElevateSlice(const Position3D& center,
                       double halfWidth,
                       double halfLength,
                       double headingRad,
                       const IMap3D& map,
                       int mapResolution) {
    const double cellSize = std::pow(10.0, -mapResolution);

    const double vec1X = snapCardinal( std::cos(headingRad));
    const double vec1Y = snapCardinal( std::sin(headingRad));
    const double vec2X = snapCardinal(-std::sin(headingRad));
    const double vec2Y = snapCardinal( std::cos(headingRad));

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
                cz * z_extent[cm],
            };
            if (map.get(sample) != 0) return true; //hit a wall
        }
    }
    return false; //no wall hit
}

bool canAdvance(const Position3D& start,
                double distanceCm,
                double headingRad,
                double halfWidth,
                double halfLength,
                double halfHeight,
                const IMap3D& map,
                int mapResolution) {
    const double fx = snapCardinal(std::cos(headingRad));
    const double fy = snapCardinal(std::sin(headingRad));

    const double step_factor = 0.5;
    const double step = step_factor * std::pow(10.0, -mapResolution);

    const double startX = start.x.force_numerical_value_in(cm);
    const double startY = start.y.force_numerical_value_in(cm);
    const double startZ = start.z.force_numerical_value_in(cm);

    for (double t = -halfLength; t <= distanceCm + halfLength + step * 0.5; t += step) {
        const double tt = std::min(t, distanceCm + halfLength);
        const Position3D sliceCenter{
            (startX + fx * tt) * x_extent[cm],
            (startY + fy * tt) * y_extent[cm],
            startZ             * z_extent[cm],
        };
        if (checkAdvanceSlice(sliceCenter, halfWidth, halfHeight, headingRad,
                              map, mapResolution)) {
            return false;
        }
    }
    return true;
}
//input: world 3D point, dist to elevate (negative for down), current xy angle, 
//drone sizes, map to check on, map resolution  
bool canElevate(const Position3D& start,
                double distanceCm,
                double headingRad,
                double halfWidth,
                double halfLength,
                double halfHeight,
                const IMap3D& map,
                int mapResolution) {
    const double absD = std::abs(distanceCm);
    const double dirZ = (distanceCm >= 0) ? 1.0 : -1.0;

    const double step_factor = 0.5;
    const double step = step_factor * std::pow(10.0, -mapResolution);

    const double startX = start.x.force_numerical_value_in(cm);
    const double startY = start.y.force_numerical_value_in(cm);
    const double startZ = start.z.force_numerical_value_in(cm);

    for (double t = -halfHeight; t <= absD + halfHeight + step * 0.5; t += step) {
        const double tt = std::min(t, absD + halfHeight);
        const Position3D sliceCenter{
            startX               * x_extent[cm],
            startY               * y_extent[cm],
            (startZ + dirZ * tt) * z_extent[cm],
        };
        if (checkElevateSlice(sliceCenter, halfWidth, halfLength, headingRad,
                              map, mapResolution)) {
            return false;
        }
    }
    return true;
}

} // namespace cpp_course::DroneMath
