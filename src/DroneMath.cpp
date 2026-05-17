#include <cpp_course/DroneMath.h>

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <vector>

namespace cpp_course::DroneMath {

namespace {

constexpr double PI = 3.1415926535;

double toRad(double d) { return d * PI / 180.0; }
double fromRad(double r) { return r * 180.0 / PI; }

// Raw-double helpers: extract degrees from the two angle types.
double hDeg(HorizontalAngle a) { return a.force_numerical_value_in(deg); }
double aDeg(Altitude a)         { return a.force_numerical_value_in(deg); }

struct Vec3 { double x, y, z; };

Vec3 normalize(Vec3 v) {
    const double len = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (len < 1e-12) return v;
    return {v.x/len, v.y/len, v.z/len};
}

} // namespace

// ---------------------------------------------------------------------------

//returns the world coordinate of the "wall" cell this beam hit
Position3D beamToWorldPoint(const Position3D& dronePos,
                             HorizontalAngle   droneHeading,
                             const LidarHit&   hit) {
    const double wh  = toRad(hDeg(droneHeading) + hDeg(hit.angle.horizontal));
    const double wa  = toRad(aDeg(hit.angle.altitude));//might need to add drone's aDeg in future exercise, but for now it's always 0, so no need to add it.
    const double d   = hit.distance.force_numerical_value_in(cm);

    const double dh  = d * std::cos(wa);
    const double dx  = dh * std::cos(wh);
    const double dy  = dh * std::sin(wh);
    const double dz  = d  * std::sin(wa);

    return {
        dronePos.x + dx * x_extent[cm],
        dronePos.y + dy * y_extent[cm],
        dronePos.z + dz * z_extent[cm],
    };
}

// ---------------------------------------------------------------------------
//given a coordinate, return the "cell" in the 3D map with resolution decimalPlace it fits in
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
        if (seen.insert(key).second)
            result.push_back(key);
    }
    return result;
}

// ---------------------------------------------------------------------------
//given an ABSOLUTE scan orientaion (=circle 0 angle), computes all ABSOLUTE angles-
//-for ALL the beams in this scan, according to lidar configs (= D, min_dist).
//this pretty much calculates the gradient of all the beams, so we can use it in rayMarch()
std::vector<Orientation> computeBeamDirections(const Orientation& scanOrientation,
                                                const LidarConfig& cfg) {
    std::vector<Orientation> result;
    if (cfg.fov_circles == 0) return result;

    const double h  = toRad(hDeg(scanOrientation.horizontal));
    const double a  = toRad(aDeg(scanOrientation.altitude));
    const double ca = std::cos(a), sa = std::sin(a);
    const double ch = std::cos(h), sh = std::sin(h);

    // Forward unit vector F in the direction of scanOrientation
    const Vec3 F{ca*ch, ca*sh, sa};

    // Right vector R: perpendicular to F in the horizontal plane
    const Vec3 R{-sh, ch, 0.0};

    // Up vector U = F cross R (perpendicular to both F and R)
    const Vec3 U{
        F.y*R.z - F.z*R.y,
        F.z*R.x - F.x*R.z,
        F.x*R.y - F.y*R.x,
    };

    // Circle 0: center beam — use scanOrientation angles directly (same as F converted back)
    result.push_back(scanOrientation);

    const double spacing  = cfg.circle_spacing.force_numerical_value_in(cm);
    const double beam_min = cfg.beam_length_min.force_numerical_value_in(cm);

    for (std::size_t circle = 1; circle < cfg.fov_circles; ++circle) {
        // beam_count = 4^circle
        const auto beam_count = static_cast<std::size_t>(
            std::round(std::pow(4.0, static_cast<double>(circle))));

        // tan(beam_angle) = N * spacing / beam_min  [since tan(atan(x)) = x]
        const double tan_angle =
            static_cast<double>(circle) * spacing / beam_min;

        for (std::size_t i = 0; i < beam_count; ++i) {
            const double theta = 2.0 * PI * static_cast<double>(i)
                               / static_cast<double>(beam_count);
            const double ct = std::cos(theta), st = std::sin(theta);

            // offset = cos(theta)*R + sin(theta)*U  (unit vector on circle rim)
            const Vec3 offset{ct*R.x + st*U.x,
                              ct*R.y + st*U.y,
                              ct*R.z + st*U.z};

            // direction = normalize(F + tan_angle * offset)
            const Vec3 d = normalize(Vec3{
                F.x + tan_angle * offset.x,
                F.y + tan_angle * offset.y,
                F.z + tan_angle * offset.z});

            result.push_back(Orientation{
                fromRad(std::atan2(d.y, d.x)) * horizontal_angle[deg],
                fromRad(std::asin(std::clamp(d.z, -1.0, 1.0))) * altitude_angle[deg],
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
    const double spacing  = cfg.circle_spacing.force_numerical_value_in(cm); //D
    const double beam_min = cfg.beam_length_min.force_numerical_value_in(cm); //Zmin
    const double step_deg = factor * fromRad(atan(spacing / beam_min));
    return step_deg * horizontal_angle[deg];
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
bool checkAdvanceSlice(const Position3D& center,
                       double halfWidth,
                       double halfHeight,
                       double headingRad,
                       const IMap3D& map,
                       int mapResolution) {
    const double cellSize = std::pow(10.0, -mapResolution);

    // vec2: horizontal unit vector perpendicular to heading.
    const double vec2X = -std::sin(headingRad);
    const double vec2Y =  std::cos(headingRad);

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
            if (map.get(sample) != 0) return true;
        }
    }
    return false;
}

bool checkElevateSlice(const Position3D& center,
                       double halfWidth,
                       double halfLength,
                       double headingRad,
                       const IMap3D& map,
                       int mapResolution) {
    const double cellSize = std::pow(10.0, -mapResolution);

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
                cz * z_extent[cm],
            };
            if (map.get(sample) != 0) return true;
        }
    }
    return false;
}

bool canAdvance(const Position3D& start,
                double distanceCm,
                double headingRad,
                double halfWidth,
                double halfLength,
                double halfHeight,
                const IMap3D& map,
                int mapResolution) {
    const double fx = std::cos(headingRad);
    const double fy = std::sin(headingRad);

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
