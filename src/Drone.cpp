#include <cpp_course/Drone.h>
#include <cpp_course/DroneMath.h>

#include <cmath>

namespace cpp_course {

namespace {

// Beam-direction matching tolerance. MockLidar uses separable horizontal/
// altitude offsets while computeBeamDirections normalises a 3D vector, so the
// two methods diverge slightly on outer circles — 0.1° absorbs that drift.
constexpr double EPS_DEG = 0.1;

// Smallest absolute difference between two horizontal angles modulo 360°.
double angleDiff(double a, double b) {
    double d = std::fmod(a - b, 360.0);
    if (d >  180.0) d -= 360.0;
    if (d < -180.0) d += 360.0;
    return std::abs(d);
}

} // namespace

Drone::Drone(const ILidarSensor&    lidar,
             const IPositionSensor& posSensor,
             IMovementDriver&       driver,
             BuildingMap&           buildingMap,
             ExplorationAlgorithm&  algorithm,
             const DroneConfig&     config,
             const MissionConfig&   mission)
    : lidar_(lidar)
    , posSensor_(posSensor)
    , driver_(driver)
    , buildingMap_(buildingMap)
    , algorithm_(algorithm)
    , config_(config)
    , mission_(mission) {}

Command Drone::getNextCommand() {
    const Position3D     pos = posSensor_.position();
    const HorizontalAngle hdg = posSensor_.heading().horizontal;
    return algorithm_.decide(pos, hdg);
}

void Drone::execute(const Command& cmd) {
    switch (cmd.type) {
        case CommandType::Scan: {
            // Lidar takes a drone-relative orientation; the algorithm already
            // tracks scanXY_/scanEl_ in drone-relative space, so forward as-is.
            const ScanResults results = lidar_.scan(cmd.scanOrientation);
            processScan(results, cmd.scanOrientation); //update map for both hits and misses.
            break;
        }
        case CommandType::Rotate:
            (void)driver_.rotate(cmd.angleValue);
            break;
        case CommandType::Advance:
            (void)driver_.advance(cmd.distanceValue);
            break;
        case CommandType::Elevate:
            (void)driver_.elevate(cmd.distanceValue);
            break;
        case CommandType::LevelMarker:
        case CommandType::Finished:
        default:
            // Internal algorithm sentinels — no-op for the Drone.
            break;
    }
}

void Drone::markEmpty(const CellKey& key) {
    // Occupied is sticky: never overwrite a confirmed hit with an Empty.
    if (buildingMap_.getCell(key) != CellValue::Occupied)
        buildingMap_.setCell(key, CellValue::Empty);
}

//the scan process is as follows:
//for each beam in results (=hits): update the map for those beams up to beam distance. (ignore 0 dist beams)
//for each beam in [allBeams\results] : update map up until max beam length.(dont overwrite walls->shouldnt happen anyway)
void Drone::processScan(const ScanResults& results,
                        const Orientation& relScanOrientation) {
    const Position3D     pos    = posSensor_.position();
    const HorizontalAngle hdg   = posSensor_.heading().horizontal;
    const double         hdgDeg = hdg.force_numerical_value_in(deg);

    // World scan orientation = drone heading + relative scan orientation.
    const Orientation worldScan{
        (hdgDeg + relScanOrientation.horizontal.force_numerical_value_in(deg))
            * horizontal_angle[deg],
        relScanOrientation.altitude,
    };

    const auto allBeams = DroneMath::computeBeamDirections(worldScan, config_.lidar);
    const int xyR = mission_.xyResolution;
    const int zR  = mission_.zResolution;

    for (const auto& beam : allBeams) {
        const double beamH = beam.horizontal.force_numerical_value_in(deg);
        const double beamA = beam.altitude.force_numerical_value_in(deg);

        // Find the hit (if any) whose world-space angle matches this beam.
        // hit.angle is drone-relative, so convert by adding hdgDeg.
        const LidarHit* matched = nullptr;
        for (const auto& hit : results) {
            const double hitWorldH = hdgDeg
                + hit.angle.horizontal.force_numerical_value_in(deg);
            const double hitA = hit.angle.altitude.force_numerical_value_in(deg);
            if (angleDiff(beamH, hitWorldH) < EPS_DEG
                && std::abs(beamA - hitA) < EPS_DEG) {
                matched = &hit;
                break;
            }
        }

        if (matched != nullptr) {
            const double distCm = matched->distance.force_numerical_value_in(cm);
            if (distCm > 0.0) {
                // Real hit: path up to hit is Empty; endpoint is Occupied.
                const auto cells = DroneMath::rayMarch(
                    pos, beam, matched->distance, xyR, zR);
                for (const auto& k : cells) markEmpty(k);

                const Position3D hitPos = DroneMath::beamToWorldPoint(
                    pos, hdg, *matched);
                const CellKey hitKey = DroneMath::snapToGrid(hitPos, xyR, zR);
                buildingMap_.setCell(hitKey, CellValue::Occupied);
            }
            // distCm == 0: too close to measure — we know SOMETHING is near
            // but not where, so we deliberately make no map update.
        } else {
            // No hit at all — clear out to beam_length_max along this ray.
            const auto cells = DroneMath::rayMarch(
                pos, beam, config_.lidar.beam_length_max, xyR, zR);
            for (const auto& k : cells) markEmpty(k);
        }
    }
}

} // namespace cpp_course
