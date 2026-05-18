#include <cpp_course/Simulator.h>
#include <cpp_course/config_parser.h>

#include <cmath>
#include <fstream>
#include <iostream>

namespace cpp_course {

bool Simulator::loadConfigs(const std::string& path) {
    const std::string sep = (path.back() == '/') ? "" : "/";

    if (!parseDroneConfig(path + sep + "drone_config.txt",
                          droneConfig_, inputErrors_)) {
        std::cout << "Fatal: cannot open drone_config.txt\n";
        return false;
    }
    if (!parseMissionConfig(path + sep + "mission_config.txt",
                            missionConfig_, inputErrors_)) {
        std::cout << "Fatal: cannot open mission_config.txt\n";
        return false;
    }
    if (!inputMap_.loadFromFile(path + sep + "map_input.txt")) {
        std::cout << "Fatal: cannot open map_input.txt\n";
        return false;
    }

    // Persist any parse warnings (missing keys, fallback defaults, etc.).
    if (!inputErrors_.empty()) {
        std::ofstream errFile(path + sep + "input_errors.txt");
        if (errFile) {
            for (const auto& e : inputErrors_) errFile << e << '\n';
        }
    }

    // Initial drone state comes from MissionConfig.
    state_.position    = missionConfig_.startPosition;
    state_.heading     = missionConfig_.startHeading;
    simulationFailed_  = false;

    // Construction order matters: posSensor / driver / lidar all reference
    // state_ or each other.
    buildingMap_ = std::make_unique<BuildingMap>(missionConfig_);
    posSensor_   = std::make_unique<MockPositionSensor>(state_);
    driver_      = std::make_unique<MockMovementDriver>(
                       state_, droneConfig_, inputMap_, simulationFailed_);
    lidar_       = std::make_unique<MockLidarSensor>(
                       droneConfig_.lidar, inputMap_, *posSensor_);
    algorithm_   = std::make_unique<ExplorationAlgorithm>(
                       *buildingMap_, droneConfig_, missionConfig_);
    drone_       = std::make_unique<Drone>(
                       *lidar_, *posSensor_, *driver_,
                       *buildingMap_, *algorithm_, droneConfig_, missionConfig_);
    return true;
}

void Simulator::run() {
    while (true) {
        const Command cmd = drone_->getNextCommand();
        if (cmd.type == CommandType::Finished) break;
        if (!drone_->execute(cmd)) {
            std::cout << "SIMULATION FAILED - collision detected\n";
            break;
        }
    }
}

float Simulator::computeScore() {
    int correct = 0;
    int total   = 0;

    const double cellSizeXY = std::pow(10.0, -static_cast<double>(missionConfig_.xyResolution));
    const double cellSizeZ  = std::pow(10.0, -static_cast<double>(missionConfig_.zResolution));

    const double minX = missionConfig_.minX.force_numerical_value_in(cm);
    const double maxX = missionConfig_.maxX.force_numerical_value_in(cm);
    const double minY = missionConfig_.minY.force_numerical_value_in(cm);
    const double maxY = missionConfig_.maxY.force_numerical_value_in(cm);
    const double minZ = missionConfig_.minZ.force_numerical_value_in(cm);
    const double maxZ = missionConfig_.maxZ.force_numerical_value_in(cm);

    // Integer iteration to avoid float drift over many cells.
    const int sizeX = static_cast<int>(std::round((maxX - minX) / cellSizeXY));
    const int sizeY = static_cast<int>(std::round((maxY - minY) / cellSizeXY));
    const int sizeZ = static_cast<int>(std::round((maxZ - minZ) / cellSizeZ));

    for (int zi = 0; zi < sizeZ; ++zi) {
        const double z = minZ + zi * cellSizeZ;
        for (int yi = 0; yi < sizeY; ++yi) {
            const double y = minY + yi * cellSizeXY;
            for (int xi = 0; xi < sizeX; ++xi) {
                const double x = minX + xi * cellSizeXY;

                const CellKey key{
                    static_cast<float>(x),
                    static_cast<float>(y),
                    static_cast<float>(z),
                };
                const CellValue outVal = buildingMap_->getCell(key);
                if (outVal == CellValue::OutOfBounds) continue;

                // Sample InputMap at the cell center to bridge any resolution
                // mismatch between input/output maps.
                const Position3D center{
                    (x + cellSizeXY / 2.0) * x_extent[cm],
                    (y + cellSizeXY / 2.0) * y_extent[cm],
                    (z + cellSizeZ  / 2.0) * z_extent[cm],
                };
                const int inputVal = inputMap_.get(center); // 0 empty/OOB, 1 occupied

                ++total;
                if ((inputVal == 1 && outVal == CellValue::Occupied) ||
                    (inputVal == 0 && outVal == CellValue::Empty)) {
                    ++correct;
                }
            }
        }
    }

    return (total > 0)
        ? static_cast<float>(correct) / static_cast<float>(total) * 100.0f
        : 0.0f;
}

void Simulator::writeOutput(const std::string& path) {
    const std::string sep = (path.back() == '/') ? "" : "/";
    std::ofstream f(path + sep + "map_output.txt");
    if (!f) {
        std::cout << "Cannot open map_output.txt for writing\n";
        return;
    }

    const double cellSizeXY = std::pow(10.0, -static_cast<double>(missionConfig_.xyResolution));
    const double cellSizeZ  = std::pow(10.0, -static_cast<double>(missionConfig_.zResolution));

    const double minX = missionConfig_.minX.force_numerical_value_in(cm);
    const double maxX = missionConfig_.maxX.force_numerical_value_in(cm);
    const double minY = missionConfig_.minY.force_numerical_value_in(cm);
    const double maxY = missionConfig_.maxY.force_numerical_value_in(cm);
    const double minZ = missionConfig_.minZ.force_numerical_value_in(cm);
    const double maxZ = missionConfig_.maxZ.force_numerical_value_in(cm);

    const int sizeX = static_cast<int>(std::round((maxX - minX) / cellSizeXY));
    const int sizeY = static_cast<int>(std::round((maxY - minY) / cellSizeXY));
    const int sizeZ = static_cast<int>(std::round((maxZ - minZ) / cellSizeZ));

    f << "resolution: " << missionConfig_.xyResolution << '\n';
    f << "size_x: " << sizeX << '\n';
    f << "size_y: " << sizeY << '\n';
    f << "size_z: " << sizeZ << '\n';
    f << "origin_x: " << minX << '\n';
    f << "origin_y: " << minY << '\n';
    f << "origin_z: " << minZ << '\n';
    f << '\n';

    // Layers bottom to top, rows space-separated values:
    // 0=Empty, 1=Occupied, -1=Unmapped, -2=OutOfBounds.
    for (int zi = 0; zi < sizeZ; ++zi) {
        const double z = minZ + zi * cellSizeZ;
        f << "# layer z=" << zi << '\n';
        for (int yi = 0; yi < sizeY; ++yi) {
            const double y = minY + yi * cellSizeXY;
            for (int xi = 0; xi < sizeX; ++xi) {
                const double x = minX + xi * cellSizeXY;
                const CellKey key{
                    static_cast<float>(x),
                    static_cast<float>(y),
                    static_cast<float>(z),
                };
                const CellValue v = buildingMap_->getCell(key);
                if (xi > 0) f << ' ';
                f << static_cast<int>(v);
            }
            f << '\n';
        }
        f << '\n';
    }
}

} // namespace cpp_course
