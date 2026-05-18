#pragma once

#include <cpp_course/BuildingMap.h>
#include <cpp_course/Drone.h>
#include <cpp_course/ExplorationAlgorithm.h>
#include <cpp_course/InputMap.h>
#include <cpp_course/MockLidarSensor.h>
#include <cpp_course/MockMovementDriver.h>
#include <cpp_course/MockPositionSensor.h>
#include <cpp_course/configs.h>
#include <cpp_course/types.h>

#include <memory>
#include <string>
#include <vector>

namespace cpp_course {

// Layer 9 — owns every piece of the simulation and drives the main loop.
//   loadConfigs(path):  parse configs and map, construct all components.
//   run():              ask drone for commands and execute until Finished or fail.
//   computeScore():     compare BuildingMap to InputMap; return % correct.
//   writeOutput(path):  emit map_output.txt in the layered format.
class Simulator {
public:
    Simulator() = default;

    [[nodiscard]] bool  loadConfigs(const std::string& path);
    void                run();
    [[nodiscard]] float computeScore();
    void                writeOutput(const std::string& path);

private:
    DroneState    state_{};
    bool          simulationFailed_{false};

    DroneConfig   droneConfig_{};
    MissionConfig missionConfig_{};

    InputMap inputMap_{};
    std::unique_ptr<BuildingMap>          buildingMap_;
    std::unique_ptr<MockPositionSensor>   posSensor_;
    std::unique_ptr<MockMovementDriver>   driver_;
    std::unique_ptr<MockLidarSensor>      lidar_;
    std::unique_ptr<ExplorationAlgorithm> algorithm_;
    std::unique_ptr<Drone>                drone_;

    std::vector<std::string> inputErrors_;
};

} // namespace cpp_course
