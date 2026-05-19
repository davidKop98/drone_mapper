cmake_minimum_required(VERSION 3.20)

project(drone_mapper LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(mp-units CONFIG REQUIRED)
add_executable(drone_mapper
    src/main.cpp
    src/MockLidarSensor.cpp
    src/config_parser.cpp
    src/DroneMath.cpp
    src/BuildingMap.cpp
    src/InputMap.cpp
    src/MockMovementDriver.cpp
    src/ExplorationAlgorithm.cpp
    src/Drone.cpp
    src/Simulator.cpp
    src/Profiler.cpp
)

target_link_libraries(drone_mapper PRIVATE
    mp-units::core
    mp-units::systems
    mp-units::mp-units
    mp-units::mp-units-contracts
)

target_include_directories(drone_mapper PRIVATE include)