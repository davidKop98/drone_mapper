#include <iostream>
#include <string>

#include <cpp_course/config_parser.h>
#include <cpp_course/configs.h>

int main(int argc, char* argv[]) {
    const std::string path = (argc > 1) ? argv[1] : ".";
    const std::string sep  = (path.back() == '/') ? "" : "/";

    std::vector<std::string> errors;

    // --- DroneConfig ---
    cpp_course::DroneConfig drone{};
    if (!cpp_course::parseDroneConfig(path + sep + "drone_config.txt", drone, errors)) {
        std::cout << "Fatal: cannot open drone_config.txt\n";
        return 1;
    }

    std::cout << "=== DroneConfig ===\n";
    std::cout << "  min_pass_width:   " << drone.minPassWidth  << "\n";
    std::cout << "  min_pass_length:  " << drone.minPassLength << "\n";
    std::cout << "  min_pass_height:  " << drone.minPassHeight << "\n";
    std::cout << "  max_rotate:       " << drone.maxRotate     << "\n";
    std::cout << "  max_advance:      " << drone.maxAdvance    << "\n";
    std::cout << "  max_elevate:      " << drone.maxElevate    << "\n";
    std::cout << "  lidar_beam_min:   " << drone.lidar.beam_length_min << "\n";
    std::cout << "  lidar_beam_max:   " << drone.lidar.beam_length_max << "\n";
    std::cout << "  lidar_spacing:    " << drone.lidar.circle_spacing  << "\n";
    std::cout << "  lidar_fov_circles:" << drone.lidar.fov_circles     << "\n";

    // --- MissionConfig ---
    cpp_course::MissionConfig mission{};
    if (!cpp_course::parseMissionConfig(path + sep + "mission_config.txt", mission, errors)) {
        std::cout << "Fatal: cannot open mission_config.txt\n";
        return 1;
    }

    std::cout << "\n=== MissionConfig ===\n";
    std::cout << "  start_x:         " << mission.startPosition.x << "\n";
    std::cout << "  start_y:         " << mission.startPosition.y << "\n";
    std::cout << "  start_z:         " << mission.startPosition.z << "\n";
    std::cout << "  start_heading:   " << mission.startHeading    << "\n";
    std::cout << "  min_x:           " << mission.minX << "\n";
    std::cout << "  max_x:           " << mission.maxX << "\n";
    std::cout << "  min_y:           " << mission.minY << "\n";
    std::cout << "  max_y:           " << mission.maxY << "\n";
    std::cout << "  min_z:           " << mission.minZ << "\n";
    std::cout << "  max_z:           " << mission.maxZ << "\n";
    std::cout << "  xy_resolution:   " << mission.xyResolution << "\n";
    std::cout << "  z_resolution:    " << mission.zResolution  << "\n";

    // --- Recoverable errors ---
    if (!errors.empty()) {
        std::cout << "\n=== Input Errors (recovered with defaults) ===\n";
        for (const auto& e : errors) std::cout << "  " << e << "\n";
    } else {
        std::cout << "\nNo input errors.\n";
    }

    return 0;
}
