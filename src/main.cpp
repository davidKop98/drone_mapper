#include <iomanip>
#include <iostream>
#include <string>

#include <cpp_course/DroneMath.h>
#include <cpp_course/config_parser.h>
#include <cpp_course/configs.h>

int main(int argc, char* argv[]) {
    const std::string path = (argc > 1) ? argv[1] : ".";
    const std::string sep  = (path.back() == '/') ? "" : "/";

    std::vector<std::string> errors;

    // --- Load configs (Layer 2) ---
    cpp_course::DroneConfig drone{};
    if (!cpp_course::parseDroneConfig(path + sep + "drone_config.txt", drone, errors)) {
        std::cout << "Fatal: cannot open drone_config.txt\n";
        return 1;
    }
    cpp_course::MissionConfig mission{};
    if (!cpp_course::parseMissionConfig(path + sep + "mission_config.txt", mission, errors)) {
        std::cout << "Fatal: cannot open mission_config.txt\n";
        return 1;
    }

    if (!errors.empty()) {
        std::cout << "Input errors (recovered with defaults):\n";
        for (const auto& e : errors) std::cout << "  " << e << "\n";
    }

    // -----------------------------------------------------------------------
    // Layer 3 tests
    // -----------------------------------------------------------------------

    namespace DM = cpp_course::DroneMath;
    using namespace cpp_course;

    const LidarConfig lidar{
        .beam_length_min = 20.0 * cm,
        .beam_length_max = 120.0 * cm,
        .circle_spacing  = 2.5 * cm,
        .fov_circles     = 3,
    };

    // --- computeBeamDirections: orientation {0 deg, 0 deg} ---
    std::cout << "\n=== computeBeamDirections (h=0, alt=0, fov_circles=3) ===\n";
    const Orientation scanDir{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    const auto beams = DM::computeBeamDirections(scanDir, lidar);
    std::cout << "  Total beams: " << beams.size() << "  (expected 21)\n";
    std::cout << std::fixed << std::setprecision(3);
    for (std::size_t i = 0; i < beams.size(); ++i) {
        std::cout << "  [" << i << "] h=" << beams[i].horizontal
                  << "  alt=" << beams[i].altitude << "\n";
    }

    // --- snapValue ---
    std::cout << "\n=== snapValue ===\n";
    std::cout << "  snapValue(2.891, 1) = " << DM::snapValue(2.891f, 1) << "  (expected 2.8)\n";
    std::cout << "  snapValue(-1.35, 1) = " << DM::snapValue(-1.35f, 1) << "  (expected -1.3)\n";
    std::cout << "  snapValue(5.0,   0) = " << DM::snapValue(5.0f,   0) << "  (expected 5.0)\n";
    std::cout << "  snapValue(-5.9,  0) = " << DM::snapValue(-5.9f,  0) << "  (expected -5.0)\n";

    // --- snapToGrid ---
    std::cout << "\n=== snapToGrid (xyRes=1, zRes=0) ===\n";
    const auto k1 = DM::snapToGrid(
        Position3D{1.37 * x_extent[cm], 2.891 * y_extent[cm], 5.6 * z_extent[cm]}, 1, 0);
    std::cout << "  (1.37, 2.891, 5.6) -> (" << k1.x << ", " << k1.y << ", " << k1.z
              << ")  (expected 1.3, 2.8, 5.0)\n";

    const auto k2 = DM::snapToGrid(
        Position3D{-1.35 * x_extent[cm], -2.01 * y_extent[cm], -0.9 * z_extent[cm]}, 1, 0);
    std::cout << "  (-1.35, -2.01, -0.9) -> (" << k2.x << ", " << k2.y << ", " << k2.z
              << ")  (expected -1.3, -2.0, 0.0)\n";

    // --- computeStepAngle ---
    const auto step = DM::computeStepAngle(lidar);
    std::cout << "\n=== computeStepAngle ===\n";
    std::cout << "  step = " << step << "\n";

    // --- computeMoveStep ---
    const auto move = DM::computeMoveStep(drone, mission);
    std::cout << "\n=== computeMoveStep ===\n";
    std::cout << "  move step = " << move << "\n";

    return 0;
}
