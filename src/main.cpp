#include <iostream>
#include <string>

#include <cpp_course/BuildingMap.h>
#include <cpp_course/InputMap.h>
#include <cpp_course/config_parser.h>
#include <cpp_course/configs.h>

static const char* cellValueName(cpp_course::CellValue v) {
    switch (v) {
        case cpp_course::CellValue::Empty:       return "Empty";
        case cpp_course::CellValue::Occupied:    return "Occupied";
        case cpp_course::CellValue::Unmapped:    return "Unmapped";
        case cpp_course::CellValue::OutOfBounds: return "OutOfBounds";
    }
    return "?";
}

int main(int argc, char* argv[]) {
    const std::string path = (argc > 1) ? argv[1] : ".";
    const std::string sep  = (path.back() == '/') ? "" : "/";

    std::vector<std::string> errors;
    cpp_course::DroneConfig drone{};
    cpp_course::MissionConfig mission{};

    if (!cpp_course::parseDroneConfig(path + sep + "drone_config.txt", drone, errors) ||
        !cpp_course::parseMissionConfig(path + sep + "mission_config.txt", mission, errors)) {
        std::cout << "Fatal: cannot open config files\n";
        return 1;
    }

    // -----------------------------------------------------------------------
    // Layer 5: InputMap test
    // -----------------------------------------------------------------------
    using namespace cpp_course;

    // 1. Load map_input.txt
    InputMap imap;
    if (!imap.loadFromFile(path + sep + "map_input.txt")) {
        std::cout << "Fatal: cannot load map_input.txt\n";
        return 1;
    }

    std::cout << "=== InputMap Layer 5 test ===\n";
    std::cout << "Map: 5x5x3, resolution=0 (cell=1cm), floor/ceiling solid, walls on z=1\n\n";

    // Helper lambda for cleaner output
    const auto query = [&](double x, double y, double z, int expected, const char* label) {
        const Position3D pos{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
        const int got = imap.get(pos);
        std::cout << "  " << label
                  << " get(" << x << "," << y << "," << z << ") = " << got
                  << (got == expected ? "  OK" : "  FAIL")
                  << "  (expected " << expected << ")\n";
    };

    // 2. Occupied positions (walls / floor / ceiling)
    query(0.0, 0.0, 0.0, 1, "floor  corner      "); // z=0 layer, row=0, col=0 → 1
    query(4.0, 4.0, 0.0, 1, "floor  far corner  "); // z=0 layer, row=4, col=4 → 1
    query(0.0, 0.0, 1.0, 1, "wall   top-left    "); // z=1 layer, row=0, col=0 → 1
    query(4.0, 0.0, 1.0, 1, "wall   top-right   "); // z=1 layer, row=0, col=4 → 1
    query(0.0, 0.0, 2.0, 1, "ceil   corner      "); // z=2 layer, row=0, col=0 → 1

    // 3. Empty positions (interior at z=1)
    query(1.0, 1.0, 1.0, 0, "interior (1,1,1)   "); // z=1, row=1, col=1 → 0
    query(2.0, 2.0, 1.0, 0, "interior (2,2,1)   "); // z=1, row=2, col=2 → 0
    query(3.0, 3.0, 1.0, 0, "interior (3,3,1)   "); // z=1, row=3, col=3 → 0

    // 4. Out-of-bounds queries must return 0
    query(5.0, 0.0, 0.0, 0, "oob    x=5          "); // cx=5 >= sizeX=5
    query(0.0, 5.0, 0.0, 0, "oob    y=5          "); // cy=5 >= sizeY=5
    query(0.0, 0.0, 3.0, 0, "oob    z=3          "); // cz=3 >= sizeZ=3
    query(-1.0, 0.0, 0.0, 0, "oob   x=-1          "); // negative x

    // 5. Sub-cell query: position 0.7 still falls in cell 0 (floor(0.7/1)=0)
    query(0.7, 0.7, 1.0, 1, "sub-cell wall (0.7)"); // floor → col=0, row=0 → wall

    return 0;
}
