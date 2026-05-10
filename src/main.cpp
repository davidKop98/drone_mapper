#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#include <cpp_course/InputMap.h>
#include <cpp_course/MockMovementDriver.h>
#include <cpp_course/MockPositionSensor.h>
#include <cpp_course/configs.h>

namespace {

// Write a test map: 0-filled grid with the given (x,y,z) cells set to 1.
void writeTestMap(const std::string& path, int sx, int sy, int sz,
                  const std::vector<std::tuple<int, int, int>>& walls) {
    std::ofstream f(path);
    f << "resolution: 0\n"
      << "size_x: " << sx << "\n"
      << "size_y: " << sy << "\n"
      << "size_z: " << sz << "\n"
      << "origin_x: 0\norigin_y: 0\norigin_z: 0\n";

    std::vector<std::vector<std::string>> layers(
        sz, std::vector<std::string>(sy, std::string(sx, '0')));
    for (const auto& w : walls) {
        const int x = std::get<0>(w);
        const int y = std::get<1>(w);
        const int z = std::get<2>(w);
        if (x >= 0 && x < sx && y >= 0 && y < sy && z >= 0 && z < sz)
            layers[z][y][x] = '1';
    }
    for (int z = 0; z < sz; ++z)
        for (int y = 0; y < sy; ++y)
            f << layers[z][y] << "\n";
}

bool dEq(double a, double b, double tol = 1e-6) {
    return std::abs(a - b) < tol;
}

} // namespace

int main(int argc, char* argv[]) {
    using namespace cpp_course;

    const std::string path = (argc > 1) ? argv[1] : ".";
    const std::string sep  = (path.back() == '/') ? "" : "/";
    const std::string mapPath = path + sep + "test_map.txt";

    int passed = 0, failed = 0;
    auto check = [&](const char* name, bool ok) {
        std::cout << (ok ? "PASS  " : "FAIL  ") << name << "\n";
        if (ok) ++passed; else ++failed;
    };

    auto makeDrone = [](double width, double length, double height) {
        DroneConfig d{};
        d.minPassWidth  = width  * cm;
        d.minPassLength = length * cm;
        d.minPassHeight = height * cm;
        d.maxRotate  = 180.0 * horizontal_angle[deg];
        d.maxAdvance = 100.0 * cm;
        d.maxElevate = 100.0 * cm;
        return d;
    };

    auto makeState = [](double x, double y, double z, double headingDeg) {
        DroneState s{};
        s.position = Position3D{
            x * x_extent[cm], y * y_extent[cm], z * z_extent[cm],
        };
        s.heading = headingDeg * horizontal_angle[deg];
        return s;
    };

    auto getX = [](const DroneState& s) { return s.position.x.force_numerical_value_in(cm);  };
    auto getY = [](const DroneState& s) { return s.position.y.force_numerical_value_in(cm);  };
    auto getZ = [](const DroneState& s) { return s.position.z.force_numerical_value_in(cm);  };
    auto getH = [](const DroneState& s) { return s.heading.force_numerical_value_in(deg);    };

    std::cout << "=== Advance Tests ===\n";

    // ADV1: empty space ahead → success, position updates
    {
        writeTestMap(mapPath, 10, 10, 5, {});
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5, 5, 2.5, 0);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.advance(1.0 * cm);
        check("ADV1 empty space ahead",
              ok && !simF
              && dEq(getX(s), 6.0) && dEq(getY(s), 5.0) && dEq(getZ(s), 2.5));
    }

    // ADV2: wall directly on center path → collision, position unchanged
    {
        writeTestMap(mapPath, 10, 10, 5, {{7, 5, 2}});
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5, 5, 2.5, 0);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.advance(2.0 * cm); // center hits wall at (7,5,2)
        check("ADV2 wall on center path",
              !ok && simF && dEq(getX(s), 5.0) && dEq(getY(s), 5.0));
    }

    // ADV3: wall at side WITHIN drone width → collision
    // Drone halfWidth=2.5; body extends to y=7.5 (cy=7). Wall at (6,7,2).
    {
        writeTestMap(mapPath, 10, 10, 5, {{6, 7, 2}});
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5, 5, 2.5, 0);
        auto dr = makeDrone(5, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.advance(1.0 * cm);
        check("ADV3 wall side WITHIN drone width",
              !ok && simF && dEq(getX(s), 5.0));
    }

    // ADV4: same wall, but drone halfWidth=1 (body reaches y=6, cy=6 ≠ 7)
    {
        writeTestMap(mapPath, 10, 10, 5, {{6, 7, 2}});
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5, 5, 2.5, 0);
        auto dr = makeDrone(2, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.advance(1.0 * cm);
        check("ADV4 wall side OUTSIDE drone width",
              ok && !simF && dEq(getX(s), 6.0) && dEq(getY(s), 5.0));
    }

    // ADV5: wall behind drone within halfLength → collision before moving
    // halfLength=0.5 → rear face at x=4.5 → samples cell (4,5,2).
    {
        writeTestMap(mapPath, 10, 10, 5, {{4, 5, 2}});
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5, 5, 2.5, 0);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.advance(1.0 * cm);
        check("ADV5 wall behind (within halfLength)",
              !ok && simF && dEq(getX(s), 5.0));
    }

    // ADV6: advance 0 cm in empty map → success, position unchanged
    {
        writeTestMap(mapPath, 10, 10, 5, {});
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5, 5, 2.5, 0);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.advance(0.0 * cm);
        check("ADV6 advance 0 cm",
              ok && !simF && dEq(getX(s), 5.0) && dEq(getY(s), 5.0));
    }

    std::cout << "\n=== Elevate Tests ===\n";

    // ELE1: empty space above → success, z updates
    {
        writeTestMap(mapPath, 10, 10, 5, {});
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5, 5, 2, 0);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.elevate(1.0 * cm);
        check("ELE1 empty space above",
              ok && !simF && dEq(getZ(s), 3.0));
    }

    // ELE2: wall directly above center → collision
    {
        writeTestMap(mapPath, 10, 10, 5, {{5, 5, 4}});
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5, 5, 2.5, 0);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.elevate(2.0 * cm);
        check("ELE2 wall above center",
              !ok && simF && dEq(getZ(s), 2.5));
    }

    // ELE3: wall to side WITHIN drone width while elevating
    // halfWidth=2.5 reaches y=7.5 (cy=7); wall at (5,7,2).
    {
        writeTestMap(mapPath, 10, 10, 5, {{5, 7, 2}});
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5, 5, 2.5, 0);
        auto dr = makeDrone(5, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.elevate(0.5 * cm);
        check("ELE3 side WITHIN drone width",
              !ok && simF && dEq(getZ(s), 2.5));
    }

    // ELE4: same wall, halfWidth=1 (body reaches y=6, cy=6 ≠ 7) → no collision
    {
        writeTestMap(mapPath, 10, 10, 5, {{5, 7, 2}});
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5, 5, 2.5, 0);
        auto dr = makeDrone(2, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.elevate(0.5 * cm);
        check("ELE4 side OUTSIDE drone width",
              ok && !simF && dEq(getZ(s), 3.0));
    }

    // ELE5: elevate downward in empty map → z decreases
    {
        writeTestMap(mapPath, 10, 10, 5, {});
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5, 5, 3, 0);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.elevate(-1.0 * cm);
        check("ELE5 elevate downward",
              ok && !simF && dEq(getZ(s), 2.0));
    }

    // ELE6: wall below while elevating downward → collision
    // From z=2.5 with halfHeight=0.5 elevating down 0.5cm, sweep reaches z=1.5 → cz=1.
    {
        writeTestMap(mapPath, 10, 10, 5, {{5, 5, 1}});
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5, 5, 2.5, 0);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.elevate(-0.5 * cm);
        check("ELE6 wall below (downward elevate)",
              !ok && simF && dEq(getZ(s), 2.5));
    }

    std::cout << "\n=== Rotate Tests ===\n";

    // ROT1: 0° + 45° → 45°
    {
        writeTestMap(mapPath, 10, 10, 5, {});
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5, 5, 2.5, 0);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.rotate(45.0 * horizontal_angle[deg]);
        check("ROT1 rotate 45°", ok && dEq(getH(s), 45.0));
    }

    // ROT2: 0° + 370° → wraps to 10°
    {
        writeTestMap(mapPath, 10, 10, 5, {});
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5, 5, 2.5, 0);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.rotate(370.0 * horizontal_angle[deg]);
        check("ROT2 rotate 370° → 10°", ok && dEq(getH(s), 10.0));
    }

    // ROT3: 0° + (-90°) → wraps to 270°
    {
        writeTestMap(mapPath, 10, 10, 5, {});
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5, 5, 2.5, 0);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.rotate(-90.0 * horizontal_angle[deg]);
        check("ROT3 rotate -90° → 270°", ok && dEq(getH(s), 270.0));
    }

    std::cout << "\n=== Summary: " << passed << " passed, " << failed << " failed ===\n";
    return failed > 0 ? 1 : 0;
}
