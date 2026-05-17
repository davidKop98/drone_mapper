#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#include <cpp_course/BuildingMap.h>
#include <cpp_course/ExplorationAlgorithm.h>
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

    std::cout << "\n=== Boxed-in tests (1-layer empty cube, walls beyond) ===\n";
    // Map: 10x10x5 with walls everywhere EXCEPT a 3x3x3 empty cube at cells
    // (4..6, 4..6, 1..3). Drone at the cube center (5.5, 5.5, 2.5) heading east.
    // testDrone halfW=halfL=halfH=0.5 → body x∈[5,6], y∈[5,6], z∈[2,3] at start
    // (all inside the empty cube). Body front face reaches wall at distance 1.
    auto makeBoxedMap = []() {
        std::vector<std::tuple<int,int,int>> walls;
        for (int z = 0; z < 5; ++z)
            for (int y = 0; y < 10; ++y)
                for (int x = 0; x < 10; ++x)
                    if (!(x >= 4 && x <= 6 && y >= 4 && y <= 6 && z >= 1 && z <= 3))
                        walls.emplace_back(x, y, z);
        return walls;
    };

    // BOX1: advance 0.5 cm east — body front at x=6.5, still inside cube → success.
    {
        writeTestMap(mapPath, 10, 10, 5, makeBoxedMap());
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5.5, 5.5, 2.5, 0);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.advance(0.5 * cm);
        check("BOX1 advance 0.5cm east within cube",
              ok && !simF && dEq(getX(s), 6.0) && dEq(getY(s), 5.5));
    }

    // BOX2: advance 1.0 cm east — body front at x=7.0, hits wall at cell (7,5,2).
    {
        writeTestMap(mapPath, 10, 10, 5, makeBoxedMap());
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5.5, 5.5, 2.5, 0);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.advance(1.0 * cm);
        check("BOX2 advance 1.0cm east hits wall",
              !ok && simF && dEq(getX(s), 5.5));
    }

    // BOX3: elevate 0.5 cm up — body top at z=3.5, still within cube → success.
    {
        writeTestMap(mapPath, 10, 10, 5, makeBoxedMap());
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5.5, 5.5, 2.5, 0);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.elevate(0.5 * cm);
        check("BOX3 elevate 0.5cm up within cube",
              ok && !simF && dEq(getZ(s), 3.0));
    }

    // BOX4: elevate 1.0 cm up — body top at z=4.0, hits ceiling at cell z=4.
    {
        writeTestMap(mapPath, 10, 10, 5, makeBoxedMap());
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5.5, 5.5, 2.5, 0);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.elevate(1.0 * cm);
        check("BOX4 elevate 1.0cm up hits ceiling",
              !ok && simF && dEq(getZ(s), 2.5));
    }

    // BOX5: elevate -1.5 cm down — body bottom at z=0.5, hits floor at cell z=0.
    {
        writeTestMap(mapPath, 10, 10, 5, makeBoxedMap());
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5.5, 5.5, 2.5, 0);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.elevate(-1.5 * cm);
        check("BOX5 elevate -1.5cm down hits floor",
              !ok && simF && dEq(getZ(s), 2.5));
    }

    // BOX6: advance 1.5cm at 45° NE — body sweeps past cube edge into corner wall.
    {
        writeTestMap(mapPath, 10, 10, 5, makeBoxedMap());
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5.5, 5.5, 2.5, 45);
        auto dr = makeDrone(1, 1, 1);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.advance(1.5 * cm);
        check("BOX6 advance 1.5cm NE hits corner wall",
              !ok && simF && dEq(getX(s), 5.5) && dEq(getY(s), 5.5));
    }

    std::cout << "\n=== Cross-corridor tests (drone 2x2x3, cardinals pass, diagonals fail) ===\n";
    // Map: 12x12x7 with a '+'-shaped corridor of empty cells.
    // A cell is empty if cx ∈ {4,5,6} OR cy ∈ {4,5,6} (all z layers identical).
    // The 4 corner quadrants are filled with walls.
    // Drone at (5.5, 5.5, 3.5) with body 2x2x3 (halfW=1, halfL=1, halfH=1.5).
    // Body at rest: x∈[4.5,6.5], y∈[4.5,6.5], z∈[2,5] → all inside the corridor.
    auto makeCrossMap = []() {
        std::vector<std::tuple<int,int,int>> walls;
        for (int z = 0; z < 7; ++z)
            for (int y = 0; y < 12; ++y)
                for (int x = 0; x < 12; ++x) {
                    const bool inCorridor =
                        (x >= 4 && x <= 6) || (y >= 4 && y <= 6);
                    if (!inCorridor) walls.emplace_back(x, y, z);
                }
        return walls;
    };

    // 4 cardinal directions — each should succeed.
    struct DirCase {
        const char* name;
        double headingDeg;
        double endX, endY;
    };
    const DirCase cardinals[] = {
        {"east",  0.0,   7.5, 5.5},
        {"north", 90.0,  5.5, 7.5},
        {"west",  180.0, 3.5, 5.5},
        {"south", 270.0, 5.5, 3.5},
    };
    for (const auto& c : cardinals) {
        writeTestMap(mapPath, 12, 12, 7, makeCrossMap());
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5.5, 5.5, 3.5, c.headingDeg);
        auto dr = makeDrone(2, 2, 3);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.advance(2.0 * cm);
        const std::string name = std::string("CROSS cardinal ") + c.name + " 2cm";
        check(name.c_str(),
              ok && !simF
              && dEq(getX(s), c.endX) && dEq(getY(s), c.endY) && dEq(getZ(s), 3.5));
    }

    // 4 diagonal directions — each should fail.
    const DirCase diagonals[] = {
        {"NE", 45.0,  5.5, 5.5},
        {"NW", 135.0, 5.5, 5.5},
        {"SW", 225.0, 5.5, 5.5},
        {"SE", 315.0, 5.5, 5.5},
    };
    for (const auto& d : diagonals) {
        writeTestMap(mapPath, 12, 12, 7, makeCrossMap());
        InputMap m; m.loadFromFile(mapPath);
        auto s = makeState(5.5, 5.5, 3.5, d.headingDeg);
        auto dr = makeDrone(2, 2, 3);
        bool simF = false;
        MockMovementDriver drv(s, dr, m, simF);
        const bool ok = drv.advance(2.0 * cm);
        const std::string name = std::string("CROSS diagonal ") + d.name + " 2cm";
        check(name.c_str(),
              !ok && simF
              && dEq(getX(s), d.endX) && dEq(getY(s), d.endY) && dEq(getZ(s), 3.5));
    }

    std::cout << "\n=== ExplorationAlgorithm smoke test ===\n";
    // Empty BuildingMap, drone at the middle of a small mission. After the
    // sphere scan no Empty cells exist (none have been mapped yet), so the
    // algorithm should immediately reach Finished.
    {
        MissionConfig mc{};
        mc.startPosition = Position3D{
            5.0 * x_extent[cm], 5.0 * y_extent[cm], 2.0 * z_extent[cm]};
        mc.startHeading  = 0.0 * horizontal_angle[deg];
        mc.minX = 0.0  * x_extent[cm]; mc.maxX = 10.0 * x_extent[cm];
        mc.minY = 0.0  * y_extent[cm]; mc.maxY = 10.0 * y_extent[cm];
        mc.minZ = 0.0  * z_extent[cm]; mc.maxZ = 5.0  * z_extent[cm];
        mc.xyResolution = 0;
        mc.zResolution  = 0;

        DroneConfig dc{};
        dc.minPassWidth       = 1.0 * cm;
        dc.minPassLength      = 1.0 * cm;
        dc.minPassHeight      = 1.0 * cm;
        dc.maxRotate          = 180.0 * horizontal_angle[deg];
        dc.maxAdvance         = 100.0 * cm;
        dc.maxElevate         = 100.0 * cm;
        dc.lidar.beam_length_min = 20.0 * cm;
        dc.lidar.beam_length_max = 120.0 * cm;
        dc.lidar.circle_spacing  = 2.5  * cm;
        dc.lidar.fov_circles     = 3;

        BuildingMap bm(mc);
        ExplorationAlgorithm algo(bm, dc, mc);

        const Position3D pos = mc.startPosition;
        const HorizontalAngle hdg = mc.startHeading;

        bool firstIsScan = false;
        int  scans       = 0;
        int  finishedAt  = -1;
        constexpr int kMaxIters = 20000;

        for (int i = 0; i < kMaxIters; ++i) {
            const Command c = algo.decide(pos, hdg);
            if (i == 0) firstIsScan = (c.type == CommandType::Scan);
            if (c.type == CommandType::Scan)     ++scans;
            if (c.type == CommandType::Finished) { finishedAt = i; break; }
        }

        check("SMOKE first command is Scan", firstIsScan);
        check("SMOKE many Scan commands issued", scans > 100);
        check("SMOKE reaches Finished within budget", finishedAt > 0);

        // Subsequent calls after Finished must keep returning Finished.
        const Command after = algo.decide(pos, hdg);
        check("SMOKE stays Finished after completion",
              after.type == CommandType::Finished);
    }

    std::cout << "\n=== Chunking tests ===\n";
    // Mini-simulator that feeds the algorithm and updates state after each
    // command. Captures non-Scan/non-Finished moves until captureN is reached
    // or maxIters elapses (typical sphere scan is ~5300 calls so budget high).
    struct AlgoTrace {
        std::vector<Command> moves;
        int  scans     = 0;
        bool finished  = false;
    };

    auto runAlgo = [](ExplorationAlgorithm& algo, DroneState state,
                      int captureN, int maxIters) -> AlgoTrace {
        constexpr double PI_LOCAL = 3.14159265358979323846;
        AlgoTrace trace;
        for (int i = 0; i < maxIters; ++i) {
            const Command c = algo.decide(state.position, state.heading);
            switch (c.type) {
                case CommandType::Scan:
                    ++trace.scans;
                    break;
                case CommandType::Finished:
                    trace.finished = true;
                    return trace;
                case CommandType::Rotate: {
                    double newDeg = state.heading.force_numerical_value_in(deg)
                                  + c.angleValue.force_numerical_value_in(deg);
                    newDeg = std::fmod(newDeg, 360.0);
                    if (newDeg < 0.0) newDeg += 360.0;
                    state.heading = newDeg * horizontal_angle[deg];
                    trace.moves.push_back(c);
                    break;
                }
                case CommandType::Advance: {
                    const double hRad = state.heading.force_numerical_value_in(deg)
                                        * PI_LOCAL / 180.0;
                    const double d    = c.distanceValue.force_numerical_value_in(cm);
                    const double newX = state.position.x.force_numerical_value_in(cm)
                                        + std::cos(hRad) * d;
                    const double newY = state.position.y.force_numerical_value_in(cm)
                                        + std::sin(hRad) * d;
                    state.position.x = newX * x_extent[cm];
                    state.position.y = newY * y_extent[cm];
                    trace.moves.push_back(c);
                    break;
                }
                case CommandType::Elevate: {
                    const double d    = c.distanceValue.force_numerical_value_in(cm);
                    const double newZ = state.position.z.force_numerical_value_in(cm) + d;
                    state.position.z  = newZ * z_extent[cm];
                    trace.moves.push_back(c);
                    break;
                }
                default: break;
            }
            if (static_cast<int>(trace.moves.size()) >= captureN) return trace;
        }
        return trace;
    };

    auto baseMission = []() {
        MissionConfig mc{};
        mc.startPosition = Position3D{
            5.5 * x_extent[cm], 5.5 * y_extent[cm], 2.5 * z_extent[cm]};
        mc.startHeading = 0.0 * horizontal_angle[deg];
        mc.minX = 0.0 * x_extent[cm]; mc.maxX = 10.0 * x_extent[cm];
        mc.minY = 0.0 * y_extent[cm]; mc.maxY = 10.0 * y_extent[cm];
        mc.minZ = 0.0 * z_extent[cm]; mc.maxZ = 5.0  * z_extent[cm];
        mc.xyResolution = 0;
        mc.zResolution  = 0;
        return mc;
    };

    auto baseDrone = []() {
        DroneConfig dc{};
        dc.minPassWidth  = 1.0 * cm;
        dc.minPassLength = 1.0 * cm;
        dc.minPassHeight = 1.0 * cm;
        dc.maxRotate     = 180.0 * horizontal_angle[deg];
        dc.maxAdvance    = 100.0 * cm;
        dc.maxElevate    = 100.0 * cm;
        dc.lidar.beam_length_min = 20.0 * cm;
        dc.lidar.beam_length_max = 120.0 * cm;
        dc.lidar.circle_spacing  = 2.5  * cm;
        dc.lidar.fov_circles     = 3;
        return dc;
    };

    auto markEmpty = [](BuildingMap& bm, int xLo, int xHi, int yLo, int yHi,
                        int zLo, int zHi) {
        for (int x = xLo; x <= xHi; ++x)
            for (int y = yLo; y <= yHi; ++y)
                for (int z = zLo; z <= zHi; ++z)
                    bm.setCell(CellKey{static_cast<float>(x),
                                       static_cast<float>(y),
                                       static_cast<float>(z)},
                               CellValue::Empty);
    };

    // CHUNK1: Advance 1cm with maxAdvance=0.4 → chunks as 0.4 + 0.4 + 0.2.
    // Drone heading east, only east marked clear so no rotation needed.
    {
        MissionConfig mc = baseMission();
        DroneConfig   dc = baseDrone();
        dc.maxAdvance    = 0.4 * cm;

        BuildingMap bm(mc);
        markEmpty(bm, 5, 7, 5, 6, 2, 3); // east body sweep cells

        ExplorationAlgorithm algo(bm, dc, mc);
        DroneState state{};
        state.position = mc.startPosition;
        state.heading  = mc.startHeading;

        const AlgoTrace t = runAlgo(algo, state, 3, 20000);
        const bool ok = t.moves.size() == 3
            && t.moves[0].type == CommandType::Advance
            && dEq(t.moves[0].distanceValue.force_numerical_value_in(cm), 0.4)
            && t.moves[1].type == CommandType::Advance
            && dEq(t.moves[1].distanceValue.force_numerical_value_in(cm), 0.4)
            && t.moves[2].type == CommandType::Advance
            && dEq(t.moves[2].distanceValue.force_numerical_value_in(cm), 0.2);
        check("CHUNK1 advance 1cm chunks 0.4+0.4+0.2", ok);
    }

    // CHUNK2: Rotate -45° with maxRotate=15° → chunks as -15 + -15 + -15,
    // then single Advance(1cm). Drone heading 45°, target east, so delta=-45.
    {
        MissionConfig mc = baseMission();
        DroneConfig   dc = baseDrone();
        dc.maxRotate     = 15.0 * horizontal_angle[deg];

        BuildingMap bm(mc);
        markEmpty(bm, 5, 7, 5, 6, 2, 3); // east body sweep cells

        ExplorationAlgorithm algo(bm, dc, mc);
        DroneState state{};
        state.position = mc.startPosition;
        state.heading  = 45.0 * horizontal_angle[deg];

        const AlgoTrace t = runAlgo(algo, state, 4, 20000);
        const bool ok = t.moves.size() == 4
            && t.moves[0].type == CommandType::Rotate
            && dEq(t.moves[0].angleValue.force_numerical_value_in(deg), -15.0)
            && t.moves[1].type == CommandType::Rotate
            && dEq(t.moves[1].angleValue.force_numerical_value_in(deg), -15.0)
            && t.moves[2].type == CommandType::Rotate
            && dEq(t.moves[2].angleValue.force_numerical_value_in(deg), -15.0)
            && t.moves[3].type == CommandType::Advance
            && dEq(t.moves[3].distanceValue.force_numerical_value_in(cm), 1.0);
        check("CHUNK2 rotate -45° chunks -15+-15+-15, then advance 1cm", ok);
    }

    // CHUNK3: Elevate 1cm with maxElevate=0.4 → chunks as 0.4 + 0.4 + 0.2.
    // Mark only the up-body cells so all 8 horizontal directions fail and
    // up (index 8) is the first valid direction.
    {
        MissionConfig mc = baseMission();
        DroneConfig   dc = baseDrone();
        dc.maxElevate    = 0.4 * cm;

        BuildingMap bm(mc);
        markEmpty(bm, 5, 6, 5, 6, 2, 4); // up body sweep cells

        ExplorationAlgorithm algo(bm, dc, mc);
        DroneState state{};
        state.position = mc.startPosition;
        state.heading  = mc.startHeading;

        const AlgoTrace t = runAlgo(algo, state, 3, 20000);
        const bool ok = t.moves.size() == 3
            && t.moves[0].type == CommandType::Elevate
            && dEq(t.moves[0].distanceValue.force_numerical_value_in(cm), 0.4)
            && t.moves[1].type == CommandType::Elevate
            && dEq(t.moves[1].distanceValue.force_numerical_value_in(cm), 0.4)
            && t.moves[2].type == CommandType::Elevate
            && dEq(t.moves[2].distanceValue.force_numerical_value_in(cm), 0.2);
        check("CHUNK3 elevate 1cm chunks 0.4+0.4+0.2", ok);
    }

    std::cout << "\n=== Summary: " << passed << " passed, " << failed << " failed ===\n";
    return failed > 0 ? 1 : 0;
}
