#include <iostream>

#include <cpp_course/configs.h>
#include <cpp_course/types.h>

int main() {
    // Layer 1 smoke-test: construct one of each Layer 1 type and verify it compiles.
    cpp_course::DroneConfig drone{};
    cpp_course::MissionConfig mission{};
    cpp_course::Command cmd{};
    cpp_course::DroneState state{};

    (void)drone; (void)mission; (void)cmd; (void)state;

    std::cout << "Layer 1 OK\n";
    return 0;
}
