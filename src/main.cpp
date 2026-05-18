#include <iostream>
#include <string>

#include <cpp_course/Simulator.h>

int main(int argc, char* argv[]) {
    const std::string path = (argc > 1) ? argv[1] : ".";

    cpp_course::Simulator sim;
    if (!sim.loadConfigs(path)) {
        std::cout << "Fatal error loading configs\n";
        return 1;
    }
    sim.run();
    sim.writeOutput(path);
    std::cout << "Score: " << sim.computeScore() << "/100\n";
    return 0;
}
