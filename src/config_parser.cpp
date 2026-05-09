#include <cpp_course/config_parser.h>

#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace cpp_course {

namespace {

using KVMap = std::unordered_map<std::string, std::string>; //key-value map

// Trim leading and trailing whitespace from a string.
std::string trim(const std::string& s) {
    const auto start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return {};
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

// Read all key:value pairs from a file into a map.
// Returns false if the file cannot be opened.
// Lines beginning with '#' (after trimming) and blank lines are skipped.
// Lines that don't contain ':' are silently skipped.
bool readKVFile(const std::string& path, KVMap& out) {
    std::ifstream file(path);
    if (!file.is_open()) return false;

    std::string line;
    while (std::getline(file, line)) {
        const std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') continue;

        const auto colon = trimmed.find(':');
        if (colon == std::string::npos) continue;

        const std::string key = trim(trimmed.substr(0, colon));
        const std::string val = trim(trimmed.substr(colon + 1));
        if (!key.empty()) out[key] = val;
    }
    return true;
}

// Extract a double from the map. Uses defaultVal if the key is absent or unparseable.
// If can_be_negative is false, a parsed negative value is also treated as invalid.
// Appends to errors on fallback.
double getDouble(const KVMap& kv, const std::string& key, double defaultVal,
                 std::vector<std::string>& errors, bool can_be_negative = true) {
    auto it = kv.find(key); //find <key,value>
    if (it == kv.end()) {
        errors.push_back("Missing key '" + key + "'; using default " + std::to_string(defaultVal));
        return defaultVal;
    }
    try {
        std::size_t pos{};
        double val = std::stod(it->second, &pos); //string to double
        if (pos != it->second.size()) throw std::invalid_argument("trailing chars");
        if (!can_be_negative && val < 0.0) throw std::invalid_argument("negative");
        return val;
    } catch (...) {
        errors.push_back("Invalid value for '" + key + "': \"" + it->second +
                         "\"; using default " + std::to_string(defaultVal));
        return defaultVal;
    }
}

// Extract a non-negative integer from the map. Uses defaultVal if absent or unparseable.
// Appends to errors on fallback.
int getInt(const KVMap& kv, const std::string& key, int defaultVal,
           std::vector<std::string>& errors) {
    auto it = kv.find(key);
    if (it == kv.end()) {
        errors.push_back("Missing key '" + key + "'; using default " + std::to_string(defaultVal));
        return defaultVal;
    }
    try {
        std::size_t pos{};
        int val = std::stoi(it->second, &pos);
        if (pos != it->second.size()) throw std::invalid_argument("trailing chars");
        return val;
    } catch (...) {
        errors.push_back("Invalid value for '" + key + "': \"" + it->second +
                         "\"; using default " + std::to_string(defaultVal));
        return defaultVal;
    }
}

// Extract a std::size_t from the map. Uses defaultVal if absent or unparseable.
// Rejects negative string values. Appends to errors on fallback.
std::size_t getSizeT(const KVMap& kv, const std::string& key, std::size_t defaultVal,
                     std::vector<std::string>& errors) {
    auto it = kv.find(key);
    if (it == kv.end()) {
        errors.push_back("Missing key '" + key + "'; using default " +
                         std::to_string(defaultVal));
        return defaultVal;
    }
    try {
        if (!it->second.empty() && it->second[0] == '-')
            throw std::invalid_argument("negative");
        std::size_t pos{};
        unsigned long val = std::stoul(it->second, &pos);
        if (pos != it->second.size()) throw std::invalid_argument("trailing chars");
        return static_cast<std::size_t>(val);
    } catch (...) {
        errors.push_back("Invalid value for '" + key + "': \"" + it->second +
                         "\"; using default " + std::to_string(defaultVal));
        return defaultVal;
    }
}

} // namespace

bool parseDroneConfig(const std::string& filePath,
                      DroneConfig&        out,
                      std::vector<std::string>& errors) {
    KVMap kv;
    if (!readKVFile(filePath, kv)) return false;

    out.minPassWidth  = getDouble(kv, "min_pass_width",  20.0, errors, false) * cm;
    out.minPassLength = getDouble(kv, "min_pass_length", 20.0, errors, false) * cm;
    out.minPassHeight = getDouble(kv, "min_pass_height", 20.0, errors, false) * cm;

    out.maxRotate  = getDouble(kv, "max_rotate",  180.0, errors, false) * deg;
    out.maxAdvance = getDouble(kv, "max_advance", 100.0, errors) * cm;
    out.maxElevate = getDouble(kv, "max_elevate", 100.0, errors) * cm;

    out.lidar.beam_length_min = getDouble(kv, "lidar_beam_min",   20.0, errors, false) * cm;
    out.lidar.beam_length_max = getDouble(kv, "lidar_beam_max",  120.0, errors, false) * cm;
    out.lidar.circle_spacing  = getDouble(kv, "lidar_spacing",     2.5, errors, false) * cm;
    out.lidar.fov_circles     = getSizeT (kv, "lidar_fov_circles",   3, errors);

    return true;
}

bool parseMissionConfig(const std::string& filePath,
                        MissionConfig&      out,
                        std::vector<std::string>& errors) {
    KVMap kv;
    if (!readKVFile(filePath, kv)) return false;

    out.startPosition.x = getDouble(kv, "start_x",  0.0, errors) * x_extent[cm];
    out.startPosition.y = getDouble(kv, "start_y",  0.0, errors) * y_extent[cm];
    out.startPosition.z = getDouble(kv, "start_z", 50.0, errors) * z_extent[cm];
    out.startHeading    = getDouble(kv, "start_heading", 0.0, errors) * deg;

    out.minX = getDouble(kv, "min_x",   0.0, errors) * x_extent[cm];
    out.maxX = getDouble(kv, "max_x", 500.0, errors) * x_extent[cm];
    out.minY = getDouble(kv, "min_y",   0.0, errors) * y_extent[cm];
    out.maxY = getDouble(kv, "max_y", 500.0, errors) * y_extent[cm];
    out.minZ = getDouble(kv, "min_z",   0.0, errors) * z_extent[cm];
    out.maxZ = getDouble(kv, "max_z", 300.0, errors) * z_extent[cm];

    out.xyResolution = getInt(kv, "xy_resolution", 1, errors);
    out.zResolution  = getInt(kv, "z_resolution",  0, errors);

    return true;
}

} // namespace cpp_course
