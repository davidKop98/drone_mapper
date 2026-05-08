#pragma once

#include <cpp_course/configs.h>
#include <string>
#include <vector>

namespace cpp_course {

// Parse drone_config.txt at filePath into out.
// Returns false if the file cannot be opened (unrecoverable error).
// Bad or missing values use sensible defaults; each such case appends a
// human-readable message to errors.
[[nodiscard]] bool parseDroneConfig(const std::string& filePath,
                                    DroneConfig&        out,
                                    std::vector<std::string>& errors);

// Parse mission_config.txt at filePath into out.
// Returns false if the file cannot be opened (unrecoverable error).
// Bad or missing values use sensible defaults; each such case appends a
// human-readable message to errors.
[[nodiscard]] bool parseMissionConfig(const std::string& filePath,
                                      MissionConfig&      out,
                                      std::vector<std::string>& errors);

} // namespace cpp_course
