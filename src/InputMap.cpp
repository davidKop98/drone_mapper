#include <cpp_course/InputMap.h>

#include <cmath>
#include <fstream>
#include <string>
#include <unordered_map>

namespace cpp_course {

namespace {

std::string trim(const std::string& s) {
    const auto start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return {};
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

int parseInt(const std::string& s, int defVal) {
    try { return std::stoi(s); } catch (...) { return defVal; }
}

double parseDouble(const std::string& s, double defVal) {
    try { return std::stod(s); } catch (...) { return defVal; }
}

} // namespace

bool InputMap::loadFromFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) return false;

    std::unordered_map<std::string, std::string> header;
    std::vector<std::string> rows; // all grid rows in order, layer by layer

    std::string line;
    while (std::getline(file, line)) {
        const std::string t = trim(line); // t= curr line
        if (t.empty() || t[0] == '#') continue;

        const auto colon = t.find(':');
        if (colon != std::string::npos) {
            // Header key-value pair
            const auto key = trim(t.substr(0, colon));
            const auto val = trim(t.substr(colon + 1));
            if (!key.empty()) header[key] = val;
        } else {
            // Grid data row: must consist only of '0' and '1'
            bool isGridRow = !t.empty();
            for (char c : t) {
                if (c != '0' && c != '1') { isGridRow = false; break; }
            }
            if (isGridRow) rows.push_back(t);
        }
    }

    // Parse header fields
    resolution_ = parseInt(header["resolution"], 0);

    const int sx = parseInt(header["size_x"], 0);
    const int sy = parseInt(header["size_y"], 0);
    const int sz = parseInt(header["size_z"], 0);
    if (sx <= 0 || sy <= 0 || sz <= 0) return false;

    sizeX_ = sx; sizeY_ = sy; sizeZ_ = sz;

    originX_ = parseDouble(header["origin_x"], 0.0) * x_extent[cm];
    originY_ = parseDouble(header["origin_y"], 0.0) * y_extent[cm];
    originZ_ = parseDouble(header["origin_z"], 0.0) * z_extent[cm];

    // Allocate grid, default all cells to 0
    grid_.assign(sizeZ_, std::vector<std::vector<int>>(
                              sizeY_, std::vector<int>(sizeX_, 0)));

    // Fill from rows. Rows arrive in layer-major, then row-major order:
    // rows[0..sizeY-1] = layer 0, rows[sizeY..2*sizeY-1] = layer 1, etc.
    const int totalRows = sizeZ_ * sizeY_;
    for (int i = 0; i < totalRows && i < static_cast<int>(rows.size()); ++i) {
        const int layer = i / sizeY_;
        const int row   = i % sizeY_;
        const auto& rowStr = rows[static_cast<std::size_t>(i)];

        for (int col = 0; col < sizeX_; ++col) {
            if (col < static_cast<int>(rowStr.size())) {
                grid_[layer][row][col] = (rowStr[static_cast<std::size_t>(col)] == '1') ? 1 : 0;
            }
        }
    }

    return true;
}

//given a world x,y,z position, and reference to 3 ints, return the grid coordinates of that position
//in the 3 ints, and return true if the position is within the grid, false otherwise.
bool InputMap::worldToGrid(const Position3D& pos,
                            int& cx, int& cy, int& cz) const {
    const double cellSize = std::pow(10.0, -static_cast<double>(resolution_));

    const double ox = originX_.force_numerical_value_in(cm);
    const double oy = originY_.force_numerical_value_in(cm);
    const double oz = originZ_.force_numerical_value_in(cm);

    const double px = pos.x.force_numerical_value_in(cm);
    const double py = pos.y.force_numerical_value_in(cm);
    const double pz = pos.z.force_numerical_value_in(cm);

    cx = static_cast<int>(std::floor((px - ox) / cellSize));
    cy = static_cast<int>(std::floor((py - oy) / cellSize));
    cz = static_cast<int>(std::floor((pz - oz) / cellSize));

    return cx >= 0 && cx < sizeX_ &&
           cy >= 0 && cy < sizeY_ &&
           cz >= 0 && cz < sizeZ_;
}

int InputMap::get(const Position3D& pos) const {
    int cx{}, cy{}, cz{};
    if (!worldToGrid(pos, cx, cy, cz)) return 0;
    return grid_[cz][cy][cx];
}

} // namespace cpp_course
