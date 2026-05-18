#include <cpp_course/BuildingMap.h>
#include <cpp_course/DroneMath.h>
#include <cpp_course/Profiler.h>

namespace cpp_course {

BuildingMap::BuildingMap(const MissionConfig& mission)
    : mission_(mission) {}

bool BuildingMap::isOutOfBounds(const CellKey& key) const {
    // Extract mission boundaries as plain floats in cm for direct comparison
    // with CellKey coordinates (which are also stored in cm).
    const auto flt = [](auto q) { return static_cast<float>(q.force_numerical_value_in(cm)); };

    return key.x < flt(mission_.minX) || key.x >= flt(mission_.maxX) || //bounds are [min,max)
           key.y < flt(mission_.minY) || key.y >= flt(mission_.maxY) ||
           key.z < flt(mission_.minZ) || key.z >= flt(mission_.maxZ);
}

//returns enum value of the request key in the map
CellValue BuildingMap::getCell(const CellKey& key) const {
    ++profiler::getCell_calls;
    if (isOutOfBounds(key)) return CellValue::OutOfBounds;
    const auto it = cells_.find(key);
    return (it == cells_.end()) ? CellValue::Unmapped : it->second;
}

void BuildingMap::setCell(const CellKey& key, CellValue value) {
    ++profiler::setCell_calls;
    if (isOutOfBounds(key)) return;
    cells_[key] = value;
}
//returns all the key/value pairs currently stored in cells_
std::vector<std::pair<CellKey, CellValue>> BuildingMap::getAllSetCells() const {
    std::vector<std::pair<CellKey, CellValue>> result;
    for (const auto& [key, value] : cells_) {
        result.push_back({key, value});
    }
    return result;
}

// IMap3D — snap world position to grid; only confirmed-Empty cells are clear.
// Unmapped and OutOfBounds count as walls so the drone never moves into unknown
// or off-map space.
int BuildingMap::get(const Position3D& pos) const {
    const CellKey key = DroneMath::snapToGrid(
        pos, mission_.xyResolution, mission_.zResolution);
    return (getCell(key) == CellValue::Empty) ? 0 : 1;
}

} // namespace cpp_course
