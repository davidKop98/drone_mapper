#pragma once

#include <cpp_course/IMap3D.h>
#include <cpp_course/configs.h>
#include <cpp_course/types.h>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cpp_course {

// The drone's sparse output map.
// Stores what the drone has learned; never visible to MockLidarSensor.
// All coordinates are stored as CellKeys (snapped to mission resolution).
// Also implements IMap3D so DroneMath collision checks can use it directly:
// get() returns 1 only for Occupied cells (Empty/Unmapped/OutOfBounds → 0).
class BuildingMap : public IMap3D {
private:                        //cell value is enum 1,0,-1,-2
    std::unordered_map<CellKey, CellValue, CellKeyHash> cells_; //CellKeyHash is hash func for Cellkey
    const MissionConfig& mission_;
public:
    explicit BuildingMap(const MissionConfig& mission);

    // Returns OutOfBounds if key is outside mission boundaries,
    // Unmapped if the cell was never set, otherwise the stored value.
    [[nodiscard]] CellValue getCell(const CellKey& key) const;

    // Set a cell value. Silently ignores keys outside mission boundaries.
    void setCell(const CellKey& key, CellValue value);

    // True if key falls outside mission boundaries.
    [[nodiscard]] bool isOutOfBounds(const CellKey& key) const;

    // All explicitly set cells — used for scoring and map output.
    [[nodiscard]] std::vector<std::pair<CellKey, CellValue>> getAllSetCells() const;

    // IMap3D implementation: snap pos to grid; return 1 iff that cell is Occupied.
    [[nodiscard]] int get(const Position3D& pos) const override;
};

} // namespace cpp_course
