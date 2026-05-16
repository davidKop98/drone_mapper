#pragma once

#include <cpp_course/Units.h>

namespace cpp_course {

// Read-only 3D occupancy map interface used by LiDAR implementations.
class IMap3D {
public:
    virtual ~IMap3D() = default;

    // Returns the voxel value at a physical position. Implementations should
    // return 0 for empty. 1 for wall.
    [[nodiscard]] virtual int get(const Position3D& pos) const = 0;
};

} // namespace cpp_course
