#pragma once
#include <algorithm>
#include <set>

#include "bonxai/bonxai.hpp"
#include "voxeland_map/semantic_object.hpp"

namespace GeometryOperations
{
    template <typename T>
    std::set<T> SetUnion(const std::set<T>& a, const std::set<T>& b)
    {
        std::set<T> union_;
        std::set_union(a.begin(), a.end(), b.begin(), b.end(), std::inserter(union_, union_.begin()));
        return union_;
    }

    template <typename T>
    std::set<T> SetIntersection(const std::set<T>& a, const std::set<T>& b)
    {
        std::set<T> intersection;
        std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), std::inserter(intersection, intersection.begin()));
        return intersection;
    }

    inline std::set<Bonxai::CoordT> DownsampleVoxels(const std::set<Bonxai::CoordT>& voxels, uint coarsening_factor)
    {
        if (coarsening_factor <= 1)
            return voxels;
        std::set<Bonxai::CoordT> voxels_coarse;

        for (const auto& coord : voxels)
            voxels_coarse.insert(coord / coarsening_factor);

        return voxels_coarse;
    }

    inline void updateBBoxBounds(BoundingBox3D& original, const BoundingBox3D& update)
    {
        // Update min bounds
        original.minX = std::min(update.minX, original.minX);
        original.minY = std::min(update.minY, original.minY);
        original.minZ = std::min(update.minZ, original.minZ);

        // Update max bounds
        original.maxX = std::max(update.maxX, original.maxX);
        original.maxY = std::max(update.maxY, original.maxY);
        original.maxZ = std::max(update.maxZ, original.maxZ);
    }

    inline bool checkBBoxIntersect(const BoundingBox3D& bbox1, const BoundingBox3D& bbox2)
    {
        // Check for no overlap along x-axis
        if (bbox1.maxX < bbox2.minX || bbox2.maxX < bbox1.minX)
            return false;

        // Check for no overlap along y-axis
        if (bbox1.maxY < bbox2.minY || bbox2.maxY < bbox1.minY)
            return false;

        // Check for no overlap along z-axis
        if (bbox1.maxZ < bbox2.minZ || bbox2.maxZ < bbox1.minZ)
            return false;

        // If there is overlap along all axes, the boxes intersect
        return true;
    }

}  // namespace GeometryOperations