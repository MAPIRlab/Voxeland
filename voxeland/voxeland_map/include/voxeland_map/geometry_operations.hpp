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

    std::set<Bonxai::IndicesT> DownsampleVoxels(const std::set<Bonxai::IndicesT>& voxels, uint coarsening_factor);

    BoundingBox3D FindBBox(const std::set<Bonxai::IndicesT>& voxels);

    void UpdateBBoxBounds(BoundingBox3D& original, const Bonxai::Point3D& update);
    void UpdateBBoxBounds(BoundingBox3D& original, const BoundingBox3D& update);

    bool CheckBBoxIntersect(const BoundingBox3D& bbox1, const BoundingBox3D& bbox2);

    std::vector<std::set<Bonxai::IndicesT>> ClusterVoxelCloud(const std::set<Bonxai::IndicesT>& voxels_in);

}  // namespace GeometryOperations