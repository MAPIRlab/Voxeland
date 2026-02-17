#include "voxeland_map/geometry_operations.hpp"

#include <dbscan/dbscan.hpp>
#include <voxeland_map/bonxai_query.hpp>

std::set<Bonxai::IndicesT> GeometryOperations::DownsampleVoxels(const std::set<Bonxai::IndicesT>& voxels, uint coarsening_factor)
{
    if (coarsening_factor <= 1)
        return voxels;
    std::set<Bonxai::IndicesT> voxels_coarse;

    for (const auto& coord : voxels)
        voxels_coarse.insert(coord / coarsening_factor);

    return voxels_coarse;
}

BoundingBox3D GeometryOperations::FindBBox(const std::set<Bonxai::IndicesT>& voxels)
{
    Bonxai::ProbabilisticMap* bonxai = VoxelandMap::g_bonxai;
    BoundingBox3D bbox;
    for (const auto& voxel : voxels)
        UpdateBBoxBounds(bbox, bonxai->indexToPos(voxel));

    return bbox;
}

void GeometryOperations::UpdateBBoxBounds(BoundingBox3D& original, const Bonxai::Point3D& update)
{
    // Update min bounds
    original.minX = std::min((float)update.x, original.minX);
    original.minY = std::min((float)update.y, original.minY);
    original.minZ = std::min((float)update.z, original.minZ);

    // Update max bounds
    original.maxX = std::max((float)update.x, original.maxX);
    original.maxY = std::max((float)update.y, original.maxY);
    original.maxZ = std::max((float)update.z, original.maxZ);
}

void GeometryOperations::UpdateBBoxBounds(BoundingBox3D& original, const BoundingBox3D& update)
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

bool GeometryOperations::CheckBBoxIntersect(const BoundingBox3D& bbox1, const BoundingBox3D& bbox2)
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

std::vector<std::set<Bonxai::IndicesT>> GeometryOperations::TrySplitInstance(const std::set<Bonxai::IndicesT>& voxels_in)
{
    std::vector<Bonxai::IndicesT> indices;
    indices.reserve(voxels_in.size());
    std::copy(voxels_in.begin(), voxels_in.end(), std::back_inserter(indices));

    std::vector<dbscan::point3> points;
    // TODO we are clustering with the voxel indices. Do we want to convert them to actual coordinates?
    for (const auto& vox : indices)
        points.push_back({ .x = (float)vox.x, .y = (float)vox.y, .z = (float)vox.z });

    constexpr float epsilon = 1.5;
    constexpr uint min_pts = 4;
    std::vector<std::vector<size_t>> clusters_idx = dbscan::dbscan(points, epsilon, min_pts);

    std::vector<std::set<Bonxai::IndicesT>> clusters_coords(clusters_idx.size());
    for (size_t i = 0; i < clusters_idx.size(); i++)
    {
        for (size_t j = 0; j < clusters_idx.at(i).size(); j++)
            clusters_coords.at(i).insert(indices.at(clusters_idx[i][j]));
    }

    return clusters_coords;
}
