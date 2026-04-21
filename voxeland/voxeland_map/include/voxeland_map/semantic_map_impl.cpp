#pragma once
#include <voxeland_map/debugging_utils.hpp>

#include "semantic_map.hpp"  // this is fine! it only affects the IDE! the pragmas will save us
#include "voxeland_map/bonxai_query.hpp"

template <typename DataT>
inline std::set<InstanceID_t> SemanticMap::getCurrentVisibleInstances(double minOccupancyZ, double maxOccupancyZ)
{
    std::vector<DataT> cell_data;
    std::vector<Bonxai::Point3D> cell_points;
    Bonxai::ProbabilisticMapT<DataT>* bonxai = BonxaiQuery<DataT>::getBonxaiT();
    bonxai->getOccupiedVoxels(cell_points, cell_data);

    std::set<InstanceID_t> visibleInstances;

    for (size_t i = 0; i < cell_points.size(); i++)
    {
        const auto& voxel = cell_points[i];
        const auto& data = cell_data[i];

        if (voxel.z >= minOccupancyZ && voxel.z <= maxOccupancyZ)
        {
            auto itInstances = std::max_element(data.instances_votes.begin(), data.instances_votes.end());
            auto idxMaxVotes = std::distance(data.instances_votes.begin(), itInstances);
            InstanceID_t bestInstanceID = data.instances_candidates[idxMaxVotes];
            if (globalSemanticMap[bestInstanceID].isStillValid())
            {
                visibleInstances.insert(bestInstanceID);
            }
            else
            {
                visibleInstances.insert(globalSemanticMap[bestInstanceID].pointsTo);
            }
        }
    }
    return visibleInstances;
}

template <typename DataT, typename PointCloudTypeT>
inline void SemanticMap::addInstancesGeometryToLocalSemanticMap(std::vector<SemanticObject>& localMap, const PointCloudTypeT& pc)
{
    Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>* bonxai = BonxaiQuery<DataT>::getBonxaiT()->grid();

    for (size_t i = 0; i < pc.points.size(); i++)
    {
        InstanceID_t instanceID = pc.points[i].instance_id;

        if (!localMap[instanceID].localGeometry.has_value())
            localMap[instanceID].localGeometry.emplace();

        localMap[instanceID].localGeometry.value().insert(
            bonxai->posToIndex(Bonxai::Point3D(pc.points[i].x, pc.points[i].y, pc.points[i].z)));

        // Update min bounds
        localMap[instanceID].bbox.minX =
            std::min(pc.points[i].x, localMap[instanceID].bbox.minX);
        localMap[instanceID].bbox.minY =
            std::min(pc.points[i].y, localMap[instanceID].bbox.minY);
        localMap[instanceID].bbox.minZ =
            std::min(pc.points[i].z, localMap[instanceID].bbox.minZ);

        // Update max bounds
        localMap[instanceID].bbox.maxX =
            std::max(pc.points[i].x, localMap[instanceID].bbox.maxX);
        localMap[instanceID].bbox.maxY =
            std::max(pc.points[i].y, localMap[instanceID].bbox.maxY);
        localMap[instanceID].bbox.maxZ =
            std::max(pc.points[i].z, localMap[instanceID].bbox.maxZ);
    }
}

template <typename DataT>
inline std::set<Bonxai::IndicesT> SemanticMap::listOfVoxelsInObject(const SemanticObject& object, std::optional<double> probabilityThr)
{
    std::set<Bonxai::IndicesT> cellsInside;

    Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>* bonxai = BonxaiQuery<DataT>::getBonxaiT()->grid();

    const Bonxai::IndicesT coordMin = bonxai->posToIndex(Bonxai::Point3D(
        object.bbox.minX - bonxai->resolution, object.bbox.minY - bonxai->resolution, object.bbox.minZ - bonxai->resolution));
    const Bonxai::IndicesT coordMax = bonxai->posToIndex(Bonxai::Point3D(
        object.bbox.maxX + bonxai->resolution, object.bbox.maxY + bonxai->resolution, object.bbox.maxZ + bonxai->resolution));

// Iterate over all points inside the bounding box
#pragma omp parallel for collapse(3)
    for (int x = coordMin.x; x <= coordMax.x; x++)
    {
        for (int y = coordMin.y; y <= coordMax.y; y++)
        {
            for (int z = coordMin.z; z <= coordMax.z; z++)
            {
                Bonxai::IndicesT coord = Bonxai::IndicesT{ x, y, z };
                Bonxai::ProbabilisticCell<DataT>* cell = BonxaiQuery<DataT>::getAccessor().value(coord);
                if (!cell)
                    continue;

                if (cell->data.getMostRepresentativeInstance() == object.instanceID  //
                    || (probabilityThr.has_value() && cell->data.GetProbabilityOfInstance(object.instanceID) >= probabilityThr))
                {
#pragma omp critical
                    cellsInside.insert(coord);
                }
            }
        }
    }

    return cellsInside;
}
