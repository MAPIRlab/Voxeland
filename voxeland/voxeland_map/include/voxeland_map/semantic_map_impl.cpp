#pragma once
#include <voxeland_map/debugging_utils.hpp>

#include "semantic_map.hpp"  // this is fine! the pragmas will save us

template <typename DataT>
inline std::set<InstanceID_t> SemanticMap::getCurrentVisibleInstances(double minOccupancyZ, double maxOccupancyZ)
{
    std::vector<DataT> cell_data;
    std::vector<Bonxai::Point3D> cell_points;
    Bonxai::ProbabilisticMapT<DataT>* bonxai = BonxaiQuery<DataT>::getBonxai();
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
    Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>* bonxai = BonxaiQuery<DataT>::getBonxai()->grid();

    for (size_t i = 0; i < pc.points.size(); i++)
    {
        InstanceID_t instanceID = pc.points[i].instance_id;

        if (!localMap[instanceID].localGeometry.has_value())
            localMap[instanceID].localGeometry.emplace();

        localMap[instanceID].localGeometry.value().insert(
            bonxai->posToCoord(Bonxai::Point3D(pc.points[i].x, pc.points[i].y, pc.points[i].z)));

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
inline void SemanticMap::integrateNewSemantics(const std::vector<SemanticObject>& localMap,
                                               float sensorX,
                                               float sensorY,
                                               float sensorZ)
{
    uint8_t integrated = 0;
    uint8_t added = 0;

    lastMapLocalToGlobal.resize(localMap.size());
    if (globalSemanticMap.empty())
    {
        SemanticObject unknown = SemanticObject(0, localMap[0].bbox);
        unknown.alphaParamsCategories = { { CategoryManager::UNKNOWN_CATEGORY, 1 } };
        updateAppearancesTimestamps(unknown, localMap[0]);

        globalSemanticMap.push_back(unknown);
    }
    else
    {
        fuseSemanticObjects(globalSemanticMap[0], localMap[0]);
    }

    // First, integrate local "unknown" with global "unknown". They are always the 0-index
    lastMapLocalToGlobal[0] = 0;
    // Further integration of unknown is required: bbox, probabilities, etc. but to be decided yet

    const InstanceID_t currentInstancesNumber = globalSemanticMap.size();

    // Both loops start at 1 to skip "unknown" class
    for (InstanceID_t localInstanceID = 1; localInstanceID < localMap.size(); localInstanceID++)
    {
        bool fused = false;
        const SemanticObject& localInstance = localMap[localInstanceID];

        if (!localInstance.localGeometry.has_value())
            continue;
        std::vector<Bonxai::CoordT> voxelsLocal(localInstance.localGeometry->begin(), localInstance.localGeometry->end());

        // Find category with maximum probability for local instance
        CategoryManager::CategoryIndex localMaxCategory = localInstance.mostLikelyCategory();

        for (InstanceID_t globalInstanceID = 1; globalInstanceID < currentInstancesNumber; globalInstanceID++)
        {
            SemanticObject& globalInstance = globalSemanticMap[globalInstanceID];
            // Find category with maximum probability for global instance
            CategoryManager::CategoryIndex globalMaxCategory = localInstance.mostLikelyCategory();

            if (globalInstance.isStillValid() && checkBBoxIntersect(localInstance.bbox, globalInstance.bbox))
            {
                std::vector<Bonxai::CoordT> voxelsGlobal = listOfVoxelsInObject<DataT>(globalInstance);
                auto [iou, ios] = compute3DIoU<DataT>(voxelsGlobal, voxelsLocal, 2);

#if 1
                const double iouThreshold = localMaxCategory == globalMaxCategory ? 0.15 : 0.5;
#else
                // Calculate distance from sensor to the local instance center
                float localCenterX = (localInstance.bbox.minX + localInstance.bbox.maxX) / 2.0f;
                float localCenterY = (localInstance.bbox.minY + localInstance.bbox.maxY) / 2.0f;
                float localCenterZ = (localInstance.bbox.minZ + localInstance.bbox.maxZ) / 2.0f;
                float distanceToSensor = std::sqrt(
                    (localCenterX - sensorX) * (localCenterX - sensorX) +
                    (localCenterY - sensorY) * (localCenterY - sensorY) +
                    (localCenterZ - sensorZ) * (localCenterZ - sensorZ));

                // Dynamic IoU threshold based on distance
                // Close objects (< 1.5m): high IoU threshold (0.35) - we expect precise segmentation
                // Medium distance (1.5-3m): medium threshold (0.25)
                // Far objects (> 3m): low IoU threshold (0.15) but rely more on semantics
                double iouThreshold;
                double semanticBonus = 0.0;

                if (distanceToSensor < 1.5f)
                {
                    iouThreshold = 0.35;
                    semanticBonus = 0.05;  // Small bonus for same class when close
                }
                else if (distanceToSensor < 3.0f)
                {
                    iouThreshold = 0.25;
                    semanticBonus = 0.10;  // Medium bonus for same class at medium distance
                }
                else
                {
                    iouThreshold = 0.15;
                    semanticBonus = 0.15;  // Large bonus for same class when far (rely more on semantics)
                }

                // Apply semantic bonus: if same class, effectively lower the threshold
                // by adding a bonus to the IoU value instead of lowering threshold
                if (localMaxCategory == globalMaxCategory)
                {
                    // Same semantic class - add bonus to make fusion more likely
                    iou += semanticBonus;

                    // Additionally, if semantic confidence is high, add extra bonus
                    double maxLocalProbability = localInstance.getCategoryAlpha(localMaxCategory);
                    double maxGlobalProbability = globalInstance.getCategoryAlpha(globalMaxCategory);
                    double semanticConfidence = std::min(maxLocalProbability, maxGlobalProbability);
                    if (semanticConfidence > 0.7)
                    {
                        iou += 0.05;  // Extra bonus for high confidence matches
                    }
                }
#endif
                if (iou > iouThreshold)
                {
                    VXL_DEBUG(fmt::fg(fmt::terminal_color::yellow), "Fusing local {} - global {}:\n\tIoU:{:.2f}  IoS: {:.2f}", localInstanceID, globalInstanceID, iou, ios);
                    PAUSE_THREAD_UNTIL_GUI_CONTINUE;
                    fuseSemanticObjects(globalInstance, localInstance);

                    lastMapLocalToGlobal[localInstanceID] = globalInstanceID;

                    fused = true;
                    globalInstance.numberObservations++;
                    integrated++;
                    break;  // don't keep iterating over the globals, we are done with this local instance
                }
                else
                    VXL_DEBUG("NOT Fusing local {} - global {}:\n\tIoU:{:.2f}  IoS: {:.2f}", localInstanceID, globalInstanceID, iou, ios);
            }
        }

        if (!fused)
        {
            lastMapLocalToGlobal[localInstanceID] = globalSemanticMap.size();
            // Create new object integrating localMap information
            SemanticObject newObject = SemanticObject(globalSemanticMap.size(), localInstance.bbox);
            newObject.alphaParamsCategories = localInstance.alphaParamsCategories;
            updateAppearancesTimestamps(newObject, localInstance);

            // Add it to the global map
            globalSemanticMap.push_back(newObject);
            added += 1;
        }
    }
    VXL_INFO("Integrating {} new local objects: {} integrated and {} added", localMap.size(), integrated, added);
}

template <typename DataT>
inline void SemanticMap::refineGlobalSemanticMap(int nObservationsToRemove)
{
    for (InstanceID_t i = 1; i < globalSemanticMap.size(); i++)
    {
        SemanticObject& firstInstance = globalSemanticMap[i];

        if (!firstInstance.isStillValid())
            continue;

        std::vector<Bonxai::CoordT> voxelsFirst = listOfVoxelsInObject<DataT>(firstInstance);
        CategoryManager::CategoryIndex firstClassIdx = firstInstance.mostLikelyCategory();

        for (InstanceID_t j = i + 1; j < globalSemanticMap.size(); j++)
        {
            SemanticObject& secondInstance = globalSemanticMap[j];

            if (secondInstance.isStillValid() && checkBBoxIntersect(firstInstance.bbox, secondInstance.bbox))
            {
                std::vector<Bonxai::CoordT> voxelsSecond = listOfVoxelsInObject<DataT>(secondInstance);

                // Adaptive threshold based on:
                // 1. Semantic similarity (same class = lower threshold)
                // 2. Number of observations (more observations = more confident, need higher IoU)
                double iouThreshold = 0.6;  // Base threshold

                CategoryManager::CategoryIndex secondClassIdx = secondInstance.mostLikelyCategory();

                // If both instances have the same category, be more permissive
                bool sameCategory = firstClassIdx == secondClassIdx;
                if (sameCategory)
                    iouThreshold = 0.2;

                // For instances with many observations, require slightly higher IoU
                // (they are more established, need stronger evidence to merge)
                if (firstInstance.numberObservations > 10 && secondInstance.numberObservations > 10)
                    iouThreshold += 0.05;

                auto [iou, ios] = compute3DIoU<DataT>(voxelsFirst, voxelsSecond, 3);
                if (iou > iouThreshold)
                {
                    // Fuse the second instance with the first one
                    secondInstance.pointsTo = i;
                    VXL_DEBUG(fmt::fg(fmt::terminal_color::yellow), "(Refine) Fusing global {} - global {}:\n\tIoU:{:.2f}  IoS: {:.2f}", i, j, iou, ios);
                    PAUSE_THREAD_UNTIL_GUI_CONTINUE;
                    fuseSemanticObjects(firstInstance, secondInstance);
                    firstInstance.numberObservations += secondInstance.numberObservations;
                }
                else
                    VXL_DEBUG("(Refine) NOT Fusing global {} - global {}:\n\tIoU:{:.2f}  IoS: {:.2f}", i, j, iou, ios);
            }
        }
    }

    // remove instances with very few observations
    for (InstanceID_t i = 1; i < globalSemanticMap.size(); i++)
    {
        if (globalSemanticMap[i].isStillValid() && globalSemanticMap[i].numberObservations <= nObservationsToRemove)
            globalSemanticMap[i].pointsTo = 0;
    }
}

template <typename DataT>
inline std::vector<Bonxai::CoordT> SemanticMap::listOfVoxelsInObject(const SemanticObject object)
{
    std::vector<Bonxai::CoordT> cellsInside;

    Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>* bonxai = BonxaiQuery<DataT>::getBonxai()->grid();

    const Bonxai::CoordT coordMin = bonxai->posToCoord(Bonxai::Point3D(
        object.bbox.minX - bonxai->resolution, object.bbox.minY - bonxai->resolution, object.bbox.minZ - bonxai->resolution));
    const Bonxai::CoordT coordMax = bonxai->posToCoord(Bonxai::Point3D(
        object.bbox.maxX + bonxai->resolution, object.bbox.maxY + bonxai->resolution, object.bbox.maxZ + bonxai->resolution));

    // Iterate over all points inside the bounding box
    for (int x = coordMin.x; x <= coordMax.x; x++)
    {
        for (int y = coordMin.y; y <= coordMax.y; y++)
        {
            for (int z = coordMin.z; z <= coordMax.z; z++)
            {
                Bonxai::CoordT coord = Bonxai::CoordT{ x, y, z };
                Bonxai::ProbabilisticCell<DataT>* cell = BonxaiQuery<DataT>::getAccessor().value(coord);
                if (!cell)
                    continue;

                // do we want to consider all voxels in which a single vote exists for this instance, or only the ones where the instance wins?
#define CONSIDER_ANY_VOTE 0
#if CONSIDER_ANY_VOTE
                auto it = std::find(cell->data.instances_candidates.begin(), cell->data.instances_candidates.end(), object.instanceID);
                if (it != cell->data.instances_candidates.end())
                    cellsInside.push_back(coord);
#else
                if (cell->data.getMostRepresentativeInstance() == object.instanceID)
                    cellsInside.push_back(coord);
#endif
            }
        }
    }

    return cellsInside;
}

template <typename DataT>
inline std::pair<double, double> SemanticMap::compute3DIoU(const std::vector<Bonxai::CoordT>& voxels1,
                                                           const std::vector<Bonxai::CoordT>& voxels2,
                                                           float coarsening_factor)
{
    std::set<Bonxai::CoordT> voxels1_coarse;
    std::set<Bonxai::CoordT> voxels2_coarse;

    for (size_t i = 0; i < voxels1.size(); i++)
    {
        Bonxai::CoordT coord = voxels1[i];
        voxels1_coarse.insert(coord / coarsening_factor);
    }

    for (size_t i = 0; i < voxels2.size(); i++)
    {
        Bonxai::CoordT coord = voxels2[i];
        voxels2_coarse.insert(coord / coarsening_factor);
    }

    auto orderFunc = [](const Bonxai::CoordT& c1, const Bonxai::CoordT& c2) {
        return c1.x < c2.x || (c1.x == c2.x && c1.y < c2.y) || (c1.x == c2.x && c1.y == c2.y && c1.z < c2.z);
    };

    std::vector<Bonxai::CoordT> intersection_;
    std::vector<Bonxai::CoordT> union_;

    std::set_intersection(voxels1_coarse.begin(),
                          voxels1_coarse.end(),
                          voxels2_coarse.begin(),
                          voxels2_coarse.end(),
                          std::back_inserter(intersection_),
                          orderFunc);
    std::set_union(voxels1_coarse.begin(),
                   voxels1_coarse.end(),
                   voxels2_coarse.begin(),
                   voxels2_coarse.end(),
                   std::back_inserter(union_),
                   orderFunc);

    double IoU = 0.;
    if (union_.size() > 0)
        IoU = ((double)intersection_.size()) / union_.size();

    double IoS = 0.;
    if (voxels1_coarse.size() > 0)
        IoS = ((double)intersection_.size()) / voxels1_coarse.size();
    if (voxels2_coarse.size() > 0)
        IoS = std::max(IoS, ((double)intersection_.size()) / voxels2_coarse.size());

    // VXL_INFO("IoU: {:.2f}\nIoS: {:.2f}", IoU, IoS);
    return std::pair<double, double>(IoU, IoS);  // TODO probably a good idea to just return both and let the caller decide what to do with them
}

template <typename DataT>
inline Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>::Accessor& BonxaiQuery<DataT>::getAccessor()
{
    return *accessor;
}

template <typename DataT>
inline void BonxaiQuery<DataT>::createAccessor(Bonxai::ProbabilisticMapT<DataT>* _bonxai)
{
    bonxai = _bonxai;
    accessor.emplace(_bonxai->grid()->createAccessor());
}
