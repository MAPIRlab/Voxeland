#pragma once

#include <cmath>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <unordered_map>
#include <vector>
#include <voxeland_map/data_modes.hpp>
#include <voxeland_map/logging.hpp>
#include <voxeland_map/pcl_utils.hpp>
#include <voxeland_map/probabilistic_map_templated.hpp>

#include "bonxai/bonxai.hpp"

struct BoundingBox3D
{
    float minX = std::numeric_limits<float>::infinity();
    float minY = std::numeric_limits<float>::infinity();
    float minZ = std::numeric_limits<float>::infinity();
    float maxX = -std::numeric_limits<float>::infinity();
    float maxY = -std::numeric_limits<float>::infinity();
    float maxZ = -std::numeric_limits<float>::infinity();
};

struct SemanticObject
{
    // Note: For now, it is supposed that in the globalSemanticMap, instances are not going to disappear.
    // Otherwise, it should be considered, as the instanceID cannot be the globalSemanticMap.size()+1
    std::string instanceID;
    std::vector<double> probabilities;
    uint32_t numberObservations = 1;
    BoundingBox3D bbox;

    std::optional<std::unordered_set<Bonxai::CoordT>> localGeometry;

    int32_t pointsTo = -1;  // In case a semantic object is integrated with another, the pointsTo variable
    // need to be set with the instanceID of the main object, hence if pointsTo is not empty, it won't check
    // the data in this SemanticObject, but instead it will check the data in the instanceID set in pointsTo.

    SemanticObject(size_t numCategories, InstanceID_t _instanceID)
        : probabilities(numCategories, 0)
        , instanceID("obj" + std::to_string(_instanceID))
    {}
    SemanticObject(const std::vector<double>& _probabilities, InstanceID_t _instanceID)
        : probabilities(_probabilities)
        , instanceID("obj" + std::to_string(_instanceID))
    {}
    SemanticObject(size_t numCategories, InstanceID_t _instanceID, BoundingBox3D _bbox)
        : probabilities(numCategories, 0)
        , instanceID("obj" + std::to_string(_instanceID))
        , bbox(_bbox)
    {}
    SemanticObject(const std::vector<double>& _probabilities, InstanceID_t _instanceID, BoundingBox3D _bbox)
        : probabilities(_probabilities)
        , instanceID("obj" + std::to_string(_instanceID))
        , bbox(_bbox)
    {}
};

class SemanticMap
{
public:
    SemanticMap();

    std::vector<std::string> default_categories;
    std::unordered_map<std::string, size_t> categoryIndexMap;
    std::vector<SemanticObject> globalSemanticMap;
    std::vector<SemanticObject> lastLocalSemanticMap;

    static SemanticMap& get_instance()
    {
        static SemanticMap instance;
        return instance;
    }

    template <typename DataT>
    class BonxaiQuery
    {
    public:
        static void createAccessor(Bonxai::ProbabilisticMapT<DataT>* _bonxai)
        {
            bonxai = _bonxai;
            accessor.emplace(_bonxai->grid()->createAccessor());
        }

        // Note that, before calling this function, the accessor object has to be created beforehand.
        static typename Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>::Accessor& getAccessor()
        {
            return *accessor;
        }

        static Bonxai::ProbabilisticMapT<DataT>* getBonxai() { return bonxai; }

    private:
        inline static Bonxai::ProbabilisticMapT<DataT>* bonxai;
        inline static std::optional<typename Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>::Accessor> accessor;
    };

    bool is_initialized();
    void initialize(std::vector<std::string> dataset_categories,
                    Bonxai::ProbabilisticMap& _bonxai,
                    Bonxai::DataMode mode);
    uint32_t getCurrentActiveInstances();
    void setLocalSemanticMap(const std::vector<SemanticObject>& localMap);
    InstanceID_t localToGlobalInstance(InstanceID_t localInstance);
    uint32_t indexToHexColor(InstanceID_t index);
    void updateCategoryProbability(SemanticObject& semanticObject, const std::string& categoryName, double probability);
    bool checkBBoxIntersect(const BoundingBox3D& box1, const BoundingBox3D& box2);
    void updateBBoxBounds(BoundingBox3D& original, const BoundingBox3D& update);
    InstanceID_t getCategoryMaxProbability(InstanceID_t objID);

    template <typename DataT>
    double compute3DIoU(const BoundingBox3D& globalBbox,
                        InstanceID_t globalID,
                        const std::unordered_set<Bonxai::CoordT>& localVoxels)
    {
        std::vector<Bonxai::CoordT> voxels1 = listOfVoxelsInsideBBox<DataT>(globalBbox, globalID);
        std::vector<Bonxai::CoordT> voxels2;
        voxels2.assign(localVoxels.begin(), localVoxels.end());

        std::set<Bonxai::CoordT> voxels1_coarse;
        std::set<Bonxai::CoordT> voxels2_coarse;

        for (size_t i = 0; i < voxels1.size(); i++)
        {
            voxels1_coarse.insert(voxels1[i] / 5);
        }

        for (size_t i = 0; i < voxels2.size(); i++)
        {
            voxels2_coarse.insert(voxels2[i] / 5);
        }

        auto orderFunc = [](const Bonxai::CoordT& c1, const Bonxai::CoordT& c2) {
            return c1.x < c2.x || (c1.x == c2.x && c1.y < c2.y) || (c1.x == c2.x && c1.y == c2.y && c1.z < c2.z);
        };

        // VXL_INFO("Global voxels size: %d / Local voxels size %d", globalVoxels.size(), localVoxels_.size());
        // std::sort(voxels1_coarse.begin(), voxels1_coarse.end(), orderFunc);
        // std::sort(voxels2_coarse.begin(), voxels2_coarse.end(), orderFunc);

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

        double iouLocal = 0.;
        if (voxels2_coarse.size() > 0)
        {
            iouLocal = (double)intersection_.size() / voxels2_coarse.size();
        }
        double iouGlobal = 0.;
        if (voxels1_coarse.size() > 0)
        {
            iouGlobal = (double)intersection_.size() / voxels1_coarse.size();
        }
        double iou = 0.;
        if (union_.size() > 0)
        {
            iou = (double)intersection_.size() / union_.size();
        }

        // VXL_INFO("IoU --> Local: {} // Global: {} // Total: {}", iouLocal, iouGlobal, iou);
        if (iouLocal > 0.3 || iouGlobal > 0.3 || iou > 0.3)
        {
            // if(iou > 0.3){
            iou = 0.35;
        }

        return iou;
    }

    template <typename DataT>
    double compute3DIoU(const BoundingBox3D& bBox1,
                        InstanceID_t id1,
                        const BoundingBox3D& bBox2,
                        InstanceID_t id2,
                        bool customIoU)
    {
        std::vector<Bonxai::CoordT> voxels1 = listOfVoxelsInsideBBox<DataT>(bBox1, id1);
        std::vector<Bonxai::CoordT> voxels2 = listOfVoxelsInsideBBox<DataT>(bBox2, id2);

        std::set<Bonxai::CoordT> voxels1_coarse;
        std::set<Bonxai::CoordT> voxels2_coarse;

        for (size_t i = 0; i < voxels1.size(); i++)
        {
            voxels1_coarse.insert(voxels1[i] / 5);
        }

        for (size_t i = 0; i < voxels2.size(); i++)
        {
            voxels2_coarse.insert(voxels2[i] / 5);
        }

        auto orderFunc = [](const Bonxai::CoordT& c1, const Bonxai::CoordT& c2) {
            return c1.x < c2.x || (c1.x == c2.x && c1.y < c2.y) || (c1.x == c2.x && c1.y == c2.y && c1.z < c2.z);
        };

        // VXL_INFO("Global voxels size: %d / Local voxels size %d", globalVoxels.size(), localVoxels_.size());
        // std::sort(voxels1_coarse.begin(), voxels1_coarse.end(), orderFunc);
        // std::sort(voxels2_coarse.begin(), voxels2_coarse.end(), orderFunc);

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

        // VXL_INFO("GLOBAL WITH GLOBAL: {} voxels en 1 y {} voxels en 2", voxels1.size(), voxels2.size());
        double iouLocal = 0.;
        if (voxels1_coarse.size() > 0 && customIoU)
        {
            iouLocal = (double)intersection_.size() / voxels1_coarse.size();
        }
        double iouGlobal = 0.;
        if (voxels2_coarse.size() > 0 && customIoU)
        {
            iouGlobal = (double)intersection_.size() / voxels2_coarse.size();
        }
        double iou = 0.;
        if (union_.size() > 0)
        {
            iou = (double)intersection_.size() / union_.size();
        }

        if (iouLocal > 0.3 || iouGlobal > 0.3 || iou > 0.3)
        {
            iou = 0.35;
        }
        VXL_INFO("local: {}, global: {}, iou: {}",
                 (double)intersection_.size() / voxels1_coarse.size(),
                 (double)intersection_.size() / voxels2_coarse.size(),
                 iou);

        return iou;
    }

    template <typename DataT>
    std::vector<Bonxai::CoordT> listOfVoxelsInsideBBox(const BoundingBox3D& bbox, InstanceID_t id)
    {
        std::vector<Bonxai::CoordT> cellsInside;

        Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>* bonxai = BonxaiQuery<DataT>::getBonxai()->grid();

        const Bonxai::CoordT coordMin = bonxai->posToCoord(Bonxai::Point3D(
            bbox.minX - bonxai->resolution, bbox.minY - bonxai->resolution, bbox.minZ - bonxai->resolution));
        const Bonxai::CoordT coordMax = bonxai->posToCoord(Bonxai::Point3D(
            bbox.maxX + bonxai->resolution, bbox.maxY + bonxai->resolution, bbox.maxZ + bonxai->resolution));

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
                    // VXL_INFO("Inicio: %d, fin: %d", cell->data.instances_candidates.begin(),
                    // cell->data.instances_candidates.end());
                    auto it =
                        std::find(cell->data.instances_candidates.begin(), cell->data.instances_candidates.end(), id);
                    if (it != cell->data.instances_candidates.end())
                    {
                        size_t idx = std::distance(cell->data.instances_candidates.begin(), it);
                        cellsInside.push_back(coord);
                    }
                }
            }
        }

        return cellsInside;
    }

    template <typename DataT>
    void refineGlobalSemanticMap(int nObservationsToRemove)
    {
        for (InstanceID_t i = 1; i < globalSemanticMap.size(); i++)
        {
            SemanticObject& firstInstance = globalSemanticMap[i];

            if (firstInstance.pointsTo != -1)
            {
                continue;
            }

            for (InstanceID_t j = i + 1; j < globalSemanticMap.size(); j++)
            {
                SemanticObject& secondInstance = globalSemanticMap[j];

                if (secondInstance.pointsTo == -1 && checkBBoxIntersect(firstInstance.bbox, secondInstance.bbox))
                {
                    bool customIoU = false;
                    if (firstInstance.numberObservations > 5 && secondInstance.numberObservations > 5)
                    {
                        customIoU = true;
                    }
                    double iou = compute3DIoU<DataT>(firstInstance.bbox, i, secondInstance.bbox, j, true);
                    if (iou > 0.3)
                    {
                        secondInstance.pointsTo = i;
                        for (size_t k = 0; k < firstInstance.probabilities.size(); k++)
                        {
                            firstInstance.probabilities[k] += secondInstance.probabilities[k];
                        }
                        updateBBoxBounds(firstInstance.bbox, secondInstance.bbox);
                        firstInstance.numberObservations += secondInstance.numberObservations;
                    }
                }
            }
        }

        for (InstanceID_t i = 1; i < globalSemanticMap.size(); i++)
        {
            if (globalSemanticMap[i].pointsTo == -1 && globalSemanticMap[i].numberObservations <= nObservationsToRemove)
            {
                globalSemanticMap[i].pointsTo = 0;
            }
        }
    }

    template <typename DataT>
    void integrateNewSemantics(const std::vector<SemanticObject>& localMap)
    {
        uint8_t integrated = 0;
        uint8_t added = 0;

        lastMapLocalToGlobal.resize(localMap.size());
        if (globalSemanticMap.empty())
        {
            globalSemanticMap.push_back(SemanticObject(localMap[0].probabilities, 0, localMap[0].bbox));
        }
        else
        {
            for (size_t i = 0; i < localMap[0].probabilities.size(); i++)
            {
                globalSemanticMap[0].probabilities[i] += localMap[0].probabilities[i];
            }
            updateBBoxBounds(globalSemanticMap[0].bbox, localMap[0].bbox);
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

            std::vector<double>::const_iterator itLocal =
                std::max_element(localInstance.probabilities.begin(), localInstance.probabilities.end());
            uint8_t localClassIdx = std::distance(localInstance.probabilities.begin(), itLocal);

            for (InstanceID_t globalInstanceID = 1; globalInstanceID < currentInstancesNumber; globalInstanceID++)
            {
                SemanticObject& globalInstance = globalSemanticMap[globalInstanceID];
                std::vector<double>::iterator itGlobal =
                    std::max_element(globalInstance.probabilities.begin(), globalInstance.probabilities.end());
                uint8_t globalClassIdx = std::distance(globalInstance.probabilities.begin(), itGlobal);

                if (globalInstance.pointsTo == -1 && checkBBoxIntersect(localInstance.bbox, globalInstance.bbox))
                {
                    // Compute 3D IoU... To Do... meanwhile, check if the best category of each one is the same
                    // if (localClassIdx == globalClassIdx)
                    double iou =
                        compute3DIoU<DataT>(globalInstance.bbox, globalInstanceID, localInstance.localGeometry.value());
                    if (iou > 0.3)
                    {
                        // VXL_INFO("Integrando {} local con {} global con IoU de {}",
                        // default_categories[localClassIdx].c_str(), default_categories[globalClassIdx].c_str(), iou);
                        for (size_t i = 0; i < globalInstance.probabilities.size(); i++)
                        {
                            globalInstance.probabilities[i] += localInstance.probabilities[i];
                        }
                        lastMapLocalToGlobal[localInstanceID] = globalInstanceID;
                        // int before_update = listOfVoxelsInsideBBox<DataT>(globalInstance.bbox,
                        // globalInstanceID).size();
                        updateBBoxBounds(globalInstance.bbox, localInstance.bbox);
                        // int after_update = listOfVoxelsInsideBBox<DataT>(globalInstance.bbox,
                        // globalInstanceID).size(); if (before_update > after_update){
                        //   VXL_INFO("Bounding box updated wrongly!");
                        // }
                        globalInstance.numberObservations += 1;
                        fused = true;
                        integrated += 1;
                        continue;
                    }
                }
            }
            if (!fused)
            {
                lastMapLocalToGlobal[localInstanceID] = globalSemanticMap.size();
                globalSemanticMap.push_back(
                    SemanticObject(localInstance.probabilities, globalSemanticMap.size(), localInstance.bbox));
                added += 1;
            }
        }
        VXL_INFO("Integrating {} new local objects: {} integrated and {} added", localMap.size(), integrated, added);
    }

    template <typename DataT, typename PointCloudTypeT>
    void addInstancesGeometryToLocalSemanticMap(std::vector<SemanticObject>& localMap, const PointCloudTypeT& pc)
    {
        Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>* bonxai = BonxaiQuery<DataT>::getBonxai()->grid();

        for (size_t i = 0; i < pc.points.size(); i++)
        {
            if (!localMap[pc.points[i].instance_id].localGeometry.has_value())
            {
                localMap[pc.points[i].instance_id].localGeometry.emplace();
            }

            localMap[pc.points[i].instance_id].localGeometry.value().insert(
                bonxai->posToCoord(Bonxai::Point3D(pc.points[i].x, pc.points[i].y, pc.points[i].z)));

            // Update min bounds
            localMap[pc.points[i].instance_id].bbox.minX =
                std::min(pc.points[i].x, localMap[pc.points[i].instance_id].bbox.minX);
            localMap[pc.points[i].instance_id].bbox.minY =
                std::min(pc.points[i].y, localMap[pc.points[i].instance_id].bbox.minY);
            localMap[pc.points[i].instance_id].bbox.minZ =
                std::min(pc.points[i].z, localMap[pc.points[i].instance_id].bbox.minZ);

            // Update max bounds
            localMap[pc.points[i].instance_id].bbox.maxX =
                std::max(pc.points[i].x, localMap[pc.points[i].instance_id].bbox.maxX);
            localMap[pc.points[i].instance_id].bbox.maxY =
                std::max(pc.points[i].y, localMap[pc.points[i].instance_id].bbox.maxY);
            localMap[pc.points[i].instance_id].bbox.maxZ =
                std::max(pc.points[i].z, localMap[pc.points[i].instance_id].bbox.maxZ);
        }
    }

    template <typename DataT>
    std::set<InstanceID_t> getCurrentVisibleInstances(double minOccupancyZ, double maxOccupancyZ)
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
                if (globalSemanticMap[bestInstanceID].pointsTo == -1)
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

    nlohmann::json mapToJSON()
    {
        nlohmann::json data_json;

        data_json["instances"] = {};

        for (size_t i = 0; i < globalSemanticMap.size(); i++)
        {
            if (globalSemanticMap[i].pointsTo == -1)
            {
                data_json["instances"][globalSemanticMap[i].instanceID] = {};
                data_json["instances"][globalSemanticMap[i].instanceID]["bbox"] = {};

                nlohmann::json center = nlohmann::json::array();
                center.push_back((globalSemanticMap[i].bbox.minX + globalSemanticMap[i].bbox.maxX) / 2.0);
                center.push_back((globalSemanticMap[i].bbox.minY + globalSemanticMap[i].bbox.maxY) / 2.0);
                center.push_back((globalSemanticMap[i].bbox.minZ + globalSemanticMap[i].bbox.maxZ) / 2.0);
                data_json["instances"][globalSemanticMap[i].instanceID]["bbox"]["center"] = center;

                nlohmann::json size = nlohmann::json::array();
                size.push_back(globalSemanticMap[i].bbox.maxX - globalSemanticMap[i].bbox.minX);
                size.push_back(globalSemanticMap[i].bbox.maxY - globalSemanticMap[i].bbox.minY);
                size.push_back(globalSemanticMap[i].bbox.maxZ - globalSemanticMap[i].bbox.minZ);
                data_json["instances"][globalSemanticMap[i].instanceID]["bbox"]["size"] = size;

                data_json["instances"][globalSemanticMap[i].instanceID]["results"] = {};
                for (InstanceID_t j = 0; j < default_categories.size(); j++)
                {
                    if (globalSemanticMap[i].probabilities[j] > 0)
                    {
                        data_json["instances"][globalSemanticMap[i].instanceID]["results"][default_categories[j]] =
                            globalSemanticMap[i].probabilities[j];
                    }
                }

                data_json["instances"][globalSemanticMap[i].instanceID]["n_observations"] =
                    globalSemanticMap[i].numberObservations;
            }
        }

        return data_json;
    }

private:
    std::vector<InstanceID_t> lastMapLocalToGlobal;
    std::vector<std::uint32_t> color_palette;
    bool initialized = false;
    double kld_threshold;

    double computeKLD(const std::vector<double>& P, const std::vector<double>& Q);
};