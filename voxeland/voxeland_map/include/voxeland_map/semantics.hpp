#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <eigen3/Eigen/Core>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <unordered_map>
#include <vector>
#include <voxeland_map/Utils/logging.hpp>
#include <voxeland_map/category_manager.hpp>
#include <voxeland_map/data_modes.hpp>
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

struct BoundingBox2D
{
    float centerX;
    float centerY;
    float sizeX;
    float sizeY;
};

struct SemanticObject
{
    // Note: For now, it is supposed that in the globalSemanticMap, instances are not going to disappear.
    // Otherwise, it should be considered, as the instanceID is also used as the index into the semantic map vector
    //TODO (pepe) we should consider changing it to a hashmap. I think any performance impact from lookups will be compensated by not having to deal with lots of old invalid instances
    InstanceID_t instanceID;
    std::string instanceName;
    // Dynamic storage for category probabilities - grows as needed
    std::unordered_map<CategoryManager::CategoryIndex, double> alphaParamsCategories;

    // Dynamic storage for appearances per category - grows as needed
    std::unordered_map<CategoryManager::CategoryIndex, std::map<uint32_t, BoundingBox2D>> appearancesTimestamps;
    uint32_t numberObservations = 1;
    BoundingBox3D bbox;

    std::optional<std::unordered_set<Bonxai::CoordT>> localGeometry;

    int32_t pointsTo = -1;  // In case a semantic object is integrated with another, the pointsTo variable
    // need to be set with the instanceID of the main object, hence if pointsTo is not empty, it won't check
    // the data in this SemanticObject, but instead it will check the data in the instanceID set in pointsTo.

    SemanticObject(InstanceID_t _instanceID)
        : instanceID(_instanceID)
        , instanceName("obj" + std::to_string(_instanceID))
    {}

    SemanticObject(InstanceID_t _instanceID, BoundingBox3D _bbox)
        : instanceID(_instanceID)
        , instanceName("obj" + std::to_string(_instanceID))
        , bbox(_bbox)
    {}

    bool isStillValid() const
    {
        return pointsTo == -1;
    }

    InstanceID_t mostLikelyCategory() const
    {
        auto it = std::max_element(alphaParamsCategories.begin(), alphaParamsCategories.end(),      //
                                   [](const std::pair<CategoryManager::CategoryIndex, double>& p1,  //
                                      const std::pair<CategoryManager::CategoryIndex, double>& p2) {
                                       return p1.second < p2.second;
                                   });
        return std::distance(alphaParamsCategories.begin(), it);
    }

    // Get probability for a specific category (returns 0 if category not found)
    double getCategoryAlpha(CategoryManager::CategoryIndex categoryIndex) const
    {
        auto it = alphaParamsCategories.find(categoryIndex);
        return it != alphaParamsCategories.end() ? it->second : 0.0;
    }

    // Set probability for a specific category
    void setCategoryAlpha(CategoryManager::CategoryIndex categoryIndex, double probability)
    {
        alphaParamsCategories[categoryIndex] = probability;
    }

    // Add to probability for a specific category
    void addToCategoryAlpha(CategoryManager::CategoryIndex categoryIndex, double probability)
    {
        alphaParamsCategories[categoryIndex] += probability;
    }
};

class SemanticMap
{
public:
    SemanticMap();

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
                    voxeland::DataMode mode);
    uint32_t getCurrentActiveInstances();
    void setLocalSemanticMap(const std::vector<SemanticObject>& localMap);
    InstanceID_t localToGlobalInstance(InstanceID_t localInstance);
    uint32_t indexToHexColor(InstanceID_t index);

    // Updated methods to work with CategoryManager
    void updateCategoryProbability(SemanticObject& semanticObject, const std::string& categoryName, double probability);
    CategoryManager::CategoryIndex addCategory(const std::string& categoryName);
    CategoryManager::CategoryIndex getCategoryIndex(const std::string& categoryName) const;
    std::string getCategoryName(CategoryManager::CategoryIndex index) const;
    size_t getNumCategories() const;

    bool checkBBoxIntersect(const BoundingBox3D& box1, const BoundingBox3D& box2);
    void updateAlphaCategories(SemanticObject& original, const SemanticObject& update);
    void updateBBoxBounds(BoundingBox3D& original, const BoundingBox3D& update);
    void updateAppearancesTimestamps(SemanticObject& original, const SemanticObject& update);
    void fuseSemanticObjects(SemanticObject& firstInstance, const SemanticObject& secondInstance);

    template <typename DataT>
    double compute3DIoU(const std::vector<Bonxai::CoordT>& voxels1,
                        const std::vector<Bonxai::CoordT>& voxels2)
    {
        std::set<Bonxai::CoordT> voxels1_coarse;
        std::set<Bonxai::CoordT> voxels2_coarse;

        constexpr uint coarse_factor = 1;

        for (size_t i = 0; i < voxels1.size(); i++)
        {
            Bonxai::CoordT coord = voxels1[i];
            voxels1_coarse.insert(coord / coarse_factor);
        }

        for (size_t i = 0; i < voxels2.size(); i++)
        {
            Bonxai::CoordT coord = voxels2[i];
            voxels2_coarse.insert(coord / coarse_factor);
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
        return std::max(IoU, IoS);  // TODO probably a good idea to just return both and let the caller decide what to do with them
    }

    template <typename DataT>
    std::vector<Bonxai::CoordT> listOfVoxelsInObject(const SemanticObject object)
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
    void refineGlobalSemanticMap(int nObservationsToRemove)
    {
        for (InstanceID_t i = 1; i < globalSemanticMap.size(); i++)
        {
            SemanticObject& firstInstance = globalSemanticMap[i];

            if (firstInstance.pointsTo != -1)
            {
                continue;
            }
            std::vector<Bonxai::CoordT> voxelsFirst = listOfVoxelsInObject<DataT>(firstInstance);

            for (InstanceID_t j = i + 1; j < globalSemanticMap.size(); j++)
            {
                SemanticObject& secondInstance = globalSemanticMap[j];

                if (secondInstance.isStillValid() && checkBBoxIntersect(firstInstance.bbox, secondInstance.bbox))
                {
                    bool customIoU = false;
                    if (firstInstance.numberObservations > 5 && secondInstance.numberObservations > 5)
                    {
                        customIoU = true;
                    }
                    std::vector<Bonxai::CoordT> voxelsSecond = listOfVoxelsInObject<DataT>(secondInstance);

                    double iou = compute3DIoU<DataT>(voxelsFirst, voxelsSecond);
                    if (iou > 0.3)
                    {
                        // Fuse the second instance with the first one
                        secondInstance.pointsTo = i;
                        fuseSemanticObjects(firstInstance, secondInstance);
                        firstInstance.numberObservations += secondInstance.numberObservations;
                    }
                }
            }
        }

        for (InstanceID_t i = 1; i < globalSemanticMap.size(); i++)
        {
            if (globalSemanticMap[i].isStillValid() && globalSemanticMap[i].numberObservations <= nObservationsToRemove)
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
                    double iou = compute3DIoU<DataT>(voxelsGlobal, voxelsLocal);

                    const double fuseThreshold = localMaxCategory == globalMaxCategory ? 0.2 : 0.4;
                    if (iou > fuseThreshold)
                    {
                        fuseSemanticObjects(globalInstance, localInstance);

                        lastMapLocalToGlobal[localInstanceID] = globalInstanceID;

                        fused = true;
                        globalInstance.numberObservations++;
                        integrated++;
                        break;  // don't keep iterating over the globals, we are done with this local instance
                    }
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

    template <typename DataT, typename PointCloudTypeT>
    void addInstancesGeometryToLocalSemanticMap(std::vector<SemanticObject>& localMap, const PointCloudTypeT& pc)
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

    nlohmann::json mapToJSON()
    {
        nlohmann::json data_json;

        data_json["instances"] = {};

        for (size_t i = 0; i < globalSemanticMap.size(); i++)
        {
            if (globalSemanticMap[i].isStillValid())
            {
                data_json["instances"][globalSemanticMap[i].instanceName] = {};
                data_json["instances"][globalSemanticMap[i].instanceName]["bbox"] = {};

                nlohmann::json center = nlohmann::json::array();
                center.push_back((globalSemanticMap[i].bbox.minX + globalSemanticMap[i].bbox.maxX) / 2.0);
                center.push_back((globalSemanticMap[i].bbox.minY + globalSemanticMap[i].bbox.maxY) / 2.0);
                center.push_back((globalSemanticMap[i].bbox.minZ + globalSemanticMap[i].bbox.maxZ) / 2.0);
                data_json["instances"][globalSemanticMap[i].instanceName]["bbox"]["center"] = center;

                nlohmann::json size = nlohmann::json::array();
                size.push_back(globalSemanticMap[i].bbox.maxX - globalSemanticMap[i].bbox.minX);
                size.push_back(globalSemanticMap[i].bbox.maxY - globalSemanticMap[i].bbox.minY);
                size.push_back(globalSemanticMap[i].bbox.maxZ - globalSemanticMap[i].bbox.minZ);
                data_json["instances"][globalSemanticMap[i].instanceName]["bbox"]["size"] = size;

                data_json["instances"][globalSemanticMap[i].instanceName]["results"] = {};

                // Convert dynamic category probabilities to JSON
                for (const auto& [categoryIndex, probability] : globalSemanticMap[i].alphaParamsCategories)
                {
                    if (probability > 0)
                    {
                        std::string categoryName = getCategoryName(categoryIndex);
                        if (!categoryName.empty())
                        {
                            data_json["instances"][globalSemanticMap[i].instanceName]["results"][categoryName] = probability;
                        }
                    }
                }

                data_json["instances"][globalSemanticMap[i].instanceName]["n_observations"] =
                    globalSemanticMap[i].numberObservations;
            }
        }

        return data_json;
    }

    void updateSemanticMapResultsFromJSON(const nlohmann::json& data_json)
    {
        if (!initialized)
        {
            throw std::runtime_error("SemanticMap is not initialized.");
        }

        for (SemanticObject& instance : globalSemanticMap)
        {
            if (instance.pointsTo != -1)
            {
                continue;
            }

            auto index_iter = data_json["instances"].find(instance.instanceName);
            if (index_iter == data_json["instances"].end())
            {
                throw std::runtime_error("Instance " + instance.instanceName + "not found in JSON data");
            }
            const nlohmann::json& instance_json = index_iter.value();
            for (const auto& [category, alpha] : instance_json["results"].items())
            {
                CategoryManager::CategoryIndex categoryIndex = getCategoryIndex(category);
                if (categoryIndex != CategoryManager::INVALID_CATEGORY)
                {
                    instance.setCategoryAlpha(categoryIndex, alpha);
                }
            }
        }
    }

    nlohmann::json appearancesToJson()
    {
        nlohmann::json data_json;

        data_json = {};
        for (size_t i = 0; i < globalSemanticMap.size(); i++)
        {
            if (globalSemanticMap[i].isStillValid())
            {
                data_json[globalSemanticMap[i].instanceName] = {};
                data_json[globalSemanticMap[i].instanceName]["timestamps"] = {};
                for (size_t j = 0; j < globalSemanticMap[i].appearancesTimestamps.size(); j++)
                {
                    std::string category = getCategoryName(j);
                    const auto& appearances_map = globalSemanticMap[i].appearancesTimestamps[j];
                    if (appearances_map.empty())
                    {
                        continue;
                    }

                    data_json[globalSemanticMap[i].instanceName]["timestamps"][category] = nlohmann::json::array();
                    for (const auto& instancePair : appearances_map)
                    {
                        nlohmann::json instanceBbox;
                        instanceBbox["instance_id"] = instancePair.first;
                        instanceBbox["bbox"]["centerX"] = instancePair.second.centerX;
                        instanceBbox["bbox"]["centerY"] = instancePair.second.centerY;
                        instanceBbox["bbox"]["sizeX"] = instancePair.second.sizeX;
                        instanceBbox["bbox"]["sizeY"] = instancePair.second.sizeY;

                        data_json[globalSemanticMap[i].instanceName]["timestamps"][category].push_back(instanceBbox);
                    }
                }
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