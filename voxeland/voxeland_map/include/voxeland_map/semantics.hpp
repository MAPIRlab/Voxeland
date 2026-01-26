#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <unordered_map>
#include <vector>
#include <voxeland_map/data_modes.hpp>
#include <voxeland_map/Utils/logging.hpp>
#include <voxeland_map/pcl_utils.hpp>
#include <voxeland_map/probabilistic_map_templated.hpp>
#include <voxeland_map/category_manager.hpp>

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
    // Otherwise, it should be considered, as the instanceID cannot be the globalSemanticMap.size()+1
    std::string instanceID;
    
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
        : instanceID("obj" + std::to_string(_instanceID))
    {}
    
    SemanticObject(InstanceID_t _instanceID, BoundingBox3D _bbox)
        : instanceID("obj" + std::to_string(_instanceID))
        , bbox(_bbox)
    {}
    
    // Get probability for a specific category (returns 0 if category not found)
    double getCategoryProbability(CategoryManager::CategoryIndex categoryIndex) const
    {
        auto it = alphaParamsCategories.find(categoryIndex);
        return it != alphaParamsCategories.end() ? it->second : 0.0;
    }
    
    // Set probability for a specific category
    void setCategoryProbability(CategoryManager::CategoryIndex categoryIndex, double probability)
    {
        alphaParamsCategories[categoryIndex] = probability;
    }
    
    // Add to probability for a specific category
    void addToCategoryProbability(CategoryManager::CategoryIndex categoryIndex, double probability)
    {
        alphaParamsCategories[categoryIndex] += probability;
    }
    
    // Get all category probabilities as a vector (for compatibility with existing code)
    std::vector<double> getCategoryProbabilityVector() const
    {
        CategoryManager& catManager = CategoryManager::getInstance();
        size_t numCategories = catManager.getNumCategories();
        std::vector<double> probabilities(numCategories, 0.0);
        
        for (const auto& [categoryIndex, probability] : alphaParamsCategories)
        {
            if (categoryIndex < numCategories)
            {
                probabilities[categoryIndex] = probability;
            }
        }
        
        return probabilities;
    }
    
    // Set category probabilities from vector (for compatibility with existing code)
    void setCategoryProbabilityVector(const std::vector<double>& probabilities)
    {
        alphaParamsCategories.clear();
        for (size_t i = 0; i < probabilities.size(); ++i)
        {
            if (probabilities[i] > 0.0)
            {
                alphaParamsCategories[i] = probabilities[i];
            }
        }
    }
    
    // Check if this instance is still valid (not fused into another)
    bool isStillValid() const
    {
        return pointsTo == -1;
    }
    
    // Get index of category with highest probability
    uint32_t mostLikelyCategory() const
    {
        CategoryManager::CategoryIndex maxIdx = CategoryManager::UNKNOWN_CATEGORY;
        double maxProb = 0.0;
        for (const auto& [catIdx, prob] : alphaParamsCategories)
        {
            if (prob > maxProb)
            {
                maxProb = prob;
                maxIdx = catIdx;
            }
        }
        return maxIdx;
    }
};

class SemanticMap
{
public:
    SemanticMap();

    // Legacy support - these will delegate to CategoryManager
    std::vector<std::string> default_categories; //list of category names. The last one is always "background"
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
    InstanceID_t getCategoryMaxProbability(InstanceID_t objID);

    template <typename DataT>
    double compute3DIoU(const SemanticObject& globalObject,
                        const std::unordered_set<Bonxai::CoordT>& localVoxels)
    {
        std::vector<Bonxai::CoordT> voxels1 = listOfVoxelsInObject<DataT>(globalObject);
        std::vector<Bonxai::CoordT> voxels2;
        voxels2.assign(localVoxels.begin(), localVoxels.end());

        std::set<Bonxai::CoordT> voxels1_coarse;
        std::set<Bonxai::CoordT> voxels2_coarse;

        // Use a smaller coarsening factor to preserve geometric precision
        const int COARSE_FACTOR = 1;
        
        for (size_t i = 0; i < voxels1.size(); i++)
        {
            voxels1_coarse.insert(voxels1[i] / COARSE_FACTOR);
        }

        for (size_t i = 0; i < voxels2.size(); i++)
        {
            voxels2_coarse.insert(voxels2[i] / COARSE_FACTOR);
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

        // Calculate directional IoUs: how much of local is in global and vice versa
        double iouLocalContainment = 0.;  // What fraction of local voxels overlap with global
        if (voxels2_coarse.size() > 0)
        {
            iouLocalContainment = (double)intersection_.size() / voxels2_coarse.size();
        }
        double iouGlobalContainment = 0.;  // What fraction of global voxels overlap with local
        if (voxels1_coarse.size() > 0)
        {
            iouGlobalContainment = (double)intersection_.size() / voxels1_coarse.size();
        }
        double iouStandard = 0.;  // Standard IoU (intersection over union)
        if (union_.size() > 0)
        {
            iouStandard = (double)intersection_.size() / union_.size();
        }

        // Return the maximum of the directional IoUs and standard IoU
        // This helps detect both containment scenarios (one inside another) and partial overlaps
        double iou = std::max({iouLocalContainment, iouGlobalContainment, iouStandard});

        // VXL_INFO("IoU --> LocalContainment: {} // GlobalContainment: {} // Standard: {} // Final: {}", 
        //          iouLocalContainment, iouGlobalContainment, iouStandard, iou);

        return iou;
    }

    template <typename DataT>
    double compute3DIoU(const SemanticObject& obj1,
                        const SemanticObject& obj2,
                        bool customIoU)
    {
        std::vector<Bonxai::CoordT> voxels1 = listOfVoxelsInObject<DataT>(obj1);
        std::vector<Bonxai::CoordT> voxels2 = listOfVoxelsInObject<DataT>(obj2);

        std::set<Bonxai::CoordT> voxels1_coarse;
        std::set<Bonxai::CoordT> voxels2_coarse;

        // Use a smaller coarsening factor to preserve geometric precision
        const int COARSE_FACTOR = 1;
        
        for (size_t i = 0; i < voxels1.size(); i++)
        {
            voxels1_coarse.insert(voxels1[i] / COARSE_FACTOR);
        }

        for (size_t i = 0; i < voxels2.size(); i++)
        {
            voxels2_coarse.insert(voxels2[i] / COARSE_FACTOR);
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

        // Calculate directional IoUs (containment ratios)
        double iouContainment1 = 0.;  // What fraction of instance1 overlaps with instance2
        if (voxels1_coarse.size() > 0)
        {
            iouContainment1 = (double)intersection_.size() / voxels1_coarse.size();
        }
        double iouContainment2 = 0.;  // What fraction of instance2 overlaps with instance1
        if (voxels2_coarse.size() > 0)
        {
            iouContainment2 = (double)intersection_.size() / voxels2_coarse.size();
        }
        double iouStandard = 0.;  // Standard IoU
        if (union_.size() > 0)
        {
            iouStandard = (double)intersection_.size() / union_.size();
        }

        // Return the maximum of directional IoUs and standard IoU for better overlap detection. This is the IoS (Intersection over Smaller)
        double iou = std::max({iouContainment1, iouContainment2, iouStandard});

        VXL_INFO("Containment1: {}, Containment2: {}, Standard: {}, Final IoU: {}",
                 iouContainment1, iouContainment2, iouStandard, iou);

        return iou;
    }

    template <typename DataT>
    std::vector<Bonxai::CoordT> listOfVoxelsInObject(const SemanticObject& object)
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

                    // Only consider voxels where this instance wins (most representative)
                    // instead of any voxel with any vote for this instance
                    // Convert InstanceID_t to string format "objN" to match object.instanceID
                    std::string winningInstanceID = "obj" + std::to_string(cell->data.getMostRepresentativeInstance());
                    if (winningInstanceID == object.instanceID)
                        cellsInside.push_back(coord);
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

                if (secondInstance.isStillValid() && checkBBoxIntersect(firstInstance.bbox, secondInstance.bbox))
                {
                    // Find the most probable category for each instance
                    CategoryManager::CategoryIndex firstClassIdx = CategoryManager::UNKNOWN_CATEGORY;
                    double maxFirstProb = 0.0;
                    for (const auto& [catIdx, prob] : firstInstance.alphaParamsCategories)
                    {
                        if (prob > maxFirstProb)
                        {
                            maxFirstProb = prob;
                            firstClassIdx = catIdx;
                        }
                    }
                    
                    CategoryManager::CategoryIndex secondClassIdx = CategoryManager::UNKNOWN_CATEGORY;
                    double maxSecondProb = 0.0;
                    for (const auto& [catIdx, prob] : secondInstance.alphaParamsCategories)
                    {
                        if (prob > maxSecondProb)
                        {
                            maxSecondProb = prob;
                            secondClassIdx = catIdx;
                        }
                    }
                    
                    double iou = compute3DIoU<DataT>(firstInstance, secondInstance, true);
                    
                    // Adaptive threshold based on:
                    // 1. Semantic similarity (same class = lower threshold)
                    // 2. Number of observations (more observations = more confident, need higher IoU)
                    double iouThreshold = 0.25;  // Base threshold (lowered from 0.3)
                    
                    // If both instances have the same category, be more permissive
                    bool sameCategory = (firstClassIdx == secondClassIdx && firstClassIdx != CategoryManager::UNKNOWN_CATEGORY);
                    if (sameCategory)
                    {
                        iouThreshold = 0.15;
                    }
                    
                    // For instances with many observations, require slightly higher IoU
                    // (they are more established, need stronger evidence to merge)
                    if (firstInstance.numberObservations > 10 && secondInstance.numberObservations > 10)
                    {
                        iouThreshold += 0.05;
                    }
                    
                    if (iou > iouThreshold)
                    {
                        // Fuse the second instance with the first one
                        secondInstance.pointsTo = i;
                        fuseSemanticObjects(firstInstance, secondInstance);
                        firstInstance.numberObservations += secondInstance.numberObservations;
                        VXL_INFO("Refined: Fused instance {} into {} (IoU: {}, threshold: {}, sameClass: {})",
                                 j, i, iou, iouThreshold, sameCategory);
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
    void integrateNewSemantics(const std::vector<SemanticObject>& localMap, 
                               float sensorX = 0.0f, float sensorY = 0.0f, float sensorZ = 0.0f)
    {
        uint8_t integrated = 0;
        uint8_t added = 0;

        lastMapLocalToGlobal.resize(localMap.size());
        if (globalSemanticMap.empty())
        {
            SemanticObject unknown = SemanticObject(0, localMap[0].bbox);
            unknown.alphaParamsCategories = localMap[0].alphaParamsCategories;
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

            // Find category with maximum probability for local instance
            CategoryManager::CategoryIndex localClassIdx = CategoryManager::UNKNOWN_CATEGORY;
            double maxLocalProbability = 0.0;
            for (const auto& [categoryIndex, probability] : localInstance.alphaParamsCategories)
            {
                if (probability > maxLocalProbability)
                {
                    maxLocalProbability = probability;
                    localClassIdx = categoryIndex;
                }
            }

            for (InstanceID_t globalInstanceID = 1; globalInstanceID < currentInstancesNumber; globalInstanceID++)
            {
                SemanticObject& globalInstance = globalSemanticMap[globalInstanceID];
                // Find category with maximum probability for global instance
                CategoryManager::CategoryIndex globalClassIdx = CategoryManager::UNKNOWN_CATEGORY;
                double maxGlobalProbability = 0.0;
                for (const auto& [categoryIndex, probability] : globalInstance.alphaParamsCategories)
                {
                    if (probability > maxGlobalProbability)
                    {
                        maxGlobalProbability = probability;
                        globalClassIdx = categoryIndex;
                    }
                }

                if (globalInstance.isStillValid() && checkBBoxIntersect(localInstance.bbox, globalInstance.bbox))
                {
                    double iou =
                        compute3DIoU<DataT>(globalInstance, localInstance.localGeometry.value());
                    
                    // Calculate distance from sensor to the local instance center
                    float localCenterX = (localInstance.bbox.minX + localInstance.bbox.maxX) / 2.0f;
                    float localCenterY = (localInstance.bbox.minY + localInstance.bbox.maxY) / 2.0f;
                    float localCenterZ = (localInstance.bbox.minZ + localInstance.bbox.maxZ) / 2.0f;
                    float distanceToSensor = std::sqrt(
                        (localCenterX - sensorX) * (localCenterX - sensorX) +
                        (localCenterY - sensorY) * (localCenterY - sensorY) +
                        (localCenterZ - sensorZ) * (localCenterZ - sensorZ)
                    );
                    
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
                    double effectiveIoU = iou;
                    if (localClassIdx == globalClassIdx && localClassIdx != CategoryManager::UNKNOWN_CATEGORY)
                    {
                        // Same semantic class - add bonus to make fusion more likely
                        effectiveIoU += semanticBonus;
                        
                        // Additionally, if semantic confidence is high, add extra bonus
                        double semanticConfidence = std::min(maxLocalProbability, maxGlobalProbability);
                        if (semanticConfidence > 0.7)
                        {
                            effectiveIoU += 0.05;  // Extra bonus for high confidence matches
                        }
                    }
                    
                    if (effectiveIoU > iouThreshold)
                    {
                        fuseSemanticObjects(globalInstance, localInstance);

                        lastMapLocalToGlobal[localInstanceID] = globalInstanceID;

                        globalInstance.numberObservations += 1;
                        fused = true;
                        integrated += 1;
                        // VXL_INFO("Fused local {} with global {} (dist: {:.2f}m, IoU: {:.3f}, effIoU: {:.3f}, thresh: {:.3f})",
                        //          localInstanceID, globalInstanceID, distanceToSensor, iou, effectiveIoU, iouThreshold);
                        continue;
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
                
                // Convert dynamic category probabilities to JSON
                for (const auto& [categoryIndex, probability] : globalSemanticMap[i].alphaParamsCategories)
                {
                    if (probability > 0)
                    {
                        std::string categoryName = getCategoryName(categoryIndex);
                        if (!categoryName.empty())
                        {
                            data_json["instances"][globalSemanticMap[i].instanceID]["results"][categoryName] = probability;
                        }
                    }
                }

                data_json["instances"][globalSemanticMap[i].instanceID]["n_observations"] =
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
            if (instance.pointsTo != -1){
                continue;
            }
            
            auto index_iter = data_json["instances"].find(instance.instanceID);
            if (index_iter == data_json["instances"].end())
            {
                throw std::runtime_error("Instance " + instance.instanceID + "not found in JSON data");
            }
            const nlohmann::json& instance_json = index_iter.value();
            for (const auto& [category, alpha] : instance_json["results"].items())
            {
                CategoryManager::CategoryIndex categoryIndex = getCategoryIndex(category);
                if (categoryIndex != CategoryManager::INVALID_CATEGORY)
                {
                    instance.setCategoryProbability(categoryIndex, alpha);
                }
            }
        }
    }

    nlohmann::json appearancesToJson(){
        nlohmann::json data_json;

        data_json = {};
        for (size_t i = 0; i < globalSemanticMap.size(); i++)
        {
            if (globalSemanticMap[i].isStillValid())
            {
                data_json[globalSemanticMap[i].instanceID] = {};
                data_json[globalSemanticMap[i].instanceID]["timestamps"] = {};
                for (size_t j = 0; j < globalSemanticMap[i].appearancesTimestamps.size(); j++){
                    std::string category = default_categories[j];
                    const auto& appearances_map = globalSemanticMap[i].appearancesTimestamps[j];
                    if (appearances_map.empty()){
                        continue;
                    }

                    data_json[globalSemanticMap[i].instanceID]["timestamps"][category] = nlohmann::json::array();
                    for (const auto& instancePair : appearances_map)
                    {
                        nlohmann::json instanceBbox;
                        instanceBbox["instance_id"] = instancePair.first;
                        instanceBbox["bbox"]["centerX"] = instancePair.second.centerX;
                        instanceBbox["bbox"]["centerY"] = instancePair.second.centerY;
                        instanceBbox["bbox"]["sizeX"] = instancePair.second.sizeX;
                        instanceBbox["bbox"]["sizeY"] = instancePair.second.sizeY;

                        data_json[globalSemanticMap[i].instanceID]["timestamps"][category].push_back(instanceBbox);
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