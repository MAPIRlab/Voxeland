#pragma once
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <voxeland_map/category_manager.hpp>

#include "voxeland_map/pcl_utils.hpp"

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
    // TODO (pepe) we should consider changing it to a hashmap. I think any performance impact from lookups will be compensated by not having to deal with lots of old invalid instances
    InstanceID_t instanceID;
    std::string instanceName;
    // Dynamic storage for category probabilities - grows as needed
    std::unordered_map<CategoryManager::CategoryIndex, double> alphaParamsCategories;

    // Dynamic storage for appearances per category - grows as needed
    std::unordered_map<CategoryManager::CategoryIndex, std::map<uint32_t, BoundingBox2D>> appearancesTimestamps;
    uint32_t numberObservations = 1;
    BoundingBox3D bbox;

    std::optional<std::set<Bonxai::IndicesT>> localGeometry;

    int32_t pointsTo = -1;  // In case a semantic object is integrated with another, the pointsTo variable
    // need to be set with the instanceID of the main object, hence if pointsTo is not empty, it won't check
    // the data in this SemanticObject, but instead it will check the data in the instanceID set in pointsTo.

    // this serves as an indication that the instance might be under-segmented
    // it is derived from the difference between the IoV and the IoS
    float underSegmentScore = 0.f;

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
        return it->first;
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

    double getSumAlphas() const
    {
        double sum = 0;
        for (const auto& [cat, alpha] : alphaParamsCategories)
            sum += alpha;
        return sum;
    }

    bool isLocalInstance() const
    {
        return instanceID == -1;
    }
};
