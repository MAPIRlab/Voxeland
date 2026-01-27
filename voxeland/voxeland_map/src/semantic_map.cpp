#include <voxeland_map/cell_types.hpp>
#include <voxeland_map/semantic_map.hpp>
#include <voxeland_map/category_manager.hpp>

SemanticMap::SemanticMap()
    : kld_threshold(0.1f)
    , color_palette({ 0xFAD4E0, 0x9DBBE3, 0xBFE3DF, 0xB59CD9, 0xFFF5CC, 0xFFD9BD, 0xEE9D94, 0xF7ADCF,
                      0xe6194B, 0x3cb44b, 0xffe119, 0x4363d8, 0xf58231, 0x911eb4, 0x42d4f4, 0xf032e6,
                      0xbfef45, 0xfabed4, 0x469990, 0xdcbeff, 0x9A6324, 0xfffac8 })
{}

/* COLOR PALETTES */

/* PASTEL COLORS: {
    0xFAD4E0, 0x9DBBE3, 0xBFE3DF, 0xB59CD9, 0xFFF5CC, 0xFFD9BD, 0xEE9D94, 0xF7ADCF,
    0xe6194B, 0x3cb44b, 0xffe119, 0x4363d8, 0xf58231, 0x911eb4, 0x42d4f4,
    0xf032e6, 0xbfef45, 0xfabed4, 0x469990, 0xdcbeff, 0x9A6324, 0xfffac8
  };
*/

/* NORMAL COLORS: {
    0xe6194B, 0x3cb44b, 0xffe119, 0x4363d8, 0xf58231, 0x911eb4, 0x42d4f4,
    0xf032e6, 0xbfef45, 0xfabed4, 0x469990, 0xdcbeff, 0x9A6324, 0xfffac8,
    0x800000, 0xaaffc3, 0x808000, 0xffd8b1, 0x000075
  };
*/

void SemanticMap::initialize(std::vector<std::string> dataset_categories,
                             Bonxai::ProbabilisticMap& _bonxai,
                             voxeland::DataMode mode)
{
    // Initialize CategoryManager with dataset categories
    CategoryManager& catManager = CategoryManager::getInstance();
    catManager.initializeWithCategories(dataset_categories);
    
    AUTO_TEMPLATE_SEMANTICS_ONLY(mode, BonxaiQuery<DataT>::createAccessor(_bonxai.With<DataT>()));
    initialized = true;
}

void SemanticMap::setLocalSemanticMap(const std::vector<SemanticObject>& localMap)
{
    lastLocalSemanticMap = localMap;
}

InstanceID_t SemanticMap::localToGlobalInstance(InstanceID_t localInstance)
{
    return lastMapLocalToGlobal[localInstance];
}

uint32_t SemanticMap::indexToHexColor(InstanceID_t index)
{
    if (index == CategoryManager::UNKNOWN_CATEGORY)
        return 0xbcbcbc;

    return color_palette[index % color_palette.size()];
}

void SemanticMap::updateCategoryProbability(SemanticObject& semanticObject,
                                            const std::string& categoryName,
                                            double probability)
{
    CategoryManager& catManager = CategoryManager::getInstance();
    CategoryManager::CategoryIndex categoryIndex = catManager.addCategory(categoryName);
        
    semanticObject.addToCategoryAlpha(categoryIndex, probability);
}

CategoryManager::CategoryIndex SemanticMap::addCategory(const std::string& categoryName)
{
    CategoryManager& catManager = CategoryManager::getInstance();
    CategoryManager::CategoryIndex index = catManager.addCategory(categoryName);
    
    return index;
}

CategoryManager::CategoryIndex SemanticMap::getCategoryIndex(const std::string& categoryName) const
{
    CategoryManager& catManager = CategoryManager::getInstance();
    return catManager.getCategoryIndex(categoryName);
}

std::string SemanticMap::getCategoryName(CategoryManager::CategoryIndex index) const
{
    CategoryManager& catManager = CategoryManager::getInstance();
    return catManager.getCategoryName(index);
}

size_t SemanticMap::getNumCategories() const
{
    CategoryManager& catManager = CategoryManager::getInstance();
    return catManager.getNumCategories();
}

double SemanticMap::computeKLD(const std::vector<double>& P, const std::vector<double>& Q)
{
    if (P.size() != Q.size())
    {
        std::cerr << "Error: Vectors must be of equal length\n";
        return false;
    }

    double kld = 0.0;
    for (size_t i = 0; i < P.size(); ++i)
    {
        if (P[i] == 0)  // To avoid log(0)
            continue;
        if (Q[i] == 0)  // Handle when Q[i] = 0
            return false;

        kld += P[i] * log(P[i] / Q[i]);
    }
    kld = std::abs(kld);  // Absolute value of KLD

    if (kld < kld_threshold)
    {
        return kld;
    }
    else
    {
        return 0.0;
    }
}

bool SemanticMap::checkBBoxIntersect(const BoundingBox3D& bbox1, const BoundingBox3D& bbox2)
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

void SemanticMap::updateAlphaCategories(SemanticObject& original, const SemanticObject& update)
{
    // Merge category probabilities from update into original
    for (const auto& [categoryIndex, probability] : update.alphaParamsCategories)
    {
        original.alphaParamsCategories[categoryIndex] += probability;
    }
}

void SemanticMap::updateBBoxBounds(BoundingBox3D& original, const BoundingBox3D& update)
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

void SemanticMap::updateAppearancesTimestamps(SemanticObject& original, const SemanticObject& update){
    
    // Merge appearances timestamps from update into original
    for (const auto& [categoryIndex, timestampMap] : update.appearancesTimestamps) {
        // Add all timestamps from the update to the original
        original.appearancesTimestamps[categoryIndex].insert(
            timestampMap.begin(),
            timestampMap.end()
        );
    }
}


/**
 * @brief Integrates the sencondInstance info into the firstInstance. That includes alphas, bbox and appearances
 *
 * @param firstInstance : the instance to be updated
 * @param secondInstance : the instance to be integrated
*/
void SemanticMap::fuseSemanticObjects(SemanticObject& firstInstance, const SemanticObject& secondInstance)
{    
    // Integrate alpha semantics
    updateAlphaCategories(firstInstance, secondInstance);

    // Update Bounding Box
    updateBBoxBounds(firstInstance.bbox, secondInstance.bbox);

    // Update appearances timestamps
    updateAppearancesTimestamps(firstInstance, secondInstance);

}

nlohmann::json SemanticMap::mapToJSON()
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
void SemanticMap::updateSemanticMapResultsFromJSON(const nlohmann::json& data_json)
{
    if (!initialized)
    {
        throw std::runtime_error("SemanticMap is not initialized.");
    }

    for (SemanticObject& instance : globalSemanticMap)
    {
        if (!instance.isStillValid())
            continue;

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
nlohmann::json SemanticMap::appearancesToJson()
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
