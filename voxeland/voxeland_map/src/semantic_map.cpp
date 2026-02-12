#include <cmath>
#include <set>
#include <voxeland_map/category_manager.hpp>
#include <voxeland_map/cell_types.hpp>
#include <voxeland_map/semantic_map.hpp>

SemanticMap::SemanticMap()
    : kld_threshold(0.1f)
    , color_palette({ 0xFAD4E0, 0x9DBBE3, 0xBFE3DF, 0xB59CD9, 0xFFF5CC, 0xFFD9BD, 0xEE9D94, 0xF7ADCF, 0xe6194B, 0x3cb44b, 0xffe119, 0x4363d8, 0xf58231, 0x911eb4, 0x42d4f4, 0xf032e6, 0xbfef45, 0xfabed4, 0x469990, 0xdcbeff, 0x9A6324, 0xfffac8 })
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

    currentMode = mode;
    AUTO_TEMPLATE_SEMANTICS_ONLY(mode, BonxaiQuery<DataT>::createAccessor(_bonxai.With<DataT>()));
    initialized = true;
}

void SemanticMap::integrateNewSemantics(const std::vector<SemanticObject>& localMap,
                                        const std::set<Bonxai::CoordT>& voxelizedLocalPointCloud,
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
        const std::set<Bonxai::CoordT>& voxelsLocal = *localInstance.localGeometry;
        std::map<InstanceID_t, std::set<Bonxai::CoordT>> globalsGeometry;  // cache the voxels for each global object to avoid repeated lookup

        // Find category with maximum probability for local instance
        CategoryManager::CategoryIndex localMaxCategory = localInstance.mostLikelyCategory();

        for (InstanceID_t globalInstanceID = 1; globalInstanceID < currentInstancesNumber; globalInstanceID++)
        {
            SemanticObject& globalInstance = globalSemanticMap[globalInstanceID];
            // Find category with maximum probability for global instance
            CategoryManager::CategoryIndex globalMaxCategory = globalInstance.mostLikelyCategory();

            if (globalInstance.isStillValid() && checkBBoxIntersect(localInstance.bbox, globalInstance.bbox))
            {
                // get all the voxels that belong to the global instance
                std::set<Bonxai::CoordT> voxelsGlobal;
                if (globalsGeometry.contains(globalInstanceID))
                    voxelsGlobal = globalsGeometry.at(globalInstanceID);
                else
                {
                    AUTO_TEMPLATE_INSTANCES_ONLY(currentMode, voxelsGlobal = listOfVoxelsInObject<DataT>(globalInstance));
                    globalsGeometry.insert({ globalInstanceID, voxelsGlobal });
                }

                auto [iou, ios] = compute3DIoU(voxelsGlobal, voxelsLocal, 2);

                double iov = computeIoV(voxelizedLocalPointCloud, voxelsGlobal, voxelsLocal, 1);

                // ============================================================
                // HYBRID FUSION: IoV + Jensen-Shannon Semantic Similarity
                // Combines geometric (IoU, IoS, IoV) with semantic evidence
                // ============================================================

                // Calculate distance from sensor to the local instance center
                float localCenterX = (localInstance.bbox.minX + localInstance.bbox.maxX) / 2.0f;
                float localCenterY = (localInstance.bbox.minY + localInstance.bbox.maxY) / 2.0f;
                float localCenterZ = (localInstance.bbox.minZ + localInstance.bbox.maxZ) / 2.0f;
                float distanceToSensor = std::sqrt(
                    (localCenterX - sensorX) * (localCenterX - sensorX) +
                    (localCenterY - sensorY) * (localCenterY - sensorY) +
                    (localCenterZ - sensorZ) * (localCenterZ - sensorZ));

                // Compute semantic similarity using Jensen-Shannon divergence
                // This compares the FULL probability distributions, not just the top class
                double semanticSimilarity = computeSemanticSimilarity(localInstance, globalInstance);

                // Base thresholds vary with distance:
                // Close objects (< 1.5m): high threshold - precise segmentation expected
                // Medium distance (1.5-3m): medium threshold
                // Far objects (> 3m): low threshold - rely more on semantics
                double baseIovThreshold;
                double semanticWeight;  // How much to weight semantic similarity vs geometric metrics

                if (distanceToSensor < 1.5f)
                {
                    baseIovThreshold = 0.35;
                    semanticWeight = 0.2;  // Trust geometry more when close
                }
                else if (distanceToSensor < 3.0f)
                {
                    baseIovThreshold = 0.25;
                    semanticWeight = 0.35;  // Balanced
                }
                else
                {
                    baseIovThreshold = 0.15;
                    semanticWeight = 0.5;  // Trust semantics more when far
                }

                // Compute combined fusion score using IoV (better than IoU for partial observations):
                // fusionScore = (1 - semanticWeight) * IoV + semanticWeight * semanticSimilarity
                // This creates a weighted combination of geometric and semantic evidence
                double fusionScore = (1.0 - semanticWeight) * iov + semanticWeight * semanticSimilarity;

                // Adaptive threshold based on semantic similarity:
                // - High semantic similarity (>0.8): lower the effective threshold (easier to fuse)
                // - Low semantic similarity (<0.3): raise the effective threshold (harder to fuse)
                double thresholdModifier = 0.0;
                if (semanticSimilarity > 0.8)
                {
                    // Very similar distributions - reduce threshold by up to 0.1
                    thresholdModifier = -0.1 * (semanticSimilarity - 0.8) / 0.2;
                }
                else if (semanticSimilarity < 0.3)
                {
                    // Very different distributions - increase threshold by up to 0.15
                    thresholdModifier = 0.15 * (0.3 - semanticSimilarity) / 0.3;
                }

                double fusionThreshold = baseIovThreshold + thresholdModifier;

                // Additional bonus for high-confidence semantic matches
                // (when both objects are confident about the same category)
                if (localMaxCategory == globalMaxCategory)
                {
                    double localConfidence = localInstance.getCategoryAlpha(localMaxCategory);
                    double globalConfidence = globalInstance.getCategoryAlpha(globalMaxCategory);

                    // Normalize confidences
                    double localSum = localInstance.getSumAlphas();
                    double globalSum = globalInstance.getSumAlphas();

                    double localProb = localSum > 0 ? localConfidence / localSum : 0;
                    double globalProb = globalSum > 0 ? globalConfidence / globalSum : 0;

                    // If both have high confidence in the same class, add bonus to fusion score
                    if (localProb > 0.5 && globalProb > 0.5)
                    {
                        fusionScore += 0.05 * std::min(localProb, globalProb);
                    }
                }

                if (fusionScore > fusionThreshold)
                {
                    VXL_DEBUG(fmt::fg(fmt::terminal_color::yellow), "Fusing local {} - global {}:\n\tIoU:{:.2f}  IoS:{:.2f}  IoV:{:.2f}  SemSim:{:.2f}  Score:{:.2f}", localInstanceID, globalInstanceID, iou, ios, iov, semanticSimilarity, fusionScore);
                    PAUSE_THREAD_UNTIL_GUI_CONTINUE;
                    fuseSemanticObjects(globalInstance, localInstance);

                    lastMapLocalToGlobal[localInstanceID] = globalInstanceID;

                    fused = true;
                    globalInstance.numberObservations++;
                    integrated++;
                    break;  // don't keep iterating over the globals, we are done with this local instance
                }
                else
                {
                    VXL_DEBUG("NOT Fusing local {} - global {}:\n\tIoU:{:.2f}  IoS:{:.2f}  IoV:{:.2f}  SemSim:{:.2f}  Score:{:.2f}",
                              localInstanceID,
                              globalInstanceID,
                              iou,
                              ios,
                              iov,
                              semanticSimilarity,
                              fusionScore);

                    // Track potential under-segmentation using IoS vs IoV difference
                    // If IoS is high but IoV is low, it suggests the local observation
                    // covers part of a larger global object (under-segmentation)
                    double iosThreshold = baseIovThreshold;  // Use same base threshold
                    if (ios > iosThreshold)
                    {
                        globalInstance.underSegmentScore += ios - iov;
                        globalInstance.numberObservations++;
                    }
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

void SemanticMap::refineGlobalSemanticMap(int nObservationsToRemove)
{
    // cache the voxels for each global object to avoid repeated lookup
    std::map<InstanceID_t, std::set<Bonxai::CoordT>> geometry;

    for (InstanceID_t i = 1; i < globalSemanticMap.size(); i++)
    {
        if (globalSemanticMap.at(i).isStillValid())
        {
            std::set<Bonxai::CoordT> voxelsGlobal;
            AUTO_TEMPLATE_INSTANCES_ONLY(currentMode,
                                         voxelsGlobal = listOfVoxelsInObject<DataT>(globalSemanticMap.at(i)));

            // remove instances with very few observations
            if (globalSemanticMap.at(i).numberObservations <= nObservationsToRemove || voxelsGlobal.size() == 0)
                globalSemanticMap.at(i).pointsTo = 0;
            else
                geometry.insert({ i, voxelsGlobal });
        }
    }

    for (InstanceID_t i = 1; i < globalSemanticMap.size(); i++)
    {
        SemanticObject& firstInstance = globalSemanticMap[i];

        if (!firstInstance.isStillValid())
            continue;

        std::set<Bonxai::CoordT> voxelsFirst = geometry.at(i);
        CategoryManager::CategoryIndex firstClassIdx = firstInstance.mostLikelyCategory();

        for (InstanceID_t j = i + 1; j < globalSemanticMap.size(); j++)
        {
            SemanticObject& secondInstance = globalSemanticMap[j];

            if (secondInstance.isStillValid() && checkBBoxIntersect(firstInstance.bbox, secondInstance.bbox))
            {
                std::set<Bonxai::CoordT> voxelsSecond = geometry.at(j);

                // Adaptive threshold based on:
                // 1. Semantic similarity (same class = lower threshold)
                // 2. Number of observations (more observations = more confident, need higher IoU)
                double iouThreshold = 0.6;  // Base threshold

                CategoryManager::CategoryIndex secondClassIdx = secondInstance.mostLikelyCategory();

                // If both instances have the same category, be more permissive
                bool sameCategory = firstClassIdx == secondClassIdx;
                if (sameCategory)
                    iouThreshold = 0.2;

                auto [iou, ios] = compute3DIoU(voxelsFirst, voxelsSecond, 3);
                if (iou > iouThreshold)
                {
                    // Fuse the second instance with the first one
                    secondInstance.pointsTo = i;
                    VXL_DEBUG(fmt::fg(fmt::terminal_color::yellow), "(Refine) Fusing global {} - global {}:\n\tIoU:{:.2f}  IoS: {:.2f}", i, j, iou, ios);
                    PAUSE_THREAD_UNTIL_GUI_CONTINUE;
                    fuseSemanticObjects(firstInstance, secondInstance);
                    break;
                }
                else
                    VXL_DEBUG("(Refine) NOT Fusing global {} - global {}:\n\tIoU:{:.2f}  IoS: {:.2f}", i, j, iou, ios);
            }
        }
    }
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

double SemanticMap::computeSemanticSimilarity(const SemanticObject& obj1, const SemanticObject& obj2)
{
    // Collect all categories present in either object
    std::set<CategoryManager::CategoryIndex> allCategories;
    for (const auto& [cat, _] : obj1.alphaParamsCategories)
        allCategories.insert(cat);
    for (const auto& [cat, _] : obj2.alphaParamsCategories)
        allCategories.insert(cat);

    if (allCategories.empty())
        return 0.0;  // No semantic information available

    // Compute total alpha sums for normalization
    double sum1 = obj1.getSumAlphas(), sum2 = obj2.getSumAlphas();

    if (sum1 <= 0.0 || sum2 <= 0.0)
        return 0.0;  // Invalid distributions

    // Build normalized probability distributions P and Q
    // Use small epsilon to avoid division by zero in KL computation
    const double epsilon = 1e-10;
    std::vector<double> P, Q, M;

    for (const auto& cat : allCategories)
    {
        double p = std::max(obj1.getCategoryAlpha(cat) / sum1, epsilon);
        double q = std::max(obj2.getCategoryAlpha(cat) / sum2, epsilon);

        P.push_back(p);
        Q.push_back(q);
        M.push_back((p + q) / 2.0);  // Midpoint distribution for JS divergence
    }

    // Compute Jensen-Shannon divergence: JS(P||Q) = 0.5 * KL(P||M) + 0.5 * KL(Q||M)
    double kl_pm = 0.0, kl_qm = 0.0;

    for (size_t i = 0; i < P.size(); ++i)
    {
        if (P[i] > epsilon)
            kl_pm += P[i] * std::log(P[i] / M[i]);
        if (Q[i] > epsilon)
            kl_qm += Q[i] * std::log(Q[i] / M[i]);
    }

    double js_divergence = 0.5 * kl_pm + 0.5 * kl_qm;

    // JS divergence is bounded [0, ln(2)] when using natural log
    // Normalize to [0, 1] and convert to similarity (1 - normalized_divergence)
    double js_normalized = js_divergence / std::log(2.0);
    double similarity = 1.0 - std::min(1.0, std::max(0.0, js_normalized));

    return similarity;
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

void SemanticMap::updateAppearancesTimestamps(SemanticObject& original, const SemanticObject& update)
{
    // Merge appearances timestamps from update into original
    for (const auto& [categoryIndex, timestampMap] : update.appearancesTimestamps)
    {
        // Add all timestamps from the update to the original
        original.appearancesTimestamps[categoryIndex].insert(
            timestampMap.begin(),
            timestampMap.end());
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

    firstInstance.numberObservations += secondInstance.numberObservations;
    firstInstance.underSegmentScore += secondInstance.underSegmentScore;
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

std::set<Bonxai::CoordT> SemanticMap::coarsenVoxels(const std::set<Bonxai::CoordT>& voxels, uint coarsening_factor)
{
    if (coarsening_factor <= 1)
        return voxels;
    std::set<Bonxai::CoordT> voxels_coarse;

    for (const auto& coord : voxels)
        voxels_coarse.insert(coord / coarsening_factor);

    return voxels_coarse;
}

std::pair<double, double> SemanticMap::compute3DIoU(const std::set<Bonxai::CoordT>& voxels1,
                                                    const std::set<Bonxai::CoordT>& voxels2,
                                                    uint coarsening_factor)
{
    std::set<Bonxai::CoordT> voxels1_coarse = coarsenVoxels(voxels1, coarsening_factor);
    std::set<Bonxai::CoordT> voxels2_coarse = coarsenVoxels(voxels2, coarsening_factor);

    std::vector<Bonxai::CoordT> intersection_;
    std::vector<Bonxai::CoordT> union_;

    std::set_intersection(voxels1_coarse.begin(),
                          voxels1_coarse.end(),
                          voxels2_coarse.begin(),
                          voxels2_coarse.end(),
                          std::back_inserter(intersection_));
    std::set_union(voxels1_coarse.begin(),
                   voxels1_coarse.end(),
                   voxels2_coarse.begin(),
                   voxels2_coarse.end(),
                   std::back_inserter(union_));

    double IoU = 0.;
    if (union_.size() > 0)
        IoU = ((double)intersection_.size()) / union_.size();

    double IoS = 0.;
    if (voxels1_coarse.size() > 0)
        IoS = ((double)intersection_.size()) / voxels1_coarse.size();
    if (voxels2_coarse.size() > 0)
        IoS = std::max(IoS, ((double)intersection_.size()) / voxels2_coarse.size());

    return std::pair<double, double>(IoU, IoS);
}

double SemanticMap::computeIoV(const std::set<Bonxai::CoordT>& localVoxels,
                               const std::set<Bonxai::CoordT>& globalInstance,
                               const std::set<Bonxai::CoordT>& localInstance,
                               uint coarsening_factor)
{
    std::set<Bonxai::CoordT> localVoxels_coarse = coarsenVoxels(localVoxels, coarsening_factor);

    // find all the voxels in the global instance which were visible in this image
    std::set<Bonxai::CoordT> visibleGlobalVoxels;
    std::set_intersection(globalInstance.begin(),
                          globalInstance.end(),
                          localVoxels_coarse.begin(),
                          localVoxels_coarse.end(),
                          std::inserter(visibleGlobalVoxels, visibleGlobalVoxels.begin()));
    size_t numVisibleVoxels = visibleGlobalVoxels.size();

    // find which of the visible voxels were identified as part of this local instance
    std::set<Bonxai::CoordT> globalVoxelsInMask;
    std::set_intersection(visibleGlobalVoxels.begin(),
                          visibleGlobalVoxels.end(),
                          localInstance.begin(),
                          localInstance.end(),
                          std::inserter(globalVoxelsInMask, globalVoxelsInMask.begin()));
    size_t numVoxelsInMask = globalVoxelsInMask.size();

    double iov = numVisibleVoxels > 0 ? numVoxelsInMask / static_cast<double>(numVisibleVoxels) : 0;
    return iov;
}