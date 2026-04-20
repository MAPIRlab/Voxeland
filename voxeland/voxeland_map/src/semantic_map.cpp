#include <cmath>
#include <random>
#include <set>
#include <voxeland_map/category_manager.hpp>
#include <voxeland_map/cell_types.hpp>
#include <voxeland_map/semantic_map.hpp>

#include "voxeland_map/geometry_operations.hpp"

SemanticMap::SemanticMap()
    : kld_threshold(0.1f)
    , color_palette({ 0xFAD4E0, 0x9DBBE3, 0xBFE3DF, 0xB59CD9, 0xFFF5CC, 0xFFD9BD, 0xEE9D94, 0xF7ADCF, 0xe6194B, 0x3cb44b, 0xffe119, 0x4363d8, 0xf58231, 0x911eb4, 0x42d4f4, 0xf032e6, 0xbfef45, 0xfabed4, 0x469990, 0xdcbeff, 0x9A6324, 0xfffac8 })
{
    color_palette_offsets.resize(1000, 0);
}

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
                                        const std::set<Bonxai::IndicesT>& voxelizedLocalPointCloud,
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
        std::optional<FusionScore> bestScore;
        const SemanticObject& localInstance = localMap[localInstanceID];

        if (!localInstance.localGeometry.has_value())
            continue;
        const std::set<Bonxai::IndicesT>& voxelsLocal = *localInstance.localGeometry;
        std::map<InstanceID_t, std::set<Bonxai::IndicesT>> globalsGeometry;  // cache the voxels for each global object to avoid repeated lookup

        // Find category with maximum probability for local instance
        CategoryManager::CategoryIndex localMaxCategory = localInstance.mostLikelyCategory();

        for (InstanceID_t globalInstanceID = 1; globalInstanceID < currentInstancesNumber; globalInstanceID++)
        {
            SemanticObject& globalInstance = globalSemanticMap[globalInstanceID];
            // Find category with maximum probability for global instance
            CategoryManager::CategoryIndex globalMaxCategory = globalInstance.mostLikelyCategory();

            if (!globalInstance.isStillValid() || !GeometryOperations::CheckBBoxIntersect(localInstance.bbox, globalInstance.bbox))
                continue;

            // get all the voxels that belong to the global instance
            std::set<Bonxai::IndicesT> voxelsGlobal;
            if (globalsGeometry.contains(globalInstanceID))
                voxelsGlobal = globalsGeometry.at(globalInstanceID);
            else
            {
                AUTO_TEMPLATE_INSTANCES_ONLY(currentMode, voxelsGlobal = listOfVoxelsInObject<DataT>(globalInstance));
                globalsGeometry.insert({ globalInstanceID, voxelsGlobal });
            }

            auto [iou, ios] = compute3DIoU(voxelsGlobal, voxelsLocal, 2);

            double iov = computeIoV(voxelizedLocalPointCloud, voxelsGlobal, voxelsLocal, 2);

            // ============================================================
            // HYBRID FUSION: IoV + Jensen-Shannon Semantic Similarity
            // Combines geometric (IoU, IoS, IoV) with semantic evidence
            // ============================================================

            // Compute semantic similarity using Jensen-Shannon divergence
            // This compares the FULL probability distributions, not just the top class
            constexpr float minIOV = 0.6;
            constexpr float maxIOV = 0.95;

            double semanticSimilarity = computeSemanticSimilarity(localInstance, globalInstance);

            double fusionThreshold = std::lerp(maxIOV, minIOV, semanticSimilarity);
            double nFusionScore = iov / fusionThreshold;

            if (nFusionScore >= 1.0)
            {
                if (!bestScore || nFusionScore > bestScore->normalizedScore)
                {
                    bestScore = FusionScore{ .fuseWithID = globalInstanceID, .normalizedScore = nFusionScore };
                    VXL_DEBUG(fmt::fg(fmt::terminal_color::yellow),
                              "Integrating local {} - global {}:\n\tIoU:{:.2f}  IoS:{:.2f}  IoV:{:.2f}  SemSim:{:.2f}",
                              localInstanceID,
                              globalInstanceID,
                              iou,
                              ios,
                              iov,
                              semanticSimilarity);
                    PAUSE_THREAD_UNTIL_GUI_CONTINUE(debugging_utils::pause_on_integration);
                }
            }
            else
            {
                VXL_DEBUG("NOT Fusing local {} - global {}:\n\tIoU:{:.2f}  IoS:{:.2f}  IoV:{:.2f}  SemSim:{:.2f}",
                          localInstanceID,
                          globalInstanceID,
                          iou,
                          ios,
                          iov,
                          semanticSimilarity);

                // Track potential under-segmentation using IoS vs IoV difference
                // If IoS is high but IoV is low, it suggests the local observation covers part of a larger global object (under-segmentation)
                if (ios > fusionThreshold)
                {
                    globalInstance.underSegmentScore += ios - iov;
                    globalInstance.numberObservations++;
                }
            }
        }

        if (bestScore.has_value())
        {
            SemanticObject& globalInstance = globalSemanticMap.at(bestScore->fuseWithID);
            fuseSemanticObjects(globalInstance, localInstance);

            lastMapLocalToGlobal[localInstanceID] = globalInstance.instanceID;

            globalInstance.numberObservations++;
            integrated++;
        }
        else
        {
            // Create new object integrating localMap information
            SemanticObject& newObject = CreateGlobalInstance();
            newObject.bbox = localInstance.bbox;
            newObject.alphaParamsCategories = localInstance.alphaParamsCategories;
            newObject.appearancesTimestamps = localInstance.appearancesTimestamps;

            lastMapLocalToGlobal[localInstanceID] = newObject.instanceID;
            VXL_DEBUG("Created global instance {} from local {}", newObject.instanceID, localInstanceID);
            added++;
        }
    }
    VXL_INFO("Integrating {} new local objects: {} integrated and {} added", localMap.size(), integrated, added);
}

void SemanticMap::refineGlobalSemanticMap(int nObservationsToRemove)
{
    // cache the voxels for each global object to avoid repeated lookup
    std::map<InstanceID_t, std::set<Bonxai::IndicesT>> geometry;

    for (InstanceID_t i = 1; i < globalSemanticMap.size(); i++)
    {
        if (globalSemanticMap.at(i).isStillValid())
        {
            std::set<Bonxai::IndicesT> voxelsGlobal;
            AUTO_TEMPLATE_INSTANCES_ONLY(currentMode,
                                         voxelsGlobal = listOfVoxelsInObject<DataT>(globalSemanticMap.at(i), 1));

            // remove instances with very few observations
            if (globalSemanticMap.at(i).numberObservations <= nObservationsToRemove || voxelsGlobal.size() == 0)
                globalSemanticMap.at(i).pointsTo = 0;
            else
                geometry.insert({ i, voxelsGlobal });
        }
    }

    // iterate over individual instances, see if they need to be split into smaller chunks
    for (InstanceID_t startInstIdx = 1; startInstIdx < globalSemanticMap.size(); startInstIdx++)
    {
        if (!globalSemanticMap[startInstIdx].isStillValid())
            continue;
        std::vector<std::set<Bonxai::IndicesT>> clusters = GeometryOperations::ClusterVoxelCloud(geometry.at(startInstIdx));

        auto filterLoosePoints = [&](const std::set<Bonxai::IndicesT>& validVoxels) {
            for (const auto& voxel : geometry.at(startInstIdx))
            {
                AUTO_TEMPLATE_INSTANCES_ONLY(currentMode,
                                             {
                                                 Bonxai::ProbabilisticCell<DataT>* cell = BonxaiQuery<DataT>::getAccessor().value(voxel);
                                                 if (!validVoxels.contains(voxel))
                                                     cell->data.ReplaceInstanceVotes(startInstIdx, 0);
                                             });
            }
        };

        // if it's all disperse points, this instance is cooked
        if (clusters.size() == 0)
        {
            globalSemanticMap[startInstIdx].pointsTo = 0;
            continue;
        }
        else if (clusters.size() == 1)
        {
            // update the geometry to remove any loose points
            auto validVoxels = clusters.at(0);
            filterLoosePoints(clusters.at(0));
            geometry.at(startInstIdx) = clusters.at(0);
        }
        else
        {
            // more than one chunk, let's split it into multiple instances
            debugInfo.mostRecentClusters = clusters;
            VXL_DEBUG("Splitting instance {} into {} chunks", startInstIdx, clusters.size());
            PAUSE_THREAD_UNTIL_GUI_CONTINUE(debugging_utils::pause_on_splitting);

            // remove loose points
            std::set<Bonxai::IndicesT> _union;
            for (size_t clusterIdx = 0; clusterIdx < clusters.size(); clusterIdx++)
                _union = GeometryOperations::SetUnion(_union, clusters.at(clusterIdx));
            filterLoosePoints(_union);

            // the first cluster will be assigned to the old instance ID
            geometry[startInstIdx] = clusters.at(0);

            // every other cluster has now been promoted to being its own instance
            for (size_t clusterIdx = 1; clusterIdx < clusters.size(); clusterIdx++)
            {
                auto thisCluster = clusters.at(clusterIdx);
                SemanticObject& newObject = CreateGlobalInstance();
                newObject.bbox = GeometryOperations::FindBBox(thisCluster);
                newObject.alphaParamsCategories = globalSemanticMap[startInstIdx].alphaParamsCategories;
                newObject.appearancesTimestamps = globalSemanticMap[startInstIdx].appearancesTimestamps;
                VXL_ASSERT(newObject.alphaParamsCategories.size() > 0);

                // update the cache
                geometry.insert({ newObject.instanceID, thisCluster });

                // update the votes on each of the voxels to point to the new instance
                for (const auto& voxel : thisCluster)
                {
                    AUTO_TEMPLATE_INSTANCES_ONLY(currentMode,
                                                 {
                                                     Bonxai::ProbabilisticCell<DataT>* cell = BonxaiQuery<DataT>::getAccessor().value(voxel);
                                                     cell->data.ReplaceInstanceVotes(startInstIdx, newObject.instanceID);
                                                 });
                }
            }
        }
    }

    // iterate over instance pairs, try to fuse them into bigger chunks
    for (InstanceID_t firstIdx = 1; firstIdx < globalSemanticMap.size(); firstIdx++)
    {
        // fusion parameters
        constexpr float semSimThr = 0.6;      // how similar the class distributions must be to allow fusing
        constexpr float votesThr = 0.3;       // when retrieving the geometry that corresponds to this instance, which proportion of votes must a voxel have to count
        constexpr uint coarseningFactor = 4;  // downsampling factor for the pointclouds when calculating IoU
        constexpr float iosThr = 0.15;        // exactly what you think this is
        constexpr float iouSkipThr = 0.7;     // if IoU is sufficiently large, the instances are overlapping entirely and we don't care about the other metrics.
                                              // This is necessary because we can sometimes end up with two overlapping instances which correspond to one class each,
                                              // and every new observation always fuses with the instance which already agrees with its class.
                                              // This can lead to a very low semantic similarity, preventing fusion

        SemanticObject& firstInstance = globalSemanticMap[firstIdx];

        if (!firstInstance.isStillValid())
            continue;

        std::set<Bonxai::IndicesT> voxelsFirst = geometry.at(firstIdx);

        for (InstanceID_t secondIdx = firstIdx + 1; secondIdx < globalSemanticMap.size(); secondIdx++)
        {
            SemanticObject& secondInstance = globalSemanticMap[secondIdx];

            if (!secondInstance.isStillValid())
                continue;
            if (GeometryOperations::CheckBBoxIntersect(firstInstance.bbox, secondInstance.bbox))
            {
                std::set<Bonxai::IndicesT> voxelsSecond = geometry.at(secondIdx);

                bool IoUSkip = false;  // if IoU is huge, don't bother with any other checks: the instances correspond to the same geometry!

                // IoU check
                float iou, ios;
                {
                    // TODO we are not caching the geometry used for this (with the votesThr). Check performance to see if it's worth bothering
                    std::set<Bonxai::IndicesT> voxelsSomeVotesFirst;
                    std::set<Bonxai::IndicesT> voxelsSomeVotesSecond;
                    AUTO_TEMPLATE_INSTANCES_ONLY(currentMode, voxelsSomeVotesFirst = listOfVoxelsInObject<DataT>(globalSemanticMap.at(firstIdx), votesThr));
                    AUTO_TEMPLATE_INSTANCES_ONLY(currentMode, voxelsSomeVotesSecond = listOfVoxelsInObject<DataT>(globalSemanticMap.at(secondIdx), votesThr));
                    std::tie(iou, ios) = compute3DIoU(voxelsSomeVotesFirst, voxelsSomeVotesSecond, coarseningFactor);

                    if (iou > iouSkipThr)
                    {
                        IoUSkip = true;
                        VXL_DEBUG("Fusing global {} - global {}.  IoU above passthrough threshold: {}", firstIdx, secondIdx, iou);
                    }

                    if (!IoUSkip && ios < iosThr)
                    {
                        VXL_DEBUG("(Refine) NOT Fusing global {} - global {}.  Insufficient IoS: {}", firstIdx, secondIdx, ios);
                        continue;
                    }
                }

                // semantics check
                double semSim = computeSemanticSimilarity(firstInstance, secondInstance);
                if (!IoUSkip && semSim < semSimThr)
                {
                    VXL_DEBUG("(Refine) NOT Fusing global {} - global {}.  Insufficient SemSim: {:.2f}", firstIdx, secondIdx, semSim);
                    continue;
                }

                std::set<Bonxai::IndicesT> _union = GeometryOperations::SetUnion(voxelsFirst, voxelsSecond);
                std::vector<std::set<Bonxai::IndicesT>> clusters = GeometryOperations::ClusterVoxelCloud(_union);

                VXL_ASSERT(clusters.size() < 3);  // given that we have already run the clustering algorithm on individual instances, we should never get more than two

                if (clusters.size() == 1)
                {
                    // Fuse the second instance with the first one
                    VXL_DEBUG(fmt::fg(fmt::terminal_color::yellow),
                              "(Refine) Fusing global {} - global {}. IoS: {:.2f}, SemSim: {:.2f}",
                              firstIdx,
                              secondIdx,
                              ios,
                              semSim);
                    PAUSE_THREAD_UNTIL_GUI_CONTINUE(debugging_utils::pause_on_fusion);
                    secondInstance.pointsTo = firstIdx;
                    fuseSemanticObjects(firstInstance, secondInstance);
                    geometry[firstIdx] = _union;
                }
                else
                    VXL_DEBUG("(Refine) NOT Fusing global {} - global {}.  {} clusters", firstIdx, secondIdx, clusters.size());
            }
            else
            {
                // VXL_DEBUG("(Refine) NOT Fusing global {} - global {}.  No BB intersection", firstIdx, secondIdx);
            }
        }
    }
}

std::pair<double, double> SemanticMap::compute3DIoU(const std::set<Bonxai::IndicesT>& voxels1,
                                                    const std::set<Bonxai::IndicesT>& voxels2,
                                                    uint coarsening_factor)
{
    std::set<Bonxai::IndicesT> voxels1_coarse = GeometryOperations::DownsampleVoxels(voxels1, coarsening_factor);
    std::set<Bonxai::IndicesT> voxels2_coarse = GeometryOperations::DownsampleVoxels(voxels2, coarsening_factor);

    std::set<Bonxai::IndicesT> intersection_ = GeometryOperations::SetIntersection(voxels1_coarse, voxels2_coarse);
    std::set<Bonxai::IndicesT> union_ = GeometryOperations::SetUnion(voxels1_coarse, voxels2_coarse);

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

double SemanticMap::computeIoV(const std::set<Bonxai::IndicesT>& visibleVoxels,
                               const std::set<Bonxai::IndicesT>& globalInstance,
                               const std::set<Bonxai::IndicesT>& localInstance,
                               uint coarsening_factor)
{
    std::set<Bonxai::IndicesT> visibleVoxels_coarse = GeometryOperations::DownsampleVoxels(visibleVoxels, coarsening_factor);
    std::set<Bonxai::IndicesT> globalVoxels_coarse = GeometryOperations::DownsampleVoxels(globalInstance, coarsening_factor);
    std::set<Bonxai::IndicesT> localVoxels_coarse = GeometryOperations::DownsampleVoxels(localInstance, coarsening_factor);

    // find all the voxels in the global instance which were visible in this image
    std::set<Bonxai::IndicesT> visibleGlobalVoxels = GeometryOperations::SetIntersection(globalVoxels_coarse, visibleVoxels_coarse);
    size_t numVisibleVoxels = visibleGlobalVoxels.size();

    // find which of the visible voxels were identified as part of this local instance
    std::set<Bonxai::IndicesT> globalVoxelsInMask = GeometryOperations::SetIntersection(visibleGlobalVoxels, localVoxels_coarse);
    size_t numVoxelsInMask = globalVoxelsInMask.size();

    double iov = numVisibleVoxels > 0 ? numVoxelsInMask / static_cast<double>(numVisibleVoxels) : 0;
    return iov;
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

    uint32_t offset = color_palette_offsets.at(index % color_palette_offsets.size());
    return color_palette[(index + offset) % color_palette.size()];
}

void SemanticMap::RandomizeColorsOrder()
{
    static std::random_device rd;
    static std::mt19937 g(rd());
    static std::uniform_int_distribution<uint32_t> uniform;

    for (size_t i = 0; i < color_palette_offsets.size(); i++)
        color_palette_offsets.at(i) = uniform(g);
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

SemanticObject& SemanticMap::CreateGlobalInstance()
{
    InstanceID_t id = globalSemanticMap.size();
    globalSemanticMap.emplace_back(id);
    VXL_DEBUG("Creating instance {}", id);
    return globalSemanticMap.back();
}

double SemanticMap::computeSemanticSimilarity(const SemanticObject& obj1, const SemanticObject& obj2)
{
    std::vector<std::string> _allCategories = CategoryManager::getInstance().getAllCategories();
    std::vector<float> P;
    P.reserve(_allCategories.size());
    std::vector<float> Q;
    Q.reserve(_allCategories.size());

    // split the uncertainty mass equally over all the classes
    constexpr float uncertaintyMassTotal = 5;
    float uncertaintyMassCat = uncertaintyMassTotal / _allCategories.size();

    for (size_t i = 0; i < _allCategories.size(); i++)
    {
        float prob1 = obj1.getCategoryAlpha(i);
        float prob2 = obj2.getCategoryAlpha(i);
        if (!obj1.isLocalInstance())
            prob1 += uncertaintyMassCat;
        if (!obj2.isLocalInstance())
            prob2 += uncertaintyMassCat;

        P.push_back(prob1);
        Q.push_back(prob2);
    }

    // Build normalized probability distributions P and Q
    Utils::Normalize(P);
    Utils::Normalize(Q);
    std::vector<double> M;

    for (size_t i = 0; i < _allCategories.size(); i++)
        M.push_back((P.at(i) + Q.at(i)) / 2.0);  // Midpoint distribution for JS divergence

    // Compute Jensen-Shannon divergence: JS(P||Q) = 0.5 * KL(P||M) + 0.5 * KL(Q||M)
    double kl_pm = 0.0, kl_qm = 0.0;

    for (size_t i = 0; i < P.size(); ++i)
    {
        if (P[i] > 0)
            kl_pm += P[i] * std::log(P[i] / M[i]);
        if (Q[i] > 0)
            kl_qm += Q[i] * std::log(Q[i] / M[i]);
    }

    double js_divergence = 0.5 * kl_pm + 0.5 * kl_qm;

    // JS divergence is bounded [0, ln(2)] when using natural log
    // Normalize to [0, 1] and convert to similarity (1 - normalized_divergence)
    double js_normalized = js_divergence / std::log(2.0);
    double similarity = 1.0 - js_normalized;

    VXL_ASSERT(similarity >= 0);
    VXL_ASSERT(similarity <= 1);
    return similarity;
}

void SemanticMap::updateAlphaCategories(SemanticObject& original, const SemanticObject& update)
{
    // Merge category probabilities from update into original
    for (const auto& [categoryIndex, probability] : update.alphaParamsCategories)
    {
        original.alphaParamsCategories[categoryIndex] += probability;
    }
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
    GeometryOperations::UpdateBBoxBounds(firstInstance.bbox, secondInstance.bbox);

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
