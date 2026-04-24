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
    RandomizeColorsOrder();
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

    lastMapLocalToGlobal.clear();
    lastMapLocalToGlobal.resize(localMap.size());
    if (globalSemanticMap.empty())
    {
        SemanticObject unknown = SemanticObject(0, localMap[0].bbox);
        unknown.alphaParamsCategories = { { CategoryManager::UNKNOWN_CATEGORY, 1 } };
        updateAppearancesTimestamps(unknown, localMap[0]);

        globalSemanticMap[0] = unknown;
    }
    else
    {
        fuseSemanticObjects(globalSemanticMap.at(0), localMap[0]);
    }

    // First, integrate local "unknown" with global "unknown". They are always the 0-index
    lastMapLocalToGlobal[0].push_back(0);

    auto globalInstanceIDList = getCurrentInstanceIDs();

    // Both loops start at 1 to skip "unknown" class
    for (InstanceID_t localInstanceID = 1; localInstanceID < localMap.size(); localInstanceID++)
    {
        const SemanticObject& localInstance = localMap[localInstanceID];

        if (!localInstance.localGeometry.has_value())
            continue;
        const std::set<Bonxai::IndicesT>& voxelsLocal = *localInstance.localGeometry;
        std::map<InstanceID_t, std::set<Bonxai::IndicesT>> globalsGeometry;  // cache the voxels for each global object to avoid repeated lookup

        // Find category with maximum probability for local instance
        CategoryManager::CategoryIndex localMaxCategory = localInstance.mostLikelyCategory();
        VXL_DEBUG("Processing local instance {}, '{}'", localInstanceID, CategoryManager::getInstance().getCategoryName(localMaxCategory));

        for (size_t globalIndex = 0; globalIndex < globalInstanceIDList.size(); globalIndex++)
        {
            InstanceID_t globalInstanceID = globalInstanceIDList.at(globalIndex);
            if (globalInstanceID == 0)
                continue;

            SemanticObject& globalInstance = globalSemanticMap.at(globalInstanceID);
            // Find category with maximum probability for global instance
            CategoryManager::CategoryIndex globalMaxCategory = globalInstance.mostLikelyCategory();

            if (globalInstanceID == 0 || !globalInstance.isValidInstance() || !GeometryOperations::CheckBBoxIntersect(localInstance.bbox, globalInstance.bbox))
                continue;

            // get all the voxels that belong to the global instance
            if (!globalsGeometry.contains(globalInstanceID))
            {
                AUTO_TEMPLATE_INSTANCES_ONLY(currentMode,
                                             globalsGeometry.insert({ globalInstanceID, listOfVoxelsInObject<DataT>(globalInstance) }););
            }
            const std::set<Bonxai::IndicesT>& voxelsGlobal = globalsGeometry.at(globalInstanceID);

            auto [iou, ios] = compute3DIoU(voxelsGlobal, voxelsLocal, 1);

            double iov = computeIoV(voxelizedLocalPointCloud, voxelsGlobal, voxelsLocal, 1);

            // ============================================================
            // HYBRID FUSION: IoV + Jensen-Shannon Semantic Similarity
            // Combines geometric (IoU, IoS, IoV) with semantic evidence
            // ============================================================

            // Compute semantic similarity using Jensen-Shannon divergence
            // This compares the FULL probability distributions, not just the top class
            double semanticSimilarity = computeSemanticSimilarity(localInstance, globalInstance);

            // if the objects are very semantically similar, be a bit more lenient with the geometrical coincidence
            constexpr float minIOVThr = 0.4;
            constexpr float maxIOVThr = 0.8;
            double fusionThreshold = std::lerp(maxIOVThr, minIOVThr, semanticSimilarity);
            double nFusionScore = iov / fusionThreshold;

            if (nFusionScore >= 1.0)
            {
                VXL_DEBUG(fmt::fg(fmt::terminal_color::yellow),
                          "Integrating local {} - global {}:\n\tIoU:{:.2f}  IoS:{:.2f}  IoV:{:.2f}  SemSim:{:.2f}",
                          localInstanceID,
                          globalInstanceID,
                          iou,
                          ios,
                          iov,
                          semanticSimilarity);
                PAUSE_THREAD_UNTIL_GUI_CONTINUE(debugging_utils::pause_on_integration);
                fuseSemanticObjects(globalInstance, localInstance);

                lastMapLocalToGlobal[localInstanceID].push_back(globalInstance.instanceID);

                globalInstance.numberObservations++;
                integrated++;
            }
            else
            {
                VXL_DEBUG("NOT integrating local {} - global {}:\n\tIoU:{:.2f}  IoS:{:.2f}  IoV:{:.2f}  SemSim:{:.2f}",
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

        if (lastMapLocalToGlobal.at(localInstanceID).empty())
        {
            // Create new object integrating localMap information
            SemanticObject& newObject = CreateGlobalInstance();
            newObject.bbox = localInstance.bbox;
            newObject.alphaParamsCategories = localInstance.alphaParamsCategories;
            newObject.appearancesTimestamps = localInstance.appearancesTimestamps;

            lastMapLocalToGlobal[localInstanceID].push_back(newObject.instanceID);
            VXL_DEBUG("Created global instance {} from local {}", newObject.instanceID, localInstanceID);
            added++;
        }
    }
    VXL_INFO("Integrated {} new local objects: {} integrated and {} added", localMap.size(), integrated, added);
}

void SemanticMap::refineGlobalSemanticMap(uint minimumObservations, uint minimumVoxels, const FusionOptions* options)
{
    if (!options)
        options = &defaultOptions;

    // cache the voxels for each global object to avoid repeated lookup
    std::map<InstanceID_t, std::set<Bonxai::IndicesT>> geometry;

    // not using structured binding because OpenMP (in listOfVoxelsInObject) does not accept them
    for (auto& pair : globalSemanticMap)
    {
        InstanceID_t id = pair.first;
        SemanticObject& instance = pair.second;
        if (instance.isValidInstance() && id != 0)
        {
            std::set<Bonxai::IndicesT> voxelsGlobal;
            AUTO_TEMPLATE_INSTANCES_ONLY(currentMode,
                                         voxelsGlobal = listOfVoxelsInObject<DataT>(instance));

            // remove instances with very few observations
            if (instance.numberObservations <= minimumObservations || voxelsGlobal.size() < minimumVoxels)
            {
                instance.pointsTo = 0;
                VXL_INFO("Removing instance {}: {} voxels after {} observations", id, voxelsGlobal.size(), instance.numberObservations);
            }
            else
                geometry.insert({ id, voxelsGlobal });
        }
    }

    auto globalInstanceIDList = getCurrentInstanceIDs();
    // iterate over individual instances, see if they need to be split into smaller chunks
    for (auto& id : globalInstanceIDList)
    {
        auto& instance = globalSemanticMap.at(id);
        if (!instance.isValidInstance() || id == 0)
            continue;
        std::vector<std::set<Bonxai::IndicesT>> clusters = GeometryOperations::ClusterVoxelCloud(geometry.at(id));

        auto filterLoosePoints = [&](const std::set<Bonxai::IndicesT>& validVoxels) {
            for (const auto& voxel : geometry.at(id))
            {
                AUTO_TEMPLATE_INSTANCES_ONLY(currentMode,
                                             {
                                                 Bonxai::ProbabilisticCell<DataT>* cell = BonxaiQuery<DataT>::getAccessor().value(voxel);
                                                 if (!validVoxels.contains(voxel))
                                                     cell->data.ReplaceInstanceVotes(id, 0);
                                             });
            }
        };

        if (clusters.size() == 0)
            continue;
        else if (clusters.size() == 1)
        {
            // update the geometry to remove any loose points
            auto validVoxels = clusters.at(0);
            filterLoosePoints(clusters.at(0));
            geometry.at(id) = clusters.at(0);
        }
        else
        {
            // more than one chunk, let's split it into multiple instances
            debugInfo.mostRecentClusters = clusters;
            VXL_DEBUG("Splitting instance {} into {} chunks", id, clusters.size());
            PAUSE_THREAD_UNTIL_GUI_CONTINUE(debugging_utils::pause_on_splitting);

            // remove loose points
            std::set<Bonxai::IndicesT> _union;
            for (size_t clusterIdx = 0; clusterIdx < clusters.size(); clusterIdx++)
                _union = GeometryOperations::SetUnion(_union, clusters.at(clusterIdx));
            filterLoosePoints(_union);

            // the first cluster will be assigned to the old instance ID
            geometry[id] = clusters.at(0);

            // every other cluster has now been promoted to being its own instance
            for (size_t clusterIdx = 1; clusterIdx < clusters.size(); clusterIdx++)
            {
                auto thisCluster = clusters.at(clusterIdx);
                SemanticObject& newObject = CreateGlobalInstance();
                newObject.bbox = GeometryOperations::FindBBox(thisCluster);
                newObject.alphaParamsCategories = instance.alphaParamsCategories;
                newObject.appearancesTimestamps = instance.appearancesTimestamps;
                VXL_ASSERT(newObject.alphaParamsCategories.size() > 0);

                // update the cache
                geometry.insert({ newObject.instanceID, thisCluster });

                // update the votes on each of the voxels to point to the new instance
                for (const auto& voxel : thisCluster)
                {
                    AUTO_TEMPLATE_INSTANCES_ONLY(currentMode,
                                                 {
                                                     Bonxai::ProbabilisticCell<DataT>* cell = BonxaiQuery<DataT>::getAccessor().value(voxel);
                                                     cell->data.ReplaceInstanceVotes(id, newObject.instanceID);
                                                 });
                }
            }
        }
    }

    // iterate over instance pairs, try to fuse them into bigger chunks
    bool fusedSomething = false;
    do
    {
        // refresh the list of instances
        globalInstanceIDList = getCurrentInstanceIDs();

        for (size_t firstIdx = 0; firstIdx < globalInstanceIDList.size(); firstIdx++)
        {
            InstanceID_t firstID = globalInstanceIDList.at(firstIdx);
            SemanticObject& firstInstance = globalSemanticMap.at(firstID);
            if (!firstInstance.isValidInstance() || firstID == 0)
                continue;

            std::set<Bonxai::IndicesT> voxelsFirst = geometry.at(firstID);

            fusedSomething = false;
            for (size_t secondIdx = firstIdx + 1; secondIdx < globalInstanceIDList.size(); secondIdx++)
            {
                InstanceID_t secondID = globalInstanceIDList.at(secondIdx);
                SemanticObject& secondInstance = globalSemanticMap.at(secondID);

                if (!secondInstance.isValidInstance() || secondID == 0)
                    continue;
                if (GeometryOperations::CheckBBoxIntersect(firstInstance.bbox, secondInstance.bbox))
                {
                    std::set<Bonxai::IndicesT> voxelsSecond = geometry.at(secondID);

                    bool IoUSkip = false;  // if IoU is huge, don't bother with any other checks: the instances correspond to the same geometry!

                    // IoU check
                    float iou, ios;
                    {
                        // TODO we are not caching the geometry used for this (with the votesThr). Check performance to see if it's worth bothering
                        std::set<Bonxai::IndicesT> voxelsSomeVotesFirst;
                        std::set<Bonxai::IndicesT> voxelsSomeVotesSecond;
                        AUTO_TEMPLATE_INSTANCES_ONLY(currentMode, voxelsSomeVotesFirst = listOfVoxelsInObject<DataT>(globalSemanticMap.at(firstID), options->votesThr));
                        AUTO_TEMPLATE_INSTANCES_ONLY(currentMode, voxelsSomeVotesSecond = listOfVoxelsInObject<DataT>(globalSemanticMap.at(secondID), options->votesThr));
                        std::tie(iou, ios) = compute3DIoU(voxelsSomeVotesFirst, voxelsSomeVotesSecond, options->coarseningFactor);

                        if (iou > options->iouSkipThr)
                        {
                            IoUSkip = true;
                            VXL_DEBUG("Fusing global {} - global {}.  IoU above passthrough threshold: {:.2f}", firstID, secondID, iou);
                        }

                        if (!IoUSkip && (ios < options->iosThr && iou < options->iouThr))
                        {
                            VXL_DEBUG("(Refine) NOT Fusing global {} - global {}.  Insufficient IoS: {:.2f} and IoU: {:.2f}", firstID, secondID, ios, iou);
                            continue;
                        }
                    }

                    // semantics check
                    double semSim = computeSemanticSimilarity(firstInstance, secondInstance);
                    CategoryManager::CategoryIndex mostLikelyFirst = firstInstance.mostLikelyCategory();
                    CategoryManager::CategoryIndex mostLikelySecond = secondInstance.mostLikelyCategory();
                    bool semanticsOk = semSim >= options->semSimThr || mostLikelyFirst == mostLikelySecond;
                    if (!IoUSkip && semanticsOk)
                    {
                        VXL_DEBUG("(Refine) NOT Fusing global {} - global {}.  Insufficient SemSim: {:.2f}", firstID, secondID, semSim);
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
                                  firstID,
                                  secondID,
                                  ios,
                                  semSim);
                        PAUSE_THREAD_UNTIL_GUI_CONTINUE(debugging_utils::pause_on_fusion);
                        secondInstance.pointsTo = firstID;
                        fuseSemanticObjects(firstInstance, secondInstance);
                        geometry[firstID] = _union;
                        fusedSomething = true;
                    }
                    else
                        VXL_DEBUG("(Refine) NOT Fusing global {} - global {}.  {} clusters", firstID, secondID, clusters.size());
                }
                else
                {
                    // VXL_DEBUG("(Refine) NOT Fusing global {} - global {}.  No BB intersection", firstIdx, secondIdx);
                }
            }
        }
    } while (fusedSomething);

    // do one last pass, removing any individual instances with no geometric clusters
    for (auto& id : globalInstanceIDList)
    {
        auto& instance = globalSemanticMap.at(id);
        if (!instance.isValidInstance() || id == 0)
            continue;
        std::vector<std::set<Bonxai::IndicesT>> clusters = GeometryOperations::ClusterVoxelCloud(geometry.at(id));

        // if it's all disperse points, this instance is cooked
        if (clusters.size() == 0)
        {
            VXL_INFO("Removing instance {}: 0 clusters in refinement", id);
            instance.pointsTo = 0;
        }
    }

    // update the global map
    deleteOldInstances();
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

void SemanticMap::deleteOldInstances()
{
    // replace all votes to outdated instances with votes to the new ones
    AUTO_TEMPLATE_INSTANCES_ONLY(currentMode,
                                 {
                                     auto bonxai = BonxaiQuery<DataT>::getBonxaiT();
                                     auto visitor = [&](Bonxai::ProbabilisticCell<DataT>& cell, const Bonxai::IndicesT& indices) {
                                         cell.data.updateCandidatesAndVotes();
                                     };
                                     bonxai->grid()->forEachCell(visitor);
                                 });

    // remove instances from the map
    auto allIDs = getCurrentInstanceIDs();
    for (InstanceID_t id : allIDs)
    {
        if (!globalSemanticMap.at(id).isValidInstance())
            globalSemanticMap.erase(id);
    }
}

void SemanticMap::setLocalSemanticMap(const std::vector<SemanticObject>& localMap)
{
    lastLocalSemanticMap = localMap;
}

std::vector<InstanceID_t>& SemanticMap::localToGlobalInstance(InstanceID_t localInstance)
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

SemanticObject& SemanticMap::CreateGlobalInstance(std::optional<InstanceID_t> forceID)
{
    static InstanceID_t nextID = 1;
    InstanceID_t id = forceID ? *forceID : nextID;
    nextID = id + 1;
    VXL_ASSERT(!globalSemanticMap.contains(id));
    VXL_DEBUG("Creating instance {}", id);
    auto pair = globalSemanticMap.emplace(id, id);
    return pair.first->second;  // this is certainly a line of code
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

    constexpr float epsilon = 1e-4;
    VXL_ASSERT(similarity >= 0 - epsilon);
    VXL_ASSERT(similarity <= 1 + epsilon);
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

std::vector<size_t> SemanticMap::getCurrentInstanceIDs()
{
    auto ks = std::views::keys(globalSemanticMap);
    std::vector<size_t> globalInstanceIDList{ ks.begin(), ks.end() };
    return globalInstanceIDList;
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

void SemanticMap::loadInstancesFromFile(const std::filesystem::path& path)
{
    using nlohmann::json;
    std::ifstream file(path);
    json json_file = json::parse(file);

    json instances = json_file["instances"];

    for (auto& [key, val] : instances.items())
    {
        std::string name = key;
        InstanceID_t id = std::atoi(name.substr(3).c_str());

        SemanticObject& object = CreateGlobalInstance(id);
        object.instanceName = name;

        float centerX = val["bbox"]["center"][0];
        float centerY = val["bbox"]["center"][1];
        float centerZ = val["bbox"]["center"][2];

        float sizeX = val["bbox"]["size"][0];
        float sizeY = val["bbox"]["size"][1];
        float sizeZ = val["bbox"]["size"][2];
        object.bbox.minX = centerX - sizeX * 0.5f;
        object.bbox.minY = centerY - sizeY * 0.5f;
        object.bbox.minZ = centerZ - sizeZ * 0.5f;
        object.bbox.maxX = centerX + sizeX * 0.5f;
        object.bbox.maxY = centerY + sizeY * 0.5f;
        object.bbox.maxZ = centerZ + sizeZ * 0.5f;

        object.numberObservations = val["n_observations"];
        object.underSegmentScore = val["undersegmentation_score"];

        auto& alphasList = val["results"];
        for (auto& [category, alpha] : alphasList.items())
            object.addToCategoryAlpha(CategoryManager::getInstance().addCategory(category), alpha);
    }
}

nlohmann::json SemanticMap::mapToJSON()
{
    nlohmann::json data_json;

    data_json["instances"] = {};

    for (auto& [id, instance] : globalSemanticMap)
    {
        if (instance.isValidInstance())
        {
            data_json["instances"][instance.instanceName] = {};
            data_json["instances"][instance.instanceName]["bbox"] = {};

            nlohmann::json center = nlohmann::json::array();
            center.push_back((instance.bbox.minX + instance.bbox.maxX) / 2.0);
            center.push_back((instance.bbox.minY + instance.bbox.maxY) / 2.0);
            center.push_back((instance.bbox.minZ + instance.bbox.maxZ) / 2.0);
            data_json["instances"][instance.instanceName]["bbox"]["center"] = center;

            nlohmann::json size = nlohmann::json::array();
            size.push_back(instance.bbox.maxX - instance.bbox.minX);
            size.push_back(instance.bbox.maxY - instance.bbox.minY);
            size.push_back(instance.bbox.maxZ - instance.bbox.minZ);
            data_json["instances"][instance.instanceName]["bbox"]["size"] = size;

            data_json["instances"][instance.instanceName]["results"] = {};

            // Convert dynamic category probabilities to JSON
            for (const auto& [categoryIndex, probability] : instance.alphaParamsCategories)
            {
                if (probability > 0)
                {
                    std::string categoryName = getCategoryName(categoryIndex);
                    if (!categoryName.empty())
                    {
                        data_json["instances"][instance.instanceName]["results"][categoryName] = probability;
                    }
                }
            }

            data_json["instances"][instance.instanceName]["n_observations"] = instance.numberObservations;
            data_json["instances"][instance.instanceName]["undersegmentation_score"] = instance.underSegmentScore;
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

    for (auto& [id, instance] : globalSemanticMap)
    {
        if (!instance.isValidInstance())
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
    for (auto& [id, instance] : globalSemanticMap)
    {
        if (instance.isValidInstance())
        {
            data_json[instance.instanceName] = {};
            data_json[instance.instanceName]["timestamps"] = {};
            for (size_t j = 0; j < instance.appearancesTimestamps.size(); j++)
            {
                std::string category = getCategoryName(j);
                const auto& appearances_map = instance.appearancesTimestamps[j];
                if (appearances_map.empty())
                {
                    continue;
                }

                data_json[instance.instanceName]["timestamps"][category] = nlohmann::json::array();
                for (const auto& instancePair : appearances_map)
                {
                    nlohmann::json instanceBbox;
                    instanceBbox["instance_id"] = instancePair.first;
                    instanceBbox["bbox"]["centerX"] = instancePair.second.centerX;
                    instanceBbox["bbox"]["centerY"] = instancePair.second.centerY;
                    instanceBbox["bbox"]["sizeX"] = instancePair.second.sizeX;
                    instanceBbox["bbox"]["sizeY"] = instancePair.second.sizeY;

                    data_json[instance.instanceName]["timestamps"][category].push_back(instanceBbox);
                }
            }
        }
    }

    return data_json;
}
