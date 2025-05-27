#include <voxeland_map/cell_types.hpp>
#include <voxeland_map/semantics.hpp>

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

bool SemanticMap::is_initialized()
{
    return initialized;
}

void SemanticMap::initialize(std::vector<std::string> dataset_categories,
                             Bonxai::ProbabilisticMap& _bonxai,
                             voxeland::DataMode mode)
{
    // Initialize objectInfoMap with SemanticObject for each object name
    for (size_t i = 0; i < dataset_categories.size(); ++i)
    {
        default_categories.push_back(dataset_categories[i]);
        categoryIndexMap[dataset_categories[i]] = i;
    }
    
    AUTO_TEMPLATE_SEMANTICS_ONLY(mode, BonxaiQuery<DataT>::createAccessor(_bonxai.With<DataT>()));
    initialized = true;
}

uint32_t SemanticMap::getCurrentActiveInstances()
{
    uint32_t activeInstances = 0;
    for (InstanceID_t i = 0; i < globalSemanticMap.size(); i++)
    {
        if (globalSemanticMap[i].pointsTo == -1)
        {
            activeInstances += 1;
        }
    }
    return activeInstances;
}

void SemanticMap::setLocalSemanticMap(const std::vector<SemanticObject>& localMap)
{
    lastLocalSemanticMap = localMap;
}

InstanceID_t SemanticMap::localToGlobalInstance(InstanceID_t localInstance)
{
    return lastMapLocalToGlobal[localInstance];
}

/*
void SemanticMap::integrateNewSemantics(const std::vector<SemanticObject>& localMap)
  {
    lastMapLocalToGlobal.resize(localMap.size());

    for (size_t localInstanceID = 0; localInstanceID < localMap.size();
         localInstanceID++)
    {
      const SemanticObject& localInstance = localMap[localInstanceID];
      std::vector<double>::const_iterator itLocal = std::max_element(
          localInstance.probabilities.begin(), localInstance.probabilities.end());
      uint8_t localClassIdx = std::distance(localInstance.probabilities.begin(), itLocal);
      bool fused = false;
      for (size_t globalInstanceID = 0; globalInstanceID < globalSemanticMap.size();
           globalInstanceID++)
      {
        SemanticObject& globalInstance = globalSemanticMap[globalInstanceID];
        std::vector<double>::iterator itGlobal =
            std::max_element(globalInstance.probabilities.begin(),
                             globalInstance.probabilities.end());
        uint8_t globalClassIdx =
            std::distance(globalInstance.probabilities.begin(), itGlobal);

        if (localClassIdx == globalClassIdx)
        {
          for (uint8_t i = 0; i < globalInstance.probabilities.size(); i++)
          {
            globalInstance.probabilities[i] += localInstance.probabilities[i];
            lastMapLocalToGlobal[localInstanceID] = globalInstanceID;
          }
          fused = true;
          break;
        }
      }
      if (!fused)
      {
        lastMapLocalToGlobal[localInstanceID] = globalSemanticMap.size();
        globalSemanticMap.push_back(SemanticObject(localInstance.probabilities, globalSemanticMap.size()+1));
      }
    }
  }
*/

uint32_t SemanticMap::indexToHexColor(InstanceID_t index)
{
    if (index == 0)
        return 0xbcbcbc;

    return color_palette[index % color_palette.size()];
}

void SemanticMap::updateCategoryProbability(SemanticObject& semanticObject,
                                            const std::string& categoryName,
                                            double probability)
{
    semanticObject.alphaParamsCategories[categoryIndexMap[categoryName]] += probability;
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
    for (size_t i = 0; i < original.alphaParamsCategories.size(); i++)
    {
        original.alphaParamsCategories[i] += update.alphaParamsCategories[i];
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

    for (size_t i = 0; i < original.appearancesTimestamps.size(); ++i) {
        // Add all timestamps from the update to the original
        original.appearancesTimestamps[i].insert(
            update.appearancesTimestamps[i].begin(),
            update.appearancesTimestamps[i].end()
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

InstanceID_t SemanticMap::getCategoryMaxProbability(InstanceID_t objID)
{
    auto itProbs =
        std::max_element(globalSemanticMap[objID].alphaParamsCategories.begin(), globalSemanticMap[objID].alphaParamsCategories.end());

    return std::distance(globalSemanticMap[objID].alphaParamsCategories.begin(), itProbs);
}