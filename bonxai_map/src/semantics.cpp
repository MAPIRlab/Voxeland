 #include <bonxai_map/semantics.hpp>

SemanticMap::SemanticMap() : kld_threshold(0.1f), color_palette({0xFAD4E0, 0x9DBBE3, 0xBFE3DF, 0xB59CD9, 0xFFF5CC, 0xFFD9BD, 0xEE9D94, 0xF7ADCF,
                                                                0xe6194B, 0x3cb44b, 0xffe119, 0x4363d8, 0xf58231, 0x911eb4, 0x42d4f4,
                                                                0xf032e6, 0xbfef45, 0xfabed4, 0x469990, 0xdcbeff, 0x9A6324, 0xfffac8})
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

bool SemanticMap::is_initialized() { return initialized; }

void SemanticMap::initialize(std::vector<std::string> dataset_categories)
  {
    // Initialize objectInfoMap with SemanticObject for each object name
    for (size_t i = 0; i < dataset_categories.size(); ++i)
    {
      default_categories.push_back(dataset_categories[i]);
      categoryIndexMap[dataset_categories[i]] = i;
    }
    initialized = true;
  }

void SemanticMap::setLocalSemanticMap(const std::vector<SemanticObject>& localMap) {
    lastLocalSemanticMap = localMap;
  }

INSTANCEIDT SemanticMap::localToGlobalInstance(INSTANCEIDT localInstance)
  {
    return lastMapLocalToGlobal[localInstance];
  }

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
        globalSemanticMap.push_back(SemanticObject(localInstance.probabilities));
      }
    }
  }

uint32_t SemanticMap::indexToHexColor(INSTANCEIDT index)
  {
    if (index == (default_categories.size() - 1))
    {
      return 0xbcbcbc;
    }

    return color_palette[index % color_palette.size()];
  }

void SemanticMap::updateCategoryProbability(SemanticObject& semanticObject,
                                 const std::string& categoryName,
                                 double probability)
  {
    semanticObject.probabilities[categoryIndexMap[categoryName]] += probability;
  };

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