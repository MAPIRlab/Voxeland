#pragma once

#include <vector>
#include <cmath>
#include <iostream>
#include <unordered_map>

#include <eigen3/Eigen/Core>
#include "bonxai/bonxai.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include "segmentation_msgs/msg/semantic_point_cloud.hpp"
#include "vision_msgs/msg/detection2_d.hpp"

struct SemanticObject
{
  std::vector<double> probabilities;

  SemanticObject(size_t numCategories)
    : probabilities(numCategories, 0)
  {}
  SemanticObject(const std::vector<double>& _probabilities)
    : probabilities(_probabilities)
  {}
};

class SemanticMap
{
public:
  static SemanticMap& get_instance()
  {
    static SemanticMap instance;
    return instance;
  };

  std::vector<std::string> default_categories;
  std::unordered_map<std::string, size_t> categoryIndexMap;

  std::vector<SemanticObject> globalSemanticMap;

  SemanticMap() = default;

  void initialize(std::vector<std::string> dataset_categories)
  {
    // Initialize objectInfoMap with SemanticObject for each object name
    for (size_t i = 0; i < dataset_categories.size(); ++i)
    {
      default_categories.push_back(dataset_categories[i]);
      categoryIndexMap[dataset_categories[i]] = i;
    }
    default_categories.push_back("unknown");
    categoryIndexMap["unknown"] = default_categories.size() - 1;

    initialized = true;
  };

  bool is_initialized() { return initialized; }

  SemanticObject
  convertDetection2DToSemanticObject(const vision_msgs::msg::Detection2D& instance)
  {
    SemanticObject semanticObject(default_categories.size());

    for (const auto& result : instance.results)
    {
      updateCategoryProbability(
          semanticObject, result.hypothesis.class_id, result.hypothesis.score);
    }

    return semanticObject;
  };

  std::vector<SemanticObject> convertROSMessageToSemanticMap(
      const std::vector<vision_msgs::msg::Detection2D>& instances)
  {
    std::vector<SemanticObject> localSemanticMap(
        instances.size(), SemanticObject(default_categories.size()));

    for (INSTANCEIDT i = 0; i < instances.size(); i++)
    {
      SemanticObject newObject = convertDetection2DToSemanticObject(instances[i]);

      localSemanticMap[std::atoi(instances[i].id.c_str())] = newObject;
    }

    return localSemanticMap;
  };

  void integrateNewSemantics(const std::vector<SemanticObject>& localMap)
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

  INSTANCEIDT localToGlobalInstance(INSTANCEIDT localInstance)
  {
    return lastMapLocalToGlobal[localInstance];
  }

  uint32_t indexToHexColor(uint8_t index)
  {
    if (index == (default_categories.size() - 1))
    {
      return 0xbcbcbc;
    }

    return color_palette[index % color_palette.size()];
  }

private:
  std::vector<INSTANCEIDT> lastMapLocalToGlobal;

  bool initialized = false;
  double kld_threshold = 0.1f;

  // Function to compute Kullback-Leibler Divergence
  double computeKLD(const std::vector<double>& P, const std::vector<double>& Q)
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
  };

  void updateCategoryProbability(SemanticObject& semanticObject,
                                 const std::string& categoryName,
                                 double probability)
  {
    semanticObject.probabilities[categoryIndexMap[categoryName]] += probability;
  };

  std::vector<std::uint32_t> color_palette = {
    0xe6194B, 0x3cb44b, 0xffe119, 0x4363d8, 0xf58231, 0x911eb4, 0x42d4f4,
    0xf032e6, 0xbfef45, 0xfabed4, 0x469990, 0xdcbeff, 0x9A6324, 0xfffac8,
    0x800000, 0xaaffc3, 0x808000, 0xffd8b1, 0x000075
  };
};