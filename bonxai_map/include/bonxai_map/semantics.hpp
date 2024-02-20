#pragma once

#include <vector>
#include <cmath>
#include <iostream>
#include <unordered_map>

#include <eigen3/Eigen/Core>
#include "bonxai/bonxai.hpp"
#include <bonxai_map/pcl_utils.hpp>

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

  SemanticMap();

  std::vector<std::string> default_categories;
  std::unordered_map<std::string, size_t> categoryIndexMap;
  std::vector<SemanticObject> globalSemanticMap;
  std::vector<SemanticObject> lastLocalSemanticMap;

  static SemanticMap& get_instance()
  {
    static SemanticMap instance;
    return instance;
  }

  bool is_initialized();
  void initialize(std::vector<std::string> dataset_categories);
  void setLocalSemanticMap(const std::vector<SemanticObject>& localMap);
  INSTANCEIDT localToGlobalInstance(INSTANCEIDT localInstance);
  void integrateNewSemantics(const std::vector<SemanticObject>& localMap);
  uint32_t indexToHexColor(INSTANCEIDT index);
  void updateCategoryProbability(SemanticObject& semanticObject,
                                 const std::string& categoryName,
                                 double probability);


private:

  std::vector<INSTANCEIDT> lastMapLocalToGlobal;
  std::vector<std::uint32_t> color_palette;
  bool initialized = false;
  double kld_threshold;

  double computeKLD(const std::vector<double>& P, const std::vector<double>& Q);

};