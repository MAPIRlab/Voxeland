#pragma once
#include "probabilistic_map.hpp"
#include "pcl_utils.hpp"
#include <pcl/io/pcd_io.h>
#include <bonxai_map/pcl_utils.hpp>
#include <bonxai_map/semantics.hpp>
#include <map>

namespace Bonxai
{

struct Color
{
  uint8_t r;
  uint8_t g;
  uint8_t b;

  Color() = default;
  Color(const Color& other) = default;

  Color(uint8_t _r, uint8_t _g, uint8_t _b)
    : r(_r)
    , g(_g)
    , b(_b)
  {}

  void update(const pcl::PointXYZRGB& pcl)
  {
    r = pcl.r;
    g = pcl.g;
    b = pcl.b;
  }

  Color toColor() { return Color(*this); }
};

struct Empty
{
  Empty() = default;

  void update(const pcl::PointXYZ& pcl) {}

  Color toColor() { return Color(255, 255, 255); }
};

struct Semantics
{
  std::vector<double> probabilities;

  Semantics() {};

  void update(const pcl::PointXYZSemantics& pcl){
    SemanticMap& semantics = SemanticMap::get_instance();
    if (!probabilities.empty()) {
      for (INSTANCEIDT i = 0; i < probabilities.size(); i++){
        probabilities[i] += semantics.lastLocalSemanticMap[pcl.instance_id].probabilities[i];
      }
      
    }
    else {
      probabilities = semantics.lastLocalSemanticMap[pcl.instance_id].probabilities;
    }
  }

  Color toColor()
  {
    SemanticMap& semantics = SemanticMap::get_instance();

    std::vector<double>::iterator it =
        std::max_element(probabilities.begin(), probabilities.end());

    uint8_t mainObjectCategory = std::distance(probabilities.begin(), it);

    uint32_t hexColor = semantics.indexToHexColor(mainObjectCategory);

    if (mainObjectCategory == (semantics.default_categories.size() - 1))
    {
      hexColor = 0xbcbcbc;
    }

    return Color((hexColor >> 16) & 0xFF, (hexColor >> 8) & 0xFF, hexColor & 0xFF);
  }
};

struct RGBSemantics
{
  Color rgb;
  std::vector<double> probabilities;

  RGBSemantics(): rgb() {}

  void update(const pcl::PointXYZRGBSemantics& pcl){

    SemanticMap& semantics = SemanticMap::get_instance();
    if (!probabilities.empty()) {
      for (INSTANCEIDT i = 0; i < probabilities.size(); i++){
        probabilities[i] += semantics.lastLocalSemanticMap[pcl.instance_id].probabilities[i];
      }
      
    }
    else {
      probabilities = semantics.lastLocalSemanticMap[pcl.instance_id].probabilities;
    }

    rgb.r = pcl.r;
    rgb.g = pcl.g;
    rgb.b = pcl.b;
  }

  Color toColor()
  {
    SemanticMap& semantics = SemanticMap::get_instance();

    std::vector<double>::iterator it =
        std::max_element(probabilities.begin(), probabilities.end());

    uint8_t mainObjectCategory = std::distance(probabilities.begin(), it);

    uint32_t hexColor = semantics.indexToHexColor(mainObjectCategory);

    if (mainObjectCategory == (semantics.default_categories.size() - 1))
    {
      hexColor = (static_cast<uint32_t>(rgb.r) << 16) | (static_cast<uint32_t>(rgb.g) << 8) | static_cast<uint32_t>(rgb.b);
      //hexColor = 0xbcbcbc;
    }

    return Color((hexColor >> 16) & 0xFF, (hexColor >> 8) & 0xFF, hexColor & 0xFF);
  }
};

struct SemanticsInstances
{
  std::vector<INSTANCEIDT> instances_candidates;
  std::vector<uint32_t> instances_votes;

  SemanticsInstances() {};

  void update(const pcl::PointXYZSemantics& pcl)
  {
    SemanticMap& semantics = SemanticMap::get_instance();
    INSTANCEIDT thisGlobalID = semantics.localToGlobalInstance(pcl.instance_id);

    auto it = std::find(instances_candidates.begin(), instances_candidates.end(), thisGlobalID);

    if (it != instances_candidates.end()) {
      //instances_votes[std::distance(instances_candidates.begin(), it)]+=1;
      if(thisGlobalID != 0){
        instances_votes[std::distance(instances_candidates.begin(), it)]+=1;
      }
    }
    else{
      instances_candidates.push_back(thisGlobalID);
      instances_votes.push_back(0);
    }
  }

  Color toColor()
  {
    SemanticMap& semantics = SemanticMap::get_instance();

    updateCandidatesAndVotes();

    auto itInstances = std::max_element(instances_votes.begin(), instances_votes.end());
    auto idxMaxVotes = std::distance(instances_votes.begin(), itInstances);
    std::vector<double> probsBestInstance = semantics.globalSemanticMap[instances_candidates[idxMaxVotes]].probabilities;

    auto itProbs = std::max_element(probsBestInstance.begin(), probsBestInstance.end());

    // Set the color of the best object category of the most probable instance
    //uint32_t hexColor = semantics.indexToHexColor(std::distance(probsBestInstance.begin(), itProbs));
    // Set a unique color for the most probable instance
    uint32_t hexColor = semantics.indexToHexColor(instances_candidates[idxMaxVotes]);

    if (std::distance(probsBestInstance.begin(), itProbs) ==
        (semantics.default_categories.size() - 1)){
      hexColor = 0xbcbcbc;
    }/*
    if (instances_candidates[std::distance(instances_votes.begin(), itInstances)] == 0){
      hexColor = 0xbcbcbc;
    }
    if ((std::distance(probsBestInstance.begin(), itProbs) ==
        (semantics.default_categories.size() - 1)) && !(instances_candidates[idxMaxVotes] == 0)){
          hexColor = 0xbcbcbc;
        }*/

    return Color((hexColor >> 16) & 0xFF, (hexColor >> 8) & 0xFF, hexColor & 0xFF);
  }

  void updateCandidatesAndVotes(){

    SemanticMap& semantics = SemanticMap::get_instance();

    std::vector<INSTANCEIDT> candidates_temp;
    std::map<INSTANCEIDT, uint32_t> combining_instances;

    for (INSTANCEIDT i = 0; i < instances_candidates.size(); i++){
      if(semantics.globalSemanticMap[instances_candidates[i]].pointsTo == -1){
        candidates_temp.push_back(instances_candidates[i]);
      }
      else{
        candidates_temp.push_back(semantics.globalSemanticMap[instances_candidates[i]].pointsTo);
      }
    }

    for (INSTANCEIDT i = 0; i < candidates_temp.size(); i++){
      combining_instances[candidates_temp[i]] += instances_votes[i];
    }

    instances_candidates.clear();
    instances_votes.clear();

    for (const std::pair<INSTANCEIDT, uint32_t>& instance: combining_instances){
      instances_candidates.push_back(instance.first);
      instances_votes.push_back(instance.second);
    }

  }

};

struct RGBSemanticsInstances
{
  Color rgb;
  std::vector<INSTANCEIDT> instances_candidates;
  std::vector<uint32_t> instances_votes;

  RGBSemanticsInstances() {};

  void update(const pcl::PointXYZRGBSemantics& pcl)
  {
    SemanticMap& semantics = SemanticMap::get_instance();
    INSTANCEIDT thisGlobalID = semantics.localToGlobalInstance(pcl.instance_id);

    auto it = std::find(instances_candidates.begin(), instances_candidates.end(), thisGlobalID);

    if (it != instances_candidates.end()) {
      instances_votes[std::distance(instances_candidates.begin(), it)]+=1;
      /*if(thisGlobalID != 0){
        instances_votes[std::distance(instances_candidates.begin(), it)]+=1;
      }*/
    }
    else{
      instances_candidates.push_back(thisGlobalID);
      instances_votes.push_back(0);
    }

    rgb.r = pcl.r;
    rgb.g = pcl.g;
    rgb.b = pcl.b;
  }

  Color toColor()
  {
    SemanticMap& semantics = SemanticMap::get_instance();

    updateCandidatesAndVotes();

    auto itInstances = std::max_element(instances_votes.begin(), instances_votes.end());
    auto idxMaxVotes = std::distance(instances_votes.begin(), itInstances);
    std::vector<double> probsBestInstance = semantics.globalSemanticMap[instances_candidates[idxMaxVotes]].probabilities;

    auto itProbs = std::max_element(probsBestInstance.begin(), probsBestInstance.end());

    // Set the color of the best object category of the most probable instance
    //uint32_t hexColor = semantics.indexToHexColor(std::distance(probsBestInstance.begin(), itProbs));
    // Set a unique color for the most probable instance
    uint32_t hexColor = semantics.indexToHexColor(instances_candidates[idxMaxVotes]);

    if (std::distance(probsBestInstance.begin(), itProbs) ==
        (semantics.default_categories.size() - 1)){
      hexColor = (static_cast<uint32_t>(rgb.r) << 16) | (static_cast<uint32_t>(rgb.g) << 8) | static_cast<uint32_t>(rgb.b);
    }/*
    if (instances_candidates[std::distance(instances_votes.begin(), itInstances)] == 0){
      hexColor = 0xbcbcbc;
    }
    if ((std::distance(probsBestInstance.begin(), itProbs) ==
        (semantics.default_categories.size() - 1)) && !(instances_candidates[idxMaxVotes] == 0)){
          hexColor = 0xbcbcbc;
        }*/

    return Color((hexColor >> 16) & 0xFF, (hexColor >> 8) & 0xFF, hexColor & 0xFF);
  }

  void updateCandidatesAndVotes(){

    SemanticMap& semantics = SemanticMap::get_instance();

    std::vector<INSTANCEIDT> candidates_temp;
    std::map<INSTANCEIDT, uint32_t> combining_instances;

    for (INSTANCEIDT i = 0; i < instances_candidates.size(); i++){
      if(semantics.globalSemanticMap[instances_candidates[i]].pointsTo == -1){
        candidates_temp.push_back(instances_candidates[i]);
      }
      else{
        candidates_temp.push_back(semantics.globalSemanticMap[instances_candidates[i]].pointsTo);
      }
    }

    for (INSTANCEIDT i = 0; i < candidates_temp.size(); i++){
      combining_instances[candidates_temp[i]] += instances_votes[i];
    }

    instances_candidates.clear();
    instances_votes.clear();

    for (const std::pair<INSTANCEIDT, uint32_t>& instance: combining_instances){
      instances_candidates.push_back(instance.first);
      instances_votes.push_back(instance.second);
    }

  }
};

}  // namespace Bonxai
