#pragma once
#include <map>
#include "probabilistic_map.hpp"
#include "pcl_utils.hpp"
#include <pcl/io/pcd_io.h>
#include <voxeland_map/pcl_utils.hpp>
#include <voxeland_map/semantics.hpp>
#include <voxeland_map/dirichlet.hpp>

namespace Bonxai
{

struct Color;

inline std::string XYZtoPLY(const Point3D& point);
inline std::string RGBtoPLY(const Color& rgb);
inline std::string getXYZheader();
inline std::string getRGBheader();

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

  std::string toPLY(const Bonxai::Point3D& point) { return fmt::format("{} {}\n", XYZtoPLY(point), RGBtoPLY(*this)); }

  static std::string getHeaderPLY() { return fmt::format("{}\n{}", getXYZheader(), getRGBheader()); }
};

struct Empty
{
  Empty() = default;

  void update(const pcl::PointXYZ& pcl) {}

  Color toColor() { return Color(255, 255, 255); }

  std::string toPLY(const Bonxai::Point3D& point) { return fmt::format("{}\n", XYZtoPLY(point)); }

  static std::string getHeaderPLY() { return getXYZheader(); }
};

struct Semantics
{
  std::vector<double> probabilities;

  Semantics(){};

  void update(const pcl::PointXYZSemantics& pcl)
  {
    SemanticMap& semantics = SemanticMap::get_instance();
    if (!probabilities.empty())
    {
      for (InstanceID_t i = 0; i < probabilities.size(); i++)
      {
        probabilities[i] += semantics.lastLocalSemanticMap[pcl.instance_id].probabilities[i];
      }
    }
    else
    {
      probabilities = semantics.lastLocalSemanticMap[pcl.instance_id].probabilities;
    }
  }

  Color toColor()
  {
    SemanticMap& semantics = SemanticMap::get_instance();

    std::vector<double>::iterator it = std::max_element(probabilities.begin(), probabilities.end());

    uint8_t mainObjectCategory = std::distance(probabilities.begin(), it);

    uint32_t hexColor = semantics.indexToHexColor(mainObjectCategory);

    if (mainObjectCategory == (semantics.default_categories.size() - 1))
    {
      hexColor = 0xbcbcbc;
    }

    return Color((hexColor >> 16) & 0xFF, (hexColor >> 8) & 0xFF, hexColor & 0xFF);
  }

  std::string toPLY(const Bonxai::Point3D& point)
  {
    double uncertainty_categories = expected_shannon_entropy<double>(probabilities);
    return fmt::format("{} {} {}\n", XYZtoPLY(point), RGBtoPLY(toColor()), uncertainty_categories);
  }

  static std::string getHeaderPLY()
  {
    return fmt::format("{}\n{}\nproperty float uncertainty_categories", getXYZheader(), getRGBheader());
  }
};

struct RGBSemantics
{
  Color rgb;
  std::vector<double> probabilities;

  RGBSemantics()
    : rgb()
  {}

  void update(const pcl::PointXYZRGBSemantics& pcl)
  {
    SemanticMap& semantics = SemanticMap::get_instance();
    if (!probabilities.empty())
    {
      for (InstanceID_t i = 0; i < probabilities.size(); i++)
      {
        probabilities[i] += semantics.lastLocalSemanticMap[pcl.instance_id].probabilities[i];
      }
    }
    else
    {
      probabilities = semantics.lastLocalSemanticMap[pcl.instance_id].probabilities;
    }

    rgb.r = pcl.r;
    rgb.g = pcl.g;
    rgb.b = pcl.b;
  }

  Color toColor()
  {
    SemanticMap& semantics = SemanticMap::get_instance();

    std::vector<double>::iterator it = std::max_element(probabilities.begin(), probabilities.end());

    uint8_t mainObjectCategory = std::distance(probabilities.begin(), it);

    uint32_t hexColor = semantics.indexToHexColor(mainObjectCategory);

    if (mainObjectCategory == (semantics.default_categories.size() - 1))
    {
      hexColor =
          (static_cast<uint32_t>(rgb.r) << 16) | (static_cast<uint32_t>(rgb.g) << 8) | static_cast<uint32_t>(rgb.b);
    }

    return Color((hexColor >> 16) & 0xFF, (hexColor >> 8) & 0xFF, hexColor & 0xFF);
  }

  std::string toPLY(const Bonxai::Point3D& point)
  {
    double uncertainty_categories = expected_shannon_entropy<double>(probabilities);
    return fmt::format("{} {} {}\n", XYZtoPLY(point), RGBtoPLY(toColor()), uncertainty_categories);
  }

  static std::string getHeaderPLY()
  {
    return fmt::format("{}\n{}\nproperty float uncertainty_categories", getXYZheader(), getRGBheader());
  }
};

struct SemanticsInstances
{
  std::vector<InstanceID_t> instances_candidates;
  std::vector<uint32_t> instances_votes;

  SemanticsInstances(){};

  void update(const pcl::PointXYZSemantics& pcl)
  {
    SemanticMap& semantics = SemanticMap::get_instance();
    InstanceID_t thisGlobalID = semantics.localToGlobalInstance(pcl.instance_id);

    auto it = std::find(instances_candidates.begin(), instances_candidates.end(), thisGlobalID);

    if (it != instances_candidates.end())
    {
      // instances_votes[std::distance(instances_candidates.begin(), it)]+=1;
      // if(thisGlobalID != 0){
      instances_votes[std::distance(instances_candidates.begin(), it)] += 1;
      //}
    }
    else
    {
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
    std::vector<double> probsBestInstance =
        semantics.globalSemanticMap[instances_candidates[idxMaxVotes]].probabilities;

    auto itProbs = std::max_element(probsBestInstance.begin(), probsBestInstance.end());

    // InstanceID_t bestInstance = instances_candidates[idxMaxVotes];
    InstanceID_t bestInstance = getMostRepresentativeInstance();
    // Set the color of the best object category of the most probable instance
    // uint32_t hexColor = semantics.indexToHexColor(std::distance(probsBestInstance.begin(), itProbs));
    // Set a unique color for the most probable instance
    uint32_t hexColor = semantics.indexToHexColor(bestInstance);

    if (bestInstance == 0)
    {
      hexColor = 0xbcbcbc;
    } /*
     if (instances_candidates[std::distance(instances_votes.begin(), itInstances)] == 0){
       hexColor = 0xbcbcbc;
     }
     if ((std::distance(probsBestInstance.begin(), itProbs) ==
         (semantics.default_categories.size() - 1)) && !(instances_candidates[idxMaxVotes] == 0)){
           hexColor = 0xbcbcbc;
         }*/

    return Color((hexColor >> 16) & 0xFF, (hexColor >> 8) & 0xFF, hexColor & 0xFF);
  }

  void updateCandidatesAndVotes()
  {
    SemanticMap& semantics = SemanticMap::get_instance();

    std::vector<InstanceID_t> candidates_temp;
    std::map<InstanceID_t, uint32_t> combining_instances;

    for (InstanceID_t i = 0; i < instances_candidates.size(); i++)
    {
      if (semantics.globalSemanticMap[instances_candidates[i]].pointsTo == -1)
      {
        candidates_temp.push_back(instances_candidates[i]);
      }
      else
      {
        candidates_temp.push_back(semantics.globalSemanticMap[instances_candidates[i]].pointsTo);
      }
    }

    for (InstanceID_t i = 0; i < candidates_temp.size(); i++)
    {
      combining_instances[candidates_temp[i]] += instances_votes[i];
    }

    instances_candidates.clear();
    instances_votes.clear();

    for (const std::pair<InstanceID_t, uint32_t>& instance : combining_instances)
    {
      instances_candidates.push_back(instance.first);
      instances_votes.push_back(instance.second);
    }
  }

  std::vector<double> getTotalProbability()
  {
    SemanticMap& semantics = SemanticMap::get_instance();
    std::vector<double> probabilities;
    size_t total_votes = 0;

    for (InstanceID_t i = 0; i < instances_votes.size(); i++)
    {
      total_votes += instances_votes[i];
    }

    for (InstanceID_t i = 0; i < instances_candidates.size(); i++)
    {
      for (InstanceID_t j = 0; j < semantics.default_categories.size(); j++)
      {
        if (i == 0)
        {
          probabilities.push_back(double(instances_votes[i]) / total_votes *
                                  semantics.globalSemanticMap[instances_candidates[i]].probabilities[j]);
        }
        else
        {
          probabilities[j] += double(instances_votes[i]) / total_votes *
                              semantics.globalSemanticMap[instances_candidates[i]].probabilities[j];
        }
      }
    }

    return probabilities;
  }

  InstanceID_t getMostRepresentativeInstance()
  {
    // auto itInstances = std::max_element(instances_votes.begin(), instances_votes.end());
    // InstanceID_t idxMaxVotes = std::distance(instances_votes.begin(), itInstances);

    InstanceID_t idxMaxVotes1 = 0;

    if (instances_votes.size() > 1)
    {
      InstanceID_t idxMaxVotes2 = 0;

      uint32_t max1 = instances_votes[0];
      uint32_t max2 = std::numeric_limits<uint32_t>::min();

      for (InstanceID_t i = 1; i < instances_votes.size(); ++i)
      {
        if (instances_votes[i] > max1)
        {
          // Update the second largest before updating the largest
          max2 = max1;
          idxMaxVotes2 = idxMaxVotes1;
          max1 = instances_votes[i];
          idxMaxVotes1 = i;
        }
        else if (instances_votes[i] > max2)
        {
          max2 = instances_votes[i];
          idxMaxVotes2 = i;
        }
      }

      if ((instances_candidates[idxMaxVotes1] == 0) && (max1 * 0.2 < max2))
      {
        idxMaxVotes1 = idxMaxVotes2;
      }
    }

    return instances_candidates[idxMaxVotes1];
  }

  std::string toPLY(const Bonxai::Point3D& point)
  {
    updateCandidatesAndVotes();
    std::vector<double> total_probability = getTotalProbability();
    InstanceID_t instanceid = getMostRepresentativeInstance();
    double uncertainty_instances = expected_shannon_entropy<uint32_t>(instances_votes);
    double uncertainty_categories = expected_shannon_entropy<double>(total_probability);
    return fmt::format("{} {} {} {} {}\n",
                       XYZtoPLY(point),
                       RGBtoPLY(toColor()),
                       instanceid,
                       uncertainty_instances,
                       uncertainty_categories);
  }

  static std::string getHeaderPLY()
  {
    return fmt::format(
        "{}\n{}\nproperty int instanceid\nproperty float uncertainty_instances\nproperty float uncertainty_categories",
        getXYZheader(),
        getRGBheader());
  }
};

struct RGBSemanticsInstances
{
  Color rgb;
  std::vector<InstanceID_t> instances_candidates;
  std::vector<uint32_t> instances_votes;

  RGBSemanticsInstances(){};

  void update(const pcl::PointXYZRGBSemantics& pcl)
  {
    SemanticMap& semantics = SemanticMap::get_instance();
    InstanceID_t thisGlobalID = semantics.localToGlobalInstance(pcl.instance_id);

    auto it = std::find(instances_candidates.begin(), instances_candidates.end(), thisGlobalID);

    if (it != instances_candidates.end())
    {
      instances_votes[std::distance(instances_candidates.begin(), it)] += 1;
      /*if(thisGlobalID != 0){
        instances_votes[std::distance(instances_candidates.begin(), it)]+=1;
      }*/
    }
    else
    {
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
    std::vector<double> probsBestInstance =
        semantics.globalSemanticMap[instances_candidates[idxMaxVotes]].probabilities;

    auto itProbs = std::max_element(probsBestInstance.begin(), probsBestInstance.end());

    // InstanceID_t bestInstance = instances_candidates[idxMaxVotes];
    InstanceID_t bestInstance = getMostRepresentativeInstance();
    // Set the color of the best object category of the most probable instance
    // uint32_t hexColor = semantics.indexToHexColor(std::distance(probsBestInstance.begin(), itProbs));
    // Set a unique color for the most probable instance
    uint32_t hexColor = semantics.indexToHexColor(bestInstance);

    // hexColor = (static_cast<uint32_t>(rgb.r) << 16) | (static_cast<uint32_t>(rgb.g) << 8) |
    // static_cast<uint32_t>(rgb.b);

    // if (std::distance(probsBestInstance.begin(), itProbs) == (semantics.default_categories.size() - 1)){
    if (bestInstance == 0)
    {
      hexColor =
          (static_cast<uint32_t>(rgb.r) << 16) | (static_cast<uint32_t>(rgb.g) << 8) | static_cast<uint32_t>(rgb.b);
    } /*
     if (instances_candidates[std::distance(instances_votes.begin(), itInstances)] == 0){
       hexColor = 0xbcbcbc;
     }
     if ((std::distance(probsBestInstance.begin(), itProbs) ==
         (semantics.default_categories.size() - 1)) && !(instances_candidates[idxMaxVotes] == 0)){
           hexColor = 0xbcbcbc;
         }*/

    return Color((hexColor >> 16) & 0xFF, (hexColor >> 8) & 0xFF, hexColor & 0xFF);
  }

  void updateCandidatesAndVotes()
  {
    SemanticMap& semantics = SemanticMap::get_instance();

    std::vector<InstanceID_t> candidates_temp;
    std::map<InstanceID_t, uint32_t> combining_instances;

    for (InstanceID_t i = 0; i < instances_candidates.size(); i++)
    {
      if (semantics.globalSemanticMap[instances_candidates[i]].pointsTo == -1)
      {
        candidates_temp.push_back(instances_candidates[i]);
      }
      else
      {
        candidates_temp.push_back(semantics.globalSemanticMap[instances_candidates[i]].pointsTo);
      }
    }

    for (InstanceID_t i = 0; i < candidates_temp.size(); i++)
    {
      combining_instances[candidates_temp[i]] += instances_votes[i];
    }

    instances_candidates.clear();
    instances_votes.clear();

    for (const std::pair<InstanceID_t, uint32_t>& instance : combining_instances)
    {
      instances_candidates.push_back(instance.first);
      instances_votes.push_back(instance.second);
    }
  }

  std::vector<double> getTotalProbability()
  {
    SemanticMap& semantics = SemanticMap::get_instance();
    std::vector<double> probabilities;
    size_t total_votes = 0;

    for (InstanceID_t i = 0; i < instances_votes.size(); i++)
    {
      total_votes += instances_votes[i];
    }

    for (InstanceID_t i = 0; i < instances_candidates.size(); i++)
    {
      for (InstanceID_t j = 0; j < semantics.default_categories.size(); j++)
      {
        if (i == 0)
        {
          probabilities.push_back(double(instances_votes[i]) / total_votes *
                                  semantics.globalSemanticMap[instances_candidates[i]].probabilities[j]);
        }
        else
        {
          probabilities[j] += double(instances_votes[i]) / total_votes *
                              semantics.globalSemanticMap[instances_candidates[i]].probabilities[j];
        }
      }
    }

    return probabilities;
  }

  InstanceID_t getMostRepresentativeInstance()
  {
    // auto itInstances = std::max_element(instances_votes.begin(), instances_votes.end());
    // InstanceID_t idxMaxVotes = std::distance(instances_votes.begin(), itInstances);

    InstanceID_t idxMaxVotes1 = 0;

    if (instances_votes.size() > 1)
    {
      InstanceID_t idxMaxVotes2 = 0;

      uint32_t max1 = instances_votes[0];
      uint32_t max2 = std::numeric_limits<uint32_t>::min();

      for (InstanceID_t i = 1; i < instances_votes.size(); ++i)
      {
        if (instances_votes[i] > max1)
        {
          // Update the second largest before updating the largest
          max2 = max1;
          idxMaxVotes2 = idxMaxVotes1;
          max1 = instances_votes[i];
          idxMaxVotes1 = i;
        }
        else if (instances_votes[i] > max2)
        {
          max2 = instances_votes[i];
          idxMaxVotes2 = i;
        }
      }

      if ((instances_candidates[idxMaxVotes1] == 0) && (max1 * 0.2 < max2))
      {
        idxMaxVotes1 = idxMaxVotes2;
      }
    }

    return instances_candidates[idxMaxVotes1];
  }

  std::string toPLY(const Bonxai::Point3D& point)
  {
    updateCandidatesAndVotes();
    std::vector<double> total_probability = getTotalProbability();
    InstanceID_t instanceid = getMostRepresentativeInstance();
    double uncertainty_instances = expected_shannon_entropy<uint32_t>(instances_votes);
    double uncertainty_categories = expected_shannon_entropy<double>(total_probability);
    return fmt::format("{} {} {} {} {}\n",
                       XYZtoPLY(point),
                       RGBtoPLY(toColor()),
                       instanceid,
                       uncertainty_instances,
                       uncertainty_categories);
  }

  static std::string getHeaderPLY()
  {
    return fmt::format(
        "{}\n{}\nproperty int instanceid\nproperty float uncertainty_instances\nproperty float uncertainty_categories",
        getXYZheader(),
        getRGBheader());
  }
};

inline std::string XYZtoPLY(const Point3D& point)
{
  return fmt::format("{} {} {}", point.x, point.y, point.z);
}

inline std::string RGBtoPLY(const Color& rgb)
{
  return fmt::format("{} {} {}", rgb.r, rgb.g, rgb.b);
}

inline std::string getXYZheader()
{
  return "property float x\nproperty float y\nproperty float z";
}

inline std::string getRGBheader()
{
  return "property uchar red\nproperty uchar green\nproperty uchar blue";
}

}  // namespace Bonxai
