#pragma once

#include <vector>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <set>
#include <optional>

#include <eigen3/Eigen/Core>
#include "bonxai/bonxai.hpp"
#include <bonxai_map/pcl_utils.hpp>
#include <bonxai_map/data_modes.hpp>
#include <bonxai_map/probabilistic_map_templated.hpp>

#include <bonxai_map/logging.hpp>

struct BoundingBox3D {
    float minX = std::numeric_limits<float>::infinity();
    float minY = std::numeric_limits<float>::infinity();
    float minZ = std::numeric_limits<float>::infinity();
    float maxX = -std::numeric_limits<float>::infinity();
    float maxY = -std::numeric_limits<float>::infinity();
    float maxZ = -std::numeric_limits<float>::infinity();
};

struct SemanticObject
{
  // Note: For now, it is supposed that in the globalSemanticMap, instances are not going to disappear.
  // Otherwise, it should be considered, as the instanceID cannot be the globalSemanticMap.size()+1
  std::string instanceID;
  std::vector<double> probabilities;
  uint32_t numberObservations = 1;
  BoundingBox3D bbox;

  std::optional<std::unordered_set<Bonxai::CoordT>> localGeometry;

  int32_t pointsTo = -1; // In case a semantic object is integrated with another, the pointsTo variable
  // need to be set with the instanceID of the main object, hence if pointsTo is not empty, it won't check
  // the data in this SemanticObject, but instead it will check the data in the instanceID set in pointsTo.

  SemanticObject(size_t numCategories, INSTANCEIDT _instanceID)
    : probabilities(numCategories, 0), instanceID("obj" + std::to_string(_instanceID))
  {}
  SemanticObject(const std::vector<double>& _probabilities, INSTANCEIDT _instanceID)
    : probabilities(_probabilities), instanceID("obj" + std::to_string(_instanceID))
  {}
  SemanticObject(size_t numCategories, INSTANCEIDT _instanceID, BoundingBox3D _bbox)
    : probabilities(numCategories, 0), instanceID("obj" + std::to_string(_instanceID)), bbox(_bbox)
  {}
  SemanticObject(const std::vector<double>& _probabilities, INSTANCEIDT _instanceID, BoundingBox3D _bbox)
    : probabilities(_probabilities), instanceID("obj" + std::to_string(_instanceID)), bbox(_bbox)
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

  template <typename DataT>
  class BonxaiQuery{

    public:

      static void createAccessor(Bonxai::ProbabilisticMapT<DataT>* _bonxai){
        bonxai = _bonxai;
        accessor.emplace(_bonxai->grid()->createAccessor());
      }

      // Note that, before calling this function, the accessor object has to be created beforehand.
      static typename Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>::Accessor& getAccessor()
      {
        return *accessor;
      }

      static Bonxai::ProbabilisticMapT<DataT>* getBonxai(){
        return bonxai;
      }

    private:
      inline static Bonxai::ProbabilisticMapT<DataT>* bonxai;
      inline static std::optional<typename Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>::Accessor> accessor;

  };

  bool is_initialized();
  void initialize(std::vector<std::string> dataset_categories, Bonxai::ProbabilisticMap& _bonxai, Bonxai::DataMode mode);
  uint32_t getCurrentActiveInstances();
  void setLocalSemanticMap(const std::vector<SemanticObject>& localMap);
  INSTANCEIDT localToGlobalInstance(INSTANCEIDT localInstance);
  uint32_t indexToHexColor(INSTANCEIDT index);
  void updateCategoryProbability(SemanticObject& semanticObject,
                                 const std::string& categoryName,
                                 double probability);
  bool checkBBoxIntersect(const BoundingBox3D& box1, const BoundingBox3D& box2);
  void updateBBoxBounds(BoundingBox3D& original, const BoundingBox3D& update);

  template <typename DataT>
  double compute3DIoU(const BoundingBox3D& globalBbox, INSTANCEIDT globalID, const std::unordered_set<Bonxai::CoordT>& localVoxels){

    std::vector<Bonxai::CoordT> globalVoxels = listOfVoxelsInsideBBox<DataT>(globalBbox, globalID);
    std::vector<Bonxai::CoordT> localVoxels_;
    localVoxels_.assign(localVoxels.begin(), localVoxels.end());

    if(globalVoxels.size() == 0 || localVoxels_.size() == 0){
      BONXAI_INFO("Empty obj caso solved!");
      std::vector<Bonxai::CoordT> globalVoxels = listOfVoxelsInsideBBox<DataT>(globalBbox, globalID);
    }

    auto orderFunc = [](const Bonxai::CoordT& c1, const Bonxai::CoordT& c2)
      {
        return c1.x < c2.x || (c1.x == c2.x && c1.y < c2.y) || (c1.x == c2.x && c1.y == c2.y && c1.z < c2.z); 
      };

    //BONXAI_INFO("Global voxels size: {} / Local voxels size {}", globalVoxels.size(), localVoxels_.size());
    std::sort(globalVoxels.begin(), globalVoxels.end(), orderFunc);
    std::sort(localVoxels_.begin(), localVoxels_.end(), orderFunc);

    std::vector<Bonxai::CoordT> intersection_;
    std::vector<Bonxai::CoordT> union_;

    std::set_intersection(globalVoxels.begin(), globalVoxels.end(), localVoxels_.begin(), localVoxels_.end(), std::back_inserter(intersection_), orderFunc);
    std::set_union(globalVoxels.begin(), globalVoxels.end(), localVoxels_.begin(), localVoxels_.end(), std::back_inserter(union_), orderFunc);
    
    double iouLocal = 0.;
    if (localVoxels_.size() > 0){
      iouLocal = (double)intersection_.size() / localVoxels_.size();
    }
    double iouGlobal = 0.;
    if (globalVoxels.size() > 0){
      iouGlobal = (double)intersection_.size() / globalVoxels.size();
    }
    double iou = 0.;
    if (union_.size() > 0){
      iou = (double)intersection_.size() / union_.size();
    }

    //BONXAI_INFO("IoU --> Local: {} // Global: {} // Total: {}", iouLocal, iouGlobal, iou);
    if(iouLocal > 0.5 || iouGlobal > 0.5 || iou > 0.3){
      iou = 0.35;
    }
    
    return iou;
    
  }

  template <typename DataT>
  double compute3DIoU(const BoundingBox3D& bBox1, INSTANCEIDT id1, const BoundingBox3D& bBox2, INSTANCEIDT id2){

    std::vector<Bonxai::CoordT> voxels1 = listOfVoxelsInsideBBox<DataT>(bBox1, id1);
    std::vector<Bonxai::CoordT> voxels2 = listOfVoxelsInsideBBox<DataT>(bBox2, id2);

    if(voxels1.size() == 0 || voxels2.size() == 0){
      BONXAI_INFO("Empty obj!");
      std::vector<Bonxai::CoordT> voxels1 = listOfVoxelsInsideBBox<DataT>(bBox1, id1);
    }

    auto orderFunc = [](const Bonxai::CoordT& c1, const Bonxai::CoordT& c2)
      {
        return c1.x < c2.x || (c1.x == c2.x && c1.y < c2.y) || (c1.x == c2.x && c1.y == c2.y && c1.z < c2.z); 
      };

    //BONXAI_INFO("Global voxels size: %d / Local voxels size %d", globalVoxels.size(), localVoxels_.size());
    std::sort(voxels1.begin(), voxels1.end(), orderFunc);
    std::sort(voxels2.begin(), voxels2.end(), orderFunc);

    std::vector<Bonxai::CoordT> intersection_;
    std::vector<Bonxai::CoordT> union_;

    std::set_intersection(voxels1.begin(), voxels1.end(), voxels2.begin(), voxels2.end(), std::back_inserter(intersection_), orderFunc);
    std::set_union(voxels1.begin(), voxels1.end(), voxels2.begin(), voxels2.end(), std::back_inserter(union_), orderFunc);
    
    //BONXAI_INFO("GLOBAL WITH GLOBAL: {} voxels en 1 y {} voxels en 2", voxels1.size(), voxels2.size());
    double iouLocal = 0.;
    if (voxels1.size() > 0){
      iouLocal = (double)intersection_.size() / voxels1.size();
    }
    double iouGlobal = 0.;
    if (voxels2.size() > 0){
      iouGlobal = (double)intersection_.size() / voxels2.size();
    }
    double iou = 0.;
    if (union_.size() > 0){
      iou = (double)intersection_.size() / union_.size();
    }

    if(iouLocal > 0.5 || iouGlobal > 0.5 || iou > 0.3){
      iou = 0.35;
    }
    
    return iou;
    
  }

  template <typename DataT>
   std::vector<Bonxai::CoordT> listOfVoxelsInsideBBox(const BoundingBox3D& bbox, INSTANCEIDT id){
        
    std::vector<Bonxai::CoordT> cellsInside;

    Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>* bonxai = BonxaiQuery<DataT>::getBonxai()->grid();
    
    const Bonxai::CoordT coordMin = bonxai->posToCoord(Bonxai::Point3D(bbox.minX - bonxai->resolution,
                                                                       bbox.minY - bonxai->resolution,
                                                                       bbox.minZ - bonxai->resolution));
    const Bonxai::CoordT coordMax = bonxai->posToCoord(Bonxai::Point3D(bbox.maxX + bonxai->resolution,
                                                                       bbox.maxY + bonxai->resolution, 
                                                                       bbox.maxZ + bonxai->resolution));

    // Iterate over all points inside the bounding box
    for (int x = coordMin.x; x <= coordMax.x; x++) {
        for (int y = coordMin.y; y <= coordMax.y; y++) {
            for (int z = coordMin.z; z <= coordMax.z; z++) {
                Bonxai::CoordT coord = Bonxai::CoordT{x,y,z};
                Bonxai::ProbabilisticCell<DataT>* cell = BonxaiQuery<DataT>::getAccessor().value(coord);
                if(!cell)
                  continue;
                //BONXAI_INFO("Inicio: %d, fin: %d", cell->data.instances_candidates.begin(), cell->data.instances_candidates.end());
                auto it = std::find(cell->data.instances_candidates.begin(), cell->data.instances_candidates.end(), id);
                if (it != cell->data.instances_candidates.end()) {
                  size_t idx = std::distance(cell->data.instances_candidates.begin(), it);
                  cellsInside.push_back(coord);
                  
                }
            }
        }
    }
    
    return cellsInside;
  }

  template <typename DataT>
  void refineGlobalSemanticMap(){
    for (INSTANCEIDT i = 1; i < globalSemanticMap.size(); i++){

      SemanticObject& firstInstance = globalSemanticMap[i];
      if (firstInstance.pointsTo != -1)
      {
        break; 
      }

      for (INSTANCEIDT j = i+1; j < globalSemanticMap.size(); j++){
        SemanticObject& secondInstance = globalSemanticMap[j];
        if (secondInstance.pointsTo == -1 && checkBBoxIntersect(firstInstance.bbox, secondInstance.bbox)){

            double iou = compute3DIoU<DataT>(firstInstance.bbox, i, secondInstance.bbox, j);
            if (iou > 0.3)
            {
              
              secondInstance.pointsTo = i;
              for (size_t k = 0; k < firstInstance.probabilities.size(); k++)
              {
                firstInstance.probabilities[k] += secondInstance.probabilities[k];
              }
              updateBBoxBounds(firstInstance.bbox, secondInstance.bbox);
              firstInstance.numberObservations += secondInstance.numberObservations;
            }
        }
      }
    }

    for (INSTANCEIDT i = 1; i < globalSemanticMap.size(); i++){
      if(globalSemanticMap[i].pointsTo == -1 && globalSemanticMap[i].numberObservations == 1)
      {
        globalSemanticMap[i].pointsTo = 0;
      }
    }
  }

  template <typename DataT>
  void integrateNewSemantics(const std::vector<SemanticObject>& localMap)
    {
      
      uint8_t integrated = 0;
      uint8_t added = 0;
      
      lastMapLocalToGlobal.resize(localMap.size());
      if(globalSemanticMap.empty()){
        globalSemanticMap.push_back(SemanticObject(localMap[0].probabilities, 0, localMap[0].bbox));
      }
      else{
        for (size_t i = 0; i < localMap[0].probabilities.size(); i++)
          {
            globalSemanticMap[0].probabilities[i] += localMap[0].probabilities[i];
          }
          updateBBoxBounds(globalSemanticMap[0].bbox, localMap[0].bbox);
      }

      // First, integrate local "unknown" with global "unknown". They are always the 0-index
      lastMapLocalToGlobal[0] = 0;
      // Further integration of unknown is required: bbox, probabilities, etc. but to be decided yet

      const INSTANCEIDT currentInstancesNumber = globalSemanticMap.size();

      // Both loops start at 1 to skip "unknown" class
      for (INSTANCEIDT localInstanceID = 1; localInstanceID < localMap.size(); localInstanceID++)
      {
        bool fused = false;
        const SemanticObject& localInstance = localMap[localInstanceID];

        if(!localInstance.localGeometry.has_value()) break;

        std::vector<double>::const_iterator itLocal = std::max_element(localInstance.probabilities.begin(), localInstance.probabilities.end());
        uint8_t localClassIdx = std::distance(localInstance.probabilities.begin(), itLocal);

        for (INSTANCEIDT globalInstanceID = 1; globalInstanceID < currentInstancesNumber; globalInstanceID++)
        {
          SemanticObject& globalInstance = globalSemanticMap[globalInstanceID];
          std::vector<double>::iterator itGlobal = std::max_element(globalInstance.probabilities.begin(), globalInstance.probabilities.end());
          uint8_t globalClassIdx = std::distance(globalInstance.probabilities.begin(), itGlobal);

          if (globalInstance.pointsTo == -1 && checkBBoxIntersect(localInstance.bbox, globalInstance.bbox)){

            // Compute 3D IoU... To Do... meanwhile, check if the best category of each one is the same
            //if (localClassIdx == globalClassIdx)
            double iou = compute3DIoU<DataT>(globalInstance.bbox, globalInstanceID, localInstance.localGeometry.value());
            if (iou > 0.3)
            {
              //BONXAI_INFO("Integrando {} local con {} global con IoU de {}", default_categories[localClassIdx].c_str(), default_categories[globalClassIdx].c_str(), iou);
              for (size_t i = 0; i < globalInstance.probabilities.size(); i++)
              {
                globalInstance.probabilities[i] += localInstance.probabilities[i];
              }
              lastMapLocalToGlobal[localInstanceID] = globalInstanceID;
              //int before_update = listOfVoxelsInsideBBox<DataT>(globalInstance.bbox, globalInstanceID).size();
              updateBBoxBounds(globalInstance.bbox, localInstance.bbox);
              //int after_update = listOfVoxelsInsideBBox<DataT>(globalInstance.bbox, globalInstanceID).size();
              //if (before_update > after_update){
              //  BONXAI_INFO("Bounding box updated wrongly!");
              //}
              globalInstance.numberObservations += 1;
              fused = true;
              integrated += 1;
              break;
            }
          }
          
        }
        if (!fused)
        {
          lastMapLocalToGlobal[localInstanceID] = globalSemanticMap.size();
          globalSemanticMap.push_back(SemanticObject(localInstance.probabilities, globalSemanticMap.size(), localInstance.bbox));
          added += 1;
        }
      }
      BONXAI_INFO("Integrating {} new local objects: {} integrated and {} added", localMap.size(), integrated, added);
    }

  template <typename DataT, typename PointCloudTypeT>
  void addInstancesGeometryToLocalSemanticMap(std::vector<SemanticObject>& localMap, const PointCloudTypeT& pc){

    Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>* bonxai = BonxaiQuery<DataT>::getBonxai()->grid();

    for (size_t i = 0; i < pc.points.size(); i++) {

      if (!localMap[pc.points[i].instance_id].localGeometry.has_value()){
        localMap[pc.points[i].instance_id].localGeometry.emplace();
      }

      localMap[pc.points[i].instance_id].localGeometry.value().insert(bonxai->posToCoord(Bonxai::Point3D(pc.points[i].x, pc.points[i].y, pc.points[i].z)));


      // Update min bounds
      localMap[pc.points[i].instance_id].bbox.minX = std::min(pc.points[i].x, localMap[pc.points[i].instance_id].bbox.minX);
      localMap[pc.points[i].instance_id].bbox.minY = std::min(pc.points[i].y, localMap[pc.points[i].instance_id].bbox.minY);
      localMap[pc.points[i].instance_id].bbox.minZ = std::min(pc.points[i].z, localMap[pc.points[i].instance_id].bbox.minZ);

      // Update max bounds
      localMap[pc.points[i].instance_id].bbox.maxX = std::max(pc.points[i].x, localMap[pc.points[i].instance_id].bbox.maxX);
      localMap[pc.points[i].instance_id].bbox.maxY = std::max(pc.points[i].y, localMap[pc.points[i].instance_id].bbox.maxY);
      localMap[pc.points[i].instance_id].bbox.maxZ = std::max(pc.points[i].z, localMap[pc.points[i].instance_id].bbox.maxZ);

  }
  }

  template <typename DataT>
  std::set<INSTANCEIDT> getCurrentVisibleInstances(double minOccupancyZ, double maxOccupancyZ){
    std::vector<DataT> cell_data;
    std::vector<Bonxai::Point3D> cell_points;
    Bonxai::ProbabilisticMapT<DataT>* bonxai = BonxaiQuery<DataT>::getBonxai();
    bonxai->getOccupiedVoxels(cell_points, cell_data);

    std::set<INSTANCEIDT> visibleInstances;

    for (size_t i = 0; i < cell_points.size(); i++)
      {
        const auto& voxel = cell_points[i];
        const auto& data = cell_data[i];

        if (voxel.z >= minOccupancyZ && voxel.z <= maxOccupancyZ)
        {
          auto itInstances = std::max_element(data.instances_votes.begin(), data.instances_votes.end());
          auto idxMaxVotes = std::distance(data.instances_votes.begin(), itInstances);
          INSTANCEIDT bestInstanceID = data.instances_candidates[idxMaxVotes];
          if(globalSemanticMap[bestInstanceID].pointsTo == -1){
            visibleInstances.insert(bestInstanceID);
          }
          else{
            visibleInstances.insert(globalSemanticMap[bestInstanceID].pointsTo);
          }
          
        }
      }
    return visibleInstances;
  } 

private:

  std::vector<INSTANCEIDT> lastMapLocalToGlobal;
  std::vector<std::uint32_t> color_palette;
  bool initialized = false;
  double kld_threshold;

  double computeKLD(const std::vector<double>& P, const std::vector<double>& Q);

};