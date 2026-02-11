#pragma once

#include <eigen3/Eigen/Core>
#include <nlohmann/json.hpp>
#include <set>
#include <vector>
#include <voxeland_map/Utils/logging.hpp>
#include <voxeland_map/data_modes.hpp>
#include <voxeland_map/pcl_utils.hpp>
#include <voxeland_map/probabilistic_map_templated.hpp>

#include "bonxai/bonxai.hpp"
#include "semantic_object.hpp"

class SemanticMap
{
public:
    SemanticMap();

    std::vector<SemanticObject> globalSemanticMap;
    std::vector<SemanticObject> lastLocalSemanticMap;

    static SemanticMap& get_instance()
    {
        static SemanticMap instance;
        return instance;
    }

    void initialize(std::vector<std::string> dataset_categories,
                    Bonxai::ProbabilisticMap& _bonxai,
                    voxeland::DataMode mode);
    bool isInitialized() { return initialized; }

    void setLocalSemanticMap(const std::vector<SemanticObject>& localMap);
    InstanceID_t localToGlobalInstance(InstanceID_t localInstance);
    uint32_t indexToHexColor(InstanceID_t index);

    // Updated methods to work with CategoryManager
    void updateCategoryProbability(SemanticObject& semanticObject, const std::string& categoryName, double probability);
    CategoryManager::CategoryIndex addCategory(const std::string& categoryName);
    CategoryManager::CategoryIndex getCategoryIndex(const std::string& categoryName) const;
    std::string getCategoryName(CategoryManager::CategoryIndex index) const;
    size_t getNumCategories() const;

    std::pair<double, double> compute3DIoU(const std::set<Bonxai::CoordT>& voxels1,
                                           const std::set<Bonxai::CoordT>& voxels2,
                                           float coarsening_factor = 3);

    double computeIoV(const std::set<Bonxai::CoordT>& localVoxels,
                      const std::set<Bonxai::CoordT>& globalInstance,
                      const std::set<Bonxai::CoordT>& localInstance,
                      float coarsening_factor);

    template <typename DataT>
    std::set<Bonxai::CoordT> listOfVoxelsInObject(const SemanticObject& object, double probabilityThr = 1.0);

    void refineGlobalSemanticMap(int nObservationsToRemove);

    void integrateNewSemantics(const std::vector<SemanticObject>& localMap,
                               const std::set<Bonxai::CoordT>& voxelizedLocalPointCloud,
                               float sensorX = 0.0f,
                               float sensorY = 0.0f,
                               float sensorZ = 0.0f);

    template <typename DataT, typename PointCloudTypeT>
    void addInstancesGeometryToLocalSemanticMap(std::vector<SemanticObject>& localMap, const PointCloudTypeT& pc);

    template <typename DataT>
    std::set<InstanceID_t> getCurrentVisibleInstances(double minOccupancyZ, double maxOccupancyZ);

    // JSON utils
    nlohmann::json mapToJSON();
    void updateSemanticMapResultsFromJSON(const nlohmann::json& data_json);
    nlohmann::json appearancesToJson();

private:
    std::vector<InstanceID_t> lastMapLocalToGlobal;
    std::vector<std::uint32_t> color_palette;
    bool initialized = false;
    double kld_threshold;
    voxeland::DataMode currentMode;
    
    std::set<Bonxai::CoordT> coarsenVoxels(const std::set<Bonxai::CoordT>& voxels, float coarsening_factor);
    double computeKLD(const std::vector<double>& P, const std::vector<double>& Q);
    
    /**
     * @brief Compute semantic similarity between two SemanticObjects using Jensen-Shannon divergence
     * 
     * Jensen-Shannon divergence is a symmetric and bounded (0-1) measure of similarity between
     * two probability distributions. Returns a similarity score where:
     * - 1.0 = identical distributions
     * - 0.0 = completely different distributions
     * 
     * @param obj1 First semantic object
     * @param obj2 Second semantic object  
     * @return Semantic similarity score in range [0, 1]
     */
    double computeSemanticSimilarity(const SemanticObject& obj1, const SemanticObject& obj2);
    bool checkBBoxIntersect(const BoundingBox3D& box1, const BoundingBox3D& box2);
    void updateBBoxBounds(BoundingBox3D& original, const BoundingBox3D& update);
    void fuseSemanticObjects(SemanticObject& firstInstance, const SemanticObject& secondInstance);

    void updateAlphaCategories(SemanticObject& original, const SemanticObject& update);
    void updateAppearancesTimestamps(SemanticObject& original, const SemanticObject& update);
};

template <typename DataT>
class BonxaiQuery
{
public:
    static void createAccessor(Bonxai::ProbabilisticMapT<DataT>* _bonxai);

    // Note that, before calling this function, the accessor object has to be created beforehand.
    static typename Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>::Accessor& getAccessor();

    static Bonxai::ProbabilisticMapT<DataT>* getBonxai() { return bonxai; }

private:
    inline static Bonxai::ProbabilisticMapT<DataT>* bonxai;
    inline static thread_local std::optional<typename Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>::Accessor> accessor;
};

//----------------------------------------------------------
//  IMPLEMENTATION OF FUNCTION TEMPLATES
//----------------------------------------------------------

#include "semantic_map_impl.cpp"  // IWYU pragma: keep