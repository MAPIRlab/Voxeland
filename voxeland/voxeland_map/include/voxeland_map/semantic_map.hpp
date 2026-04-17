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
    void RandomizeColorsOrder();

    // Updated methods to work with CategoryManager
    void updateCategoryProbability(SemanticObject& semanticObject, const std::string& categoryName, double probability);
    CategoryManager::CategoryIndex addCategory(const std::string& categoryName);
    CategoryManager::CategoryIndex getCategoryIndex(const std::string& categoryName) const;
    std::string getCategoryName(CategoryManager::CategoryIndex index) const;
    size_t getNumCategories() const;

    std::pair<double, double> compute3DIoU(const std::set<Bonxai::IndicesT>& voxels1,
                                           const std::set<Bonxai::IndicesT>& voxels2,
                                           uint coarsening_factor = 3);

    double computeIoV(const std::set<Bonxai::IndicesT>& localVoxels,
                      const std::set<Bonxai::IndicesT>& globalInstance,
                      const std::set<Bonxai::IndicesT>& localInstance,
                      uint coarsening_factor);

    template <typename DataT>
    std::set<Bonxai::IndicesT> listOfVoxelsInObject(const SemanticObject& object, double probabilityThr = 1.0);

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

    void refineGlobalSemanticMap(int nObservationsToRemove);

    void integrateNewSemantics(const std::vector<SemanticObject>& localMap,
                               const std::set<Bonxai::IndicesT>& voxelizedLocalPointCloud,
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

    struct DebugInformation
    {
        std::vector<std::set<Bonxai::IndicesT>> mostRecentClusters;
    };
    DebugInformation debugInfo;

private:
    std::vector<InstanceID_t> lastMapLocalToGlobal;
    std::vector<std::uint32_t> color_palette;
    std::vector<std::uint32_t> color_palette_offsets; // used to re-randomize the colors in case of an unlucky coincidence on nearby instances
    bool initialized = false;
    double kld_threshold;
    voxeland::DataMode currentMode;
    
    double computeKLD(const std::vector<double>& P, const std::vector<double>& Q);
    
    SemanticObject& CreateGlobalInstance();
    void fuseSemanticObjects(SemanticObject& firstInstance, const SemanticObject& secondInstance);

    void updateAlphaCategories(SemanticObject& original, const SemanticObject& update);
    void updateAppearancesTimestamps(SemanticObject& original, const SemanticObject& update);

    struct FusionScore
    {
        InstanceID_t fuseWithID;
        double normalizedScore;
    };
};

//----------------------------------------------------------
//  IMPLEMENTATION OF FUNCTION TEMPLATES
//----------------------------------------------------------

#include "semantic_map_impl.cpp"  // IWYU pragma: keep