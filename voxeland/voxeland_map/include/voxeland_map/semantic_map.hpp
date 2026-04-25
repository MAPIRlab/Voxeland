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

    std::map<InstanceID_t, SemanticObject> globalSemanticMap;
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

    void integrateNewSemantics(const std::vector<SemanticObject>& localMap,
                               const std::set<Bonxai::IndicesT>& voxelizedLocalPointCloud,
                               float sensorX = 0.0f,
                               float sensorY = 0.0f,
                               float sensorZ = 0.0f);

    // options for global refinement
    struct FusionOptions
    {
        // fusion parameters
        float semSimThr = 0.6;      // how similar the class distributions must be to allow fusing
        float votesThr = 0.3;       // when retrieving the geometry that corresponds to this instance, which proportion of votes must a voxel have to count
        uint coarseningFactor = 2;  // downsampling factor for the pointclouds when calculating IoU
        float iosThr = 0.4;         // exactly what you think this is
        float iouThr = 0.3;         // exactly what you think this is
        float iouSkipThr = 0.7;     // if IoU is sufficiently large, the instances are overlapping entirely and we don't care about the other metrics.
                                    // This is necessary because we can sometimes end up with two overlapping instances which correspond to one class each,
                                    // and every new observation always fuses with the instance which already agrees with its class.
                                    // This can lead to a very low semantic similarity, preventing fusion
    };

    struct RemovalOptions
    {
        uint minimumObservations = 5;
        uint32_t secondsSinceLastObs = 10;
        uint minimumVoxels = 0;
    };

    void refineGlobalSemanticMap(uint32_t timestamp, const RemovalOptions* removeOptions = nullptr, const FusionOptions* fuseOptions = nullptr);
    void deleteOldInstances();

    std::pair<double, double> compute3DIoU(const std::set<Bonxai::IndicesT>& voxels1,
                                           const std::set<Bonxai::IndicesT>& voxels2,
                                           uint coarsening_factor = 3);

    double computeIoV(const std::set<Bonxai::IndicesT>& localVoxels,
                      const std::set<Bonxai::IndicesT>& globalInstance,
                      const std::set<Bonxai::IndicesT>& localInstance,
                      uint coarsening_factor);

    template <typename DataT>
    std::set<Bonxai::IndicesT> listOfVoxelsInObject(const SemanticObject& object, std::optional<double> probabilityThr = std::nullopt);

    void setLocalSemanticMap(const std::vector<SemanticObject>& localMap);
    std::vector<InstanceID_t>& localToGlobalInstance(InstanceID_t localInstance);
    uint32_t indexToHexColor(InstanceID_t index);
    void RandomizeColorsOrder();

    // Updated methods to work with CategoryManager
    void updateCategoryProbability(SemanticObject& semanticObject, const std::string& categoryName, double probability);
    CategoryManager::CategoryIndex addCategory(const std::string& categoryName);
    CategoryManager::CategoryIndex getCategoryIndex(const std::string& categoryName) const;
    std::string getCategoryName(CategoryManager::CategoryIndex index) const;
    size_t getNumCategories() const;

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

    template <typename DataT, typename PointCloudTypeT>
    void addInstancesGeometryToLocalSemanticMap(std::vector<SemanticObject>& localMap, const PointCloudTypeT& pc);

    template <typename DataT>
    std::set<InstanceID_t> getCurrentVisibleInstances(double minOccupancyZ, double maxOccupancyZ);

    // JSON utils
    void loadInstancesFromFile(const std::filesystem::path& path);
    nlohmann::json mapToJSON();
    void updateSemanticMapResultsFromJSON(const nlohmann::json& data_json);
    nlohmann::json appearancesToJson();

    struct DebugInformation
    {
        std::vector<std::set<Bonxai::IndicesT>> mostRecentClusters;
    };
    DebugInformation debugInfo;

    FusionOptions defaultFuseOptions;
    RemovalOptions defaultRemoveOptions;
private:
    std::vector<std::vector<InstanceID_t>> lastMapLocalToGlobal;
    std::vector<std::uint32_t> color_palette;
    std::vector<std::uint32_t> color_palette_offsets;  // used to re-randomize the colors in case of an unlucky coincidence on nearby instances
    bool initialized = false;
    double kld_threshold;
    voxeland::DataMode currentMode;

    double computeKLD(const std::vector<double>& P, const std::vector<double>& Q);

    SemanticObject& CreateGlobalInstance(std::optional<InstanceID_t> forceID = std::nullopt);
    void fuseSemanticObjects(SemanticObject& firstInstance, const SemanticObject& secondInstance);

    void updateAlphaCategories(SemanticObject& original, const SemanticObject& update);
    void updateAppearancesTimestamps(SemanticObject& original, const SemanticObject& update);

    std::vector<size_t> getCurrentInstanceIDs();  // when iterating over the map directly, we can't create or delete instances (due to iterator invalidation)
                                                  // so, when we need to modify, we can get the list of keys and just access the map element by element without iterators
};

//----------------------------------------------------------
//  IMPLEMENTATION OF FUNCTION TEMPLATES
//----------------------------------------------------------

#include "semantic_map_impl.cpp"  // IWYU pragma: keep