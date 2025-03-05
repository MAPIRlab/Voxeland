#include "json_semantics.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <exception>
#include <fstream>
#include <map>
#include <string>
#include "voxeland_map/Utils/logging.hpp"
#include "voxeland_map/semantics.hpp"

using json = nlohmann::json;

/**
 * @brief Read a JSON file containing a semantic map and load it into a JsonSemanticMap object. The file must be located in the package's "params" directory.
 * 
 * @param json_file Name of the JSON file containing the semantic map.
 * @return JsonSemanticMap Semantic map object.
 * 
 * @throws std::exception If the file cannot be found or opened.
 */
JsonSemanticMap JsonSemanticMap::load_map(const std::string& json_file){

    // Get the path to the package
    std::string package_path = ament_index_cpp::get_package_share_directory("voxeland");
    std::string file_path = package_path + "/params/" + json_file;

    // Open the json file (if found)
    std::ifstream file(file_path);
    if(!file.is_open()){
        VXL_ERROR("Cannot open file {}", file_path);
        throw std::exception();
    }
    
    // Parse the json file
    json j = json::parse(file);
    json instances = j["instances"];
    
    JsonSemanticMap map;
    for (json::iterator object_it = instances.begin();  object_it != instances.end(); ++object_it) {
        json object_value = object_it.value();
        JsonSemanticObject instance;

        instance.InstanceID = object_it.key();
        instance.bbox = parse_bbox(object_value["bbox"]);
        instance.n_observations = object_value["n_observations"];
        instance.results = parse_results(object_value["results"]);

        VXL_INFO("Loaded instance: {}", instance.InstanceID);
        map.instances.push_back(instance);
    }

    return map;
}

JsonSemanticObject* JsonSemanticMap::get_instance(const std::string& instanceID){
    for (JsonSemanticObject& instance : instances){
        if (instance.InstanceID == instanceID){
            return &instance;
        }
    }
    return nullptr;
}

std::vector<JsonSemanticObject>* JsonSemanticMap::get_instances(){
    return &instances;
}

const std::string JsonSemanticMap::to_string(){
    std::string str = "";
    for (JsonSemanticObject& instance : instances){
        str += "Instance: " + instance.InstanceID + "\n";
        str += "BBox: " + std::to_string(instance.bbox.minX) + ", " + std::to_string(instance.bbox.minY) + ", " + std::to_string(instance.bbox.minZ) + "\n";
        str += "       " + std::to_string(instance.bbox.maxX) + ", " + std::to_string(instance.bbox.maxY) + ", " + std::to_string(instance.bbox.maxZ) + "\n";
        str += "n_observations: " + std::to_string(instance.n_observations) + "\n";
        str += "Results: \n";
        for (std::pair<std::string, double> result : instance.results){
            str += "    " + result.first + ": " + std::to_string(result.second) + "\n";
        }
    }
    return str;
}

/**
 * @brief Parse the "bbox" JSON object into a voxeland_map/BoundingBox3D object.
 * 
 * @param bbox JSON object containing the bounding box.
 * @return BoundingBox3D Bounding box object.
*/
BoundingBox3D JsonSemanticMap::parse_bbox(json& bbox){
    json center = bbox["center"];
    json size = bbox["size"];

    BoundingBox3D box;
    box.minX = (double) center[0] - ((double) size[0] / 2);
    box.maxX = (double) center[0] + ((double) size[0] / 2);
    box.minY = (double) center[1] - ((double) size[1] / 2);
    box.maxY = (double) center[1] + ((double) size[1] / 2);
    box.minZ = (double) center[2] - ((double) size[2] / 2);
    box.maxZ = (double) center[2] + ((double) size[2] / 2);
    return box;
}

/**
 * @brief Parse the "results" JSON object into a map of category names and probabilities.
 * 
 * @param results JSON object containing the results.
 * @return std::map<std::string, double> Map of category names and probabilities.
 */
std::map<std::string, double> JsonSemanticMap::parse_results(json& results){
    std::map<std::string, double> results_map;
    for (json::iterator results_it = results.begin(); results_it != results.end(); ++results_it) {
        std::string category = results_it.key();
        double probability = results_it.value();
        results_map[category] = probability;
    }
    return results_map;
}