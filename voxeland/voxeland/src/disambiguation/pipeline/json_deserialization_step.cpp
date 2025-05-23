#include "disambiguation/pipeline/pipeline_steps.hpp"
#include "disambiguation/json_semantics.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include "nlohmann/json.hpp"
#include "voxeland_map/Utils/logging.hpp"
using json = nlohmann::json;

JsonDeserializationStep::JsonDeserializationStep(const std::string& json_file, const std::string& json_appearances_file){
    this->json_file = json_file;
    this->json_appearances_file = json_appearances_file;
}

void JsonDeserializationStep::execute() {
    VXL_INFO("[JSON_DESERIALIZATION] Executing JSON deserialization step...");
    
    auto map = context->get_semantic_map();
    serialize_map((*map));

    std::cout << map->to_string() << std::endl;

    VXL_INFO("[JSON_DESERIALIZATION] JSON deserialization step completed.");
}

/**
 * @brief Read a JSON file containing a semantic map and load it into a JsonSemanticMap object. The file must be located in the package's "params" directory.
 * 
 * @param map Reference to a JsonSemanticMap object where the data will be loaded.
 * 
 * @throws std::exception If the file cannot be found or opened.
 */
void JsonDeserializationStep::serialize_map(JsonSemanticMap& map) {
    // Get the path to the package
    std::string package_path = ament_index_cpp::get_package_share_directory("voxeland");
    std::string map_file_path = package_path + "/params/" + json_file;
    std::string appearances_file_path = package_path + "/params/" + json_appearances_file;

    // Open the json file (if found)
    std::ifstream map_file(map_file_path);
    if(!map_file.is_open()){
        VXL_ERROR("Cannot open file {}", map_file_path);
        throw std::exception();
    }

    std::ifstream appearances_file(appearances_file_path);
    if(!appearances_file.is_open()){
        VXL_ERROR("Cannot open file {}", json_appearances_file);
        throw std::exception();
    }
    
    // Parse the json file
    json j = json::parse(map_file);
    json apppearances_j = json::parse(appearances_file); // Timestamps json file
    json instances = j["instances"];
    
    for (json::iterator object_it = instances.begin();  object_it != instances.end(); ++object_it) {
        json object_value = object_it.value();
        std::string instance_id = object_it.key();
        JsonSemanticObject instance = serialize_instance(instance_id, object_value, apppearances_j[instance_id]);

        map.add_instance(instance);
    }
}


/**
 * @brief Parse the "bbox" JSON object into a voxeland_map/BoundingBox3D object.
 * 
 * @param bbox JSON object containing the bounding box.
 * @return BoundingBox3D Bounding box object.
*/
BoundingBox3D JsonDeserializationStep::parse_bbox(json& bbox){
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
std::map<std::string, double> JsonDeserializationStep::parse_results(json& results){
    std::map<std::string, double> results_map;
    for (json::iterator results_it = results.begin(); results_it != results.end(); ++results_it) {
        std::string category = results_it.key();
        double probability = results_it.value();
        results_map[category] = probability;
    }

    return results_map;
}

std::map<std::string, std::map<uint32_t,BoundingBox2D>> JsonDeserializationStep::parse_appearances_timestamps(json& appearances_timestamps){
    std::map<std::string, std::map<uint32_t,BoundingBox2D>> appearances_map;
    for(auto& [category, instances] : appearances_timestamps["timestamps"].items()){
        for (auto& instance : instances){
            BoundingBox2D bbox;
            bbox.centerX = instance["bbox"]["centerX"];
            bbox.centerY = instance["bbox"]["centerY"];
            bbox.sizeX = instance["bbox"]["sizeX"];
            bbox.sizeY = instance["bbox"]["sizeY"];
            appearances_map[category][instance["instance_id"]] = bbox;
        }
    }

    return appearances_map;
}

JsonSemanticObject JsonDeserializationStep::serialize_instance(const std::string& instance_id, json& instance_json, json& instance_appeareances_json) {
    JsonSemanticObject instance;
    instance.InstanceID = instance_id;
    instance.bbox = parse_bbox(instance_json["bbox"]);
    instance.appearances_timestamps = parse_appearances_timestamps(instance_appeareances_json);
    instance.n_observations = instance_json["n_observations"];
    instance.results = parse_results(instance_json["results"]);

    return instance;
}