#include "json_semantics.hpp"
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include "voxeland_map/Utils/logging.hpp"
#include "voxeland_map/semantics.hpp"

using json = nlohmann::json;

JsonSemanticObject* UncertainInstance::get_instance(){
    return instance;
}

std::map<std::string, std::vector<uint32_t>>* UncertainInstance::get_selected_appearances(){
    return &selected_appearances;
}

std::map<std::string, std::vector<cv_bridge::CvImagePtr>>* UncertainInstance::get_selected_images(){
    return &selected_images;
}

double UncertainInstance::get_entropy(){
    return entropy;
}

void UncertainInstance::set_selected_appearances(std::map<std::string, std::vector<uint32_t>> selected_appearances){
    this->selected_appearances = selected_appearances;
}

std::string UncertainInstance::get_final_category(){
    return final_category;
}

void UncertainInstance::set_final_category(std::string final_category){
    this->final_category = final_category;
}

std::string UncertainInstance::to_string(){
    std::string str = "Instance: " + instance->InstanceID + "\n";
    str += "Entropy: " + std::to_string(entropy) + "\n";
    str += "Selected appearances: \n";
    str += "    " + std::to_string(selected_appearances.size()) + " categories\n";
    for (auto& [category, timestamps] : selected_appearances){
        str += "    " + category + " : ";
        for (uint32_t timestamp : timestamps){
            str += std::to_string(timestamp) + ", ";
        }
        str += "\n";
    }
    return str;
}

std::map<std::string, double> sort_map(std::map<std::string, double>& map){
    std::vector<std::pair<std::string, double>> vector(map.begin(), map.end());
    std::sort(vector.begin(), vector.end(), [](const auto& a, const auto& b) {
        return a.second > b.second;
    });


    // Verificar que, efectivamnete, se ordena
    std::cout << " --- Sorted map:  ---" << std::endl;
    std::map<std::string, double> sorted_map;
    for (const auto& pair : vector){
        sorted_map[pair.first] = pair.second;
        std::cout << pair.first << " : " << pair.second << std::endl;
    }

    return sorted_map;
}


/**
 * @brief Read a JSON file containing a semantic map and load it into a JsonSemanticMap object. The file must be located in the package's "params" directory.
 * 
 * @param json_file Name of the JSON file containing the semantic map.
 * @return JsonSemanticMap Semantic map object.
 * 
 * @throws std::exception If the file cannot be found or opened.
 */
JsonSemanticMap JsonSemanticMap::load_map(const std::string& json_file, const std::string& json_appearances_file){

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
    
    JsonSemanticMap map;
    for (json::iterator object_it = instances.begin();  object_it != instances.end(); ++object_it) {
        json object_value = object_it.value();
        JsonSemanticObject instance;

        instance.InstanceID = object_it.key();
        instance.bbox = parse_bbox(object_value["bbox"]);
        instance.appearances_timestamps = parse_appearances_timestamps(apppearances_j[instance.InstanceID]);
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
        str += "Appearances timestamps: ";
        for (auto& [category, timestamps] : instance.appearances_timestamps){
            str += category + " (" + std::to_string(timestamps.size()) + ") : ";
            for (uint32_t timestamp : timestamps){
                str += std::to_string(timestamp) + ", ";
            }
            str += "\n";
        }
        str += "\n";
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

std::map<std::string, std::vector<uint32_t>> JsonSemanticMap::parse_appearances_timestamps(json& appearances_timestamps){
    std::map<std::string, std::vector<uint32_t>> appearances_map;
    for(auto& [category, timestamps] : appearances_timestamps["timestamps"].items()){
        std::vector<uint32_t> timestamps_vector;
        for (uint32_t timestamp : timestamps){
            timestamps_vector.push_back(timestamp);
        }
        appearances_map[category] = timestamps_vector;
    }

    return appearances_map;
}

