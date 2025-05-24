#include "disambiguation/json_semantics.hpp"
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include "voxeland_map/semantics.hpp"

using json = nlohmann::json;

std::shared_ptr<JsonSemanticObject> UncertainInstance::get_instance(){
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

std::map<std::string, uint32_t>* UncertainInstance::get_disambiguation_results(){
    return &disambiguation_results;
}

void UncertainInstance::set_selected_appearances(std::map<std::string, std::vector<uint32_t>> selected_appearances){
    this->selected_appearances = selected_appearances;
}

void UncertainInstance::increase_one_disambiguation_result(const std::string& category){
    if (disambiguation_results.find(category) != disambiguation_results.end()){
        disambiguation_results[category]++;
    } else {
        disambiguation_results[category] = 1;
    }
}

std::string UncertainInstance::to_string(){
    std::string str = "Instance: " + instance->InstanceID + "\n";
    str += "Entropy: " + std::to_string(entropy) + "\n";
    str += "Selected appearances: \n";
    str += "    " + std::to_string(selected_appearances.size()) + " categories\n";
    for (auto& [category, timestamps] : selected_appearances){
        str += "    " + category + " : " + std::to_string(timestamps.size()) + " timestamps\n";
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

void JsonSemanticMap::add_instance(JsonSemanticObject instance){
    instances.push_back(instance);
}

std::shared_ptr<JsonSemanticObject> JsonSemanticMap::get_instance(const std::string& instanceID){
    for (JsonSemanticObject& instance : instances){
        if (instance.InstanceID == instanceID){
            return std::make_shared<JsonSemanticObject>(instance);
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
        str += "Appearances timestamps (" + std::to_string(instance.appearances_timestamps.size()) + "): \n";
        for (auto& [category, timestamps] : instance.appearances_timestamps){
            str += "    " + category + " (" + std::to_string(timestamps.size()) + ")\n";
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


cv_bridge::CvImagePtr UncertainInstance::get_bbox_image(cv_bridge::CvImagePtr full_image,std::string category, uint32_t timestamp){
    auto& appearances_map = this->get_instance()->appearances_timestamps;
    BoundingBox2D bbox = appearances_map[category][timestamp];

    int paddinng = 75;

    int width = static_cast<int>(bbox.sizeX);
    int height = static_cast<int>(bbox.sizeY);

    // Esquina superior izquierda
    int x = static_cast<int>(bbox.centerX - width/2 ) - paddinng;
    int y = static_cast<int>(bbox.centerY - height/2 ) - paddinng;
    // Añadir el padding
    width += 2 * paddinng;
    height += 2 * paddinng;
    
    // Crear el rectángulo
    cv::Rect bbox_rect(x, y, width, height);

    // Asegurar que el bounding box esté dentro de los límites de la imagen
    bbox_rect = bbox_rect & cv::Rect(0, 0, full_image->image.cols, full_image->image.rows);

    cv::Mat roi = full_image->image(bbox_rect).clone();

    // Crear una nueva CvImagePtr con el ROI
    cv_bridge::CvImagePtr bbox_image(new cv_bridge::CvImage);
    bbox_image->header = full_image->header;             // mantener timestamp y frame_id
    bbox_image->encoding = full_image->encoding;         // mantener el encoding
    bbox_image->image = roi;

    // Ahora `output_ptr` contiene la imagen recortada
    return bbox_image;
}