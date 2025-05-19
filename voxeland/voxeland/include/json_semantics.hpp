#pragma once

#include <sys/types.h>
#include <cstdint>
#include <map>
#include <string>
#include <vector>
#include "voxeland_map/semantics.hpp"
#include "cv_bridge/cv_bridge.h"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

struct JsonSemanticObject{
    std::string InstanceID;
    BoundingBox3D bbox;
    std::map<std::string, std::map<uint32_t,BoundingBox2D>> appearances_timestamps;
    uint32_t n_observations;
    std::map<std::string, double> results;
};

class UncertainInstance {
    public:
        UncertainInstance(JsonSemanticObject* instance, double entropy){
            this->instance = instance;
            this->entropy = entropy;
        };
        JsonSemanticObject* get_instance();
        std::map<std::string, std::vector<uint32_t>>* get_selected_appearances();
        std::map<std::string, std::vector<cv_bridge::CvImagePtr>>* get_selected_images();
        double get_entropy();
        std::string get_final_category();
        cv_bridge::CvImagePtr get_bbox_image(cv_bridge::CvImagePtr full_image, std::string category, uint32_t timestamp);
        void set_selected_appearances(std::map<std::string, std::vector<uint32_t>> selected_appearances);
        void set_final_category(std::string final_category);
        std::string to_string();
    private:
        JsonSemanticObject* instance;
        std::map<std::string, std::vector<uint32_t>> selected_appearances;
        std::map<std::string, std::vector<cv_bridge::CvImagePtr>> selected_images;
        double entropy;
        std::string final_category;
};

class JsonSemanticMap{
    public:
        static JsonSemanticMap load_map(const std::string& json_file, const std::string& json_appearances_file);
        JsonSemanticObject* get_instance(const std::string& instanceID);
        std::vector<JsonSemanticObject>* get_instances();
        const std::string to_string();
    protected:
        std::vector<JsonSemanticObject> instances;
    private:
        static BoundingBox3D parse_bbox(json& bbox);
        static std::map<std::string, std::map<uint32_t,BoundingBox2D>> parse_appearances_timestamps(json& appearances_timestamps);
        static std::map<std::string, double> parse_results(json& results);
};
