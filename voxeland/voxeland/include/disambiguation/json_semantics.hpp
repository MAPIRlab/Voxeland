#pragma once

#include <sys/types.h>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "voxeland_map/semantics.hpp"
#include "cv_bridge/cv_bridge.h"



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
            this->instance = std::make_shared<JsonSemanticObject>(*instance);
            this->entropy = entropy;
        };
        
        std::shared_ptr<JsonSemanticObject> get_instance();
        std::map<std::string, std::vector<uint32_t>>* get_selected_appearances();
        std::map<std::string, std::vector<cv_bridge::CvImagePtr>>* get_selected_images();
        double get_entropy();
        std::string get_final_category();
        cv_bridge::CvImagePtr get_bbox_image(cv_bridge::CvImagePtr full_image, std::string category, uint32_t timestamp);

        void set_selected_appearances(std::map<std::string, std::vector<uint32_t>> selected_appearances);
        void set_final_category(std::string final_category);
        std::string to_string();
    private:
        std::shared_ptr<JsonSemanticObject> instance;
        std::map<std::string, std::vector<uint32_t>> selected_appearances;
        std::map<std::string, std::vector<cv_bridge::CvImagePtr>> selected_images;
        double entropy;
        std::string final_category;
};

class JsonSemanticMap{
    public:
        std::vector<JsonSemanticObject>* get_instances();
    
        void add_instance(JsonSemanticObject instance);
        std::shared_ptr<JsonSemanticObject> get_instance(const std::string& instanceID);    
        const std::string to_string();
    private:
        std::vector<JsonSemanticObject> instances; 

};
