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
        UncertainInstance(std::shared_ptr<JsonSemanticObject> instance, double entropy){
            this->instance = instance;
            this->entropy = entropy;
        };
        
        std::shared_ptr<JsonSemanticObject> get_instance();
        std::map<std::string, std::vector<uint32_t>>* get_selected_appearances();
        std::map<std::string, std::vector<cv_bridge::CvImagePtr>>* get_selected_images();
        double get_entropy();
        std::map<std::string, uint32_t>* get_disambiguation_results();
        cv_bridge::CvImagePtr get_bbox_image(cv_bridge::CvImagePtr full_image, std::string category, uint32_t timestamp);

        void set_selected_appearances(std::map<std::string, std::vector<uint32_t>> selected_appearances);
        void increase_one_disambiguation_result(const std::string& category);
        std::string to_string();
    private:
        std::shared_ptr<JsonSemanticObject> instance;
        std::map<std::string, std::vector<uint32_t>> selected_appearances;
        std::map<std::string, std::vector<cv_bridge::CvImagePtr>> selected_images;
        double entropy;
        std::map<std::string, uint32_t> disambiguation_results;
};

class JsonSemanticMap{
    public:
        std::vector<std::shared_ptr<JsonSemanticObject>>& get_instances();
    
        void add_instance(std::shared_ptr<JsonSemanticObject> instance);
        std::shared_ptr<JsonSemanticObject> get_instance(const std::string& instanceID);    
        const std::string to_string();
    private:
        std::vector<std::shared_ptr<JsonSemanticObject>> instances; 

};
