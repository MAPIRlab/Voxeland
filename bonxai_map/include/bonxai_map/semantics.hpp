#pragma once

#include <vector>
#include <cmath>
#include <iostream>
#include <unordered_map>


#include <eigen3/Eigen/Core>
#include "bonxai/bonxai.hpp"

#include <segmentation_msgs/msg/semantic_point_cloud.hpp>
#include "vision_msgs/msg/detection2_d.hpp"


struct SemanticObject {

    std::vector<double> probabilities;

    SemanticObject(size_t numCategories) : probabilities(numCategories, 0) {}
    SemanticObject(const std::vector<double>& _probabilities) : probabilities(_probabilities) {}

};

class SemanticMap {

public:

    static SemanticMap& get_instance() {
        static SemanticMap instance;
        return instance;
    };

    std::vector<std::string> default_categories;
    std::unordered_map<std::string, size_t> categoryIndexMap;

    std::vector<SemanticObject> globalSemanticMap;

    SemanticMap() = default;

    void initialize(std::vector<std::string> dataset_categories) {
        // Initialize objectInfoMap with SemanticObject for each object name
        for (size_t i = 0; i < dataset_categories.size(); ++i) {
            categoryIndexMap[dataset_categories[i]] = i;
        }
        default_categories.push_back("unknown");
        categoryIndexMap["unknown"] = default_categories.size() - 1;

        initialized = true;
        
    };

    bool is_initialized() {
        return initialized;
    }           

    SemanticObject convertDetection2DToSemanticObject(const vision_msgs::msg::Detection2D& instance) {

        SemanticObject semanticObject(default_categories.size());

            for (const auto& result: instance.results) {

                updateCategoryProbability(semanticObject, result.hypothesis.class_id, result.hypothesis.score);

            }

        return semanticObject;
    };

    std::vector<SemanticObject> convertROSMessageToSemanticMap(const std::vector<vision_msgs::msg::Detection2D>& instances) {

        std::vector<SemanticObject> localSemanticMap(instances.size(), default_categories.size());

        for (uint8_t i = 0; i < instances.size(); i++) {
            
            SemanticObject newObject = convertDetection2DToSemanticObject(instances[i]);

            localSemanticMap[std::atoi(instances[i].id.c_str())] = newObject;

        }

        return localSemanticMap;
    };

    void integrateNewSemantics(const std::vector<SemanticObject>& localMap) {

        for(const SemanticObject& localInstance: localMap) {
            std::vector<double>::const_iterator it = std::max_element(localInstance.probabilities.begin(), localInstance.probabilities.end());
            uint8_t localIdx = std::distance(localInstance.probabilities.begin(), it);
            bool fused = false;
            for(SemanticObject& globalInstance: globalSemanticMap){
                std::vector<double>::iterator it = std::max_element(globalInstance.probabilities.begin(), globalInstance.probabilities.end());
                uint8_t globalIdx = std::distance(globalInstance.probabilities.begin(), it);

                if(localIdx == globalIdx){
                    for(uint8_t i = 0; i < globalInstance.probabilities.size(); i++){
                        globalInstance.probabilities[i] += localInstance.probabilities[i];
                    }
                    fused = true;
                    break;
                }

            }
            if(!fused){
                globalSemanticMap.push_back(SemanticObject(localInstance.probabilities));
            }
        }

    }

private:

    bool initialized = false;
    double kld_threshold = 0.1f;

    // Function to compute Kullback-Leibler Divergence
    double computeKLD(const std::vector<double>& P, const std::vector<double>& Q) {
        if (P.size() != Q.size()) {
            std::cerr << "Error: Vectors must be of equal length\n";
            return false;
        }

        double kld = 0.0;
        for (size_t i = 0; i < P.size(); ++i) {
            if (P[i] == 0) // To avoid log(0)
                continue;
            if (Q[i] == 0) // Handle when Q[i] = 0
                return false;
            
            kld += P[i] * log(P[i] / Q[i]);
        }
        kld = std::abs(kld); // Absolute value of KLD

        if (kld < kld_threshold) {
            return kld;
        }
        else {
            return 0.0;
        }
    };

    void updateCategoryProbability(SemanticObject& semanticObject, const std::string& categoryName, double probability) {

        semanticObject.probabilities[categoryIndexMap[categoryName]] += probability;
        
    };

    
    

};