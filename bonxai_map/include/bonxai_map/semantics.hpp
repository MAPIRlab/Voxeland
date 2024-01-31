#pragma once

#include <vector>
#include <cmath>
#include <iostream>
#include <unordered_map>


#include <eigen3/Eigen/Core>
#include "bonxai/bonxai.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "segmentation_msgs/msg/semantic_point_cloud.hpp"
#include "vision_msgs/msg/detection_2_d.hpp"


class Semantics {

    private:

        double kld_threshold = 0.1f;

        std::vector<std::string> default_categories;
        std::vector<double> default_probabilities;
        std::unordered_map<std::string, size_t> categoryIndexMap;

        struct SemanticObject {

            std::vector<std::string> categories;
            std::vector<double> probabilities;

            SemanticObject() : categories(default_categories), probabilities(default_probabilities) {}

        };

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
            auto it = categoryIndexMap.find(categoryName);
            if (it != categoryIndexMap.end()) {
                size_t index = it->second;
                semanticObject.probabilities[index] += probability;
            }
        };

    public:

        Semantics(const std::vector<std::string>& dataset_categories)
        : default_categories(dataset_categories) {
            // Initialize objectInfoMap with SemanticObject for each object name
            for (size_t i = 0; i < dataset_categories.size(); ++i) {
                    default_probabilities.push_back(0.0);
                    categoryIndexMap[dataset_categories[i]] = i;
                }
                default_categories.push_back("unknown");
                default_probabilities.push_back(0.0);
                categoryIndexMap["unknown"] = default_categories.size() - 1;
            };
        

        std::unordered_map<std::string, SemanticObject> globalSemanticMap;

        SemanticObject convertDetection2DToSemanticObject(const vision_msgs::msg::Detection2D& instance) {

            SemanticObject semanticObject;

                for (const auto& result: instance.results) {

                    updateCategoryProbability(semanticObject, result.hypothesis.class_id, result.hypothesis.score);

                }

            return semanticObject;
        };

        std::unordered_map<std::string, SemanticObject> convertROSMessageToSemanticMap(const std::vector<vision_msgs::msg::Detection2D>& instances) {

            std::unordered_map<std::string, SemanticObject> localSemanticMap;

            for (const vision_msgs::msg::Detection2D& instance: instances) {
                
                SemanticObject newObject = convertDetection2DToSemanticObject(instance);

                std::string instanceName = "obj" + instance.id;
                localSemanticMap.emplace(instanceName, newObject);

            }

            return localSemanticMap;
        };

}