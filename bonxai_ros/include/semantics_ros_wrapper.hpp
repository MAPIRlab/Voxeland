#pragma once

#include <bonxai_map/semantics.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/object_hypothesis.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

#include "segmentation_msgs/msg/semantic_point_cloud.hpp"
#include "segmentation_msgs/msg/instance_semantic_map.hpp"


class SemanticsROSWrapper
{
    public:

        SemanticsROSWrapper() = default;

        SemanticObject
        convertDetection2DToSemanticObject(const vision_msgs::msg::Detection2D& instance)
        {
            SemanticObject semanticObject(semantics.default_categories.size(), semantics.globalSemanticMap.size()+1);

            for (const auto& result : instance.results)
            {
            semantics.updateCategoryProbability(
                semanticObject, result.hypothesis.class_id, result.hypothesis.score);
            }

            return semanticObject;
        }

        std::vector<SemanticObject> convertROSMessageToSemanticMap(
            const std::vector<vision_msgs::msg::Detection2D>& instances)
        {
            std::vector<SemanticObject> localSemanticMap(
                instances.size(), SemanticObject(semantics.default_categories.size(), semantics.globalSemanticMap.size()+1));

            for (INSTANCEIDT i = 0; i < instances.size(); i++)
            {
                // Note that, always the 0-index refers to the "unknown" class
                SemanticObject newObject = convertDetection2DToSemanticObject(instances[i]);
                localSemanticMap[std::atoi(instances[i].id.c_str())] = newObject;
            }

            return localSemanticMap;
        }

        segmentation_msgs::msg::InstanceSemanticMap getSemanticMapAsROSMessage(const rclcpp::Time& rostime){

            segmentation_msgs::msg::InstanceSemanticMap map;

            map.header.stamp = rostime;
            
            for (size_t i=0; i < semantics.globalSemanticMap.size(); i++){
                
                if(semantics.globalSemanticMap[i].pointsTo.empty()){
                    vision_msgs::msg::Detection3D instance;
                    instance.id = semantics.globalSemanticMap[i].instanceID;
                    for (size_t j = 0; j < semantics.default_categories.size(); j++){
                        if(semantics.globalSemanticMap[i].probabilities[j] > 0){
                            vision_msgs::msg::ObjectHypothesisWithPose instanceHypothesis;
                            instanceHypothesis.hypothesis.class_id = semantics.default_categories[j];
                            instanceHypothesis.hypothesis.score = semantics.globalSemanticMap[i].probabilities[j];
                            instance.results.push_back(instanceHypothesis);
                        }
                    }
                    map.semantic_map.push_back(instance);
                }
            }

            return map;
        }

        template <typename PointCloudTypeT, typename DataT>
        void addLocalInstanceSemanticMap(const std::vector<vision_msgs::msg::Detection2D>& instances, const PointCloudTypeT& pc){

            std::vector<SemanticObject> localMap = convertROSMessageToSemanticMap(instances);

            semantics.addInstancesGeometryToLocalSemanticMap<DataT, PointCloudTypeT>(localMap, pc);
            semantics.integrateNewSemantics<DataT>(localMap);

        }

        template <typename PointCloudTypeT>
        void addLocalSemanticMap(const std::vector<vision_msgs::msg::Detection2D>& instances, const PointCloudTypeT& pc){

            std::vector<SemanticObject> localMap = convertROSMessageToSemanticMap(instances);
                
            semantics.setLocalSemanticMap(localMap);

        }

    protected:

        SemanticMap& semantics = SemanticMap::get_instance();

};