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
                instances.size(), SemanticObject(semantics.default_categories.size(), 1));

            for (INSTANCEIDT i = 0; i < instances.size(); i++)
            {
                // Note that, always the 0-index refers to the "unknown" class
                SemanticObject newObject = convertDetection2DToSemanticObject(instances[i]);
                localSemanticMap[std::atoi(instances[i].id.c_str())] = newObject;
            }

            return localSemanticMap;
        }

        segmentation_msgs::msg::InstanceSemanticMap getSemanticMapAsROSMessage(const rclcpp::Time& rostime, const std::set<INSTANCEIDT> visibleInstances){

            segmentation_msgs::msg::InstanceSemanticMap map;

            map.header.stamp = rostime;
            
            for (size_t i=0; i < semantics.globalSemanticMap.size(); i++){
                
                if(visibleInstances.count(i) > 0 && semantics.globalSemanticMap[i].pointsTo == -1){
                    vision_msgs::msg::Detection3D instance;
                    instance.id = semantics.globalSemanticMap[i].instanceID;
                    instance.bbox.center.position.x = (semantics.globalSemanticMap[i].bbox.minX + semantics.globalSemanticMap[i].bbox.maxX) / 2.0;
                    instance.bbox.center.position.y = (semantics.globalSemanticMap[i].bbox.minY + semantics.globalSemanticMap[i].bbox.maxY) / 2.0;
                    instance.bbox.center.position.z = (semantics.globalSemanticMap[i].bbox.minZ + semantics.globalSemanticMap[i].bbox.maxZ) / 2.0;
                    instance.bbox.size.x = semantics.globalSemanticMap[i].bbox.maxX - semantics.globalSemanticMap[i].bbox.minX;
                    instance.bbox.size.y = semantics.globalSemanticMap[i].bbox.maxY - semantics.globalSemanticMap[i].bbox.minY;
                    instance.bbox.size.z = semantics.globalSemanticMap[i].bbox.maxZ - semantics.globalSemanticMap[i].bbox.minZ;
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

            /*for (INSTANCEIDT i = 0; i < localMap.size(); i++){
                if(!localMap[i].localGeometry.has_value()){
                    localMap.erase(localMap.begin()+i);
                    i--;
                }
            }*/

            BONXAI_INFO("################# LOCAL MAP #################");
            for(INSTANCEIDT j = 0; j < localMap.size(); j++){
                if(localMap[j].localGeometry.has_value()){
                    BONXAI_INFO("Local instance {}: with {} voxels", localMap[j].instanceID, localMap[j].localGeometry.value().size());
                }
            }

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