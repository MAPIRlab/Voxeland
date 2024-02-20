#pragma once

#include <bonxai_map/semantics.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include "segmentation_msgs/msg/semantic_point_cloud.hpp"
#include "vision_msgs/msg/detection2_d.hpp"


class SemanticsROSWrapper
{
    public:

        SemanticsROSWrapper() = default;

        SemanticObject
        convertDetection2DToSemanticObject(const vision_msgs::msg::Detection2D& instance)
        {
            SemanticObject semanticObject(semantics.default_categories.size());

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
                instances.size(), SemanticObject(semantics.default_categories.size()));

            for (INSTANCEIDT i = 0; i < instances.size(); i++)
            {
            SemanticObject newObject = convertDetection2DToSemanticObject(instances[i]);

            localSemanticMap[std::atoi(instances[i].id.c_str())] = newObject;
            }

            return localSemanticMap;
        }

    protected:

        SemanticMap& semantics = SemanticMap::get_instance();

};