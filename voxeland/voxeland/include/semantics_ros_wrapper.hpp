#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <voxeland_map/cell_types/Color.hpp>
#include <voxeland_map/semantic_map.hpp>

#include "Profiling.hpp"
#include "segmentation_msgs/msg/instance_semantic_map.hpp"
#include "segmentation_msgs/msg/semantic_point_cloud.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/object_hypothesis.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class SemanticsROSWrapper
{
public:
    SemanticsROSWrapper() = default;

    SemanticObject convertDetection2DToSemanticObject(const vision_msgs::msg::Detection2D& instance)
    {
        SemanticObject semanticObject(-1);

        for (const auto& result : instance.results)
        {
            semantics.updateCategoryProbability(semanticObject, result.hypothesis.class_id, result.hypothesis.score);
            // Insert image instance in the appearances array

            // Create mask bbox object
            BoundingBox2D bbox;
            bbox.centerX = instance.bbox.center.position.x;
            bbox.centerY = instance.bbox.center.position.y;
            bbox.sizeX = instance.bbox.size_x;
            bbox.sizeY = instance.bbox.size_y;

            CategoryManager::CategoryIndex categoryIndex = semantics.getCategoryIndex(result.hypothesis.class_id);
            if (categoryIndex != CategoryManager::INVALID_CATEGORY)
            {
                semanticObject.appearancesTimestamps[categoryIndex][instance.header.stamp.sec] = bbox;
            }
        }

        return semanticObject;
    }

    std::vector<SemanticObject>
    convertROSMessageToSemanticMap(const std::vector<vision_msgs::msg::Detection2D>& instances)
    {
        std::vector<SemanticObject> localSemanticMap;
        localSemanticMap.reserve(instances.size());

        for (InstanceID_t i = 0; i < instances.size(); i++)
        {
            // Note that, always the 0-index refers to the "unknown" class
            SemanticObject newObject = convertDetection2DToSemanticObject(instances.at(i));

            // Use instance ID from message or create sequential
            int instanceIndex = std::atoi(instances.at(i).id.c_str());
            if (instanceIndex >= localSemanticMap.size())
            {
                localSemanticMap.resize(instanceIndex + 1, SemanticObject(1));
            }
            localSemanticMap[instanceIndex] = newObject;
        }

        return localSemanticMap;
    }

    struct InstanceMapMsgs
    {
        segmentation_msgs::msg::InstanceSemanticMap instanceMap;
        visualization_msgs::msg::MarkerArray textMarkers;
    };

    InstanceMapMsgs getSemanticMapAsROSMessage(const rclcpp::Time& rostime, const std::set<InstanceID_t> visibleInstances)
    {
        segmentation_msgs::msg::InstanceSemanticMap map;

        map.header.stamp = rostime;
        visualization_msgs::msg::MarkerArray textMarkers;

        for (size_t i = 0; i < semantics.globalSemanticMap.size(); i++)
        {
            if (visibleInstances.count(i) > 0 && semantics.globalSemanticMap.at(i).isStillValid())
            {
                vision_msgs::msg::Detection3D instance;
                instance.id = semantics.globalSemanticMap.at(i).instanceName;
                instance.bbox.center.position.x =
                    (semantics.globalSemanticMap.at(i).bbox.minX + semantics.globalSemanticMap.at(i).bbox.maxX) / 2.0;
                instance.bbox.center.position.y =
                    (semantics.globalSemanticMap.at(i).bbox.minY + semantics.globalSemanticMap.at(i).bbox.maxY) / 2.0;
                instance.bbox.center.position.z =
                    (semantics.globalSemanticMap.at(i).bbox.minZ + semantics.globalSemanticMap.at(i).bbox.maxZ) / 2.0;
                instance.bbox.size.x =
                    semantics.globalSemanticMap.at(i).bbox.maxX - semantics.globalSemanticMap.at(i).bbox.minX;
                instance.bbox.size.y =
                    semantics.globalSemanticMap.at(i).bbox.maxY - semantics.globalSemanticMap.at(i).bbox.minY;
                instance.bbox.size.z =
                    semantics.globalSemanticMap.at(i).bbox.maxZ - semantics.globalSemanticMap.at(i).bbox.minZ;

                // Convert category probabilities to results
                for (const auto& [categoryIndex, probability] : semantics.globalSemanticMap.at(i).alphaParamsCategories)
                {
                    if (probability > 0)
                    {
                        std::string categoryName = semantics.getCategoryName(categoryIndex);
                        if (!categoryName.empty())
                        {
                            vision_msgs::msg::ObjectHypothesisWithPose instanceHypothesis;
                            instanceHypothesis.hypothesis.class_id = categoryName;
                            instanceHypothesis.hypothesis.score = probability;
                            instance.results.push_back(instanceHypothesis);
                        }
                    }
                }
                map.semantic_map.push_back(instance);

                // marker with instance ID for RViz
                visualization_msgs::msg::Marker textMarker;
                {
                    textMarker.header.frame_id = "map";
                    textMarker.id = i;
                    textMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                    textMarker.scale.z = 0.2;
                    textMarker.text = instance.id;
                    textMarker.pose.position.x = instance.bbox.center.position.x;
                    textMarker.pose.position.y = instance.bbox.center.position.y;
                    textMarker.pose.position.z = instance.bbox.center.position.z + 1.0f;

                    auto color = voxeland::Color::FromHex(SemanticMap::get_instance().indexToHexColor(i));
                    textMarker.color.r = color.r / 255.f;
                    textMarker.color.g = color.g / 255.f;
                    textMarker.color.b = color.b / 255.f;
                    textMarker.color.a = 1;
                }
                textMarkers.markers.push_back(textMarker);
            }
        }

        return { map, textMarkers };
    }

    template <typename PointCloudTypeT, typename DataT>
    void addLocalInstanceSemanticMap(const std::vector<vision_msgs::msg::Detection2D>& instances,
                                     const PointCloudTypeT& pc,
                                     float sensorX = 0.0f, float sensorY = 0.0f, float sensorZ = 0.0f)
    {
        voxeland::ScopedStopwatch watch("Add Instances to Map");
        std::vector<SemanticObject> localMap = convertROSMessageToSemanticMap(instances);
        
        // generate a voxelized version of the entire point cloud, to check whether a given pre-existing voxel is visible or not
        // this will be used to measure how much of a global instance is being identified as a single object in this image
        Bonxai::VoxelGrid<Bonxai::ProbabilisticCell<DataT>>* bonxai = BonxaiQuery<DataT>::getBonxai()->grid();
        std::set<Bonxai::CoordT> voxelizedLocalPointCloud;
        for (size_t i = 0; i < pc.points.size(); i++)
            voxelizedLocalPointCloud.insert(bonxai->posToCoord(Bonxai::Point3D(pc.points[i].x, pc.points[i].y, pc.points[i].z)));

        semantics.addInstancesGeometryToLocalSemanticMap<DataT, PointCloudTypeT>(localMap, pc);

        semantics.setLocalSemanticMap(localMap);

        semantics.integrateNewSemantics<DataT>(localMap, voxelizedLocalPointCloud);
    }

    template <typename PointCloudTypeT>
    void addLocalSemanticMap(const std::vector<vision_msgs::msg::Detection2D>& instances, const PointCloudTypeT& pc)
    {
        std::vector<SemanticObject> localMap = convertROSMessageToSemanticMap(instances);
        semantics.setLocalSemanticMap(localMap);
    }

protected:
    SemanticMap& semantics = SemanticMap::get_instance();
};