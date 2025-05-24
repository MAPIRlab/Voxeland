#include <iostream>
#include <memory>
#include <opencv2/highgui.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/serialized_message.hpp>
#include <ros_lm_interfaces/srv/detail/open_llm_request__struct.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <string>

#include <disambiguation/voxeland_disambiguation.hpp>
#include "disambiguation/json_semantics.hpp"
#include "disambiguation/pipeline/interface_pipeline_step.hpp"
#include "voxeland_map/Utils/logging.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "disambiguation/appearances_classifier/classifier_factory.hpp"
#include "disambiguation/pipeline/pipeline_steps.hpp"


namespace voxeland_disambiguation {


    VoxelandDisambiguation::VoxelandDisambiguation(const rclcpp::NodeOptions& node_options)
        : Node("voxeland_disambiguation_node", node_options)
    {
        // DECLARE NODE PARAMETERS
        json_file = declare_parameter("json_map", "scenenn/061.json");
        VXL_INFO("json_map parameter defined, value: {}", json_file);

        json_appearances_file = declare_parameter("json_appearances", "scenenn/061_appearances.json");
        VXL_INFO("json_appearances parameter defined, value: {}", json_appearances_file);

        bag_path = declare_parameter("bag_path", "/home/ubuntu/ros2_ws/bag/SceneNN/to_ros/ROS2_bags/061/061.db3");
        VXL_INFO("bag_path parameter defined, value: {}", bag_path);

        appearances_classifier = declare_parameter("appearances_classifier", "random");
        VXL_INFO("appearances_classifier parameter defined, value: {}", appearances_classifier);

        n_images_per_category = declare_parameter("n_images_per_category", 3);
        VXL_INFO("n_images_per_category parameter defined, value: {}", n_images_per_category);

        n_categories_per_instance = declare_parameter("n_categories_per_instance", 3);
        VXL_INFO("n_categories_per_instance parameter defined, value: {}", n_categories_per_instance);

        output_file = declare_parameter("output_file", "/home/ubuntu/Desktop/061_disambiguated.json");
        VXL_INFO("output_file parameter defined, value: {}", output_file);

        lvlm_model = declare_parameter("lvlm_model", "openbmb/MiniCPM-o-2_6");
        VXL_INFO("lvlm_model parameter defined, value: {}", lvlm_model);

        initialize_pipeline();
        execute_pipeline();
    }

    void VoxelandDisambiguation::initialize_pipeline() {

        auto json_deserialization = std::make_unique<JsonDeserializationStep>(json_file, json_appearances_file);
        auto uncertain_instance_identification = std::make_unique<UncertainInstanceIdentificationStep>();
        auto classifier_instance = ClassifierFactory::create_classifier(appearances_classifier);
        auto appearances_selection = std::make_unique<AppeareancesSelectionStep>(std::move(classifier_instance), n_images_per_category, n_categories_per_instance);
        auto image_bag_reading = std::make_unique<ImageBagReading>(bag_path);
        auto lvlm_disambiguation = std::make_unique<LVLMDisambiguationStep>(lvlm_model);
        auto json_serialization = std::make_unique<JsonSerializationStep>(output_file);
        
        pipeline_steps.push_back(std::move(json_deserialization));
        pipeline_steps.push_back(std::move(uncertain_instance_identification));
        pipeline_steps.push_back(std::move(appearances_selection));
        pipeline_steps.push_back(std::move(image_bag_reading));
        pipeline_steps.push_back(std::move(lvlm_disambiguation));
        pipeline_steps.push_back(std::move(json_serialization));

    }

    void VoxelandDisambiguation::execute_pipeline() {
        for (auto& step : pipeline_steps) {
            step -> execute();
            std::cout << std::endl;
        }
    }

    
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(voxeland_disambiguation::VoxelandDisambiguation)