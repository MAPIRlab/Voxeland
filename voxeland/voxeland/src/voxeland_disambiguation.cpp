#include <iostream>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/serialized_message.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <string>
#include <voxeland_disambiguation.hpp>
#include "json_semantics.hpp"
#include "voxeland_map/dirichlet.hpp"
#include "voxeland_map/Utils/logging.hpp"
#include "appearances_classifier.hpp"

#include "rosbag2_transport/reader_writer_factory.hpp"

namespace voxeland_disambiguation {
    VoxelandDisambiguation::VoxelandDisambiguation(const rclcpp::NodeOptions& node_options)
        : Node("voxeland_disambiguation_node", node_options)
    {
        // DECLARE NODE PARAMETERS
        json_file = declare_parameter("json_map", "scenenn065.json");
        VXL_INFO("json_map parameter defined, value: {}", json_file);

        json_appearances_file = declare_parameter("json_appearances", "scenenn065_appearances.json");
        VXL_INFO("json_appearances parameter defined, value: {}", json_appearances_file);

        bag_path = declare_parameter("bag_path", "/home/ubuntu/ros2_ws/bag/SceneNN/to_ros/ROS2_bags/065/065.db3");
        VXL_INFO("bag_path parameter defined, value: {}", bag_path);

        // Get the path to the package
        std::string package_path = ament_index_cpp::get_package_share_directory("voxeland");
        std::string map_file_path = package_path + "/params/" + json_file;
        std::string appearances_file_path = package_path + "/params/" + json_appearances_file;

        std::cout << appearances_file_path << std::endl;

        execute_pipeline();
    }

    void VoxelandDisambiguation::execute_pipeline(){
        // Load the semantic map
        semantic_map = JsonSemanticMap::load_map(json_file, json_appearances_file);

        find_uncertain_instances();

        select_appearances();

        obtain_bag_images();
        VXL_INFO("------- IMAGES OBTAINED ----------");
        for(UncertainInstance& instance : uncertain_instances){
            std::cout << "Instance: " << instance.get_instance()->InstanceID << std::endl;
            for(auto& [category, images] : *instance.get_selected_images()){
                std::cout << "     Category: " << category << " - " << images.size() << " images" << std::endl;
            }
        }
    }
    /**
     * @brief Select the uncertain instances by computing the entropy of the results map
     * @brief If the entropy is greater than a fixed value, the instance is considered uncertain
    */
    void VoxelandDisambiguation::find_uncertain_instances(){
        for (JsonSemanticObject& instance : *semantic_map.get_instances()){
            
            // Retrieve all alphas from the results
            std::vector<double> alphas;
            for (auto results_pair : instance.results){
                alphas.push_back(results_pair.second);
            }
            
            // Compute the entropy
            double entropy = expected_shannon_entropy(alphas);
            std::cout << "Expected entropy for instance " << instance.InstanceID << " : " << entropy << std::endl;

            // Fixed threshold
            if(entropy > 0.7){
                uncertain_instances.push_back(UncertainInstance(&instance, entropy));
            }
        }
    }
    
    /**
     * @brief Select the categories with the highest probabilities and choose the appearances to be used for re-classification.
     * @brief The way to choose the appearances is defined by the AppearanceClassifier strategy.
    */
    void VoxelandDisambiguation::select_appearances(){
        // Select an strategy
        AppearancesClassifier* classifier = new SplitAppearancesClassifier(3);
        for (UncertainInstance& instance : uncertain_instances){

            classifier->classify_instance_appearances(instance);
        }
    }

    void VoxelandDisambiguation::obtain_bag_images(){
        // Setup the bag reader
        rosbag2_storage::StorageOptions storage_options;
        storage_options.storage_id = "sqlite3";
        storage_options.uri = bag_path;
        reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
        reader->open(storage_options);

        // Read all images from bag file
        while (reader->has_next()){
            rosbag2_storage::SerializedBagMessageSharedPtr message = reader->read_next();

            // std::cout << "Topic: " << message->topic_name << std::endl;
            if(message->topic_name != "camera/rgb"){
                // Accept only image messages
                continue;
            }

            //Desearialize the message
            rclcpp::SerializedMessage serialized_message(*message->serialized_data);
            sensor_msgs::msg::Image::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::Image>();
            image_serializer.deserialize_message(&serialized_message, ros_msg.get());

            // Check for every uncertain instance, if the image timestamp is included in the selected appearances
            for (UncertainInstance& instance : uncertain_instances){
                add_selected_images(ros_msg, instance);
            }
        }
    }

    /**
     * @brief Checks if the img_timestamp is included in any of the selected appearances of the instance
     * @brief If it is included, the image is added to the selected images map
     */
    void VoxelandDisambiguation::add_selected_images(sensor_msgs::msg::Image::SharedPtr image_msg, UncertainInstance& instance){
        std::map<std::string, std::vector<sensor_msgs::msg::Image>>* appearances = instance.get_selected_images();

        // Lopp through all the selected appearances
        for(auto& [category, appearances_vector] : *instance.get_selected_appearances()){
            for(uint32_t timestamp : appearances_vector){
                // If the timestamp is included in the selected appearances,
                // the image is added to the selected images map
                if(timestamp == image_msg->header.stamp.sec){
                    (*appearances)[category].push_back(*image_msg);
                    VXL_INFO("Image {} added to instance: {} - Category: {}", image_msg->header.stamp.sec,instance.get_instance()->InstanceID, category);
                }
            }
        }
    }

}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(voxeland_disambiguation::VoxelandDisambiguation)