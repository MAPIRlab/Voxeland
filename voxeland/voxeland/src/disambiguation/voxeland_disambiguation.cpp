#include <iostream>
#include <memory>
#include <opencv2/highgui.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <thread>

#include <rclcpp/serialized_message.hpp>
#include <ros_lm_interfaces/srv/detail/open_llm_request__struct.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <string>
#include <vector>
#include <voxeland_disambiguation.hpp>
#include "json_semantics.hpp"
#include "voxeland_map/dirichlet.hpp"
#include "voxeland_map/Utils/logging.hpp"
#include "appearances_classifier.hpp"
#include "lvlm_processing.hpp"

#include "rosbag2_transport/reader_writer_factory.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


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

        client = this->create_client<ros_lm_interfaces::srv::OpenLLMRequest>("llm_generate_text");
        VXL_INFO("Awaiting llm_generate_text service...");
        while (!client->wait_for_service(std::chrono::seconds(1))){
            if (!rclcpp::ok()){
                VXL_ERROR("Interrupted while waiting for the service. Exiting...");
                return;
            }
            VXL_WARN("sertvice not available, waiting...");
        }
        VXL_INFO("llm_generate_text service available!");

        execute_pipeline();
    }

    void VoxelandDisambiguation::execute_pipeline(){
        // Load the semantic map
        semantic_map = JsonSemanticMap::load_map(json_file, json_appearances_file);

        find_uncertain_instances();

        select_appearances();

        obtain_bag_images();

        ask_llm_for_disambiguation();
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
        AppearancesClassifier* classifier = new RandomAppearancesClassifier(3);
        for (UncertainInstance& instance : uncertain_instances){
            classifier->classify_instance_appearances(instance);
        }
    }

    void show_all_images(std::string category ,std::vector<cv_bridge::CvImagePtr> images){
        for (cv_bridge::CvImagePtr image : images){
            cv::imshow(category, image->image);
            cv::waitKey(0);
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

            if(message->topic_name != "camera/rgb"){
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
        VXL_INFO("All images added to instances");

        // Show all images
        // for (UncertainInstance& instance : uncertain_instances){
        //     std::map<std::string, std::vector<cv_bridge::CvImagePtr>>* selected_images = instance.get_selected_images();
        //     for (auto& [category, images] : *selected_images){
        //         show_all_images(category,images);
        //     }
        // }
    }

    void VoxelandDisambiguation::ask_llm_for_disambiguation(){
        auto load_request = std::make_shared<ros_lm_interfaces::srv::OpenLLMRequest::Request>();
        load_request -> action = 1;
        // load_request-> model_id = "llava-hf/llava-1.5-7b-hf";
        load_request-> model_id = "openbmb/MiniCPM-o-2_6";

        VXL_INFO("Loading model {} into ros_lm server", load_request->model_id);
        auto future = client->async_send_request(load_request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS){
            auto response = future.get();
            VXL_INFO("Response: {}", response->status_message);
        } else {
            VXL_ERROR("Failed to call service");
        }
        int i = 0;
        for (UncertainInstance& instance : uncertain_instances){
            if( i >= 1){
                break;
            }
            std::vector<std::string> categories;
            std::vector<std::string> base64_images;
            std::string prompt;
            for (auto& [category, images] : *instance.get_selected_images()){
                categories.push_back(category);
                for (cv_bridge::CvImagePtr image : images){
                    std::string base64_image = image_to_base64(image);
                    base64_images.push_back(base64_image);
                }
            }
            prompt = load_and_format_prompt("prompt.txt", categories,3);
            std::cout << prompt << std::endl;
            std::cout << "Selected images: " << base64_images.size() << std::endl;

            // TEST REQUEST

            auto text_request = std::make_shared<ros_lm_interfaces::srv::OpenLLMRequest::Request>();
            text_request->action = 2;
            text_request->prompt = prompt;
            text_request->model_id = "openbmb/MiniCPM-o-2_6";
            text_request->images = base64_images;

            auto future = client -> async_send_request(text_request);
            VXL_INFO("Waiting for response for instance: {}", instance.get_instance()->InstanceID);
            // Espera hasta que la respuesta esté disponible
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = future.get();
                VXL_INFO("Instance {} disambiguated to category: {}", instance.get_instance()->InstanceID, response->generated_text);
            } else {
                VXL_ERROR("Failed to call service");
            }
            
            i++;
            
            // // Execute request
            // auto text_request = std::make_shared<ros_lm_interfaces::srv::OpenLLMRequest::Request>();
            // text_request->action = 2;
            // text_request->prompt = prompt;
            // text_request->model_id = "llava-hf/llava-1.5-7b-hf";
            // text_request->images = selected_images;

            // std::cout << text_request->images.size() << std::endl;

            // VXL_INFO("Requesting disambiguation for instance: {}", instance.get_instance()->InstanceID);
            // auto future = client -> async_send_request(text_request);

            // // Create a thread to handle the response
            // std::thread([this, &instance, future = std::move(future)]() mutable {
            //     VXL_INFO("Waiting for response for instance: {}", instance.get_instance()->InstanceID);
            //     // Espera hasta que la respuesta esté disponible
            //     if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            //         rclcpp::FutureReturnCode::SUCCESS)
            //     {
            //         auto response = future.get();
            //         VXL_INFO("Instance {} disambiguated to category: {}", instance.get_instance()->InstanceID, response->generated_text);
            //     } else {
            //         VXL_ERROR("Failed to call service");
            //     }
            // }).detach();
            
        }

    }


    /**
     * @brief Checks if the img_timestamp is included in any of the selected appearances of the instance
     * @brief If it is included, the image is added to the selected images map
     */
    void VoxelandDisambiguation::add_selected_images(sensor_msgs::msg::Image::SharedPtr image_msg, UncertainInstance& instance){
        std::map<std::string, std::vector<cv_bridge::CvImagePtr>>* appearances = instance.get_selected_images();
        cv_bridge::CvImagePtr cv_ptr;
        // Lopp through all the selected appearances
        for(auto& [category, appearances_vector] : *instance.get_selected_appearances()){
            for(uint32_t timestamp : appearances_vector){
                // If the timestamp is included in the selected appearances,
                // the image is added to the selected images map
                if(timestamp == image_msg->header.stamp.sec){
                    // Convert the image to a open_cv image
                    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, "bgr8");
                    cv_bridge::CvImagePtr cv_image_bbox = instance.get_bbox_image(cv_image, category, timestamp);
                    (*appearances)[category].push_back(cv_image_bbox);
                    VXL_INFO("Image {} added to instance: {} - Category: {}", image_msg->header.stamp.sec,instance.get_instance()->InstanceID, category);
                }
            }
        }
    }


    void VoxelandDisambiguation::llm_response(auto future, UncertainInstance& instance){
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),future) == rclcpp::FutureReturnCode::SUCCESS){
            auto response = future.get();
            instance.set_final_category(response->generated_text);
            VXL_INFO("Instance {} disambiguated to category: {}", instance.get_instance()->InstanceID, response->generated_text);
        } else {
            VXL_ERROR("Failed to call service");
        }
    }



}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(voxeland_disambiguation::VoxelandDisambiguation)