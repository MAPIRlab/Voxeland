
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <ros_lm_interfaces/srv/detail/open_llm_request__struct.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <string>
#include <vector>
#include <voxeland_map/semantics.hpp>
#include "json_semantics.hpp"
#include <ros_lm_interfaces/srv/open_llm_request.hpp>

#include "rclcpp/serialization.hpp"

namespace voxeland_disambiguation {
    class VoxelandDisambiguation : public rclcpp::Node
    {
        public:
            explicit VoxelandDisambiguation(const rclcpp::NodeOptions& node_options);

            //Pipeline methods
            void execute_pipeline();
            void find_uncertain_instances();
            void select_appearances();
            void obtain_bag_images();
            void ask_llm_for_disambiguation();
        protected:
            void add_selected_images(sensor_msgs::msg::Image::SharedPtr image_msg, UncertainInstance& instance);

            std::string json_file;
            std::string json_appearances_file;
            std::string bag_path;
            
            JsonSemanticMap semantic_map;
            std::vector<UncertainInstance> uncertain_instances;

            rclcpp::Client<ros_lm_interfaces::srv::OpenLLMRequest>::SharedPtr client;

            // Instance variables for the scene bag reader
            rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer;
            std::unique_ptr<rosbag2_cpp::Reader> reader;
        
    };
};