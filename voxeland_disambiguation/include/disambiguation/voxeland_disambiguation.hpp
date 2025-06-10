
#include <sys/types.h>
#include <cstdint>
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
#include "disambiguation/pipeline/interface_pipeline_step.hpp"

#include "rclcpp/serialization.hpp"

namespace voxeland_disambiguation {
    class VoxelandDisambiguation : public rclcpp::Node
    {
        public:
            explicit VoxelandDisambiguation(const rclcpp::NodeOptions& node_options);
        private:
            // Configuration parameters
            std::string json_file;
            std::string json_appearances_file;
            std::string bag_path;
            std::string appearances_classifier;
            uint32_t n_images_per_category;
            uint32_t n_categories_per_instance;
            std::string output_file;
            std::string lvlm_model;
            uint32_t disambiguation_iters;
            bool update_map_service;

            std::vector<std::unique_ptr<PipelineStep>> pipeline_steps;

            void initialize_pipeline();
            void execute_pipeline();
    };
};