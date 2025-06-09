#pragma once

#include <memory>
#include <rclcpp/node.hpp>
#include <ros_lm_interfaces/srv/detail/open_llm_request__struct.hpp>
#include <voxeland_msgs/srv/detail/load_map__struct.hpp>
#include <voxeland_msgs/srv/detail/update_map_results__struct.hpp>
#include <voxeland_msgs/srv/load_map.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/client.hpp>
#include <string>
#include <vector>
#include "disambiguation/disambiguation_context.hpp"
#include "disambiguation/json_semantics.hpp"
#include "disambiguation/appearances_classifier/appearances_classifier.hpp"
#include "disambiguation/pipeline/interface_pipeline_step.hpp"
#include "nlohmann/json.hpp"

class AbstractPipelineStep : public PipelineStep{
    protected:
        std::shared_ptr<DisambiguationContext> context = DisambiguationContext::get_context_instance();

};

// ------- DEFINED PIPELINE STEPS -------

class JsonDeserializationStep : public AbstractPipelineStep{
    public:
        JsonDeserializationStep(const std::string& json_file, const std::string& json_appearances_file);
        bool execute() override;
    private:
        std::string json_file;
        std::string json_appearances_file;

        void deserialize_map(JsonSemanticMap& map);
        
        // Auxiliary functions
        std::map<std::string, std::map<uint32_t,BoundingBox2D>> parse_appearances_timestamps(nlohmann::json& appearances_timestamps);
        std::map<std::string, double> parse_results(nlohmann::json& results);
        BoundingBox3D parse_bbox(nlohmann::json& bbox);
        JsonSemanticObject deserialize_instance(const std::string& instance_id, nlohmann::json& instance_json, nlohmann::json& instance_appeareances_json);
};

class UncertainInstanceIdentificationStep : public AbstractPipelineStep{
    public:
        bool execute() override;
    private:
        std::vector<UncertainInstance> identify_uncertain_instances(JsonSemanticMap& map);
};

class AppeareancesSelectionStep : public AbstractPipelineStep{
    public:
        AppeareancesSelectionStep(std::unique_ptr<AppearancesClassifier> classifier, uint32_t n_images_per_category, uint32_t n_categories_per_instance);
        bool execute() override;
    private:
        std::unique_ptr<AppearancesClassifier> appearances_classifier;
        uint32_t n_images_per_category;
        uint32_t n_categories_per_instance;

        void select_category_appearances(std::vector<UncertainInstance>& uncertain_instances);

        // Auxiliary functions
        std::vector<std::string> choose_selected_categories(const std::map<std::string, double>& results);
        std::vector<std::pair<std::string, double>> sort_results_map(const std::map<std::string, double>& results);
};

class ImageBagReading : public AbstractPipelineStep{
    public:
        ImageBagReading(const std::string& bag_path);
        bool execute() override;
    private:
        std::string bag_path;
        rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer;
        std::unique_ptr<rosbag2_cpp::Reader> reader;

        void obtain_bag_images(std::vector<UncertainInstance>& uncertain_instances);
        void add_selected_images(sensor_msgs::msg::Image::SharedPtr image_msg, UncertainInstance& instance);

        // Debug
        void show_all_images(std::vector<UncertainInstance>& uncertain_instances);
        void show_category_images(std::string category, std::vector<cv_bridge::CvImagePtr> images);
};

class LVLMDisambiguationStep : public AbstractPipelineStep{
    public:
        LVLMDisambiguationStep(const std::string& lvlm_model);
        bool execute() override;
    private:
        std::string lvlm_model;
        rclcpp::Node::SharedPtr node;
        rclcpp::Client<ros_lm_interfaces::srv::OpenLLMRequest>::SharedPtr client;

        void disambiguate_instances(std::vector<UncertainInstance>& uncertain_instances);
        
        void init_client();
        bool load_model();
        void send_and_handle_request(std::shared_ptr<ros_lm_interfaces::srv::OpenLLMRequest::Request> request, std::vector<std::string>& categories, UncertainInstance& instance);
        std::string get_category_from_response(std::string response, const std::vector<std::string>& categories);
};

class UncertainResultsUpdateStep : public AbstractPipelineStep{
    public:
        bool execute() override;
    private:
        void update_uncertain_instances_results(std::vector<UncertainInstance>& uncertain_instances);
};

class JsonSerializationStep : public AbstractPipelineStep{
    public:
        JsonSerializationStep(const std::string& output_file);
        bool execute() override;
    private:
        std::string output_file;
        rclcpp::Node::SharedPtr node;
        rclcpp::Client<voxeland_msgs::srv::UpdateMapResults>::SharedPtr client;

        nlohmann::json serialize_map(JsonSemanticMap& map);
        void save_map(const nlohmann::json& map_json);
        void send_map_to_server(const nlohmann::json& map_json);

        void init_client();
        nlohmann::json serialize_instance(JsonSemanticObject& instance);
        nlohmann::json serialize_bbox(BoundingBox3D& bbox);
        nlohmann::json serialize_results(std::map<std::string, double>& results);
};

class MetricsSaveStep : public AbstractPipelineStep{
    public:
        MetricsSaveStep(const std::string& classifer_name, const std::string& lvlm_model);
        bool execute() override;
    private:
        std::string classifier_name;
        std::string lvlm_model;
        std::string output_file;

        nlohmann::json serialize_metrics(std::vector<UncertainInstance>& uncertain_instances);
        void save_metrics(const nlohmann::json& metrics_json);
};