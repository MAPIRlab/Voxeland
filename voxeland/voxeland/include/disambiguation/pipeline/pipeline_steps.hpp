#pragma once

#include <memory>
#include <rclcpp/node.hpp>
#include <ros_lm_interfaces/srv/detail/open_llm_request__struct.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/client.hpp>
#include <string>
#include <vector>
#include "disambiguation/disambiguation_context.hpp"
#include "disambiguation/json_semantics.hpp"
#include "disambiguation/appearances_classifier/appearances_classifier.hpp"
#include "disambiguation/pipeline/interface_pipeline_step.hpp"

class AbstractPipelineStep : public PipelineStep{
    protected:
        std::shared_ptr<DisambiguationContext> context = DisambiguationContext::get_context_instance();

};

// ------- DEFINED PIPELINE STEPS -------

class JsonDeserializationStep : public AbstractPipelineStep{
    public:
        JsonDeserializationStep(const std::string& json_file, const std::string& json_appearances_file);
        void execute() override;
    private:
        std::string json_file;
        std::string json_appearances_file;

        void serialize_map(JsonSemanticMap& map);
        
        // Auxiliary functions
        std::map<std::string, std::map<uint32_t,BoundingBox2D>> parse_appearances_timestamps(nlohmann::json& appearances_timestamps);
        std::map<std::string, double> parse_results(nlohmann::json& results);
        BoundingBox3D parse_bbox(nlohmann::json& bbox);
        JsonSemanticObject serialize_instance(const std::string& instance_id, nlohmann::json& instance_json);
};

class UncertainInstanceIdentificationStep : public AbstractPipelineStep{
    public:
        void execute() override;
    private:
        std::vector<UncertainInstance> identify_uncertain_instances(JsonSemanticMap& map);
};

class AppeareancesSelectionStep : public AbstractPipelineStep{
    public:
        AppeareancesSelectionStep(std::unique_ptr<AppearancesClassifier> classifier, uint32_t n_images_per_category, uint32_t n_categories_per_instance);
        void execute() override;
    private:
        std::unique_ptr<AppearancesClassifier> appearances_classifier;
        uint32_t n_images_per_category;
        uint32_t n_categories_per_instance;

        void select_category_appearances(std::vector<UncertainInstance>& uncertain_instances);

        // Auxiliary functions
        std::vector<std::string> choose_selected_categories(std::map<std::string, double>& results);
        std::vector<std::pair<std::string, double>> sort_results_map(std::map<std::string, double>& results);
};

class ImageBagReading : public AbstractPipelineStep{
    public:
        ImageBagReading(const std::string& bag_path);
        void execute() override;
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
        void execute() override;
    private:
        std::string lvlm_model;
        rclcpp::Node::SharedPtr node;
        rclcpp::Client<ros_lm_interfaces::srv::OpenLLMRequest>::SharedPtr client;

        void disambiguate_instances(std::vector<UncertainInstance>& uncertain_instances);
        
        void init_client();
        bool load_model();
};

class JsonSerializationStep : public AbstractPipelineStep{
    public:
        JsonSerializationStep(const std::string& output_file);
        void execute() override;
    private:
        std::string output_file;
};