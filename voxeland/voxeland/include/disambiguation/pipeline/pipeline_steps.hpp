#include <memory>
#include <ros_lm_interfaces/srv/detail/open_llm_request__struct.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/client.hpp>
#include "disambiguation/disambiguation_context.hpp"
#include "disambiguation/json_semantics.hpp"
#include "disambiguation/pipeline/interface_pipeline_step.hpp"

class AbstractPipelineStep : public PipelineStep{
    
    public:
        void set_next(PipelineStep* next_step) override;
        
    protected:
        std::shared_ptr<DisambiguationContext> context = DisambiguationContext::get_context_instance();
        
        void execute_next();

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
        BoundingBox3D parse_bbox(nlohmann::json& bbox);
        std::map<std::string, std::map<uint32_t,BoundingBox2D>> parse_appearances_timestamps(nlohmann::json& appearances_timestamps);
        std::map<std::string, double> parse_results(nlohmann::json& results);
        JsonSemanticObject serialize_instance(nlohmann::json& instance_json);
};

class UncertainInstanceIdentificationStep : public AbstractPipelineStep{
    public:
        void execute() override;
};

class AppeareancesSelectionStep : public AbstractPipelineStep{
    public:
        AppeareancesSelectionStep(std::string appearances_classifier, uint32_t n_images_per_category, uint32_t n_categories_per_instance);
        void execute() override;
    private:
        std::string appearances_classifier;
        uint32_t n_images_per_category;
        uint32_t n_categories_per_instance;
};

class ImageBagReading : public AbstractPipelineStep{
    public:
        ImageBagReading(const std::string& bag_path);
        void execute() override;
    private:
        std::string bag_path;
        rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer;
};

class LVLMDisambiguationStep : public AbstractPipelineStep{
    public:
        LVLMDisambiguationStep(const std::string& lvlm_model);
        void execute() override;
    private:
        std::string lvlm_model;
        rclcpp::Client<ros_lm_interfaces::srv::OpenLLMRequest>::SharedPtr client;
        std::unique_ptr<rosbag2_cpp::Reader> reader;
};

class JsonSerializationStep : public AbstractPipelineStep{
    public:
        JsonSerializationStep(const std::string& output_file);
        void execute() override;
    private:
        std::string output_file;
};