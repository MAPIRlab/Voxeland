#include <gtest/gtest.h>
#include "disambiguation/disambiguation_context.hpp"
#include "disambiguation/json_semantics.hpp"
#include "disambiguation/pipeline/pipeline_steps.hpp"

TEST(JsonSerializationStep, test_execute_empty_map_creates_empty_json) {
    JsonSerializationStep step("test_output.json");

    step.execute();

    std::ifstream in_file("test_output.json");
    ASSERT_TRUE(in_file.is_open());    
    nlohmann::json map_json;
    in_file >> map_json;
    in_file.close();
    ASSERT_TRUE(map_json.empty());
}

TEST(JsonSerializationStep, test_execute_with_instances_creates_json_with_instances) {
    std::shared_ptr<JsonSemanticObject> instance = std::make_shared<JsonSemanticObject>();
    instance->InstanceID = "test_instance";
    instance->bbox = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    instance->n_observations = 5;
    instance->results = {{"category1", 0.9}, {"category2", 0.1}};
    std::shared_ptr<DisambiguationContext> context = DisambiguationContext::get_context_instance();
    auto semantic_map = context->get_semantic_map();
    semantic_map->add_instance(instance);
    JsonSerializationStep step("test_output.json");

    step.execute();

    std::ifstream in_file("test_output.json");
    ASSERT_TRUE(in_file.is_open());
    nlohmann::json map_json;
    in_file >> map_json;
    in_file.close();
    ASSERT_FALSE(map_json.empty());
    ASSERT_TRUE(map_json.contains("instances"));
    ASSERT_TRUE(map_json["instances"].contains("test_instance"));
    context.reset();
}