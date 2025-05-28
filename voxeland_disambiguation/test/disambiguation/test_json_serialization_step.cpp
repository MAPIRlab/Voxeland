#include <gtest/gtest.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include "disambiguation/disambiguation_context.hpp"
#include "disambiguation/json_semantics.hpp"
#include "disambiguation/pipeline/pipeline_steps.hpp"

TEST(JsonSerializationStep, test_execute_empty_map_returns_false) {
    // Arrange
    JsonSerializationStep step("test_output.json");
    auto context = DisambiguationContext::get_context_instance();
    context->get_semantic_map()->get_instances().clear();

    // Act
    bool result = step.execute();

    // Assert
    EXPECT_FALSE(result);
}

TEST(JsonSerializationStep, test_execute_with_instances_returns_true_and_creates_json_with_instances) {
    // Arrange
    std::shared_ptr<JsonSemanticObject> instance = std::make_shared<JsonSemanticObject>();
    instance->InstanceID = "test_instance";
    instance->bbox = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    instance->n_observations = 5;
    instance->results = {{"category1", 0.9}, {"category2", 0.1}};
    auto context = DisambiguationContext::get_context_instance();
    auto semantic_map = context->get_semantic_map();
    semantic_map->get_instances().clear();
    semantic_map->add_instance(instance);
    JsonSerializationStep step("test_output.json");

    // Act
    bool result = step.execute();

    // Assert
    EXPECT_TRUE(result);
    std::ifstream in_file("test_output.json");
    ASSERT_TRUE(in_file.is_open());
    nlohmann::json map_json;
    in_file >> map_json;
    in_file.close();
    ASSERT_FALSE(map_json.empty());
    ASSERT_TRUE(map_json.contains("instances"));
    ASSERT_TRUE(map_json["instances"].contains("test_instance"));
    // Limpieza
    std::remove("test_output.json");
}

TEST(JsonSerializationStep, test_execute_throws_if_file_cannot_be_opened) {
    // Arrange
    // Intenta escribir en un directorio que no existe para forzar la excepción
    JsonSerializationStep step("/no_such_dir/test_output.json");
    std::shared_ptr<JsonSemanticObject> instance = std::make_shared<JsonSemanticObject>();
    instance->InstanceID = "test_instance";
    auto context = DisambiguationContext::get_context_instance();
    auto semantic_map = context->get_semantic_map();
    semantic_map->get_instances().clear();
    semantic_map->add_instance(instance);

    // Act
    bool result = step.execute();

    // Assert
    EXPECT_FALSE(result);
}