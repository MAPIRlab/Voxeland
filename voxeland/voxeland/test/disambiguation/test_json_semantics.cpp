#include <gtest/gtest.h>
#include "disambiguation/json_semantics.hpp"

// Test para JsonSemanticMap: add_instance y get_instance
TEST(JsonSemanticMapTest, AddAndGetInstance) {
    JsonSemanticMap map;
    auto instance = std::make_shared<JsonSemanticObject>();
    instance->InstanceID = "test_id";

    map.add_instance(instance);

    auto retrieved = map.get_instance("test_id");
    ASSERT_NE(retrieved, nullptr);
    EXPECT_EQ(retrieved->InstanceID, "test_id");
}

// Test para JsonSemanticMap: get_instance con ID inexistente
TEST(JsonSemanticMapTest, GetInstanceNotFound) {
    JsonSemanticMap map;

    auto retrieved = map.get_instance("no_such_id");

    EXPECT_EQ(retrieved, nullptr);
}

// Test para JsonSemanticMap: get_instances devuelve referencia válida
TEST(JsonSemanticMapTest, GetInstancesReference) {
    JsonSemanticMap map;
    auto& vec = map.get_instances();
    EXPECT_TRUE(vec.empty());
    auto instance = std::make_shared<JsonSemanticObject>();
    instance->InstanceID = "id";

    map.add_instance(instance);

    EXPECT_EQ(vec.size(), 1);
}

// Test para UncertainInstance: increase_one_disambiguation_result
TEST(UncertainInstanceTest, IncreaseOneDisambiguationResult) {
    auto instance = std::make_shared<JsonSemanticObject>();
    UncertainInstance uncertain(instance, 0.5);

    uncertain.increase_one_disambiguation_result("cat");
    uncertain.increase_one_disambiguation_result("cat");
    uncertain.increase_one_disambiguation_result("dog");

    auto* results = uncertain.get_disambiguation_results();
    EXPECT_EQ((*results)["cat"], 2);
    EXPECT_EQ((*results)["dog"], 1);
}

// Test para UncertainInstance: set_selected_appearances y get_selected_appearances
TEST(UncertainInstanceTest, SetAndGetSelectedAppearances) {
    auto instance = std::make_shared<JsonSemanticObject>();
    UncertainInstance uncertain(instance, 0.1);
    std::map<std::string, std::vector<uint32_t>> appearances = {{"cat", {1,2,3}}};

    uncertain.set_selected_appearances(appearances);
    auto* selected = uncertain.get_selected_appearances();

    ASSERT_EQ((*selected)["cat"].size(), 3);
}

// Test para UncertainInstance: get_entropy
TEST(UncertainInstanceTest, GetEntropy) {
    auto instance = std::make_shared<JsonSemanticObject>();
    UncertainInstance uncertain(instance, 0.42);

    auto res_entropy = uncertain.get_entropy();

    EXPECT_DOUBLE_EQ(res_entropy, 0.42);
}