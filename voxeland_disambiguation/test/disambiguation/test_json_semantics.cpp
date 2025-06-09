#include <gtest/gtest.h>
#include "disambiguation/json_semantics.hpp"

TEST(JsonSemanticMapTest, test_add_instance_updates_map) {
    JsonSemanticMap map;
    auto instance = std::make_shared<JsonSemanticObject>();
    instance->InstanceID = "test_id";

    map.add_instance(instance);

    std::shared_ptr<JsonSemanticObject> expected_instance = map.get_instance("test_id");
    ASSERT_NE(expected_instance, nullptr);
    EXPECT_EQ(expected_instance->InstanceID, "test_id");
}

TEST(JsonSemanticMapTest, test_get_instance_non_existent_returns_null) {
    JsonSemanticMap map;

    auto retrieved = map.get_instance("no_such_id");

    EXPECT_EQ(retrieved, nullptr);
}

TEST(JsonSemanticMapTest, test_get_instances_returns_valid_reference) {
    JsonSemanticMap map;
    auto instance = std::make_shared<JsonSemanticObject>();
    instance->InstanceID = "id";
    
    auto& vec = map.get_instances();
    
    map.add_instance(instance);
    EXPECT_EQ(vec.size(), 1);
}

TEST(UncertainInstanceTest, test_increase_one_disambiguation_result_updates_category_count) {
    auto instance = std::make_shared<JsonSemanticObject>();
    UncertainInstance uncertain(instance, 0.5);

    uncertain.increase_one_disambiguation_result("cat");
    uncertain.increase_one_disambiguation_result("cat");
    uncertain.increase_one_disambiguation_result("dog");

    auto* results = uncertain.get_disambiguation_results();
    EXPECT_EQ((*results)["cat"], 2);
    EXPECT_EQ((*results)["dog"], 1);
}

TEST(UncertainInstanceTest, test_set_selected_appearances_sets_correctly) {
    auto instance = std::make_shared<JsonSemanticObject>();
    UncertainInstance uncertain(instance, 0.1);
    std::map<std::string, std::vector<uint32_t>> appearances = {{"cat", {1,2,3}}};

    uncertain.set_selected_appearances(appearances);

    auto* selected = uncertain.get_selected_appearances();
    ASSERT_EQ((*selected)["cat"].size(), 3);
}

TEST(UncertainInstanceTest, test_get_selected_appearances_returns_reference) {
    auto instance = std::make_shared<JsonSemanticObject>();
    UncertainInstance uncertain(instance, 0.1);

    auto* selected = uncertain.get_selected_appearances();

    ASSERT_NE(selected, nullptr);
}

TEST(UncertainInstanceTest, test_get_entropy_returns_correct_value) {
    auto instance = std::make_shared<JsonSemanticObject>();
    UncertainInstance uncertain(instance, 0.42);

    auto res_entropy = uncertain.get_entropy();

    EXPECT_DOUBLE_EQ(res_entropy, 0.42);
}