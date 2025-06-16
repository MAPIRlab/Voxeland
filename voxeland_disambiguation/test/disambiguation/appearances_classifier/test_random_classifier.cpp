#include <gtest/gtest.h>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <memory>
#include "disambiguation/appearances_classifier/appearances_classifier.hpp"
#include "disambiguation/json_semantics.hpp"


TEST(RandomAppearancesClassifierTest, test_classify_instance_appearances_with_few_appearances_selects_all) {
    RandomAppearancesClassifier classifier;
    JsonSemanticObject instance;
    std::vector<std::string> categories = {"cat"};
    instance.appearances_timestamps["cat"][10] = BoundingBox2D{};
    instance.appearances_timestamps["cat"][20] = BoundingBox2D{};
    UncertainInstance uncertain(std::make_shared<JsonSemanticObject>(instance), 0.5);

    classifier.classify_instance_appearances(uncertain, categories, 3);

    auto* selected = uncertain.get_selected_appearances();
    ASSERT_EQ(selected->size(), 1);
    EXPECT_EQ((*selected)["cat"].size(), 2);
    for (uint32_t id : (*selected)["cat"]) {
        EXPECT_TRUE(id == 10 || id == 20);
    }
}

TEST(RandomAppearancesClassifierTest, test_classify_instance_appearances_with_many_appearances_selects_max) {
    srand(42);
    RandomAppearancesClassifier classifier;
    JsonSemanticObject instance;
    std::vector<std::string> categories = {"dog"};
    instance.appearances_timestamps["dog"][100] = BoundingBox2D{};
    instance.appearances_timestamps["dog"][200] = BoundingBox2D{};
    instance.appearances_timestamps["dog"][300] = BoundingBox2D{};
    instance.appearances_timestamps["dog"][400] = BoundingBox2D{};
    UncertainInstance uncertain(std::make_shared<JsonSemanticObject>(instance), 0.5);

    classifier.classify_instance_appearances(uncertain, categories, 3);

    auto* selected = uncertain.get_selected_appearances();
    ASSERT_EQ(selected->size(), 1);
    EXPECT_EQ((*selected)["dog"].size(), 3);
    for (uint32_t id : (*selected)["dog"]) {
        EXPECT_TRUE(id == 100 || id == 200 || id == 300 || id == 400);
    }
}

TEST(RandomAppearancesClassifierTest, test_classify_instance_appearances_with_empty_category_throws_exception) {
    srand(123);
    RandomAppearancesClassifier classifier;
    auto instance = std::make_shared<JsonSemanticObject>();
    std::vector<std::string> categories = {"cat"};
    UncertainInstance uncertain(instance, 0.1);

    ASSERT_THROW(classifier.classify_instance_appearances(uncertain, categories, 1), std::exception);
};

TEST(RandomAppearancesClassifierTest, test_get_name_returns_correct_name) {
    RandomAppearancesClassifier classifier;

    std::string name = classifier.get_name();

    EXPECT_EQ(name, "random");
};