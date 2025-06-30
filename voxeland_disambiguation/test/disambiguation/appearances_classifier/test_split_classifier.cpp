#include <gtest/gtest.h>
#include "disambiguation/appearances_classifier/appearances_classifier.hpp"
#include "disambiguation/json_semantics.hpp"


TEST(SplitAppearancesClassifierTest, test_classify_instance_appearances_with_few_appearances_selects_all) {
    SplitAppearancesClassifier classifier;
    auto instance = std::make_shared<JsonSemanticObject>();
    std::vector<std::string> categories = {"cat", "dog"};
    instance->appearances_timestamps["cat"][10] = BoundingBox2D{};
    instance->appearances_timestamps["cat"][30] = BoundingBox2D{};
    instance->appearances_timestamps["dog"][100] = BoundingBox2D{};
    instance->appearances_timestamps["dog"][500] = BoundingBox2D{};
    UncertainInstance uncertain(instance, 0.5);

    classifier.classify_instance_appearances(uncertain, categories, 3);

    auto* selected = uncertain.get_selected_appearances();
    ASSERT_EQ(selected->size(), 2);
    EXPECT_EQ((*selected)["cat"].size(), 2);
    EXPECT_EQ((*selected)["dog"].size(), 2);
    std::vector<uint32_t> expected_cat = {10,  30};
    std::vector<uint32_t> expected_dog = {100, 500};
    EXPECT_EQ((*selected)["cat"], expected_cat);
    EXPECT_EQ((*selected)["dog"], expected_dog);
}

TEST(SplitAppearancesClassifierTest, test_classify_instance_appearances_with_many_appearances_selects_split) {
    SplitAppearancesClassifier classifier;
    auto instance = std::make_shared<JsonSemanticObject>();
    std::vector<std::string> categories = {"dog"};
    instance->appearances_timestamps["dog"][100] = BoundingBox2D{};
    instance->appearances_timestamps["dog"][200] = BoundingBox2D{};
    instance->appearances_timestamps["dog"][300] = BoundingBox2D{};
    instance->appearances_timestamps["dog"][400] = BoundingBox2D{};
    instance->appearances_timestamps["dog"][500] = BoundingBox2D{};
    instance->appearances_timestamps["dog"][600] = BoundingBox2D{};
    instance->appearances_timestamps["dog"][700] = BoundingBox2D{};
    instance->appearances_timestamps["dog"][800] = BoundingBox2D{};
    UncertainInstance uncertain(instance, 0.5);

    classifier.classify_instance_appearances(uncertain, categories, 3);

    auto* selected = uncertain.get_selected_appearances();
    ASSERT_EQ(selected->size(), 1);
    EXPECT_EQ((*selected)["dog"].size(), 3);
    std::vector<uint32_t> expected_dog = {100, 500, 800}; // Ejemplo de selección
    EXPECT_EQ((*selected)["dog"], expected_dog);
}

TEST(SplitAppearancesClassifierTest, test_classify_instance_appearances_with_empty_category_throws_exception) {
    SplitAppearancesClassifier classifier;
    auto instance = std::make_shared<JsonSemanticObject>();
    std::vector<std::string> categories = {"cat"};
    UncertainInstance uncertain(instance, 0.1);

    EXPECT_ANY_THROW(classifier.classify_instance_appearances(uncertain, categories, 1));
}

TEST(SplitAppearancesClassifierTest, test_get_name_returns_correct_name) {
    SplitAppearancesClassifier classifier;

    std::string name = classifier.get_name();

    EXPECT_EQ(name, "split");
}