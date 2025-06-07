#include <gtest/gtest.h>
#include "disambiguation/appearances_classifier/appearances_classifier.hpp"
#include "disambiguation/json_semantics.hpp"


TEST(SplitBboxAreaAppearancesClassifierTest, AppearancesSizeLowerThanMaxAppearances) {
    SplitBboxAreaAppearancesClassifier classifier;
    auto instance = std::make_shared<JsonSemanticObject>();
    std::vector<std::string> categories = {"cat", "dog"};
    instance->appearances_timestamps["cat"][10] = BoundingBox2D{0, 0, 5, 5}; // area = 25
    instance->appearances_timestamps["cat"][30] = BoundingBox2D{0, 0, 3, 3}; // area = 9
    instance->appearances_timestamps["dog"][100] = BoundingBox2D{0, 0, 4, 4}; // area = 16
    instance->appearances_timestamps["dog"][500] = BoundingBox2D{0, 0, 6, 6}; // area = 36
    UncertainInstance uncertain(instance, 0.5);

    classifier.classify_instance_appearances(uncertain, categories, 3);

    auto* selected = uncertain.get_selected_appearances();
    ASSERT_EQ(selected->size(), 2);
    EXPECT_EQ((*selected)["cat"].size(), 2);
    EXPECT_EQ((*selected)["dog"].size(), 2);
    // Should select all appearances since we have less than max_appearances
    std::vector<uint32_t> expected_cat = {10, 30};
    std::vector<uint32_t> expected_dog = {100, 500};
    EXPECT_EQ((*selected)["cat"], expected_cat);
    EXPECT_EQ((*selected)["dog"], expected_dog);
}

TEST(SplitBboxAreaAppearancesClassifierTest, AppearancesSizeGreaterThanMaxAppearances) {
    SplitBboxAreaAppearancesClassifier classifier;
    auto instance = std::make_shared<JsonSemanticObject>();
    std::vector<std::string> categories = {"dog"};
    instance->appearances_timestamps["dog"][100] = BoundingBox2D{0, 0, 2, 2}; // area = 4
    instance->appearances_timestamps["dog"][200] = BoundingBox2D{0, 0, 5, 5}; // area = 25 (largest in first subset)
    instance->appearances_timestamps["dog"][300] = BoundingBox2D{0, 0, 3, 3}; // area = 9
    instance->appearances_timestamps["dog"][400] = BoundingBox2D{0, 0, 7, 7}; // area = 49 (largest in second subset)
    instance->appearances_timestamps["dog"][500] = BoundingBox2D{0, 0, 4, 4}; // area = 16
    instance->appearances_timestamps["dog"][600] = BoundingBox2D{0, 0, 1, 1}; // area = 1
    instance->appearances_timestamps["dog"][700] = BoundingBox2D{0, 0, 6, 6}; // area = 36 (largest in third subset)
    instance->appearances_timestamps["dog"][800] = BoundingBox2D{0, 0, 2, 2}; // area = 4
    UncertainInstance uncertain(instance, 0.5);

    classifier.classify_instance_appearances(uncertain, categories, 3);

    auto* selected = uncertain.get_selected_appearances();
    ASSERT_EQ(selected->size(), 1);
    EXPECT_EQ((*selected)["dog"].size(), 3);
    std::vector<uint32_t> expected_dog = {200, 400, 700};
    EXPECT_EQ((*selected)["dog"], expected_dog);
}

TEST(SplitBboxAreaAppearancesClassifierTest, SingleAppearancePerSubset) {
    SplitBboxAreaAppearancesClassifier classifier;
    auto instance = std::make_shared<JsonSemanticObject>();
    std::vector<std::string> categories = {"cat"};
    instance->appearances_timestamps["cat"][100] = BoundingBox2D{0, 0, 2, 2}; // area = 4
    instance->appearances_timestamps["cat"][200] = BoundingBox2D{0, 0, 5, 5}; // area = 25
    instance->appearances_timestamps["cat"][300] = BoundingBox2D{0, 0, 3, 3}; // area = 9
    UncertainInstance uncertain(instance, 0.5);

    classifier.classify_instance_appearances(uncertain, categories, 3);

    auto* selected = uncertain.get_selected_appearances();
    ASSERT_EQ(selected->size(), 1);
    EXPECT_EQ((*selected)["cat"].size(), 3);
    std::vector<uint32_t> expected_cat = {100, 200, 300};
    EXPECT_EQ((*selected)["cat"], expected_cat);
}

TEST(SplitBboxAreaAppearancesClassifierTest, HandlesEmptyAppearancesGracefully) {
    SplitBboxAreaAppearancesClassifier classifier;
    auto instance = std::make_shared<JsonSemanticObject>();
    std::vector<std::string> categories = {"cat"};
    UncertainInstance uncertain(instance, 0.1);

    EXPECT_ANY_THROW(classifier.classify_instance_appearances(uncertain, categories, 1));
}

TEST(SplitBboxAreaAppearancesClassifierTest, GetNameReturnsCorrectName) {
    SplitBboxAreaAppearancesClassifier classifier;

    std::string name = classifier.get_name();

    EXPECT_EQ(name, "split_bbox");
}