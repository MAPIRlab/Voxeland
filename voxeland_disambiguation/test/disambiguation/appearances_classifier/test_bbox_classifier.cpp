#include <gtest/gtest.h>
#include "disambiguation/appearances_classifier/appearances_classifier.hpp"
#include "disambiguation/json_semantics.hpp"


TEST(BBoxAppearancesClassifierTest, SelectsAppearancesWithLargestArea) {
    BBoxAppearancesClassifier classifier;
    auto instance = std::make_shared<JsonSemanticObject>();
    std::vector<std::string> categories = {"cat"};
    instance->appearances_timestamps["cat"][1] = BoundingBox2D{0, 0, 1, 2}; // área 2
    instance->appearances_timestamps["cat"][2] = BoundingBox2D{0, 0, 2, 3}; // área 6
    instance->appearances_timestamps["cat"][3] = BoundingBox2D{0, 0, 2, 2}; // área 4
    UncertainInstance uncertain(instance, 0.5);

    classifier.classify_instance_appearances(uncertain, categories, 2);

    auto* selected = uncertain.get_selected_appearances();
    ASSERT_EQ(selected->size(), 1);
    EXPECT_EQ((*selected)["cat"].size(), 2);
    EXPECT_EQ((*selected)["cat"][0], 2u);
    EXPECT_EQ((*selected)["cat"][1], 3u);
}

TEST(BBoxAppearancesClassifierTest, SelectsAllIfLessThanMaxAppearances) {
    BBoxAppearancesClassifier classifier;
    auto instance = std::make_shared<JsonSemanticObject>();
    std::vector<std::string> categories = {"dog"};
    instance->appearances_timestamps["dog"][10] = BoundingBox2D{0, 0, 1, 1}; // área 1
    instance->appearances_timestamps["dog"][20] = BoundingBox2D{0, 0, 2, 2}; // área 4
    UncertainInstance uncertain(instance, 0.5);

    classifier.classify_instance_appearances(uncertain, categories, 5);

    auto* selected = uncertain.get_selected_appearances();
    ASSERT_EQ(selected->size(), 1);
    EXPECT_EQ((*selected)["dog"].size(), 2);
    EXPECT_EQ((*selected)["dog"][0], 20u);
    EXPECT_EQ((*selected)["dog"][1], 10u);
}

TEST(BBoxAppearancesClassifierTest, ThrowsIfNoAppearancesForCategory) {
    BBoxAppearancesClassifier classifier;
    auto instance = std::make_shared<JsonSemanticObject>();
    std::vector<std::string> categories = {"bird"};
    UncertainInstance uncertain(instance, 0.5);

    EXPECT_THROW(classifier.classify_instance_appearances(uncertain, categories, 2), std::exception);
}

TEST(BBoxAppearancesClassifierTest, GetNameReturnsCorrectName) {
    BBoxAppearancesClassifier classifier;

    std::string name = classifier.get_name();

    EXPECT_EQ(name, "bbox");
}