#include <gtest/gtest.h>
#include "disambiguation/appearances_classifier/appearances_classifier.hpp"
#include "disambiguation/json_semantics.hpp"

// AAA: Arrange, Act, Assert

TEST(SplitAppearancesClassifierTest, AppearancesSizeLowerThanMaxAppearances) {
    // Arrange
    SplitAppearancesClassifier classifier;
    auto instance = std::make_shared<JsonSemanticObject>();
    std::vector<std::string> categories = {"cat", "dog"};
    // Añade 3 apariencias para "cat" y 2 para "dog"
    instance->appearances_timestamps["cat"][10] = BoundingBox2D{};
    instance->appearances_timestamps["cat"][30] = BoundingBox2D{};
    instance->appearances_timestamps["dog"][100] = BoundingBox2D{};
    instance->appearances_timestamps["dog"][500] = BoundingBox2D{};
    UncertainInstance uncertain(instance, 0.5);

    // Act
    classifier.classify_instance_appearances(uncertain, categories, 3);

    // Assert
    auto* selected = uncertain.get_selected_appearances();
    ASSERT_EQ(selected->size(), 2);
    EXPECT_EQ((*selected)["cat"].size(), 2);
    EXPECT_EQ((*selected)["dog"].size(), 2);
    std::vector<uint32_t> expected_cat = {10,  30};
    std::vector<uint32_t> expected_dog = {100, 500};
    EXPECT_EQ((*selected)["cat"], expected_cat);
    EXPECT_EQ((*selected)["dog"], expected_dog);
}

TEST(SplitAppearancesClassifierTest, AppearancesSizeGreaterThanMaxAppearances) {
    // Arrange
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

    // Act
    classifier.classify_instance_appearances(uncertain, categories, 3);

    // Assert
    auto* selected = uncertain.get_selected_appearances();
    ASSERT_EQ(selected->size(), 1);
    EXPECT_EQ((*selected)["dog"].size(), 3);
    std::vector<uint32_t> expected_dog = {100, 500, 800}; // Ejemplo de selección
    EXPECT_EQ((*selected)["dog"], expected_dog);
}

TEST(SplitAppearancesClassifierTest, HandlesEmptyAppearancesGracefully) {
    // Arrange
    SplitAppearancesClassifier classifier;
    auto instance = std::make_shared<JsonSemanticObject>();
    std::vector<std::string> categories = {"cat"};
    UncertainInstance uncertain(instance, 0.1);

    // Act & Assert
    EXPECT_ANY_THROW(classifier.classify_instance_appearances(uncertain, categories, 1));
}

TEST(SplitAppearancesClassifierTest, GetNameReturnsCorrectName) {
    // Arrange
    SplitAppearancesClassifier classifier;

    // Act
    std::string name = classifier.get_name();

    // Assert
    EXPECT_EQ(name, "split");
}