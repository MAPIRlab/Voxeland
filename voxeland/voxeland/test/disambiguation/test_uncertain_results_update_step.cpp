#include <gtest/gtest.h>
#include "disambiguation/pipeline/pipeline_steps.hpp"
#include "disambiguation/json_semantics.hpp"

TEST(UncertainResultsUpdateStepTest, UpdatesResultsCorrectlyWithExecute) {
    // Arrange
    auto instance = std::make_shared<JsonSemanticObject>();
    instance->InstanceID = "test";
    instance->results = {{"cat", 1.0}, {"dog", 2.0}};
    UncertainInstance uncertain(instance, 0.5);

    // Simula resultados de desambiguación
    std::map<std::string, uint32_t> disambiguation_results = {{"cat", 2}, {"dog", 1}};
    uncertain.get_disambiguation_results()->clear();
    *(uncertain.get_disambiguation_results()) = disambiguation_results;

    // Prepara el contexto global
    auto context = DisambiguationContext::get_context_instance();
    context->get_uncertain_instances()->clear();
    context->get_uncertain_instances()->push_back(uncertain);

    UncertainResultsUpdateStep step;

    // Act
    step.execute();

    // Assert
    auto& updated_results = uncertain.get_instance()->results;
    EXPECT_DOUBLE_EQ(updated_results["cat"], 3.0); // 1.0 + 2
    EXPECT_DOUBLE_EQ(updated_results["dog"], 3.0); // 2.0 + 1
}

TEST(UncertainResultsUpdateStepTest, AddsNewCategoryIfNotPresentWithExecute) {
    // Arrange
    auto instance = std::make_shared<JsonSemanticObject>();
    instance->InstanceID = "test2";
    instance->results = {{"cat", 1.0}};
    UncertainInstance uncertain(instance, 0.5);

    // Solo hay resultado para "dog", que no estaba antes
    std::map<std::string, uint32_t> disambiguation_results = {{"dog", 5}};
    uncertain.get_disambiguation_results()->clear();
    *(uncertain.get_disambiguation_results()) = disambiguation_results;

    // Prepara el contexto global
    auto context = DisambiguationContext::get_context_instance();
    context->get_uncertain_instances()->clear();
    context->get_uncertain_instances()->push_back(uncertain);

    UncertainResultsUpdateStep step;

    // Act
    step.execute();

    // Assert
    auto& updated_results = uncertain.get_instance()->results;
    EXPECT_DOUBLE_EQ(updated_results["cat"], 1.0);
    EXPECT_DOUBLE_EQ(updated_results["dog"], 5.0); // Se añade nueva categoría
}