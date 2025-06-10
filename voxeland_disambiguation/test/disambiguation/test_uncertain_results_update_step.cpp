#include <gtest/gtest.h>
#include "disambiguation/pipeline/pipeline_steps.hpp"
#include "disambiguation/json_semantics.hpp"

// AAA: Arrange, Act, Assert

TEST(UncertainResultsUpdateStep,test_execute_with_valid_disambiguation_results_returns_true_and_updates_results) {
    auto instance = std::make_shared<JsonSemanticObject>();
    instance->InstanceID = "test";
    instance->results = {{"cat", 1.0}, {"dog", 2.0}};
    UncertainInstance uncertain(instance, 0.5);
    std::map<std::string, uint32_t> disambiguation_results = {{"cat", 2}, {"dog", 1}};
    uncertain.get_disambiguation_results()->clear();
    *(uncertain.get_disambiguation_results()) = disambiguation_results;

    auto context = DisambiguationContext::get_context_instance();
    context->get_uncertain_instances()->clear();
    context->get_uncertain_instances()->push_back(uncertain);
    UncertainResultsUpdateStep step = UncertainResultsUpdateStep(100);

    bool result = step.execute();

    EXPECT_TRUE(result);
    auto& updated_results = uncertain.get_instance()->results;
    EXPECT_GE(updated_results["cat"], 3.0); // 1.0 + 2
    EXPECT_GE(updated_results["dog"], 3.0); // 2.0 + 1
}

TEST(UncertainResultsUpdateStep,test_execute_with_empty_uncertain_instances_returns_false) {
    UncertainResultsUpdateStep step = UncertainResultsUpdateStep(100);
    auto context = DisambiguationContext::get_context_instance();
    context->get_uncertain_instances()->clear();

    bool result = step.execute();

    EXPECT_FALSE(result);
}

TEST(UncertainResultsUpdateStep,test_execute_with_missing_disambiguation_results_returns_false) {
    auto instance = std::make_shared<JsonSemanticObject>();
    instance->InstanceID = "test3";
    instance->results = {{"cat", 1.0}};
    UncertainInstance uncertain(instance, 0.5);
    uncertain.get_disambiguation_results()->clear();
    auto context = DisambiguationContext::get_context_instance();
    context->get_uncertain_instances()->clear();
    context->get_uncertain_instances()->push_back(uncertain);
    UncertainResultsUpdateStep step = UncertainResultsUpdateStep(100);

    bool result = step.execute();

    EXPECT_FALSE(result);
}