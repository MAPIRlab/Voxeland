#include <gtest/gtest.h>
#include "disambiguation/disambiguation_context.hpp"
#include "disambiguation/json_semantics.hpp"
#include "disambiguation/pipeline/pipeline_steps.hpp"

TEST(UncertainInstanceIdentifiacionStep, test_execute_empty_map_returns_false_and_empty_vector) {
    UncertainInstanceIdentificationStep step;
    auto context = DisambiguationContext::get_context_instance();
    context->get_semantic_map()->get_instances().clear();

    bool result = step.execute();

    std::vector<UncertainInstance> uncertain_instances = *context->get_uncertain_instances();
    EXPECT_TRUE(uncertain_instances.empty());
    EXPECT_FALSE(result);
    context.reset();
}

TEST(UncertainInstanceIdentifiacionStep, test_execute_with_instances_below_entropy_threshold_returns_true_and_empty_vector) {
    JsonSemanticObject test_object;
    test_object.InstanceID = "low_entropy_object";
    test_object.results = {
        {"sensor1", 100},
        {"sensor2", 7},
        {"sensor3", 8}
    };
    auto context = DisambiguationContext::get_context_instance();
    auto semantic_map = context->get_semantic_map();
    semantic_map->get_instances().clear();
    semantic_map->add_instance(std::make_shared<JsonSemanticObject>(test_object));
    UncertainInstanceIdentificationStep step;

    bool result = step.execute();

    std::vector<UncertainInstance> uncertain_instances = *context->get_uncertain_instances();
    EXPECT_TRUE(uncertain_instances.empty());
    EXPECT_TRUE(result);
    context.reset();
}

TEST(UncertainInstanceIdentifiacionStep, test_execute_with_instances_identifies_uncertain_instances_and_returns_true) {
    JsonSemanticObject test_object;
    test_object.InstanceID = "test_object";
    test_object.results = {
        {"sensor1", 0.5},
        {"sensor2", 0.5}
    };
    auto context = DisambiguationContext::get_context_instance();
    auto semantic_map = context->get_semantic_map();
    semantic_map->get_instances().clear();
    semantic_map->add_instance(std::make_shared<JsonSemanticObject>(test_object));
    UncertainInstanceIdentificationStep step;

    bool result = step.execute();

    std::vector<UncertainInstance> uncertain_instances = *context->get_uncertain_instances();
    ASSERT_EQ(uncertain_instances.size(), 1);
    ASSERT_EQ(uncertain_instances[0].get_instance()->InstanceID, "test_object");
    EXPECT_TRUE(result);
    context.reset();
}

