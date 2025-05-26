#include <gtest/gtest.h>
#include "disambiguation/disambiguation_context.hpp"
#include "disambiguation/json_semantics.hpp"
#include "disambiguation/pipeline/pipeline_steps.hpp"


TEST(UncertainInstanceIdentifiacionStep, test_execute_empty_map_returns_empty_vector){

    UncertainInstanceIdentificationStep step;

    step.execute();

    std::shared_ptr<DisambiguationContext> context = DisambiguationContext::get_context_instance();
    std::vector<UncertainInstance> uncertain_instances = *context->get_uncertain_instances();
    ASSERT_TRUE(uncertain_instances.empty());
    context.reset();
};

TEST(UncertainInstanceIdentifiacionStep, test_execute_with_instances_identifies_uncertain_instances){
    JsonSemanticObject test_object;
    test_object.InstanceID = "test_object";
    test_object.results= {
        {"sensor1", 0.8},
        {"sensor2", 0.5} 
    };
    std::shared_ptr<DisambiguationContext> context = DisambiguationContext::get_context_instance();
    auto semantic_map = context->get_semantic_map();
    semantic_map->add_instance(std::make_shared<JsonSemanticObject>(test_object));
    UncertainInstanceIdentificationStep step;

    step.execute();

    std::vector<UncertainInstance> uncertain_instances = *context->get_uncertain_instances();
    ASSERT_EQ(uncertain_instances.size(), 1);
    ASSERT_EQ(uncertain_instances[0].get_instance()->InstanceID, "test_object");
    context.reset();
};