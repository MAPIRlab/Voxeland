#include <gtest/gtest.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include "disambiguation/pipeline/pipeline_steps.hpp"
#include "disambiguation/json_semantics.hpp"

using json = nlohmann::json;


TEST(MetricsSaveStepTest, test_execute_saves_metrics_to_file) {
    std::string classifier = "split";
    std::string lvlm = "minicpm";
    MetricsSaveStep step(classifier, lvlm);
    auto instance = std::make_shared<JsonSemanticObject>();
    instance->InstanceID = "id2";
    UncertainInstance uncertain(instance, 0.99);
    *(uncertain.get_disambiguation_results()) = {{"bird", 7}};
    auto context = DisambiguationContext::get_context_instance();
    context->get_uncertain_instances()->clear();
    context->get_uncertain_instances()->push_back(uncertain);

    // Act
    bool result = step.execute();

    EXPECT_TRUE(result);
    std::string filename = "metrics_" + classifier + "_" + lvlm + ".json";
    std::ifstream in(filename);
    ASSERT_TRUE(in.is_open());
    json loaded;
    in >> loaded;
    in.close();
    EXPECT_EQ(loaded["classifier"], classifier);
    EXPECT_EQ(loaded["lvlm_model"], lvlm);
    ASSERT_EQ(loaded["uncertain_instances"].size(), 1);
    EXPECT_EQ(loaded["uncertain_instances"][0]["instance_id"], "id2");
    EXPECT_EQ(loaded["uncertain_instances"][0]["disambiguation_results"]["bird"], 7);
    std::remove(filename.c_str());
}

TEST(MetricsSaveStepTest, test_execute_with_empty_uncertain_instances_returns_false) {
    MetricsSaveStep step("any", "any");
    auto context = DisambiguationContext::get_context_instance();
    context->get_uncertain_instances()->clear();

    bool result = step.execute();

    EXPECT_FALSE(result);
    std::string filename = "metrics_any_any.json";
    std::ifstream in(filename);
    EXPECT_FALSE(in.is_open());
}

TEST(MetricsSaveStepTest, test_execute_with_no_disambiguation_results_returns_false) {
    MetricsSaveStep step("split", "vicuna");
    auto instance = std::make_shared<JsonSemanticObject>();
    instance->InstanceID = "id3";
    UncertainInstance uncertain(instance, 0.5);
    auto context = DisambiguationContext::get_context_instance();
    context->get_uncertain_instances()->clear();
    context->get_uncertain_instances()->push_back(uncertain);

    bool result = step.execute();

    EXPECT_FALSE(result);
    std::string filename = "metrics_split_vicuna.json";
    std::ifstream in(filename);
    EXPECT_FALSE(in.is_open());
}