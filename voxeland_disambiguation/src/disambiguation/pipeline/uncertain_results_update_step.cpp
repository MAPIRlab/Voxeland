
#include <vector>
#include "disambiguation/json_semantics.hpp"
#include "disambiguation/pipeline/pipeline_steps.hpp"

void UncertainResultsUpdateStep::execute(){
    VXL_INFO("[UNCERTAIN_RESULTS_UPDATER] Updating uncertain instances results...");

    std::vector<UncertainInstance>* uncertain_instances = context->get_uncertain_instances();
    update_uncertain_instances_results(*uncertain_instances);

    VXL_INFO("[UNCERTAIN_RESULTS_UPDATER] Uncertain instances results updated successfully.");
}

void UncertainResultsUpdateStep::update_uncertain_instances_results(std::vector<UncertainInstance>& uncertain_instances) {
    
    for (auto& uncertain_instance : uncertain_instances) {
        auto& instance_results = uncertain_instance.get_instance()->results;
        auto lvlm_results = uncertain_instance.get_disambiguation_results();

        VXL_INFO("FINAL STEP IS RECEIVING results for instance {}:", uncertain_instance.get_instance()->InstanceID);
        for (const auto& [category, count] : *lvlm_results) {
            VXL_INFO("  Category: {}, Count: {}", category, count);
        }
        
        for (const auto& [category, result] : *lvlm_results){
            instance_results[category] += result;
        }
    }

}