
#include <stdexcept>
#include <vector>
#include "disambiguation/json_semantics.hpp"
#include "disambiguation/pipeline/pipeline_steps.hpp"

bool UncertainResultsUpdateStep::execute(){
    VXL_INFO("[UNCERTAIN_RESULTS_UPDATER] Updating uncertain instances results...");
    std::vector<UncertainInstance>* uncertain_instances = context->get_uncertain_instances();
    if (uncertain_instances->empty()) {
        VXL_ERROR("[UNCERTAIN_RESULTS_UPDATER] No uncertain instances to update.");
        return false;
    }
    try{
        update_uncertain_instances_results(*uncertain_instances);
    }catch(std::runtime_error& e){
        VXL_ERROR("[UNCERTAIN_RESULTS_UPDATER] Error during uncertain results update step: {}", e.what());
        return false;
    }

    VXL_INFO("[UNCERTAIN_RESULTS_UPDATER] Uncertain instances results updated successfully.");
    return true;
}

void UncertainResultsUpdateStep::update_uncertain_instances_results(std::vector<UncertainInstance>& uncertain_instances) {
    
    for (auto& uncertain_instance : uncertain_instances) {
        auto& instance_results = uncertain_instance.get_instance()->results;
        auto lvlm_results = uncertain_instance.get_disambiguation_results();
        if (!lvlm_results || lvlm_results->empty()) {
            VXL_ERROR("No LVLM results found for instance: {}", uncertain_instance.get_instance()->InstanceID);
           throw std::runtime_error("No LVLM results found for instance: " + uncertain_instance.get_instance()->InstanceID);            
        }

        VXL_INFO("FINAL STEP IS RECEIVING results for instance {}:", uncertain_instance.get_instance()->InstanceID);
        for (const auto& [category, count] : *lvlm_results) {
            VXL_INFO("  Category: {}, Count: {}", category, count);
        }
        
        for (const auto& [category, result] : *lvlm_results){
            instance_results[category] += result;
        }
    }

}