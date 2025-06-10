
#include <stdexcept>
#include <vector>
#include "disambiguation/json_semantics.hpp"
#include "disambiguation/pipeline/pipeline_steps.hpp"


double sigmoid_factor(double N, double N0, double k) {
    double f = 1.0 / (1.0 + std::exp(-k * (N - N0)));
    return std::min(1.0, std::max(0.0, f));
}

UncertainResultsUpdateStep::UncertainResultsUpdateStep(uint32_t disambiguation_iters){
    this->disambiguation_iters = disambiguation_iters;
}

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
        
        double n_categories_factor = sigmoid_factor(instance_results.size(), 5, 0.6);
        double observations_factor = sigmoid_factor(uncertain_instance.get_instance()->n_observations, 75, 0.08);

        VXL_INFO("FINAL STEP IS RECEIVING results for instance {}:", uncertain_instance.get_instance()->InstanceID);
        for (const auto& [category, result] : *lvlm_results){
            VXL_INFO("  Category: {}, Count: {}", category, result);
            
            double confidence_factor = static_cast<double>(result) / static_cast<double>(disambiguation_iters);

            double total_factor = n_categories_factor + observations_factor + confidence_factor;
            instance_results[category] += total_factor * result;
        }
    }

}

