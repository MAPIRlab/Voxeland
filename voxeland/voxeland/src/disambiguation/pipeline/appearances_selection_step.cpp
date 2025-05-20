#include <string>
#include <vector>
#include "disambiguation/json_semantics.hpp"
#include "disambiguation/pipeline/pipeline_steps.hpp"
#include "voxeland_map/Utils/logging.hpp"


void AppeareancesSelectionStep::execute(){
    VXL_INFO("[APPEARANCES_SELECTION] Executing appearances selection step using %s classifier ...", appearances_classifier->name);

    std::vector<UncertainInstance> uncertain_instances = *context->get_uncertain_instances();
    select_category_appearances(uncertain_instances);

    VXL_INFO("[APPEARANCES_SELECTION] Appearances selection step completed.");
    execute_next();
}

void AppeareancesSelectionStep::select_category_appearances(std::vector<UncertainInstance>& uncertain_instances){
    
    for(auto& uncertain_instance : uncertain_instances){
        // Get the categories with the highest probabilities
        auto categories = choose_selected_categories(uncertain_instance.get_instance()->results);
        
        // Classify the instance appearances
        appearances_classifier->classify_instance_appearances(uncertain_instance, categories, n_images_per_category);
    }
}


/**
 * @brief Choose the n_categories with the highest probabilities.
 * 
 * @param sorted_results map of category names and probabilities sorted in descending order.
 */
std::vector<std::string> AppeareancesSelectionStep::choose_selected_categories(std::map<std::string, double>& results){
    // Sort the results map in descending order
    std::vector<std::pair<std::string, double>> sorted_results = sort_results_map(results);
    std::vector<std::string> selected_categories;

    // Iterate n_categories times to choose the categories with the highest probabilities
    int count = 0;
    for (auto it = sorted_results.begin(); it != sorted_results.end() && count < n_categories_per_instance; ++it, ++count) {
        selected_categories.push_back(it->first);
    }

    return selected_categories;
}

/**
 * @brief Sort the results map in descending order.
 * 
 * @param results Map of category names and probabilities.
 * @return Sorted vector of category names and probabilities.
 */
std::vector<std::pair<std::string, double>> AppeareancesSelectionStep::sort_results_map(std::map<std::string, double>& results){
    std::vector<std::pair<std::string, double>> sorted_vector(results.begin(), results.end());
    std::sort(sorted_vector.begin(), sorted_vector.end(), [](const auto& a, const auto& b) {
        return a.second > b.second;
    });

    return sorted_vector;
}