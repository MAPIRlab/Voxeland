#include <string>
#include <vector>

#include "disambiguation/json_semantics.hpp"
#include "disambiguation/pipeline/pipeline_steps.hpp"
#include "voxeland_map/Utils/logging.hpp"

AppeareancesSelectionStep::AppeareancesSelectionStep(
    std::unique_ptr<AppearancesClassifier> classifier,
    uint32_t n_images_per_category,
    uint32_t n_categories_per_instance)
  : appearances_classifier(std::move(classifier)),
    n_images_per_category(n_images_per_category),
    n_categories_per_instance(n_categories_per_instance)
{}

bool AppeareancesSelectionStep::execute() {
  VXL_INFO(
  "[APPEARANCES_SELECTION] Executing with “{}” classifier, ",
  appearances_classifier->get_name());
  if (!appearances_classifier) {
    VXL_ERROR("[APPEARANCES_SELECTION] No classifier provided");
    return false;
  }
  auto vec_ptr = context->get_uncertain_instances();
  if (!vec_ptr || vec_ptr->empty()) {
    VXL_ERROR("[APPEARANCES_SELECTION] No uncertain instances found.");
    return false;
  }

  auto &uncertain_instances = *vec_ptr;
  select_category_appearances(uncertain_instances);

  VXL_INFO("[APPEARANCES_SELECTION] Completado correctamente.");
  return true;
}

void AppeareancesSelectionStep::select_category_appearances(
    std::vector<UncertainInstance> &uncertain_instances) {
  for (auto &uncertain_instance : uncertain_instances) {

    auto categories = choose_selected_categories(
      uncertain_instance.get_instance()->results);
    
    appearances_classifier->classify_instance_appearances(
      uncertain_instance, categories, n_images_per_category);
  }
}

std::vector<std::string> AppeareancesSelectionStep::choose_selected_categories(
    const std::map<std::string, double> &results) {
  std::vector<std::pair<std::string, double>> sorted_results = sort_results_map(results);

  std::vector<std::string> selected_categories;
  int cateogries_count = 0;
  for (auto it = sorted_results.begin(); it != sorted_results.end() && cateogries_count < n_categories_per_instance; ++it, ++cateogries_count) {
    selected_categories.push_back(it->first);
  }

  return selected_categories;
}

std::vector<std::pair<std::string, double>>AppeareancesSelectionStep::sort_results_map(const std::map<std::string, double> &results) {
  std::vector<std::pair<std::string, double>> sorted_vector(results.begin(),
                                                            results.end());
  std::sort(sorted_vector.begin(), sorted_vector.end(),
            [](const auto &a, const auto &b) { return a.second > b.second; });
  return sorted_vector;
}
