#include <algorithm>
#include <iostream>
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

void AppeareancesSelectionStep::execute() {
  if (!appearances_classifier) {
    VXL_ERROR("[APPEARANCES_SELECTION] El classifier es nullptr ¡no se puede continuar!");
    return;
  }

  VXL_INFO(
    "[APPEARANCES_SELECTION] Ejecutando step con classifier “{}” …",
    appearances_classifier->get_name());

  auto vec_ptr = context->get_uncertain_instances();
  if (!vec_ptr) {
    VXL_ERROR("[APPEARANCES_SELECTION] El contexto no tiene lista de instancias");
    return;
  }
  auto &uncertain_instances = *vec_ptr;
  if (uncertain_instances.empty()) {
    VXL_WARN("[APPEARANCES_SELECTION] No hay instancias inciertas para procesar");
    return;
  }

  select_category_appearances(uncertain_instances);
  VXL_INFO("[APPEARANCES_SELECTION] Completado correctamente.");
}

void AppeareancesSelectionStep::select_category_appearances(
    std::vector<UncertainInstance> &uncertain_instances) {
  for (auto &uncertain_instance : uncertain_instances) {

    // Paso seguro: la función recibe el mapa por const&
    auto categories = choose_selected_categories(
      uncertain_instance.get_instance()->results);
    
    VXL_INFO("[APPEARANCES_SELECTION] BOMBOCLAT IN");
    appearances_classifier->classify_instance_appearances(
      uncertain_instance, categories, n_images_per_category);
    VXL_INFO("[APPEARANCES_SELECTION] BOMBOCLAT OUT");
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
