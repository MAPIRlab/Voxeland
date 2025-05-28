#include "disambiguation/appearances_classifier/appearances_classifier.hpp"
#include <vector>
#include <algorithm>

void BBoxAppearancesClassifier::classify_instance_appearances(
    UncertainInstance &instance, std::vector<std::string> categories,
    uint32_t max_appearances) {

    std::map<std::string, std::vector<uint32_t>> selected_appearances_map;

    for (const std::string& category : categories) {
        // Obtén el mapa de apariencias para la categoría
        const auto& appearances = instance.get_instance()->appearances_timestamps[category];
        if (appearances.empty()) {
            VXL_ERROR("No appearances found for category: {}", category);
            throw std::exception();
        }

        // Vector de pares <id, área>
        std::vector<std::pair<uint32_t, float>> id_area_vec;
        for (const auto& [id, bbox] : appearances) {
            float area = bbox.sizeX * bbox.sizeY;
            id_area_vec.emplace_back(id, area);
        }

        // Ordena por área descendente
        std::sort(id_area_vec.begin(), id_area_vec.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });

        // Selecciona los <max_appearances> ids con mayor área
        std::vector<uint32_t> selected_appearances;
        for (size_t i = 0; i < std::min<size_t>(max_appearances, id_area_vec.size()); ++i) {
            selected_appearances.push_back(id_area_vec[i].first);
        }

        selected_appearances_map[category] = selected_appearances;
    }

    instance.set_selected_appearances(selected_appearances_map);
}

std::string BBoxAppearancesClassifier::get_name() const {
    return name;
}