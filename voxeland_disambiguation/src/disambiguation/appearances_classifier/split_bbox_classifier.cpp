#include "disambiguation/appearances_classifier/appearances_classifier.hpp"
#include <vector>
#include <algorithm>

void SplitBboxAreaAppearancesClassifier::classify_instance_appearances(UncertainInstance& instance, std::vector<std::string> categories, uint32_t max_appearances) {
    std::map<std::string, std::vector<uint32_t>> selected_appearances_map;

    for (const std::string& category : categories) {
        std::map<uint32_t,BoundingBox2D> appearances = instance.get_instance()->appearances_timestamps[category];
        if(appearances.empty()) {
            VXL_ERROR("No appearances found for category: {}", category);
            throw std::exception();
        }
        
        // Convert map to vector of pairs <id, bbox>
        std::vector<std::pair<uint32_t, BoundingBox2D>> appearances_vec;
        for (const auto& [id, bbox] : appearances) {
            appearances_vec.emplace_back(id, bbox);
        }
        
        // Sort by timestamp
        std::sort(appearances_vec.begin(), appearances_vec.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        std::vector<uint32_t> selected_appearances;
        size_t n = appearances_vec.size();
        
        if (n <= max_appearances) {
            // If we have less appearances than max_appearances, select them all
            for (const auto& [id, bbox] : appearances_vec) {
                selected_appearances.push_back(id);
            }
        } else {
            // Divide into max_appearances subsets and select the largest bbox from each
            for (uint32_t i = 0; i < max_appearances; i++) {
                // Calculate subset boundaries
                size_t start_idx = i * n / max_appearances;
                size_t end_idx = (i + 1) * n / max_appearances;
                if (i == max_appearances - 1) end_idx = n; // Ensure we include all elements
                
                // Find the appearance with the largest bounding box area in this subset
                uint32_t best_id = appearances_vec[start_idx].first;
                float max_area = appearances_vec[start_idx].second.sizeX * appearances_vec[start_idx].second.sizeY;
                
                for (size_t j = start_idx + 1; j < end_idx; j++) {
                    float area = appearances_vec[j].second.sizeX * appearances_vec[j].second.sizeY;
                    if (area > max_area) {
                        max_area = area;
                        best_id = appearances_vec[j].first;
                    }
                }
                
                selected_appearances.push_back(best_id);
            }
        }

        selected_appearances_map[category] = selected_appearances;
    }

    instance.set_selected_appearances(selected_appearances_map);
}

std::string SplitBboxAreaAppearancesClassifier::get_name() const {
    return name;
}