#include "disambiguation/appearances_classifier/appearances_classifier.hpp"

/**
 * @brief Select the appearances to be used for re-classification by dividing the appearances vector in splits.
 * @brief We take the first and last index of each split. Naturally, the last index is the first index of the next split.
 * 
 * @param instance Uncertain instance to classify. 
 */
 void SplitAppearancesClassifier::classify_instance_appearances(UncertainInstance& instance, std::vector<std::string> categories, uint32_t max_appearances) {
    // Choose n_categories with the highest probabilities
    std::map<std::string, std::vector<uint32_t>> selected_appearances_map;

    for (const std::string& category : categories) {
        std::map<uint32_t,BoundingBox2D> appearances = instance.get_instance()->appearances_timestamps[category];
        if( appearances.empty()) {
            VXL_ERROR("No appearances found for category: {}", category);
            throw std::exception();
        }

        // Take the keys vector
        std::vector<uint32_t> instance_ids_vector;
        for (auto& [instance_id, bbox] : appearances){
            instance_ids_vector.push_back(instance_id);
        }
        std::vector<uint32_t> selected_appearances;

        size_t n = appearances.size();
        if (n <= max_appearances) {
            // If the number of appearances is less than max_appearances, use all of them
            selected_appearances = instance_ids_vector;
        } else {
            // Distribute the appearances in max_appearances splits
            for (int i = 0; i < max_appearances; i++) {
                size_t index = std::round(i * (n - 1.0) / (max_appearances - 1));
                selected_appearances.push_back(instance_ids_vector.at(index));
            }
        }

        selected_appearances_map[category] = selected_appearances;
    }

    instance.set_selected_appearances(selected_appearances_map);
}

std::string SplitAppearancesClassifier::get_name() const {
    return name;
}