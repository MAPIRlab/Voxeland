
#include "appearances_classifier.hpp"
#include <cstdint>
#include <cstdlib>
#include "voxeland_map/semantics.hpp"


/**
 * @brief Sort the results map in descending order.
 * 
 * @param results Map of category names and probabilities.
 * @return Sorted vector of category names and probabilities.
 */
std::vector<std::pair<std::string, double>> sort_results_map(std::map<std::string, double>& results){
    std::vector<std::pair<std::string, double>> sorted_vector(results.begin(), results.end());
    std::sort(sorted_vector.begin(), sorted_vector.end(), [](const auto& a, const auto& b) {
        return a.second > b.second;
    });

    return sorted_vector;
}

/**
 * @brief Choose the n_categories with the highest probabilities.
 * 
 * @param sorted_results map of category names and probabilities sorted in descending order.
 */
std::vector<std::string> AppearancesClassifier::choose_selected_categories(std::map<std::string, double>& results){
    std::vector<std::pair<std::string, double>> sorted_results = sort_results_map(results);
    std::vector<std::string> selected_categories;

    // Iterate n_categories times to choose the categories with the highest probabilities
    int count = 0;
    for (auto it = sorted_results.begin(); it != sorted_results.end() && count < n_categories; ++it, ++count) {
        selected_categories.push_back(it->first);
    }

    return selected_categories;
}

/**
 * @brief Select the appearances to be used for re-classification by choosing max_appearances random appearances.
 * 
 * @param instance Uncertain instance to classify. 
 */
void RandomAppearancesClassifier::classify_instance_appearances(UncertainInstance& instance){
    // Choose n_categories with the highest probabilities
    std::vector<std::string> selected_categories = choose_selected_categories(instance.get_instance()->results);
    std::map<std::string, std::vector<uint32_t>> selected_appearances_map;

    for (std::string category : selected_categories){
        std::map<uint32_t,BoundingBox2D> appearances = instance.get_instance()->appearances_timestamps[category];

        // Take the keys vector
        std::vector<uint32_t> instance_ids_vector;
        for (auto& [instance_id, bbox] : appearances){
            instance_ids_vector.push_back(instance_id);
        }

        std::vector<uint32_t> selected_appearances;

        // Choose max_appearances random appearances and add them to the final vector
        for (int i = 0; i < max_appearances; i++){
            int random_index = rand() % appearances.size();
            selected_appearances.push_back(instance_ids_vector.at(random_index));
        }

        selected_appearances_map[category] = selected_appearances;
    }

    std::cout << "Selected appearances: " << selected_appearances_map.size() << std::endl;

    instance.set_selected_appearances(selected_appearances_map);
}


/**
 * @brief Select the appearances to be used for re-classification by dividing the appearances vector in splits.
 * @brief We take the first and last index of each split. Naturally, the last index is the first index of the next split.
 * 
 * @param instance Uncertain instance to classify. 
 */
 void SplitAppearancesClassifier::classify_instance_appearances(UncertainInstance& instance) {
    // Choose n_categories with the highest probabilities
    std::vector<std::string> selected_categories = choose_selected_categories(instance.get_instance()->results);
    std::map<std::string, std::vector<uint32_t>> selected_appearances_map;

    for (const std::string& category : selected_categories) {
        std::map<uint32_t,BoundingBox2D> appearances = instance.get_instance()->appearances_timestamps[category];

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


