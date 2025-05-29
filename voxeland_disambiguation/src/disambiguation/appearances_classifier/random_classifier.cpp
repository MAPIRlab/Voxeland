
#include <exception>
#include <stdexcept>
#include "disambiguation/appearances_classifier/appearances_classifier.hpp"
#include "voxeland_map/Utils/logging.hpp"

/**
 * @brief Select the appearances to be used for re-classification by choosing
 * max_appearances random appearances.
 *
 * @param instance Uncertain instance to classify.
 */
void RandomAppearancesClassifier::classify_instance_appearances(
    UncertainInstance &instance, std::vector<std::string> categories,
    uint32_t max_appearances) {

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
            for (int i = 0; i < max_appearances; i++) {
              int random_index = rand() % appearances.size();
              selected_appearances.push_back(instance_ids_vector.at(random_index));
            }
        }

        selected_appearances_map[category] = selected_appearances;
    }

    instance.set_selected_appearances(selected_appearances_map);
}

std::string RandomAppearancesClassifier::get_name() const { return name; }

