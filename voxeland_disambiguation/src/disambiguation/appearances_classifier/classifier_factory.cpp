#include "disambiguation/appearances_classifier/classifier_factory.hpp"
#include <memory>

std::unique_ptr<AppearancesClassifier> ClassifierFactory::create_classifier(const std::string& classifier_type) {
    if (classifier_type == "random") {
        return std::make_unique<RandomAppearancesClassifier>();
    } else if (classifier_type == "split") {
        return std::make_unique<SplitAppearancesClassifier>();
    } else if (classifier_type == "bbox") {
        return std::make_unique<BBoxAppearancesClassifier>();
    } else {
        throw std::invalid_argument("Unknown classifier type: " + classifier_type);
    }
}