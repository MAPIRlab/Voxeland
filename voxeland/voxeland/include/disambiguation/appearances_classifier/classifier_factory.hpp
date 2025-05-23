#pragma once

#include "disambiguation/appearances_classifier/appearances_classifier.hpp"

class ClassifierFactory {
    public:
        static std::unique_ptr<AppearancesClassifier> create_classifier(const std::string& classifier_type);
};