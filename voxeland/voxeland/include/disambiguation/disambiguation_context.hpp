#pragma once

#include <memory>
#include <vector>
#include "disambiguation/json_semantics.hpp"

class DisambiguationContext {
private:
    // Private constructor
    DisambiguationContext();

    static void reset_instance();

    JsonSemanticMap semantic_map;
    std::vector<UncertainInstance> uncertain_instances;

    // Instancia compartida
    static std::shared_ptr<DisambiguationContext> singleton_;

public:
    // Don't allow copy or assignment
    DisambiguationContext(DisambiguationContext &other) = delete;
    void operator=(const DisambiguationContext &) = delete;

    // Static method to get the singleton instance
    static std::shared_ptr<DisambiguationContext> get_instance();

    // Attribute getters
    JsonSemanticMap* get_semantic_map();
    std::vector<UncertainInstance>* get_uncertain_instances();
};
