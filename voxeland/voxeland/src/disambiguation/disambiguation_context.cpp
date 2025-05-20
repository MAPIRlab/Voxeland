#include "disambiguation/disambiguation_context.hpp"

std::shared_ptr<DisambiguationContext> DisambiguationContext::singleton_ = nullptr;

void DisambiguationContext::reset_instance() {
    singleton_.reset();  // Free the existing instance
}

std::shared_ptr<DisambiguationContext> DisambiguationContext::get_instance() {
    if (!singleton_) {
        singleton_ = std::make_shared<DisambiguationContext>();
    }
    return singleton_;
}

DisambiguationContext::DisambiguationContext() {
    semantic_map = JsonSemanticMap();
    uncertain_instances = std::vector<UncertainInstance>();
}

JsonSemanticMap* DisambiguationContext::get_semantic_map() {
    return &semantic_map;
}

std::vector<UncertainInstance>* DisambiguationContext::get_uncertain_instances() {
    return &uncertain_instances;
}
