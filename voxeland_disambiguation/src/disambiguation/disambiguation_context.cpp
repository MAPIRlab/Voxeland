#include "disambiguation/disambiguation_context.hpp"
#include "disambiguation/json_semantics.hpp"

std::shared_ptr<DisambiguationContext> DisambiguationContext::singleton_ = nullptr;

void DisambiguationContext::reset_instance() {
    singleton_.reset();  // Free the existing instance
}

std::shared_ptr<DisambiguationContext> DisambiguationContext::get_context_instance() {
    if (!singleton_) {
        singleton_ = std::shared_ptr<DisambiguationContext>(new DisambiguationContext());
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

UncertainInstance* DisambiguationContext::get_uncertain_instance_by_id(std::string instanceId) {
    for (auto& instance : uncertain_instances) {
        if (instance.get_instance()->InstanceID == instanceId) {
            return &instance;
        }
    }
    return nullptr;  // Return nullptr if not found
}

void DisambiguationContext::set_uncertain_instances(std::vector<UncertainInstance> uncertain_instances) {
    this->uncertain_instances = uncertain_instances;
}
