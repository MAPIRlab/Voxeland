#include "disambiguation/json_semantics.hpp"
#include "disambiguation/pipeline/pipeline_steps.hpp"
#include "voxeland_map/Utils/logging.hpp"
#include "voxeland_map/dirichlet.hpp"


bool UncertainInstanceIdentificationStep::execute(){
    VXL_INFO("[UNCERTAIN_INSTANCE_IDENTIFICATION] Executing uncertain instance identification step");

    JsonSemanticMap map = *(context->get_semantic_map());
    if (map.get_instances().empty()) {
        VXL_ERROR("[UNCERTAIN_INSTANCE_IDENTIFICATION] Semantic map is empty or not initialized.");
        return false;
    }

    std::vector<UncertainInstance> uncertain_instances = identify_uncertain_instances(map);
    context->set_uncertain_instances(uncertain_instances);
    
    VXL_INFO("[UNCERTAIN_INSTANCE_IDENTIFICATION] Completed, found {} uncertain instances",uncertain_instances.size()); 
    return true;
}

/**
     * @brief Select the uncertain instances by computing the entropy of the results map
     * @brief If the entropy is greater than a fixed value, the instance is considered uncertain
*/
std::vector<UncertainInstance> UncertainInstanceIdentificationStep::identify_uncertain_instances(JsonSemanticMap& map){
    std::vector<UncertainInstance> uncertain_instances;
    for (auto instance : map.get_instances()){
            
            // Retrieve all alphas from the results
            std::vector<double> alphas;
            for (auto results_pair : instance->results){
                alphas.push_back(results_pair.second);
            }
            
            // Compute the entropy
            double entropy = expected_shannon_entropy(alphas);
            VXL_INFO("[UNCERTAIN_INSTANCE_IDENTIFICATION] Instance {} has entropy {}", instance->InstanceID, entropy);

            // Fixed threshold
            if(entropy > 0.7){
                uncertain_instances.emplace_back(instance, entropy);
            }
        }
    return uncertain_instances;
}
