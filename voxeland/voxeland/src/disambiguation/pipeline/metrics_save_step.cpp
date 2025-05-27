
#include "disambiguation/pipeline/pipeline_steps.hpp"
using json = nlohmann::json;

MetricsSaveStep::MetricsSaveStep(const std::string& classifier_name, const std::string& lvlm_model){
    this->classifier_name = classifier_name;
    this->lvlm_model = lvlm_model;

    auto slash_pos = lvlm_model.find('/');
    if (slash_pos != std::string::npos) {
        this->lvlm_model = lvlm_model.substr(0, slash_pos);
    }
    this->output_file = "metrics_" + this->classifier_name + "_" + this->lvlm_model + ".json";
}   

void MetricsSaveStep::execute() {
    VXL_INFO("[METRICS] Executing Metrics Save Step...");

    std::vector<UncertainInstance> uncertain_instances = *context -> get_uncertain_instances();
    if (uncertain_instances.empty()) {
        VXL_INFO("No uncertain instances to save metrics for.");
        return;
    }

    json metrics_json = serialize_metrics(uncertain_instances);
    save_metrics(metrics_json);

    VXL_INFO("[METRICS] Metrics saved successfully.");
}

json MetricsSaveStep::serialize_metrics(std::vector<UncertainInstance>& uncertain_instances){
    json metrics_json = {};
    metrics_json["classifier"] = classifier_name;
    metrics_json["lvlm_model"] = lvlm_model;
    metrics_json["uncertain_instances"] = json::array();

    for (auto& instance : uncertain_instances) {
        json instance_json = {};
        std::map<std::string, uint32_t> disambiguation_results = *instance.get_disambiguation_results();

        instance_json["instance_id"] = instance.get_instance()->InstanceID;
        instance_json["entropy"] = instance.get_entropy();
        instance_json["disambiguation_results"] = json::object();
        for (const auto& result : disambiguation_results) {
            instance_json["disambiguation_results"][result.first] = result.second;
        }
        
        metrics_json["uncertain_instances"].push_back(instance_json);
    }

    return metrics_json;
}

void MetricsSaveStep::save_metrics(const json& metrics_json) {
    std::ofstream out_file(output_file);
    if (!out_file.is_open()) {
        VXL_ERROR("Cannot open output file: {}", output_file);
        return;
    }
    out_file << metrics_json.dump(4); // Pretty print with 4 spaces
    out_file.close();
}

