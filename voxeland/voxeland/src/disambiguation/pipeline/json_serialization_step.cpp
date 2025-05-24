#include "disambiguation/json_semantics.hpp"
#include "disambiguation/pipeline/pipeline_steps.hpp"
using json = nlohmann::json;

JsonSerializationStep::JsonSerializationStep(const std::string& output_file)
    : output_file(output_file) {}

void JsonSerializationStep::execute() {
    VXL_INFO("Executing JSON serialization step...");

    JsonSemanticMap map = *context->get_semantic_map();
    json map_json = serialize_map(map);
    save_map(map_json);

    VXL_INFO("JSON serialization completed successfully, output written to: {}", output_file);
}

void JsonSerializationStep::save_map(const json& map_json) {
    std::ofstream out_file(output_file);
    if (!out_file.is_open()) {
        VXL_ERROR("Cannot open output file: {}", output_file);
        return;
    }
    out_file << map_json.dump(4); // Pretty print with 4 spaces
    out_file.close();
}

json JsonSerializationStep::serialize_map(JsonSemanticMap& map) {
    std::vector<JsonSemanticObject> instances = *map.get_instances();
    json map_json = {};

    for (auto& instance : instances){
        map_json["instances"][instance.InstanceID] = serialize_instance(instance);
    }

    return map_json;
}

json JsonSerializationStep::serialize_instance(JsonSemanticObject& instance) {
    json instance_json = {};
    
    instance_json["bbox"] = serialize_bbox(instance.bbox);
    instance_json["n_observations"] = instance.n_observations;
    instance_json["results"] = serialize_results(instance.results);

    return instance_json;
}

json JsonSerializationStep::serialize_bbox(BoundingBox3D& bbox) {
    json bbox_json = {};
    
    json center = json::array();
    center.push_back((bbox.minX + bbox.maxX) / 2.0);
    center.push_back((bbox.minY + bbox.maxY) / 2.0);
    center.push_back((bbox.minZ + bbox.maxZ) / 2.0);

    json size = json::array();
    size.push_back(bbox.maxX - bbox.minX);
    size.push_back(bbox.maxY - bbox.minY);
    size.push_back(bbox.maxZ - bbox.minZ);

    bbox_json["center"] = center;
    bbox_json["size"] = size;

    return bbox_json;
}

json JsonSerializationStep::serialize_results(std::map<std::string, double>& results) {
    json results_json = {};
    
    for (const auto& [category, value] : results) {
        results_json[category] = value;
    }

    return results_json;
}