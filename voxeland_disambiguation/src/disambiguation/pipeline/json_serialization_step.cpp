#include <stdexcept>
#include "disambiguation/json_semantics.hpp"
#include "disambiguation/pipeline/pipeline_steps.hpp"
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <voxeland_msgs/srv/detail/update_map_results__struct.hpp>

using json = nlohmann::json;

JsonSerializationStep::JsonSerializationStep(const std::string& output_file, bool update_map_service){
    this->output_file = output_file;
    this->update_map_service = update_map_service;

    if (update_map_service) {
        VXL_WARN("[JSON_SERIALIZATION] update_map_service is set to false, the map will not be sent to the server.");
        rclcpp::NodeOptions options;
        options.allow_undeclared_parameters(true);
        options.automatically_declare_parameters_from_overrides(true);
        std::string node_name = "disambiguation_serialization_node_" + std::to_string(std::rand());
        this->node = std::make_shared<rclcpp::Node>(node_name, options);
        init_client();
    }
}

bool JsonSerializationStep::execute() {
    VXL_INFO("[JSON_SERIALIZATION] Executing JSON serialization step...");
    
    JsonSemanticMap map = *context->get_semantic_map();
    if (map.get_instances().empty()) {
        VXL_ERROR("[JSON_SERIALIZATION] Semantic map is empty or not initialized.");
        return false;
    }

    try {
        json map_json = serialize_map(map);
        save_map(map_json);
        send_map_to_server(map_json);
    } catch (std::runtime_error& e) {
        VXL_ERROR("[JSON_SERIALIZATION] Error during JSON serialization step: {}", e.what());
        return false;
    }

    VXL_INFO("[JSON_SERIALIZATION] JSON serialization completed successfully, output written to: {}", output_file);
    return true;
}

void JsonSerializationStep::save_map(const json& map_json) {
    std::ofstream out_file(output_file);
    if (!out_file.is_open()) {
        throw std::runtime_error("Cannot open output file: " + output_file);
    }
    out_file << map_json.dump(4); // Pretty print with 4 spaces
    out_file.close();
}

void JsonSerializationStep::send_map_to_server(const json& map_json) {
    if (!update_map_service) {
        VXL_INFO("[JSON_SERIALIZATION] update_map_service is set to false, skipping sending map to server.");
        return;
    }
    auto request = std::make_shared<voxeland_msgs::srv::UpdateMapResults::Request>();
    request->json_map = map_json.dump(); // Convert JSON to string

    VXL_INFO("Sending map to /update_map_results service...");
    auto future = client->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
        VXL_ERROR("Failed to call /update_map_results service");
        return;
    }

    auto response = future.get();
    if (response->success) {
        VXL_INFO("Map loaded successfully. Message: {}", response->message);
    } else {
        VXL_ERROR("Failed to load map on the server. Message {}", response->message);
    }
}

void JsonSerializationStep::init_client() {
    this->client = node->create_client<voxeland_msgs::srv::UpdateMapResults>("/voxeland_server/update_map_results");
    VXL_INFO("Awaiting /update_map_results service...");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            VXL_ERROR("Interrupted while waiting for the service. Exiting...");
            return;
        }
        VXL_WARN("/update_map_results service not available, waiting...");
    }
    VXL_INFO("/update_map_results service available!");
}

json JsonSerializationStep::serialize_map(JsonSemanticMap& map) {
    std::vector<std::shared_ptr<JsonSemanticObject>> instances = map.get_instances();
    json map_json = {};

    for (auto& instance : instances){
        map_json["instances"][instance->InstanceID] = serialize_instance(*instance);
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