#include "disambiguation/json_semantics.hpp"
#include "disambiguation/pipeline/pipeline_steps.hpp"
#include "disambiguation/prompt_utils.hpp"
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

LVLMDisambiguationStep::LVLMDisambiguationStep(const std::string& lvlm_model){
    this->lvlm_model = lvlm_model;

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    this->node = std::make_shared<rclcpp::Node>("lvlm_disambiguation_node", options);

    init_client();
}

void LVLMDisambiguationStep::execute(){
    VXL_INFO("[LVLM_DISAMBIGUATION] Disambiguating instances using LVLM model: {} ...", lvlm_model);
    
    std::vector<UncertainInstance>* uncertain_instances = context -> get_uncertain_instances();
    if (uncertain_instances->empty()){
        VXL_INFO("[LVLM_DISAMBIGUATION] No uncertain instances to disambiguate.");
        return;
    }
    disambiguate_instances(*uncertain_instances);
    
    VXL_INFO("[LVLM_DISAMBIGUATION] All instances disambiguated ");
}

void LVLMDisambiguationStep::disambiguate_instances(std::vector<UncertainInstance>& uncertain_instances){
    load_model();

    for (UncertainInstance& instance : uncertain_instances){
        VXL_INFO("Disambiguating instance: {}", instance.get_instance()->InstanceID);
        std::vector<std::string> categories;
        std::vector<std::string> base64_images;
        std::string prompt;

        // Convert images to base64 and extract categories
        for (auto& [category, images] : *instance.get_selected_images()){
            categories.push_back(category);
            for (cv_bridge::CvImagePtr image : images){
                std::string base64_image = image_to_base64(image);
                base64_images.push_back(base64_image);
            }
        }

        prompt = load_and_format_prompt("prompt.txt", categories,3);
        auto prompt_request = std::make_shared<ros_lm_interfaces::srv::OpenLLMRequest::Request>();
        prompt_request->action = 2;
        prompt_request->prompt = prompt;
        prompt_request->model_id = lvlm_model;
        prompt_request->images = base64_images;

        send_and_handle_request(prompt_request, categories, instance);

        // DEBUG
        const auto& results = instance.get_disambiguation_results();
        VXL_INFO("Disambiguation results for instance {}:", instance.get_instance()->InstanceID);
        for (const auto& [category, count] : *results) {
            VXL_INFO("  Category: {}, Count: {}", category, count);
        }
    }
}

void LVLMDisambiguationStep::init_client(){
    this -> client = node->create_client<ros_lm_interfaces::srv::OpenLLMRequest>("llm_generate_text");
    VXL_INFO("Awaiting llm_generate_text service...");
    while (!client->wait_for_service(std::chrono::seconds(1))){
        if (!rclcpp::ok()){
            VXL_ERROR("Interrupted while waiting for the service. Exiting...");
            return;
        }
        VXL_WARN("Service not available, waiting...");
    }
    VXL_INFO("llm_generate_text service available!");
}

bool LVLMDisambiguationStep::load_model(){
    auto load_request = std::make_shared<ros_lm_interfaces::srv::OpenLLMRequest::Request>();
    load_request -> action = 1;
    load_request-> model_id = lvlm_model;
    
    VXL_INFO("Loading model {} into ros_lm server", load_request->model_id);
    auto future = client->async_send_request(load_request);
    if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS){
        auto response = future.get();
        VXL_INFO("Response: {}", response->status_message);
    } else {
        VXL_ERROR("Failed to call service");
        return false;
    }

    return true;
}

void LVLMDisambiguationStep::send_and_handle_request(std::shared_ptr<ros_lm_interfaces::srv::OpenLLMRequest::Request> request, std::vector<std::string>& categories, UncertainInstance& instance){
    int valid_responses_count = 0;

    while (valid_responses_count < 100){
        auto future = client -> async_send_request(request);

        if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS){
            VXL_ERROR("Failed to call service");
            continue;
        }

        auto response = future.get();
        std::string selected_category = get_category_from_response(response->generated_text, categories);
        if (selected_category.empty()) {
            VXL_WARN("No valid category found in response for instance: {}", instance.get_instance()->InstanceID);
            continue;
        }

        VXL_INFO("Instance {} disambiguated to category: {}", instance.get_instance()->InstanceID, selected_category);
        instance.increase_one_disambiguation_result(selected_category);
        valid_responses_count++;
    }

}


std::string LVLMDisambiguationStep::get_category_from_response(std::string response, const std::vector<std::string>& categories) {
    // Find one the categories in the response
    for (const std::string& category : categories) {
        if (response.find(category) != std::string::npos) {
            return category;
        }
    }
    // If no category is found, return an empty string
    return "";
}

