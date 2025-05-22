#include "disambiguation/pipeline/pipeline_steps.hpp"
#include "disambiguation/prompt_utils.hpp"
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

LVLMDisambiguationStep::LVLMDisambiguationStep(const std::string& lvlm_model, rclcpp::Node::SharedPtr node){
    this->lvlm_model = lvlm_model;
    this->node = node;

    init_client();
}

void LVLMDisambiguationStep::execute(){
    VXL_INFO("[LVLM_DISAMBIGUATION] Disambiguating instances using LVLM model: {} ...", lvlm_model);
    
    std::vector<UncertainInstance>* uncertain_instances = context -> get_uncertain_instances();
    disambiguate_instances(*uncertain_instances);
    
    VXL_INFO("[LVLM_DISAMBIGUATION] All instances disambiguated ");
    execute_next();
}

void LVLMDisambiguationStep::disambiguate_instances(std::vector<UncertainInstance>& uncertain_instances){
    // Load the model
    load_model();

    // Loop through all the uncertain instances
    for (UncertainInstance& instance : uncertain_instances){
        std::vector<std::string> categories;
        std::vector<std::string> base64_images;
        std::string prompt;
        // Convert images to base64
        for (auto& [category, images] : *instance.get_selected_images()){
            categories.push_back(category);
            for (cv_bridge::CvImagePtr image : images){
                std::string base64_image = image_to_base64(image);
                base64_images.push_back(base64_image);
            }
        }
        prompt = load_and_format_prompt("prompt.txt", categories,3);
        VXL_INFO("Prompt: {}", prompt);
        VXL_INFO("Selected images: {}", base64_images.size());

        auto prompt_request = std::make_shared<ros_lm_interfaces::srv::OpenLLMRequest::Request>();
        prompt_request->action = 2;
        prompt_request->prompt = prompt;
        prompt_request->model_id = lvlm_model;
        prompt_request->images = base64_images;

        // Call the service
        auto future = client -> async_send_request(prompt_request);
        
        // Create a thread to handle the response
        std::thread([this, &instance, future = std::move(future)]() mutable {
            VXL_INFO("Waiting for response for instance: {}", instance.get_instance()->InstanceID);
            // Espera hasta que la respuesta esté disponible
            if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), future) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = future.get();
                instance.set_final_category(response->generated_text);
                VXL_INFO("Instance {} disambiguated to category: {}", instance.get_instance()->InstanceID, response->generated_text);
            } else {
                VXL_ERROR("Failed to call service");
            }
        }).detach();
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

