#include <iostream>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <string>
#include <voxeland_disambiguation.hpp>
#include "json_semantics.hpp"
#include "voxeland_map/dirichlet.hpp"
#include "voxeland_map/Utils/logging.hpp"
#include "appearances_classifier.hpp"

#include "rosbag2_transport/reader_writer_factory.hpp"

namespace voxeland_disambiguation {
    VoxelandDisambiguation::VoxelandDisambiguation(const rclcpp::NodeOptions& node_options)
        : Node("voxeland_disambiguation_node", node_options)
    {
        json_file = declare_parameter("json_map", "scenenn065.json");
        json_appearances_file = declare_parameter("json_appearances", "scenenn065_appearances.json");
        bag_path = declare_parameter("bag_path", "/ros_ws/bag/SceneNN/to_ros/ROS2_bags/065/065.db3");

        VXL_INFO("json_map parameter defined, value: {}", json_file);
        VXL_INFO("json_appearances parameter defined, value: {}", json_appearances_file);

        semantic_map = JsonSemanticMap::load_map(json_file, json_appearances_file);
        VXL_INFO("Semantic map loaded");
        // std::cout << semantic_map.to_string() << std::endl;

        VXL_INFO("Checking uncertain instances...");
        find_uncertain_instances();
        
        VXL_INFO("Uncertain instances found : {}", uncertain_instances.size());
        for (UncertainInstance instance : uncertain_instances){
            VXL_INFO("Uncertain instance : {} - Categories: {}", instance.get_instance()->InstanceID, instance.get_instance()->appearances_timestamps.size());
        }

        VXL_INFO("Selecting appearances...");
        select_appearances();
        for (UncertainInstance instance : uncertain_instances){
            std::cout << instance.to_string() << std::endl;
        }
    }

    void VoxelandDisambiguation::find_uncertain_instances(){
        for (JsonSemanticObject& instance : *semantic_map.get_instances()){
            
            // Retrieve all alphas from the results
            std::vector<double> alphas;
            for (auto results_pair : instance.results){
                alphas.push_back(results_pair.second);
            }
            
            // Compute the entropy
            double entropy = expected_shannon_entropy(alphas);
            std::cout << "Expected entropy for instance " << instance.InstanceID << " : " << entropy << std::endl;

            if(entropy > 0.7){
                uncertain_instances.push_back(UncertainInstance(&instance, entropy));
            }
        }
    }
    
    /**
     * @brief Select the categories with the highest probabilities and choose the appearances to be used for re-classification.
     * @brief The way to choose the appearances is defined by the AppearanceClassifier strategy.
    */
    void VoxelandDisambiguation::select_appearances(){
        // Select an strategy
        AppearancesClassifier* classifier = new SplitAppearancesClassifier(3);
        for (UncertainInstance& instance : uncertain_instances){

            classifier->classify_instance_appearances(instance);
        }
    }

}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(voxeland_disambiguation::VoxelandDisambiguation)