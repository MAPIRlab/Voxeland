
#include <rclcpp/node.hpp>
#include <string>
#include <vector>
#include <voxeland_map/semantics.hpp>
#include "json_semantics.hpp"

namespace voxeland_disambiguation {
    class VoxelandDisambiguation : public rclcpp::Node
    {
        public:
            explicit VoxelandDisambiguation(const rclcpp::NodeOptions& node_options);
            void find_uncertain_instances();
            void select_appearances();
        protected:
            std::string json_file;
            std::string json_appearances_file;
            std::string bag_path;
            
            JsonSemanticMap semantic_map;
            std::vector<UncertainInstance> uncertain_instances;
        
    };
};