
#include <rclcpp/node.hpp>
#include <string>
#include <voxeland_map/semantics.hpp>

namespace voxeland_disambiguation {
    class VoxelandDisambiguation : public rclcpp::Node
    {
        public:
            explicit VoxelandDisambiguation(const rclcpp::NodeOptions& node_options);

        protected:

            std::string json_file;
            SemanticMap semantic_map = SemanticMap();

            void load_map();
        
    };
};