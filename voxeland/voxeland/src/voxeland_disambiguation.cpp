#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <voxeland_disambiguation.hpp>
#include "voxeland_map/Utils/logging.hpp"

namespace voxeland_disambiguation {
    VoxelandDisambiguation::VoxelandDisambiguation(const rclcpp::NodeOptions& node_options)
        : Node("voxeland_disambiguation_node", node_options)
    {
        json_file = declare_parameter("json_map", "voxeland_instanceMap.json");
        load_map();
    }

    void VoxelandDisambiguation::load_map()
    {
        VXL_INFO("Loading dededd from {} json file", json_file);
    }

    // int main(int argc, char* argv[])
    // {
    //     rclcpp::init(argc, argv);
    //     auto node = std::make_shared<voxeland_disambiguation::VoxelandDisambiguation>();
    //     rclcpp::spin(node);
    //     rclcpp::shutdown();
    //     return 0;
    // }
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(voxeland_disambiguation::VoxelandDisambiguation)