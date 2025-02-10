#include <rclcpp/node.hpp>

#include <voxeland_desambiguation.hpp>

namespace voxeland_desambiguation {
    VoxelandDesambiguation::VoxelandDesambiguation(const rclcpp::NodeOptions& node_options)
        : Node("voxeland_desambiguation_node", node_options)
    {
        
    }
}