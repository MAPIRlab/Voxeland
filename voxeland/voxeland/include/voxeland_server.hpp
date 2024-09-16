#ifndef voxeland_server__voxeland_server_HPP_
#define voxeland_server__voxeland_server_HPP_

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <semantics_ros_wrapper.hpp>
#include <voxeland_map/cell_types.hpp>
#include <voxeland_map/data_modes.hpp>
#include <voxeland_map/logging.hpp>
#include <voxeland_map/semantics.hpp>

#include "bonxai/bonxai.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_srvs/srv/empty.hpp"
#include "voxeland_map/pcl_utils.hpp"
#include "voxeland_map/probabilistic_map_templated.hpp"

/* Added by JL Matez */
#include <algorithm>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "message_filters/subscriber.h"
#include "segmentation_msgs/msg/instance_semantic_map.hpp"
#include "segmentation_msgs/msg/semantic_point_cloud.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "voxeland/srv/get_class_distributions.hpp"

namespace voxeland_server
{

    using sensor_msgs::msg::PointCloud2;

    using DataMode = voxeland::DataMode;

    class VoxelandServer : public rclcpp::Node
    {
    public:
        using ResetSrv = std_srvs::srv::Empty;
        using GetClassDistributions = voxeland::srv::GetClassDistributions;

        DataMode currentMode = DataMode::Uninitialized;

        double data_association_time = 0.0f;
        int data_association_k = 0;
        double map_integration_time = 0.0f;
        int map_integration_k = 0;
        double map_refinement_time = 0.0f;
        int map_refinement_k = 0;

        explicit VoxelandServer(const rclcpp::NodeOptions& node_options);

        bool resetSrv(const std::shared_ptr<ResetSrv::Request> req, const std::shared_ptr<ResetSrv::Response> resp);

        void saveMapSrv(const std::shared_ptr<std_srvs::srv::Empty::Request> req, const std::shared_ptr<std_srvs::srv::Empty::Response> resp);

        void getClassDistributionsSrv(GetClassDistributions::Request::SharedPtr request, GetClassDistributions::Response::SharedPtr response);

        /* Modified by JL Matez: changing PointCloud2 msg to SemanticPointCloud msg */
        virtual void insertCloudCallback(const segmentation_msgs::msg::SemanticPointCloud::ConstSharedPtr cloud);

        bool modeHas(DataMode mode) { return static_cast<int>(currentMode & mode) != 0; }

        SemanticsROSWrapper semantics_ros_wrapper;

    protected:
        void initializeBonxaiObject();

        template <typename DataT>
        void publishAll(const rclcpp::Time& rostime);

        template <typename DataT>
        void publishAllWithInstances(const rclcpp::Time& rostime);

        template <typename PointCloudTypeT, typename DataT>
        pcl::PointXYZ transformPointCloudToGlobal(PointCloudTypeT& pc, geometry_msgs::msg::PoseWithCovariance pose);

        template <typename DataT>
        std::string mapToPLY();

        template <typename DataT>
        void insertPointCloudBasic(const segmentation_msgs::msg::SemanticPointCloud::ConstSharedPtr cloud);

        template <typename DataT>
        void insertPointCloudSemantics(const segmentation_msgs::msg::SemanticPointCloud::ConstSharedPtr cloud);

        template <typename DataT>
        void insertPointCloudSemanticInstances(const segmentation_msgs::msg::SemanticPointCloud::ConstSharedPtr cloud);

        template <typename DataT>
        void fillClassSrvResponse(GetClassDistributions::Request::SharedPtr request, GetClassDistributions::Response::SharedPtr response);

        OnSetParametersCallbackHandle::SharedPtr set_param_res_;

        rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);

        /* Modified by JL Matez: changing PointCloud2 msg to SemanticPointCloud msg */
        rclcpp::Publisher<PointCloud2>::SharedPtr point_cloud_pub_;
        rclcpp::Publisher<segmentation_msgs::msg::InstanceSemanticMap>::SharedPtr semantic_map_pub_;
        rclcpp::Subscription<segmentation_msgs::msg::SemanticPointCloud>::SharedPtr point_cloud_sub_;
        rclcpp::Service<ResetSrv>::SharedPtr reset_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_map_srv_;
        rclcpp::Service<GetClassDistributions>::SharedPtr get_distributions_srv_;

        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

        SemanticMap& semantics = SemanticMap::get_instance();
        std::unique_ptr<Bonxai::ProbabilisticMap> bonxai_;
        std::vector<Bonxai::CoordT> key_ray_;

        double max_range_;
        std::string world_frame_id_;  // the map frame
        std::string base_frame_id_;   // base of the robot for ground plane filtering

        bool latched_topics_;

        double res_;

        double occupancy_min_z_;
        double occupancy_max_z_;

        bool publish_2d_map_;
        bool map_origin_changed;
        // octomap::OcTreeKey padded_min_key_;
        unsigned multires_2d_scale_;
        bool project_complete_map_;

        // Added by JL Matez: SemanticBonxai Parameters
        bool semantics_as_instances_;
        u_int32_t number_iterations = 0;
    };

}  // namespace voxeland_server

#endif  // voxeland_server__voxeland_server_HPP_