#pragma once

#include <semantics_ros_wrapper.hpp>
#include <voxeland_map/Utils/logging.hpp>
#include <voxeland_map/cell_types.hpp>
#include <voxeland_map/data_modes.hpp>
#include <voxeland_map/semantic_map.hpp>

#include "FunctionQueue.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <visualization_msgs/msg/marker.hpp>

/* Added by JL Matez */
#include <memory>
#include <string>
#include <vector>
#include <voxeland_msgs/srv/get_class_distributions.hpp>
#include <voxeland_msgs/srv/update_map_results.hpp>

#include "tf2_eigen/tf2_eigen.hpp"  // IWYU pragma: keep
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace voxeland_server
{

    using sensor_msgs::msg::PointCloud2;

    using DataMode = voxeland::DataMode;

    class VoxelandServer : public rclcpp::Node
    {
    public:
        using ResetSrv = std_srvs::srv::Empty;
        using GetClassDistributions = voxeland_msgs::srv::GetClassDistributions;
        using UpdateMapResultsSrv = voxeland_msgs::srv::UpdateMapResults;

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

        void loadMapSrv(const std::shared_ptr<voxeland_msgs::srv::UpdateMapResults::Request> req, const std::shared_ptr<voxeland_msgs::srv::UpdateMapResults::Response> resp);

        bool getClassDistributionsSrv(const std::shared_ptr<rmw_request_id_t> requestHeader, GetClassDistributions::Request::SharedPtr request, GetClassDistributions::Response::SharedPtr response);

        /* Modified by JL Matez: changing PointCloud2 msg to SemanticPointCloud msg */
        virtual void insertCloudCallback(const segmentation_msgs::msg::SemanticPointCloud::ConstSharedPtr cloud);

        bool modeHas(DataMode mode)
        {
            return static_cast<int>(currentMode & mode) != 0;
        }

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

        std::string semanticsMapToPLY();

        void autoSaveMapCallback();
        
        template <typename DataT>
        std::string fullSemanticMapToPLY();

        template <typename DataT>
        void insertPointCloudBasic(const segmentation_msgs::msg::SemanticPointCloud::ConstSharedPtr cloud);

        template <typename DataT>
        void insertPointCloudSemantics(const segmentation_msgs::msg::SemanticPointCloud::ConstSharedPtr cloud);

        template <typename DataT>
        void insertPointCloudSemanticInstances(const segmentation_msgs::msg::SemanticPointCloud::ConstSharedPtr cloud);

        template <typename DataT>
        void fillClassSrvResponse(GetClassDistributions::Request::SharedPtr request, GetClassDistributions::Response::SharedPtr response);

        void doGlobalRefinement();

        OnSetParametersCallbackHandle::SharedPtr set_param_res_;

        rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);

        /* Modified by JL Matez: changing PointCloud2 msg to SemanticPointCloud msg */
        rclcpp::Publisher<PointCloud2>::SharedPtr point_cloud_pub_;
        rclcpp::Publisher<segmentation_msgs::msg::InstanceSemanticMap>::SharedPtr semantic_map_pub_;
        rclcpp::Subscription<segmentation_msgs::msg::SemanticPointCloud>::SharedPtr point_cloud_sub_;
        rclcpp::Service<ResetSrv>::SharedPtr reset_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_map_srv_;
        rclcpp::Service<GetClassDistributions>::SharedPtr get_distributions_srv_;
        rclcpp::Service<UpdateMapResultsSrv>::SharedPtr load_map_srv_;
        
        rclcpp::TimerBase::SharedPtr auto_save_timer_;

        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

        SemanticMap& semantics = SemanticMap::get_instance();
        std::unique_ptr<Bonxai::ProbabilisticMap> bonxai_;
        std::vector<Bonxai::IndicesT> key_ray_;

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
        bool auto_save_enabled_;
        u_int32_t number_iterations = 0;

        bool paused = false;  // stop processing new observations. To be toggled from the GUI
        
        // Scene and detector parameters for output organization
        std::string scene_name_;
        std::string detector_name_;
        std::string output_ply_path_;  // Full path to the PLY file determined at startup
        std::string output_dir_;       // Directory path for outputs

#if ENABLE_DEBUG_GUI
        void SetupGUI();
        void RenderGUI();

        template <typename DataT>
        void SelectObjectsAndDraw();
        void GetQueryPoint();

        template <typename DataT>
        void PrintVoxelInfo(const Bonxai::Point3D& point);

        void PrintInstanceInfo();
        void PauseButton();

        template <typename DataT>
        void ShowObservationPointCloud();

        void PrintCategoriesList();
        
        rclcpp::Publisher<PointCloud2>::SharedPtr debugInstancesPub;
        rclcpp::Publisher<PointCloud2>::SharedPtr debugInputPub;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clickedPointSub;
        
        segmentation_msgs::msg::SemanticPointCloud::ConstSharedPtr mostRecentPointCloud;
        std::vector<uint8_t> globalObjectsToDraw;
        std::vector<uint8_t> localObjectsToDraw;
        Bonxai::Point3D selectedCoordinates;
        FunctionQueue functionQueue;
        rclcpp::TimerBase::SharedPtr functionQueueTimer;

        // these are mutually exclusive
        // the GUI is normally rendered as part of the spin cycle, from the main thread, to avoid data sync issues
        // however, while using a debugger, we need a separate thread to be able to use the GUI while the execution is paused
        // this is controlled in SetupGUI()
        rclcpp::TimerBase::SharedPtr renderTimer;
        std::jthread renderThread;
#endif
    };

}  // namespace voxeland_server
