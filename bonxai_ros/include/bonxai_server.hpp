#ifndef BONXAI_SERVER__BONXAI_SERVER_HPP_
#define BONXAI_SERVER__BONXAI_SERVER_HPP_

#include "bonxai_map/pcl_utils.hpp"
#include "bonxai_map/probabilistic_map_templated.hpp"
#include "bonxai/bonxai.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/empty.hpp"

/* Added by JL Matez */
#include "segmentation_msgs/msg/semantic_point_cloud.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <bonxai_map/semantics.hpp>

namespace bonxai_server
{

using sensor_msgs::msg::PointCloud2;

enum class MsgType
{
  Empty = 0,
  RGB = 1 << 0,
  Semantics = 1 << 1,
  RGBSemantics = RGB | Semantics
};
MsgType currentMode = MsgType::Empty;

inline MsgType operator|(MsgType a, MsgType b)
{
  return static_cast<MsgType>(static_cast<int>(a) | static_cast<int>(b));
}

inline MsgType operator&(MsgType a, MsgType b)
{
  return static_cast<MsgType>(static_cast<int>(a) & static_cast<int>(b));
}

class BonxaiServer : public rclcpp::Node
{
public:
  using ResetSrv = std_srvs::srv::Empty;

  explicit BonxaiServer(const rclcpp::NodeOptions& node_options);

  bool resetSrv(const std::shared_ptr<ResetSrv::Request> req,
                const std::shared_ptr<ResetSrv::Response> resp);

  /* Modified by JL Matez: changing PointCloud2 msg to SemanticPointCloud msg */
  virtual void insertCloudCallback(
      const segmentation_msgs::msg::SemanticPointCloud::ConstSharedPtr cloud);

protected:
  SemanticMap& semantics = SemanticMap::get_instance();

  template <typename DataT>
  void publishAll(const rclcpp::Time& rostime)
  {
    const auto start_time = rclcpp::Clock{}.now();
    std::vector<DataT> cell_data;
    std::vector<Bonxai::Point3D> cell_points;
    cell_points.clear();
    bonxai_->With<DataT>()->getOccupiedVoxels(cell_points, cell_data);

    if (cell_points.size() <= 1)
    {
      RCLCPP_WARN(get_logger(), "Nothing to publish, bonxai is empty");
      return;
    }

    bool publish_point_cloud =
        (latched_topics_ ||
         point_cloud_pub_->get_subscription_count() +
                 point_cloud_pub_->get_intra_process_subscription_count() >
             0);

    // init pointcloud for occupied space:
    if (publish_point_cloud)
    {
      pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
      pcl_cloud.clear();

      for (size_t i = 0; i < cell_points.size(); i++)
      {
        const auto& voxel = cell_points[i];

        if (voxel.z >= occupancy_min_z_ && voxel.z <= occupancy_max_z_)
        {
          Bonxai::Color vizualization_color = cell_data[i].toColor();
          pcl_cloud.emplace_back((float)voxel.x,
                                 (float)voxel.y,
                                 (float)voxel.z,
                                 vizualization_color.r,
                                 vizualization_color.g,
                                 vizualization_color.b);
        }
      }
      PointCloud2 cloud;
      pcl::toROSMsg(pcl_cloud, cloud);

      cloud.header.frame_id = world_frame_id_;
      cloud.header.stamp = rostime;
      point_cloud_pub_->publish(cloud);
      RCLCPP_WARN(get_logger(),
                  "Published occupancy grid with %ld voxels",
                  pcl_cloud.points.size());
    }
  }

  template <typename PointCloudTypeT, typename DataT>
  void addObservation(PointCloudTypeT& pc, std_msgs::msg::Header header)
  {
    // Sensor In Global Frames Coordinates
    geometry_msgs::msg::TransformStamped sensor_to_world_transform_stamped;
    try
    {
      sensor_to_world_transform_stamped =
          tf2_buffer_->lookupTransform(world_frame_id_,
                                       header.frame_id,
                                       header.stamp,
                                       rclcpp::Duration::from_seconds(1.0));
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }

    Eigen::Matrix4f sensor_to_world =
        tf2::transformToEigen(sensor_to_world_transform_stamped.transform)
            .matrix()
            .cast<float>();

    // Transforming Points to Global Reference Frame
    pcl::transformPointCloud(pc, pc, sensor_to_world);

    // Getting the Translation from the sensor to the Global Reference Frame
    const auto& t = sensor_to_world_transform_stamped.transform.translation;

    const pcl::PointXYZ sensor_to_world_vec3((float)t.x, (float)t.y, (float)t.z);

    auto bonxai__ = dynamic_cast<Bonxai::ProbabilisticMapT<DataT>*>(bonxai_.get());
    bonxai__->insertPointCloud(pc.points, sensor_to_world_vec3, 30.0);
  }

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  rcl_interfaces::msg::SetParametersResult
  onParameter(const std::vector<rclcpp::Parameter>& parameters);

  /* Modified by JL Matez: changing PointCloud2 msg to SemanticPointCloud msg */
  rclcpp::Publisher<PointCloud2>::SharedPtr point_cloud_pub_;
  message_filters::Subscriber<segmentation_msgs::msg::SemanticPointCloud>
      point_cloud_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<segmentation_msgs::msg::SemanticPointCloud>>
      tf_point_cloud_sub_;
  // rclcpp::Service<BBoxSrv>::SharedPtr clear_bbox_srv_;
  rclcpp::Service<ResetSrv>::SharedPtr reset_srv_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  std::unique_ptr<Bonxai::ProbabilisticMap> bonxai_;
  void initializeBonxaiObject();
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
};
}  // namespace bonxai_server

#endif  // BONXAI_SERVER__BONXAI_SERVER_HPP_