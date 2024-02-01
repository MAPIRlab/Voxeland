#include <bonxai_server.hpp>
#include <bonxai_map/cell_types.hpp>

namespace
{
template <typename T>
bool update_param(const std::vector<rclcpp::Parameter>& p,
                  const std::string& name,
                  T& value)
{
  auto it = std::find_if(
      p.cbegin(), p.cend(), [&name](const rclcpp::Parameter& parameter) {
        return parameter.get_name() == name;
      });
  if (it != p.cend())
  {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}
}  // namespace

namespace bonxai_server
{
BonxaiServer::BonxaiServer(const rclcpp::NodeOptions& node_options)
  : Node("bonxai_server_node", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  {
    world_frame_id_ = declare_parameter("frame_id", "map");
    base_frame_id_ = declare_parameter("base_frame_id", "base_footprint");
  }

  latched_topics_ = declare_parameter("latch", true);
  if (latched_topics_)
  {
    RCLCPP_INFO(get_logger(),
                "Publishing latched (single publish will take longer, "
                "all topics are prepared)");
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "Publishing non-latched (topics are only prepared as needed, "
                "will only be re-published on map change");
  }

  auto qos = latched_topics_ ? rclcpp::QoS{ 1 }.transient_local() : rclcpp::QoS{ 1 };
  point_cloud_pub_ =
      create_publisher<PointCloud2>("bonxai_point_cloud_centers", qos);

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  /* Modified by JL Matez: changing PointCloud2 msg to SemanticPointCloud msg */
  using std::chrono_literals::operator""s;
  point_cloud_sub_.subscribe(this, "cloud_in", rmw_qos_profile_sensor_data);
  tf_point_cloud_sub_ = std::make_shared<tf2_ros::MessageFilter<segmentation_msgs::msg::SemanticPointCloud>>(
      point_cloud_sub_,
      *tf2_buffer_,
      world_frame_id_,
      5,
      this->get_node_logging_interface(),
      this->get_node_clock_interface(),
      5s);


  tf_point_cloud_sub_->registerCallback(&BonxaiServer::insertCloudCallback, this);

  reset_srv_ = create_service<ResetSrv>(
      "~/reset", std::bind(&BonxaiServer::resetSrv, this, _1, _2));

  // set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(
      std::bind(&BonxaiServer::onParameter, this, _1));
}

void BonxaiServer::initializeBonxaiObject()
{
  {
    rcl_interfaces::msg::ParameterDescriptor occupancy_min_z_desc;
    occupancy_min_z_desc.description =
        "Minimum height of occupied cells to consider in the final map";
    rcl_interfaces::msg::FloatingPointRange occupancy_min_z_range;
    occupancy_min_z_range.from_value = -100.0;
    occupancy_min_z_range.to_value = 100.0;
    occupancy_min_z_desc.floating_point_range.push_back(occupancy_min_z_range);
    occupancy_min_z_ =
        declare_parameter("occupancy_min_z", -100.0, occupancy_min_z_desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor occupancy_max_z_desc;
    occupancy_max_z_desc.description =
        "Maximum height of occupied cells to consider in the final map";
    rcl_interfaces::msg::FloatingPointRange occupancy_max_z_range;
    occupancy_max_z_range.from_value = -100.0;
    occupancy_max_z_range.to_value = 100.0;
    occupancy_max_z_desc.floating_point_range.push_back(occupancy_max_z_range);
    occupancy_max_z_ =
        declare_parameter("occupancy_max_z", 100.0, occupancy_max_z_desc);
  }

  {
    rcl_interfaces::msg::ParameterDescriptor max_range_desc;
    max_range_desc.description = "Sensor maximum range";
    rcl_interfaces::msg::FloatingPointRange max_range_range;
    max_range_range.from_value = -1.0;
    max_range_range.to_value = 100.0;
    max_range_desc.floating_point_range.push_back(max_range_range);
    max_range_ = declare_parameter("sensor_model.max_range", -1.0, max_range_desc);
  }

  res_ = declare_parameter("resolution", 0.1);

  rcl_interfaces::msg::ParameterDescriptor prob_hit_desc;
  prob_hit_desc.description =
      "Probabilities for hits in the sensor model when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_hit_range;
  prob_hit_range.from_value = 0.5;
  prob_hit_range.to_value = 1.0;
  prob_hit_desc.floating_point_range.push_back(prob_hit_range);
  const double prob_hit = declare_parameter("sensor_model.hit", 0.7, prob_hit_desc);

  rcl_interfaces::msg::ParameterDescriptor prob_miss_desc;
  prob_miss_desc.description =
      "Probabilities for misses in the sensor model when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_miss_range;
  prob_miss_range.from_value = 0.0;
  prob_miss_range.to_value = 0.5;
  prob_miss_desc.floating_point_range.push_back(prob_miss_range);
  const double prob_miss =
      declare_parameter("sensor_model.miss", 0.4, prob_miss_desc);

  rcl_interfaces::msg::ParameterDescriptor prob_min_desc;
  prob_min_desc.description =
      "Minimum probability for clamping when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_min_range;
  prob_min_range.from_value = 0.0;
  prob_min_range.to_value = 1.0;
  prob_min_desc.floating_point_range.push_back(prob_min_range);
  const double thres_min =
      declare_parameter("sensor_model.min", 0.12, prob_min_desc);

  rcl_interfaces::msg::ParameterDescriptor prob_max_desc;
  prob_max_desc.description =
      "Maximum probability for clamping when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_max_range;
  prob_max_range.from_value = 0.0;
  prob_max_range.to_value = 1.0;
  prob_max_desc.floating_point_range.push_back(prob_max_range);
  const double thres_max =
      declare_parameter("sensor_model.max", 0.97, prob_max_desc);

  // initialize bonxai object & params
  RCLCPP_INFO(get_logger(), "Voxel resolution %f", res_);
  if(currentMode == MsgType::Empty)
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::Empty>>(res_);
  else if(currentMode == MsgType::RGB)
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::Color>>(res_);
  else if(currentMode == MsgType::Semantics)
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::Semantics>>(res_);
  else if(currentMode == MsgType::RGBSemantics)
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::RGBSemantics>>(res_);
  
  Bonxai::ProbabilisticMap::Options options = { 
                                Bonxai::logodds(prob_miss),
                                Bonxai::logodds(prob_hit),
                                Bonxai::logodds(thres_min),
                                Bonxai::logodds(thres_max) };
  bonxai_->setOptions(options);
}

/* Modified by JL Matez: changing PointCloud2 msg to SemanticPointCloud msg */
void BonxaiServer::insertCloudCallback(const segmentation_msgs::msg::SemanticPointCloud::ConstSharedPtr cloud)
{
  const auto start_time = rclcpp::Clock{}.now();

  /* Added by JL Matez */
  for (size_t i = 0; i < cloud->cloud.fields.size(); i++)
  {  
    if(cloud->cloud.fields[i].name == "rgb") 
      currentMode = currentMode | MsgType::RGB;
    else if(cloud->cloud.fields[i].name == "instance_id") 
      currentMode = currentMode | MsgType::Semantics;
  }

  if(bonxai_.get()==nullptr)
    initializeBonxaiObject();

  if(static_cast<int>(currentMode & MsgType::Semantics) != 0 && !semantics.is_initialized()) {
    semantics.initialize(cloud->categories);
  }

  if(static_cast<int>(currentMode & MsgType::Semantics) != 0) {
    std::vector<SemanticObject> localMap = semantics.convertROSMessageToSemanticMap(cloud->instances);
    semantics.integrateNewSemantics(localMap);
  }

  if(currentMode == MsgType::Empty) 
  {
    using PointCloudType = pcl::PointCloud<pcl::PointXYZ>;
    PointCloudType pc;
    pcl::fromROSMsg(cloud->cloud, pc);
    addObservation<PointCloudType, Bonxai::Empty>(pc, cloud->header);
    publishAll<Bonxai::Empty>(cloud->header.stamp);
  }
  else if(currentMode == MsgType::RGB)
  {
    using PointCloudType = pcl::PointCloud<pcl::PointXYZRGB>;
    PointCloudType pc;
    pcl::fromROSMsg(cloud->cloud, pc);
    addObservation<PointCloudType, Bonxai::Color>(pc, cloud->header);
    publishAll<Bonxai::Color>(cloud->header.stamp);
  }
  else if(currentMode == MsgType::Semantics)
  {
    using PointCloudType = pcl::PointCloud<pcl::PointXYZSemantics>;
    PointCloudType pc;
    pcl::fromROSMsg(cloud->cloud, pc);

    addObservation<PointCloudType, Bonxai::Semantics>(pc, cloud->header);
    publishAll<Bonxai::Semantics>(cloud->header.stamp);
  }
  else if(currentMode == MsgType::RGBSemantics)
  {
    using PointCloudType = pcl::PointCloud<pcl::PointXYZRGBSemantics>;
    PointCloudType pc;
    pcl::fromROSMsg(cloud->cloud, pc);
    addObservation<PointCloudType, Bonxai::RGBSemantics>(pc, cloud->header);
    publishAll<Bonxai::RGBSemantics>(cloud->header.stamp);
  }
  
  double total_elapsed = (rclcpp::Clock{}.now() - start_time).seconds();
  RCLCPP_DEBUG(get_logger(), "Pointcloud insertion in Bonxai done, %f sec)", total_elapsed);

}

rcl_interfaces::msg::SetParametersResult
BonxaiServer::onParameter(const std::vector<rclcpp::Parameter>& parameters)
{/*
  update_param(parameters, "occupancy_min_z", occupancy_min_z_);
  update_param(parameters, "occupancy_max_z", occupancy_max_z_);

  double sensor_model_min{ get_parameter("sensor_model.min").as_double() };
  update_param(parameters, "sensor_model.min", sensor_model_min);
  double sensor_model_max{ get_parameter("sensor_model.max").as_double() };
  update_param(parameters, "sensor_model.max", sensor_model_max);
  double sensor_model_hit{ get_parameter("sensor_model.hit").as_double() };
  update_param(parameters, "sensor_model.hit", sensor_model_hit);
  double sensor_model_miss{ get_parameter("sensor_model.miss").as_double() };
  update_param(parameters, "sensor_model.miss", sensor_model_miss);

  Bonxai::ProbabilisticMap::Options options = { 
                            Bonxai::logodds(sensor_model_miss),
                            Bonxai::logodds(sensor_model_hit),
                            Bonxai::logodds(sensor_model_min),
                            Bonxai::logodds(sensor_model_max) };

  bonxai_->setOptions(options);
  */
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
  
}

bool BonxaiServer::resetSrv(const std::shared_ptr<ResetSrv::Request>,
                            const std::shared_ptr<ResetSrv::Response>)
{
  const auto rostime = now();
  if(currentMode == MsgType::Empty)
  {
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::ProbabilisticCell<Bonxai::Empty>>>(res_);
    publishAll<Bonxai::Empty>(rostime);
  }
  else if(currentMode == MsgType::RGB)
  {
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::ProbabilisticCell<Bonxai::Color>>>(res_);
    publishAll<Bonxai::Color>(rostime);
  }
  else if(currentMode == MsgType::Semantics)
  {
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::ProbabilisticCell<Bonxai::Semantics>>>(res_);
    publishAll<Bonxai::Semantics>(rostime);
  }
  else if(currentMode == MsgType::RGBSemantics)
  {
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::ProbabilisticCell<Bonxai::RGBSemantics>>>(res_);
    publishAll<Bonxai::RGBSemantics>(rostime);
  }

  RCLCPP_INFO(get_logger(), "Cleared Bonxai");

  return true;
}

}  // namespace bonxai_server

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(bonxai_server::BonxaiServer)
