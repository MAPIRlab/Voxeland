#include <bonxai_server.hpp>

namespace
{
template <typename T>
bool update_param(const std::vector<rclcpp::Parameter>& p, const std::string& name, T& value)
{
  auto it = std::find_if(
      p.cbegin(), p.cend(), [&name](const rclcpp::Parameter& parameter) { return parameter.get_name() == name; });
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

  {
    semantics_as_instances_ = declare_parameter("semantics_as_instances", false);
  }

  latched_topics_ = declare_parameter("latch", true);
  if (latched_topics_)
  {
    BONXAI_INFO(
        "Publishing latched (single publish will take longer, "
        "all topics are prepared)");
  }
  else
  {
    BONXAI_INFO(
        "Publishing non-latched (topics are only prepared as needed, "
        "will only be re-published on map change");
  }

  auto qos = latched_topics_ ? rclcpp::QoS{ 1 }.transient_local() : rclcpp::QoS{ 1 };
  point_cloud_pub_ = create_publisher<PointCloud2>("bonxai_point_cloud_centers", qos);

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface =
      std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  using std::chrono_literals::operator""s;
  /* Modified by JL Matez: changing PointCloud2 msg to SemanticPointCloud msg
  point_cloud_sub_.subscribe(this, "cloud_in", rmw_qos_profile_sensor_data);
  tf_point_cloud_sub_ = std::make_shared<
      tf2_ros::MessageFilter<segmentation_msgs::msg::SemanticPointCloud>>(
      point_cloud_sub_,
      *tf2_buffer_,
      world_frame_id_,
      5,
      this->get_node_logging_interface(),
      this->get_node_clock_interface(),
      5s);

  tf_point_cloud_sub_->registerCallback(&BonxaiServer::insertCloudCallback, this);
  */
  point_cloud_sub_ = create_subscription<segmentation_msgs::msg::SemanticPointCloud>(
      "cloud_in", 1, std::bind(&BonxaiServer::insertCloudCallback, this, _1));

  reset_srv_ = create_service<ResetSrv>("~/reset", std::bind(&BonxaiServer::resetSrv, this, _1, _2));

  save_map_srv_ = create_service<ResetSrv>("~/save_map", std::bind(&BonxaiServer::saveMapSrv, this, _1, _2));

  get_distributions_srv_ = create_service<GetClassDistributions>(
      "~/get_class_distributions", std::bind(&BonxaiServer::getClassDistributionsSrv, this, _1, _2));

  // set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(std::bind(&BonxaiServer::onParameter, this, _1));
}

void BonxaiServer::initializeBonxaiObject()
{
  {
    rcl_interfaces::msg::ParameterDescriptor occupancy_min_z_desc;
    occupancy_min_z_desc.description = "Minimum height of occupied cells to consider in the final map";
    rcl_interfaces::msg::FloatingPointRange occupancy_min_z_range;
    occupancy_min_z_range.from_value = -100.0;
    occupancy_min_z_range.to_value = 100.0;
    occupancy_min_z_desc.floating_point_range.push_back(occupancy_min_z_range);
    occupancy_min_z_ = declare_parameter("occupancy_min_z", -100.0, occupancy_min_z_desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor occupancy_max_z_desc;
    occupancy_max_z_desc.description = "Maximum height of occupied cells to consider in the final map";
    rcl_interfaces::msg::FloatingPointRange occupancy_max_z_range;
    occupancy_max_z_range.from_value = -100.0;
    occupancy_max_z_range.to_value = 100.0;
    occupancy_max_z_desc.floating_point_range.push_back(occupancy_max_z_range);
    occupancy_max_z_ = declare_parameter("occupancy_max_z", 100.0, occupancy_max_z_desc);
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
  prob_hit_desc.description = "Probabilities for hits in the sensor model when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_hit_range;
  prob_hit_range.from_value = 0.5;
  prob_hit_range.to_value = 1.0;
  prob_hit_desc.floating_point_range.push_back(prob_hit_range);
  const double prob_hit = declare_parameter("sensor_model.hit", 0.7, prob_hit_desc);

  rcl_interfaces::msg::ParameterDescriptor prob_miss_desc;
  prob_miss_desc.description = "Probabilities for misses in the sensor model when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_miss_range;
  prob_miss_range.from_value = 0.0;
  prob_miss_range.to_value = 0.5;
  prob_miss_desc.floating_point_range.push_back(prob_miss_range);
  const double prob_miss = declare_parameter("sensor_model.miss", 0.4, prob_miss_desc);

  rcl_interfaces::msg::ParameterDescriptor prob_min_desc;
  prob_min_desc.description = "Minimum probability for clamping when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_min_range;
  prob_min_range.from_value = 0.0;
  prob_min_range.to_value = 1.0;
  prob_min_desc.floating_point_range.push_back(prob_min_range);
  const double thres_min = declare_parameter("sensor_model.min", 0.12, prob_min_desc);

  rcl_interfaces::msg::ParameterDescriptor prob_max_desc;
  prob_max_desc.description = "Maximum probability for clamping when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_max_range;
  prob_max_range.from_value = 0.0;
  prob_max_range.to_value = 1.0;
  prob_max_desc.floating_point_range.push_back(prob_max_range);
  const double thres_max = declare_parameter("sensor_model.max", 0.97, prob_max_desc);

  // initialize bonxai object & params
  BONXAI_INFO("Voxel resolution: {}m", res_);
  if (currentMode == DataMode::Empty)
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::Empty>>(res_);
  else if (currentMode == DataMode::RGB)
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::Color>>(res_);
  else if (currentMode == DataMode::Semantics)
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::Semantics>>(res_);
  else if (currentMode == DataMode::RGBSemantics)
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::RGBSemantics>>(res_);
  else if (currentMode == DataMode::SemanticsInstances)
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::SemanticsInstances>>(res_);
  else if (currentMode == DataMode::RGBSemanticsInstances)
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::RGBSemanticsInstances>>(res_);

  Bonxai::ProbabilisticMap::Options options = {
    Bonxai::logodds(prob_miss), Bonxai::logodds(prob_hit), Bonxai::logodds(thres_min), Bonxai::logodds(thres_max)
  };
  bonxai_->setOptions(options);
}

/* Modified by JL Matez: changing PointCloud2 msg to SemanticPointCloud msg */
void BonxaiServer::insertCloudCallback(const segmentation_msgs::msg::SemanticPointCloud::ConstSharedPtr cloud)
{
  number_iterations++;

  const auto start_time = rclcpp::Clock{}.now();

  // Checking the operation mode:
  // XYZ, XYZRGB, XYZSemantics, XYZSemanticsInstances, XYZRGBSemantics, XYZRGBSemanticsInstances
  // Note that RGB and Semantics options are set in the PointCloud2 message, based on the received fields.
  // On the other hand, Instances is set as parameter of Bonxai, to indicate if semantics should be
  // considered as instances or isolated voxels.
  if (currentMode == DataMode::Uninitialized)
  {
    currentMode = DataMode::Empty;

    for (size_t i = 0; i < cloud->cloud.fields.size(); i++)
    {
      if (cloud->cloud.fields[i].name == "rgb")
        currentMode = currentMode | DataMode::RGB;
      else if (cloud->cloud.fields[i].name == "instance_id")
        currentMode = currentMode | DataMode::Semantics;
    }
    if (semantics_as_instances_ && modeHasSemantics())
    {
      currentMode = currentMode | DataMode::SemanticsInstances;
    }
  }

  if (bonxai_.get() == nullptr)
    initializeBonxaiObject();

  // If semantics are included in the point cloud, the possible object categories are retrieved from the
  // first message.
  if (modeHasSemantics() && !semantics.is_initialized())
  {
    semantics.initialize(cloud->categories, *bonxai_, currentMode);
    if (semantics_as_instances_)
    {
      semantic_map_pub_ = create_publisher<segmentation_msgs::msg::InstanceSemanticMap>("semantic_map_instances", 1);
    }
  }

  //TODO move all this to a templated function to avoid repeating ourselves so much
  if (currentMode == DataMode::Empty)  // Mode XYZ
  {
    BONXAI_INFO("Mode Empty");
    using PointCloudType = pcl::PointCloud<pcl::PointXYZ>;
    insertPointCloud<PointCloudType, Bonxai::Empty>(cloud);
  }
  else if (currentMode == DataMode::RGB)  // Mode XYZRGB
  {
    BONXAI_INFO("Mode RGB");
    using PointCloudType = pcl::PointCloud<pcl::PointXYZRGB>;
    insertPointCloud<PointCloudType, Bonxai::Color>(cloud);
  }
  else if (currentMode == DataMode::Semantics)  // Mode XYZSemantics
  {
    BONXAI_INFO("Mode Semantics");
    using PointCloudType = pcl::PointCloud<pcl::PointXYZSemantics>;
    insertPointCloud<PointCloudType, Bonxai::Semantics>(cloud);
  }
  else if (currentMode == DataMode::RGBSemantics)  // Mode XYZRGBSemantics
  {
    BONXAI_INFO("Mode RGBSemantics");
    using PointCloudType = pcl::PointCloud<pcl::PointXYZRGBSemantics>;
    insertPointCloud<PointCloudType, Bonxai::RGBSemantics>(cloud);
  }
  else if (currentMode == DataMode::SemanticsInstances)  // Mode XYZSemanticsInstances
  {
    BONXAI_INFO("Mode SemanticsInstances");
    using PointCloudType = pcl::PointCloud<pcl::PointXYZSemantics>;
    PointCloudType pc;
    pcl::fromROSMsg(cloud->cloud, pc);
    pcl::PointXYZ sensorPosition =
        transformPointCloudToGlobal<PointCloudType, Bonxai::SemanticsInstances>(pc, cloud->pose);
    const auto stime = rclcpp::Clock{}.now();
    semantics_ros_wrapper.addLocalInstanceSemanticMap<PointCloudType, Bonxai::SemanticsInstances>(cloud->instances, pc);
    data_association_time += (rclcpp::Clock{}.now() - stime).seconds();
    data_association_k++;
    const auto stime2 = rclcpp::Clock{}.now();
    bonxai_->With<Bonxai::SemanticsInstances>()->insertPointCloud(pc.points, sensorPosition, 30.0);
    map_integration_time += (rclcpp::Clock{}.now() - stime2).seconds();
    map_integration_k++;
    if (number_iterations % 20 == 0)
    {
      const auto stime3 = rclcpp::Clock{}.now();
      semantics.refineGlobalSemanticMap<Bonxai::SemanticsInstances>(5);
      map_refinement_time += (rclcpp::Clock{}.now() - stime3).seconds();
      map_refinement_k++;
    }
    publishAllWithInstances<Bonxai::SemanticsInstances>(cloud->header.stamp);

    std::set<InstanceID_t> visibleInstances =
        semantics.getCurrentVisibleInstances<Bonxai::SemanticsInstances>(occupancy_min_z_, occupancy_max_z_);
    semantic_map_pub_->publish(semantics_ros_wrapper.getSemanticMapAsROSMessage(cloud->header.stamp, visibleInstances));
    /*RCLCPP_INFO(get_logger(), "############### MAP AFTER UPDATE ################");
    for(InstanceID_t idx = 0; idx < semantics.globalSemanticMap.size(); idx++){
      if(semantics.listOfVoxelsInsideBBox<Bonxai::SemanticsInstances>(semantics.globalSemanticMap[idx].bbox, idx).size()
    == 1){ BONXAI_INFO("aqui 1 voxel solo");
      }
      BONXAI_INFO("{} with: {} voxels", semantics.globalSemanticMap[idx].instanceID,
    semantics.listOfVoxelsInsideBBox<Bonxai::SemanticsInstances>(semantics.globalSemanticMap[idx].bbox, idx).size());
    }
    uint32_t oneObservationInstances = 0;
    uint32_t activeInstances = 0;
    for(InstanceID_t i = 0; i < semantics.globalSemanticMap.size(); i++){
      if(semantics.globalSemanticMap[i].pointsTo == -1 && semantics.globalSemanticMap[i].numberObservations < 2){
        oneObservationInstances += 1;
      }
      if(semantics.globalSemanticMap[i].pointsTo == -1){
        activeInstances += 1;
      }
    }
    BONXAI_INFO("Single observation instances / Total active instances: {} / {}", oneObservationInstances,
    activeInstances); RCLCPP_INFO(get_logger(), "Global map: %ld visible and %ld active instances",
    visibleInstances.size(), semantics.globalSemanticMap.size());*/
  }
  else if (currentMode == DataMode::RGBSemanticsInstances)  // Mode XYZRGBSemanticsInstances
  {
    BONXAI_INFO("Mode RGBSemanticsInstances");
    using PointCloudType = pcl::PointCloud<pcl::PointXYZRGBSemantics>;
    PointCloudType pc;
    pcl::fromROSMsg(cloud->cloud, pc);
    pcl::PointXYZ sensorPosition =
        transformPointCloudToGlobal<PointCloudType, Bonxai::RGBSemanticsInstances>(pc, cloud->pose);
    semantics_ros_wrapper.addLocalInstanceSemanticMap<PointCloudType, Bonxai::RGBSemanticsInstances>(cloud->instances,
                                                                                                     pc);
    bonxai_->With<Bonxai::RGBSemanticsInstances>()->insertPointCloud(pc.points, sensorPosition, 30.0);
    if (number_iterations % 30 == 0)
    {
      semantics.refineGlobalSemanticMap<Bonxai::RGBSemanticsInstances>(5);
    }
    publishAllWithInstances<Bonxai::RGBSemanticsInstances>(cloud->header.stamp);
    std::set<InstanceID_t> visibleInstances =
        semantics.getCurrentVisibleInstances<Bonxai::RGBSemanticsInstances>(occupancy_min_z_, occupancy_max_z_);
    semantic_map_pub_->publish(semantics_ros_wrapper.getSemanticMapAsROSMessage(cloud->header.stamp, visibleInstances));
    BONXAI_INFO(
        "Global map: {} visible and {} active instances", visibleInstances.size(), semantics.globalSemanticMap.size());
  }

  double total_elapsed = (rclcpp::Clock{}.now() - start_time).seconds();
  BONXAI_INFO("Pointcloud insertion in Bonxai done, {} sec)", total_elapsed);

  if (data_association_k > 0)
  {
    BONXAI_INFO("Average data association time: {} ms)", 1000. * data_association_time / float(data_association_k));
    BONXAI_INFO("Average map integration time: {} ms)", 1000. * map_integration_time / float(map_integration_k));
    BONXAI_INFO("Average map refinement time: {} ms)", 1000. * map_refinement_time / float(map_refinement_k));
  }
}

rcl_interfaces::msg::SetParametersResult BonxaiServer::onParameter(const std::vector<rclcpp::Parameter>& parameters)
{ /*
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

bool BonxaiServer::resetSrv(const std::shared_ptr<ResetSrv::Request>, const std::shared_ptr<ResetSrv::Response>)
{
  const auto rostime = now();
  if (currentMode == DataMode::Empty)
  {
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::ProbabilisticCell<Bonxai::Empty>>>(res_);
    publishAll<Bonxai::Empty>(rostime);
  }
  else if (currentMode == DataMode::RGB)
  {
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::ProbabilisticCell<Bonxai::Color>>>(res_);
    publishAll<Bonxai::Color>(rostime);
  }
  else if (currentMode == DataMode::Semantics)
  {
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::ProbabilisticCell<Bonxai::Semantics>>>(res_);
    publishAll<Bonxai::Semantics>(rostime);
  }
  else if (currentMode == DataMode::RGBSemantics)
  {
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::ProbabilisticCell<Bonxai::RGBSemantics>>>(res_);
    publishAll<Bonxai::RGBSemantics>(rostime);
  }
  else if (currentMode == DataMode::SemanticsInstances)
  {
    bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::ProbabilisticCell<Bonxai::SemanticsInstances>>>(res_);
    publishAll<Bonxai::SemanticsInstances>(rostime);
  }
  else if (currentMode == DataMode::RGBSemanticsInstances)
  {
    bonxai_ =
        std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::ProbabilisticCell<Bonxai::RGBSemanticsInstances>>>(res_);
    publishAll<Bonxai::RGBSemanticsInstances>(rostime);
  }

  BONXAI_INFO("Cleared Bonxai");

  return true;
}

void BonxaiServer::saveMapSrv(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                              const std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  // Refine map (if necessary)
  BONXAI_INFO("Service working!");

  std::string ply;

  if (currentMode == DataMode::Empty)
  {
    ply = mapToPLY<Bonxai::Empty>();
  }
  else if (currentMode == DataMode::RGB)
  {
    ply = mapToPLY<Bonxai::Color>();
  }
  else if (currentMode == DataMode::Semantics)
  {
    ply = mapToPLY<Bonxai::Semantics>();
  }
  else if (currentMode == DataMode::RGBSemantics)
  {
    ply = mapToPLY<Bonxai::RGBSemantics>();
  }
  else if (currentMode == DataMode::SemanticsInstances)
  {
    semantics.refineGlobalSemanticMap<Bonxai::SemanticsInstances>(15);
    ply = mapToPLY<Bonxai::SemanticsInstances>();
    nlohmann::json json_data = semantics.mapToJSON();

    std::string json_filename = "map_cplusplus.json";
    std::ofstream outfile(json_filename);

    if (!outfile.is_open())
    {
      std::cerr << "Cannot save .JSON file in: " << json_filename << std::endl;
      return;
    }
    outfile << json_data.dump(4);
    outfile.close();
  }
  else if (currentMode == DataMode::RGBSemanticsInstances)
  {
    semantics.refineGlobalSemanticMap<Bonxai::RGBSemanticsInstances>(15);
    ply = mapToPLY<Bonxai::RGBSemanticsInstances>();
    nlohmann::json json_data = semantics.mapToJSON();

    std::string json_filename = "map_cplusplus.json";
    std::ofstream outfile(json_filename);

    if (!outfile.is_open())
    {
      std::cerr << "Cannot save .JSON file in: " << json_filename << std::endl;
      return;
    }
    outfile << json_data.dump(4);
    outfile.close();
  }

  std::string ply_filename = "pointcloud_cplusplus.ply";
  std::ofstream outfile(ply_filename);

  if (!outfile.is_open())
  {
    std::cerr << "Cannot save .PLY file in: " << ply_filename << std::endl;
    return;
  }
  outfile << ply;
  outfile.close();

  // Save .PLY file (with instance_id, uncertainty_instances, uncertainty_categories)

  // Save .json file

  // [ToDo] Save two .PNG files: top-view uncertainty instances and top-view uncertainty_categories)
}

void BonxaiServer::getClassDistributionsSrv(GetClassDistributions::Request::SharedPtr request,
                                            GetClassDistributions::Response::SharedPtr response)
{
  if (!modeHasSemantics())
  {
    BONXAI_ERROR(
        "Tried to get class distributions through service, but current mode does not have semantic information!");
    return;
  }

  //TODO

}

}  // namespace bonxai_server

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(bonxai_server::BonxaiServer)
