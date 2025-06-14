#include <tf2_ros/create_timer_ros.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <voxeland_map/Utils/Stopwatch.hpp>
#include <voxeland_server.hpp>
#include <segmentation_msgs/msg/instance_semantic_map.hpp>

#include <rclcpp/serialization.hpp>

#include "nlohmann/json.hpp"
#include "voxeland_map/Utils/logging.hpp"

namespace
{
    template <typename T>
    bool update_param(const std::vector<rclcpp::Parameter>& p, const std::string& name, T& value)
    {
        auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter& parameter) {
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

namespace voxeland_server
{
    VoxelandServer::VoxelandServer(const rclcpp::NodeOptions& node_options)
        : Node("voxeland_server_node", node_options)
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
            VXL_INFO(
                "Publishing latched (single publish will take longer, "
                "all topics are prepared)");
        }
        else
        {
            VXL_INFO(
                "Publishing non-latched (topics are only prepared as needed, "
                "will only be re-published on map change");
        }

        auto qos = rclcpp::QoS{ 1 };
        point_cloud_pub_ = create_publisher<PointCloud2>("bonxai_point_cloud_centers", qos);

        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
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

        tf_point_cloud_sub_->registerCallback(&VoxelandServer::insertCloudCallback, this);
        */
        point_cloud_sub_ =
            create_subscription<segmentation_msgs::msg::SemanticPointCloud>("cloud_in", 1, std::bind(&VoxelandServer::insertCloudCallback, this, _1));

        reset_srv_ = create_service<ResetSrv>("~/reset", std::bind(&VoxelandServer::resetSrv, this, _1, _2));

        save_map_srv_ = create_service<ResetSrv>("~/save_map", std::bind(&VoxelandServer::saveMapSrv, this, _1, _2));

        load_map_srv_ = create_service<UpdateMapResultsSrv>("~/update_map_results", std::bind(&VoxelandServer::loadMapSrv, this, _1, _2));

        // set parameter callback
        set_param_res_ = this->add_on_set_parameters_callback(std::bind(&VoxelandServer::onParameter, this, _1));
    }

    void VoxelandServer::initializeBonxaiObject()
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
            max_range_ = declare_parameter("sensor_model.max_range", 100.0, max_range_desc);
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
        VXL_INFO("Voxel resolution: {}m", res_);
        AUTO_TEMPLATE(currentMode,
                      bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<DataT>>(res_));
        Bonxai::ProbabilisticMap::Options options = {
            Bonxai::logodds(prob_miss), Bonxai::logodds(prob_hit), Bonxai::logodds(thres_min), Bonxai::logodds(thres_max)
        };
        bonxai_->setOptions(options);
    }

    /* Modified by JL Matez: changing PointCloud2 msg to SemanticPointCloud msg */
    void VoxelandServer::insertCloudCallback(const segmentation_msgs::msg::SemanticPointCloud::ConstSharedPtr cloud)
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
            if (semantics_as_instances_ && modeHas(DataMode::Semantics))
            {
                currentMode = currentMode | DataMode::SemanticsInstances;
            }

            rmw_qos_profile_t qos{ .reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE };
            get_distributions_srv_ = create_service<GetClassDistributions>("voxeland/get_class_distributions",
                                                                           std::bind(&VoxelandServer::getClassDistributionsSrv, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                                                           qos);
        }

        if (bonxai_.get() == nullptr)
            initializeBonxaiObject();

        // If semantics are included in the point cloud, the possible object categories are retrieved from the
        // first message.
        if (modeHas(DataMode::Semantics) && !semantics.is_initialized())
        {
            semantics.initialize(cloud->categories, *bonxai_, currentMode);
            if (semantics_as_instances_)
            {
                semantic_map_pub_ = create_publisher<segmentation_msgs::msg::InstanceSemanticMap>("semantic_map_instances", 1);
            }
        }

        if (currentMode == DataMode::Empty)  // Mode XYZ
        {
            // VXL_INFO("Mode Empty");
            insertPointCloudBasic<voxeland::Empty>(cloud);
        }
        else if (currentMode == DataMode::RGB)  // Mode XYZRGB
        {
            // VXL_INFO("Mode RGB");
            insertPointCloudBasic<voxeland::Color>(cloud);
        }

        // Semantics - No instances
        else if (currentMode == DataMode::Semantics)  // Mode XYZSemantics
        {
            // VXL_INFO("Mode Semantics");
            insertPointCloudSemantics<voxeland::Semantics>(cloud);
        }
        else if (currentMode == DataMode::RGBSemantics)  // Mode XYZRGBSemantics
        {
            // VXL_INFO("Mode RGBSemantics");
            insertPointCloudSemantics<voxeland::RGBSemantics>(cloud);
        }

        // Semantics with instances
        else if (currentMode == DataMode::SemanticsInstances)  // Mode XYZSemanticsInstances
        {
            // VXL_INFO("Mode SemanticsInstances");
            insertPointCloudSemanticInstances<voxeland::SemanticsInstances>(cloud);
        }
        else if (currentMode == DataMode::RGBSemanticsInstances)  // Mode XYZRGBSemanticsInstances
        {
            // VXL_INFO("Mode RGBSemanticsInstances");
            insertPointCloudSemanticInstances<voxeland::RGBSemanticsInstances>(cloud);
        }

        double total_elapsed = (rclcpp::Clock{}.now() - start_time).seconds();
        // VXL_INFO("Pointcloud insertion in Bonxai done, {} sec)", total_elapsed);
    }

    rcl_interfaces::msg::SetParametersResult VoxelandServer::onParameter(const std::vector<rclcpp::Parameter>& parameters)
    {
        /*
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

    bool VoxelandServer::resetSrv(const std::shared_ptr<ResetSrv::Request>, const std::shared_ptr<ResetSrv::Response>)
    {
        const auto rostime = now();
        AUTO_TEMPLATE(currentMode,
                      {
                          bonxai_ = std::make_unique<Bonxai::ProbabilisticMapT<Bonxai::ProbabilisticCell<DataT>>>(res_);
                          publishAll<DataT>(rostime);
                      });

        VXL_INFO("Cleared Bonxai");

        return true;
    }

    void VoxelandServer::saveMapSrv(const std::shared_ptr<std_srvs::srv::Empty::Request>, const std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        VXL_INFO("Saving map files");

        if (modeHas(DataMode::SemanticsInstances))
        {
            AUTO_TEMPLATE_INSTANCES_ONLY(currentMode, semantics.refineGlobalSemanticMap<DataT>(15));
            nlohmann::json json_data = semantics.mapToJSON();

            std::string json_filename = "voxeland_instanceMap.json";
            std::ofstream mapOutfile(json_filename);

            if (!mapOutfile.is_open())
            {
                VXL_ERROR("Cannot save .JSON file in: {}/{}", std::filesystem::current_path().string(), json_filename);
                return;
            }
            mapOutfile << json_data.dump(4);
            mapOutfile.close();

            nlohmann::json json_appearances = semantics.appearancesToJson();

            std::string json_appearances_filename = "voxeland_instanceMap_appearances.json";
            std::ofstream appearancesOutfile(json_appearances_filename);

            if(!appearancesOutfile.is_open())
            {
                VXL_ERROR("Cannot save .JSON file in: {}/{}", std::filesystem::current_path().string(), json_appearances_filename);
                return;
            }
            appearancesOutfile << json_appearances.dump(4);
            appearancesOutfile.close();

        }

        std::string ply;

        AUTO_TEMPLATE(currentMode,
                      ply = mapToPLY<DataT>());

        std::string ply_filename = "voxeland_pointcloud.ply";
        std::ofstream outfile(ply_filename);

        if (!outfile.is_open())
        {
            VXL_ERROR("Cannot save .PLY file in: {}/{}", std::filesystem::current_path().string(), ply_filename);
            return;
        }
        outfile << ply;
        outfile.close();
    }

    void VoxelandServer::loadMapSrv(const std::shared_ptr<UpdateMapResultsSrv::Request> req, const std::shared_ptr<UpdateMapResultsSrv::Response> resp)
    {
        VXL_INFO("Loading map files");
        try {
            nlohmann::json json_map = nlohmann::json::parse(req->json_map);
            semantics.updateSemanticMapResultsFromJSON(json_map);
        } catch (nlohmann::json::parse_error& e) {
            VXL_ERROR("Failed to parse JSON map: {}", e.what());
            resp->success = false;
            resp->message = "Failed to parse JSON map";
            return;
        } catch(std::runtime_error& e) {
            VXL_ERROR("An error occurred while loading the map: {}", e.what());
            resp->success = false;
            resp->message = "Error loading map: " + std::string(e.what());
            return;
        }

        resp->success = true;
        resp->message = "Map loaded successfully";
        VXL_INFO("Map loaded successfully");
    }


    bool VoxelandServer::getClassDistributionsSrv(
        const std::shared_ptr<rmw_request_id_t> requestHeader,
        GetClassDistributions::Request::SharedPtr request,
        GetClassDistributions::Response::SharedPtr response)
    {
        if (!modeHas(DataMode::Semantics))
        {
            VXL_ERROR("Tried to get class distributions through service, but current mode does not have semantic information!");
            return false;
        }

        VXL_WARN("Received class distribution request {}", requestHeader->sequence_number);
        Stopwatch watch;
        AUTO_TEMPLATE_SEMANTICS_ONLY(currentMode, fillClassSrvResponse<DataT>(request, response));
        VXL_WARN("Took {}s to process the service", watch.ellapsed());
        return true;
    }

    template <typename DataT>
    void VoxelandServer::fillClassSrvResponse(GetClassDistributions::Request::SharedPtr request, GetClassDistributions::Response::SharedPtr response)
    {
        auto grid = bonxai_->With<DataT>()->grid();
        auto accessor = grid->createAccessor();

        auto setDefaultClassDistribution = [&](std::vector<double>& dist) {
            size_t numCategories = SemanticMap::get_instance().default_categories.size();
            dist.resize(numCategories, 1. / numCategories);
        };

        response->distributions.resize(request->query_points.size());

        // query each point to get the probabilities vector
        for (size_t i = 0; i < request->query_points.size(); i++)
        {
            geometry_msgs::msg::Point point = request->query_points[i];
            Bonxai::CoordT coord = grid->posToCoord(point.x, point.y, point.z);
            Bonxai::ProbabilisticCell<DataT>* cell = accessor.value(coord);

            // get p(class | occupied) and p(occupied) from the cell
            std::vector<double> classProbabilities;
            float occupancyProb;
            if (cell)
            {
                occupancyProb = Bonxai::prob(cell->probability_log);
                classProbabilities = cell->data.GetClassProbabilities();
                if (classProbabilities.size() == 0)  // the cell exists but has only ever been observed to be empty, give it the default class distribution
                    setDefaultClassDistribution(classProbabilities);
            }
            else
            {
                // the cell does not exist! this means we've never even had a ray pass through it
                occupancyProb = 0.5f;
                setDefaultClassDistribution(classProbabilities);
            }

            // combine both probs into p(class)
            // this computation implicitly considers that p(class | !occupied) = 1 for the background and = 0 for every other class
            {
                for (size_t classIndex = 0; classIndex < classProbabilities.size() - 1; classIndex++)
                    classProbabilities[classIndex] = std::lerp(0., classProbabilities[classIndex], occupancyProb);

                classProbabilities.back() = std::lerp(1., classProbabilities.back(), occupancyProb);  // the last element is always the background class
                VXL_ASSERT(classProbabilities.back() <= 1);
            }

            // retrieve the corresponding class names and fill in the response
            voxeland_msgs::msg::ClassDistribution& distribution = response->distributions[i];
            for (size_t class_id = 0; class_id < classProbabilities.size(); class_id++)
            {
                vision_msgs::msg::ObjectHypothesis& hypothesis = distribution.probabilities.emplace_back();
                hypothesis.class_id = SemanticMap::get_instance().default_categories[class_id];
                hypothesis.score = classProbabilities[class_id];
            }
        }
    }

    template <typename DataT>
    void VoxelandServer::insertPointCloudBasic(const segmentation_msgs::msg::SemanticPointCloud::ConstSharedPtr cloud)
    {
        using PointCloudType = typename DataT::PointCloudType;
        PointCloudType pc;
        pcl::fromROSMsg(cloud->cloud, pc);
        pcl::PointXYZ sensorPosition = transformPointCloudToGlobal<PointCloudType, DataT>(pc, cloud->pose);
        bonxai_->With<DataT>()->insertPointCloud(pc.points, sensorPosition, max_range_);
        publishAll<DataT>(cloud->header.stamp);
    }

    template <typename DataT>
    void VoxelandServer::insertPointCloudSemantics(const segmentation_msgs::msg::SemanticPointCloud::ConstSharedPtr cloud)
    {
        using PointCloudType = typename DataT::PointCloudType;
        PointCloudType pc;
        pcl::fromROSMsg(cloud->cloud, pc);
        pcl::PointXYZ sensorPosition = transformPointCloudToGlobal<PointCloudType, DataT>(pc, cloud->pose);

        semantics_ros_wrapper.addLocalSemanticMap<PointCloudType>(cloud->instances, pc);

        bonxai_->With<DataT>()->insertPointCloud(pc.points, sensorPosition, max_range_);
        publishAll<DataT>(cloud->header.stamp);
    }

    template <typename DataT>
    void VoxelandServer::insertPointCloudSemanticInstances(const segmentation_msgs::msg::SemanticPointCloud::ConstSharedPtr cloud)
    {
        using PointCloudType = typename DataT::PointCloudType;
        PointCloudType pc;
        pcl::fromROSMsg(cloud->cloud, pc);
        pcl::PointXYZ sensorPosition = transformPointCloudToGlobal<PointCloudType, DataT>(pc, cloud->pose);
        semantics_ros_wrapper.addLocalInstanceSemanticMap<PointCloudType, DataT>(cloud->instances, pc);
        bonxai_->With<DataT>()->insertPointCloud(pc.points, sensorPosition, max_range_);
        if (number_iterations % 20 == 0)
        {
            const auto stime3 = rclcpp::Clock{}.now();
            semantics.refineGlobalSemanticMap<DataT>(5);
        }
        publishAllWithInstances<DataT>(cloud->header.stamp);

        std::set<InstanceID_t> visibleInstances =
            semantics.getCurrentVisibleInstances<DataT>(occupancy_min_z_, occupancy_max_z_);
        
        SemanticsROSWrapper::InstanceMapMsgs msgs = semantics_ros_wrapper.getSemanticMapAsROSMessage(cloud->header.stamp, visibleInstances);
        semantic_map_pub_->publish(msgs.instanceMap);
        
        static auto textPub = create_publisher<visualization_msgs::msg::MarkerArray>("/voxeland/IDs", 1);
        textPub->publish(msgs.textMarkers);
        
        // VXL_INFO("Global map: {} visible and {} active instances", visibleInstances.size(), semantics.globalSemanticMap.size());
    }

    template <typename DataT>
    void VoxelandServer::publishAll(const rclcpp::Time& rostime)
    {
        std::vector<DataT> cell_data;
        std::vector<Bonxai::Point3D> cell_points;
        cell_points.clear();
        bonxai_->With<DataT>()->getOccupiedVoxels(cell_points, cell_data);

        if (cell_points.size() <= 1)
        {
            VXL_WARN("Nothing to publish, bonxai is empty");
            return;
        }

        bool publish_point_cloud =
            (latched_topics_ || point_cloud_pub_->get_subscription_count() + point_cloud_pub_->get_intra_process_subscription_count() > 0);

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
                    voxeland::Color vizualization_color = cell_data[i].toColor();
                    pcl_cloud.emplace_back(
                        (float)voxel.x, (float)voxel.y, (float)voxel.z, vizualization_color.r, vizualization_color.g, vizualization_color.b);
                }
            }
            PointCloud2 cloud;
            pcl::toROSMsg(pcl_cloud, cloud);

            cloud.header.frame_id = world_frame_id_;
            cloud.header.stamp = rostime;
            point_cloud_pub_->publish(cloud);
            VXL_INFO("Published occupancy grid with {} voxels", pcl_cloud.points.size());
        }
    }

    template <typename DataT>
    void VoxelandServer::publishAllWithInstances(const rclcpp::Time& rostime)
    {
        std::vector<DataT> cell_data;
        std::vector<Bonxai::Point3D> cell_points;
        cell_points.clear();
        bonxai_->With<DataT>()->getOccupiedVoxels(cell_points, cell_data);

        if (cell_points.size() <= 1)
        {
            VXL_WARN("Nothing to publish, bonxai is empty");
            return;
        }

        bool publish_point_cloud =
            (latched_topics_ || point_cloud_pub_->get_subscription_count() + point_cloud_pub_->get_intra_process_subscription_count() > 0);

        // init pointcloud for occupied space:
        if (publish_point_cloud)
        {
            pcl::PointCloud<pcl::PointXYZRGBSemantics> pcl_cloud;

            pcl_cloud.clear();

            for (size_t i = 0; i < cell_points.size(); i++)
            {
                const auto& voxel = cell_points[i];

                if (voxel.z >= occupancy_min_z_ && voxel.z <= occupancy_max_z_)
                {
                    voxeland::Color vizualization_color = cell_data[i].toColor();
                    std::uint32_t rgb = ((std::uint32_t)vizualization_color.r << 16 | (std::uint32_t)vizualization_color.g << 8 |
                                         (std::uint32_t)vizualization_color.b);
                    auto itInstances = std::max_element(cell_data[i].instances_votes.begin(), cell_data[i].instances_votes.end());
                    auto idxMaxVotes = std::distance(cell_data[i].instances_votes.begin(), itInstances);
                    InstanceID_t instanceID = cell_data[i].instances_candidates[idxMaxVotes];
                    pcl_cloud.emplace_back((float)voxel.x, (float)voxel.y, (float)voxel.z, *reinterpret_cast<float*>(&rgb), instanceID);
                }
            }
            PointCloud2 cloud;
            pcl::toROSMsg(pcl_cloud, cloud);

            cloud.header.frame_id = world_frame_id_;
            cloud.header.stamp = rostime;
            point_cloud_pub_->publish(cloud);
            VXL_INFO("Published occupancy grid with {} voxels", pcl_cloud.points.size());
        }
    }

    template <typename PointCloudTypeT, typename DataT>
    pcl::PointXYZ VoxelandServer::transformPointCloudToGlobal(PointCloudTypeT& pc, geometry_msgs::msg::PoseWithCovariance pose)
    {
        Eigen::Isometry3d sensor_to_world_iso;
        tf2::fromMsg(pose.pose, sensor_to_world_iso);
        Eigen::Matrix4f sensor_to_world = sensor_to_world_iso.matrix().cast<float>();

        // Transforming Points to Global Reference Frame
        pcl::transformPointCloud(pc, pc, sensor_to_world);

        // Getting the Translation from the sensor to the Global Reference Frame
        const auto& t = pose.pose.position;

        return pcl::PointXYZ((float)t.x, (float)t.y, (float)t.z);
    }

    template <typename DataT>
    std::string VoxelandServer::mapToPLY()
    {
        std::vector<DataT> cell_data;
        std::vector<Bonxai::Point3D> cell_points;

        bonxai_->With<DataT>()->getOccupiedVoxels(cell_points, cell_data);

        std::string ply = fmt::format("ply\nformat ascii 1.0\nelement vertex {}\n{}\nend_header\n", cell_points.size(), DataT::getHeaderPLY());

        for (size_t i = 0; i < cell_points.size(); i++)
        {
            ply += cell_data[i].toPLY(cell_points[i]);
        }

        return ply;
    }

}  // namespace voxeland_server

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(voxeland_server::VoxelandServer)
