#include <tf2_ros/create_timer_ros.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <rclcpp/serialization.hpp>
#include <segmentation_msgs/msg/instance_semantic_map.hpp>
#include <stdexcept>
#include <string>
#include <voxeland_map/Utils/Stopwatch.hpp>
#include <voxeland_server.hpp>
#include <Profiling.hpp>

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

        {
            auto_save_enabled_ = declare_parameter("automatic_map_saving", false);
            
            // Only declare scene and detector parameters if automatic saving is enabled
            if (auto_save_enabled_)
            {
                scene_name_ = declare_parameter("scene_name", "unknown_scene");
                detector_name_ = declare_parameter("detector_name", "unknown_detector");
                VXL_INFO("Automatic map saving ENABLED - Scene: {}, Detector: {}", scene_name_, detector_name_);
                
                // Determine output directory and file path at startup (only once)
                // Use current working directory (usually the workspace root)
                std::filesystem::path workspace_root = std::filesystem::current_path();
                std::filesystem::path base_output_dir = workspace_root / "src" / "Voxeland" / "evaluation" / "voxeland_output";
                
                // Use scene_name directly as folder name (e.g., scannet_scene0000_01 or scenenn_011)
                output_dir_ = (base_output_dir / scene_name_).string();
                
                // Create scene directory if it doesn't exist
                std::filesystem::create_directories(output_dir_);
                
                // Check if PLY file already exists
                std::string base_filename = "voxeland_semantic_map_" + detector_name_ + "_" + scene_name_;
                std::string candidate_path = output_dir_ + "/" + base_filename + ".ply";
                
                if (std::filesystem::exists(candidate_path))
                {
                    // File exists, add timestamp
                    auto now = std::chrono::system_clock::now();
                    auto time_t_now = std::chrono::system_clock::to_time_t(now);
                    std::tm tm_now;
                    localtime_r(&time_t_now, &tm_now);
                    
                    char timestamp[64];
                    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm_now);
                    
                    output_ply_path_ = output_dir_ + "/" + base_filename + "_" + timestamp + ".ply";
                    VXL_INFO("Output file already exists. Will save to: {}", output_ply_path_);
                }
                else
                {
                    // File doesn't exist, use base name
                    output_ply_path_ = candidate_path;
                    VXL_INFO("Will save map to: {}", output_ply_path_);
                }
            }
            else
            {
                VXL_INFO("Automatic map saving DISABLED");
            }
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

        // Auto-save timer: save map every 30 seconds (only if enabled)
        if (auto_save_enabled_)
        {
            auto_save_timer_ = create_wall_timer(
                std::chrono::seconds(30),
                std::bind(&VoxelandServer::autoSaveMapCallback, this));
            
            VXL_INFO("Auto-save timer initialized: map will be saved every 30 seconds to {}", output_ply_path_);
        }

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
        voxeland::ScopedStopwatch watch("Inserting pointcloud");

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
        // first message and can grow dynamically.
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

            if (!appearancesOutfile.is_open())
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

        // If PLY is empty and we have semantics instances, generate PLY from semantic map
        if (ply.empty() && modeHas(DataMode::SemanticsInstances))
        {
            VXL_INFO("Generating PLY from semantic instances map");
            std::string instances_ply = semanticsMapToPLY();
            
            std::string instances_ply_filename = "voxeland_instances_map.ply";
            std::ofstream instances_outfile(instances_ply_filename);
            
            if (!instances_outfile.is_open())
            {
                VXL_ERROR("Cannot save instances .PLY file in: {}/{}", std::filesystem::current_path().string(), instances_ply_filename);
                return;
            }
            instances_outfile << instances_ply;
            instances_outfile.close();
            VXL_INFO("Saved semantic instances to PLY: {} vertices", semantics.globalSemanticMap.size());
        }
    }

    void VoxelandServer::loadMapSrv(const std::shared_ptr<UpdateMapResultsSrv::Request> req, const std::shared_ptr<UpdateMapResultsSrv::Response> resp)
    {
        VXL_INFO("Loading map files");
        try
        {
            nlohmann::json json_map = nlohmann::json::parse(req->json_map);
            semantics.updateSemanticMapResultsFromJSON(json_map);
        }
        catch (nlohmann::json::parse_error& e)
        {
            VXL_ERROR("Failed to parse JSON map: {}", e.what());
            resp->success = false;
            resp->message = "Failed to parse JSON map";
            return;
        }
        catch (std::runtime_error& e)
        {
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
        voxeland::ScopedStopwatch watch("Get class distributions");

        AUTO_TEMPLATE_SEMANTICS_ONLY(currentMode, fillClassSrvResponse<DataT>(request, response));
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
                std::string categoryName = SemanticMap::get_instance().getCategoryName(class_id);
                hypothesis.class_id = categoryName.empty() ? "unknown" : categoryName;
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

    std::string VoxelandServer::semanticsMapToPLY()
    {
        // Generate PLY from semantic instances (bounding box centers only)
        std::stringstream ply_header;
        std::stringstream ply_data;
        size_t vertex_count = 0;

        for (const auto& instance : semantics.globalSemanticMap)
        {
            if (instance.pointsTo != -1)
                continue;

            float center_x = (instance.bbox.minX + instance.bbox.maxX) / 2.0f;
            float center_y = (instance.bbox.minY + instance.bbox.maxY) / 2.0f;
            float center_z = (instance.bbox.minZ + instance.bbox.maxZ) / 2.0f;

            std::string dominant_category = "unknown";
            float max_probability = 0.0f;
            
            for (const auto& [categoryIndex, probability] : instance.alphaParamsCategories)
            {
                if (probability > max_probability)
                {
                    max_probability = static_cast<float>(probability);
                    dominant_category = semantics.getCategoryName(categoryIndex);
                }
            }

            int instance_id = 0;
            try
            {
                if (instance.instanceID.length() > 3 && instance.instanceID.substr(0, 3) == "obj")
                {
                    instance_id = std::stoi(instance.instanceID.substr(3));
                }
            }
            catch (const std::exception& e)
            {
                VXL_WARN("Failed to parse instance ID: {}", instance.instanceID);
                continue;
            }

            uint32_t hexColor = semantics.indexToHexColor(instance_id);
            uint8_t r = (hexColor >> 16) & 0xFF;
            uint8_t g = (hexColor >> 8) & 0xFF;
            uint8_t b = hexColor & 0xFF;

            ply_data << fmt::format("{} {} {} {} {} {} {} {} {}\n",
                                   center_x, center_y, center_z,
                                   static_cast<int>(r), static_cast<int>(g), static_cast<int>(b),
                                   instance_id,
                                   dominant_category,
                                   max_probability);
            vertex_count++;
        }

        ply_header << "ply\n"
                   << "format ascii 1.0\n"
                   << "element vertex " << vertex_count << "\n"
                   << "property float x\n"
                   << "property float y\n"
                   << "property float z\n"
                   << "property uchar red\n"
                   << "property uchar green\n"
                   << "property uchar blue\n"
                   << "property int instance_id\n"
                   << "property string dominant_category\n"
                   << "property float confidence\n"
                   << "end_header\n";

        return ply_header.str() + ply_data.str();
    }

    template <typename DataT>
    std::string VoxelandServer::fullSemanticMapToPLY()
    {
        // Generate complete PLY with all voxels including semantic information
        std::vector<DataT> cell_data;
        std::vector<Bonxai::Point3D> cell_points;

        bonxai_->With<DataT>()->getOccupiedVoxels(cell_points, cell_data);

        if (cell_points.size() == 0)
        {
            VXL_WARN("No voxels to save in map");
            return "";
        }

        std::stringstream ply_header;
        std::stringstream ply_data;
        size_t vertex_count = 0;

        // Process each voxel
        for (size_t i = 0; i < cell_points.size(); i++)
        {
            const auto& voxel = cell_points[i];
            
            // Apply height filter (same as visualization)
            if (voxel.z < occupancy_min_z_ || voxel.z > occupancy_max_z_)
                continue;

            // Get color from visualization
            voxeland::Color viz_color = cell_data[i].toColor();
            
            // Get instance ID
            InstanceID_t instanceID = 0;
            
            if (!cell_data[i].instances_candidates.empty() && !cell_data[i].instances_votes.empty())
            {
                auto itInstances = std::max_element(cell_data[i].instances_votes.begin(), 
                                                   cell_data[i].instances_votes.end());
                auto idxMaxVotes = std::distance(cell_data[i].instances_votes.begin(), itInstances);
                instanceID = cell_data[i].instances_candidates[idxMaxVotes];
            }

            // Calculate uncertainty_instances and uncertainty_categories
            float uncertainty_instances = 0.0f;
            float uncertainty_categories = 0.0f;
            
            // For instances uncertainty: entropy of instance votes if available
            if (!cell_data[i].instances_votes.empty())
            {
                float total_votes = 0.0f;
                for (const auto& vote : cell_data[i].instances_votes)
                {
                    total_votes += vote;
                }
                
                if (total_votes > 0)
                {
                    for (const auto& vote : cell_data[i].instances_votes)
                    {
                        if (vote > 0)
                        {
                            float p = vote / total_votes;
                            uncertainty_instances -= p * std::log(p);
                        }
                    }
                }
            }
            
            // For categories uncertainty: get from the instance's alpha parameters
            if (instanceID > 0)
            {
                for (const auto& instance : semantics.globalSemanticMap)
                {
                    if (instance.pointsTo != -1)
                        continue;
                        
                    int instance_numeric_id = 0;
                    try
                    {
                        if (instance.instanceID.length() > 3 && instance.instanceID.substr(0, 3) == "obj")
                        {
                            instance_numeric_id = std::stoi(instance.instanceID.substr(3));
                        }
                    }
                    catch (...) { continue; }
                    
                    if (instance_numeric_id == static_cast<int>(instanceID))
                    {
                        // Calculate expected Shannon entropy from Dirichlet parameters
                        double alpha_sum = 0.0;
                        for (const auto& [catIdx, alpha] : instance.alphaParamsCategories)
                        {
                            alpha_sum += alpha;
                        }
                        
                        if (alpha_sum > 0)
                        {
                            double expected_entropy = 0.0;
                            // E[H] = ψ(α₀) - (1/α₀) Σ αᵢ ψ(αᵢ)
                            // Simplified approximation for efficiency
                            for (const auto& [catIdx, alpha] : instance.alphaParamsCategories)
                            {
                                double p = alpha / alpha_sum;
                                if (p > 0)
                                {
                                    expected_entropy -= p * std::log(p);
                                }
                            }
                            uncertainty_categories = static_cast<float>(expected_entropy);
                        }
                        break;
                    }
                }
            }

            // Write voxel data: x y z r g b instanceid uncertainty_instances uncertainty_categories
            ply_data << fmt::format("{} {} {} {} {} {} {} {} {}\n",
                                   voxel.x, voxel.y, voxel.z,
                                   static_cast<int>(viz_color.r),
                                   static_cast<int>(viz_color.g),
                                   static_cast<int>(viz_color.b),
                                   instanceID,
                                   uncertainty_instances,
                                   uncertainty_categories);
            vertex_count++;
        }

        // Build PLY header (old format without string field)
        ply_header << "ply\n"
                   << "format ascii 1.0\n"
                   << "element vertex " << vertex_count << "\n"
                   << "property float x\n"
                   << "property float y\n"
                   << "property float z\n"
                   << "property uchar red\n"
                   << "property uchar green\n"
                   << "property uchar blue\n"
                   << "property int instanceid\n"
                   << "property float uncertainty_instances\n"
                   << "property float uncertainty_categories\n"
                   << "end_header\n";

        return ply_header.str() + ply_data.str();
    }

    void VoxelandServer::autoSaveMapCallback()
    {
        if (currentMode == DataMode::Uninitialized || !bonxai_)
        {
            VXL_WARN("Map not initialized yet, skipping auto-save");
            return;
        }

        VXL_INFO("Auto-saving map...");

        // Save full semantic voxel map
        if (modeHas(DataMode::SemanticsInstances))
        {
            std::string ply_content;
            AUTO_TEMPLATE_INSTANCES_ONLY(currentMode, ply_content = fullSemanticMapToPLY<DataT>());
            
            if (!ply_content.empty())
            {
                // Use the pre-determined output path
                std::ofstream outfile(output_ply_path_);
                
                if (outfile.is_open())
                {
                    outfile << ply_content;
                    outfile.close();
                    VXL_INFO("Saved semantic map to {}", output_ply_path_);
                }
                else
                {
                    VXL_ERROR("Failed to open file: {}", output_ply_path_);
                }
            }
            
            // Save instance map to JSON file (old format)
            std::filesystem::path ply_path(output_ply_path_);
            std::string json_path = ply_path.parent_path() / (ply_path.stem().string() + ".json");
            
            nlohmann::json map_json;
            nlohmann::json instances_json;
            
            for (const auto& instance : semantics.globalSemanticMap)
            {
                if (instance.pointsTo != -1)
                    continue;
                
                int instance_id = 0;
                try
                {
                    if (instance.instanceID.length() > 3 && instance.instanceID.substr(0, 3) == "obj")
                    {
                        instance_id = std::stoi(instance.instanceID.substr(3));
                    }
                    else
                    {
                        continue;
                    }
                }
                catch (const std::exception& e) 
                { 
                    VXL_WARN("Failed to parse instance ID from {}: {}", instance.instanceID, e.what());
                    continue; 
                }
                
                // Build category scores dictionary (results)
                nlohmann::json results;
                for (const auto& [categoryIndex, probability] : instance.alphaParamsCategories)
                {
                    try
                    {
                        std::string category_name = semantics.getCategoryName(categoryIndex);
                        results[category_name] = probability;
                    }
                    catch (const std::out_of_range& e)
                    {
                        VXL_WARN("Category index {} out of range for instance {}", categoryIndex, instance.instanceID);
                        continue;
                    }
                    catch (const std::exception& e)
                    {
                        VXL_WARN("Error getting category name for index {} in instance {}: {}", 
                                categoryIndex, instance.instanceID, e.what());
                        continue;
                    }
                }
                
                // Skip instances with no valid categories
                if (results.empty())
                {
                    VXL_WARN("Instance {} has no valid categories, skipping", instance.instanceID);
                    continue;
                }
                
                // Calculate bounding box center and size from bbox
                const BoundingBox3D& bbox = instance.bbox;
                
                // Check if bbox is valid (not all infinity values)
                if (std::isinf(bbox.minX) || std::isinf(bbox.minY) || std::isinf(bbox.minZ) ||
                    std::isinf(bbox.maxX) || std::isinf(bbox.maxY) || std::isinf(bbox.maxZ))
                {
                    VXL_WARN("Instance {} has invalid bounding box, using default values", instance.instanceID);
                    nlohmann::json bbox_json;
                    bbox_json["center"] = {0.0, 0.0, 0.0};
                    bbox_json["size"] = {0.1, 0.1, 0.1};
                    
                    nlohmann::json instance_entry;
                    instance_entry["bbox"] = bbox_json;
                    instance_entry["n_observations"] = 1;
                    instance_entry["results"] = results;
                    
                    instances_json["obj" + std::to_string(instance_id)] = instance_entry;
                    continue;
                }
                
                // Calculate center
                float centerX = (bbox.minX + bbox.maxX) / 2.0f;
                float centerY = (bbox.minY + bbox.maxY) / 2.0f;
                float centerZ = (bbox.minZ + bbox.maxZ) / 2.0f;
                
                // Calculate size
                float sizeX = bbox.maxX - bbox.minX;
                float sizeY = bbox.maxY - bbox.minY;
                float sizeZ = bbox.maxZ - bbox.minZ;
                
                nlohmann::json bbox_json;
                bbox_json["center"] = {centerX, centerY, centerZ};
                bbox_json["size"] = {sizeX, sizeY, sizeZ};
                
                // Build instance entry
                nlohmann::json instance_entry;
                instance_entry["bbox"] = bbox_json;
                instance_entry["n_observations"] = 1;  // Always 1 as requested
                instance_entry["results"] = results;
                
                // Add to instances map with objN key
                instances_json["obj" + std::to_string(instance_id)] = instance_entry;
            }
            
            map_json["instances"] = instances_json;
            
            std::ofstream json_outfile(json_path);
            if (json_outfile.is_open())
            {
                json_outfile << map_json.dump(4);  // Pretty print with 4 spaces
                json_outfile.close();
                VXL_INFO("Saved instance map to {}", json_path);
            }
            else
            {
                VXL_ERROR("Failed to save map JSON: {}", json_path);
            }
        }
    }

}  // namespace voxeland_server

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(voxeland_server::VoxelandServer)
