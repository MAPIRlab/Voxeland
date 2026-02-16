#if ENABLE_DEBUG_GUI
#include <imgui_gl/imgui_gl.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <imgui_gl/utils.hpp>
#include <voxeland_map/debugging_utils.hpp>
#include <voxeland_server.hpp>

using visualization_msgs::msg::Marker;

namespace ImColors
{
    constexpr uint32_t InfoText = 0xffbedb1a;
    constexpr uint32_t ErrorText = 0xff2e48c9;

    inline ImVec4 AsVec(uint32_t hex)
    {
        ImVec4 vec;
        vec.w = (hex >> 24 & 0xff) / 255.f;
        vec.z = (hex >> 16 & 0xff) / 255.f;
        vec.y = (hex >> 8 & 0xff) / 255.f;
        vec.x = (hex >> 0 & 0xff) / 255.f;
        return vec;
    }
}  // namespace ImColors

namespace voxeland_server
{
    void VoxelandServer::SetupGUI()
    {
        debugInstancesPub = create_publisher<PointCloud2>("/voxeland/debugInstances", 1);
        debugInputPub = create_publisher<PointCloud2>("/voxeland/debugInput", 1);

        clickedPointSub = create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 1, [this](const geometry_msgs::msg::PointStamped::SharedPtr point) {
                selectedCoordinates.x = point->point.x;
                selectedCoordinates.y = point->point.y;
                selectedCoordinates.z = point->point.z;
            });

        // this needs to be run from the same thread that does the actual rendering
        auto createWindow = [&]() {
            ImguiGL::Setup(
                fmt::format("{}/resources/debug_gui.ini", ament_index_cpp::get_package_share_directory("voxeland")).c_str(),
                "voxeland_gui",
                1200,
                900);
        };

        // decide whether to run GUI in main thread (usually preferrable) or not (necessary to visualize while using a debugger)
#define SEPARATE_RENDER_THREAD 1
#if SEPARATE_RENDER_THREAD
        renderThread = std::jthread([&]() {
            // wait until initialization is done to minimize the chances of multithreading issues
            // This is horrible, but whatever :)
            while (currentMode == voxeland::DataMode::Uninitialized)
                std::this_thread::sleep_for(std::chrono::milliseconds(50));

            createWindow();
            rclcpp::Rate rate(30);
            while (rclcpp::ok())
            {
                rate.sleep();
                RenderGUI();
            }
        });
#else
        createWindow();
        renderTimer = create_wall_timer(std::chrono::milliseconds(30), std::bind(&VoxelandServer::RenderGUI, this));
#endif
    }

    void VoxelandServer::RenderGUI()
    {
        std::scoped_lock<std::mutex> lock(debugging_utils::mutex);
        ImguiGL::StartFrame();
        ImGui::DockSpaceOverViewport();

        // list of checkboxes for object viz
        ImGui::Begin("Object Visualization");
        if (modeHas(DataMode::SemanticsInstances))
            AUTO_TEMPLATE_INSTANCES_ONLY(currentMode, SelectObjectsAndDraw<DataT>(););
        ImGui::End();

        // selected point to query voxel info
        GetQueryPoint();

        // print info for a single instance
        PrintInstanceInfo();

        // pause button
        PauseButton();

        // local instances in most recent observation
        ImGui::Begin("Local Geometry");
        if (modeHas(voxeland::DataMode::SemanticsInstances))
            AUTO_TEMPLATE_INSTANCES_ONLY(currentMode, ShowObservationPointCloud<DataT>());
        ImGui::End();

        PrintCategoriesList();

        ImguiGL::Render();
    }

    template <typename DataT>
    void VoxelandServer::SelectObjectsAndDraw()
    {
        if (semantics.globalSemanticMap.size() == 0)
        {
            ImGui::Text("There are no instances to show");
            return;
        }

        static bool viewUnderSegmentationScore = false;
        ImGui::Checkbox("View under-segmentation score", &viewUnderSegmentationScore);

        static bool enableByDefault = false;
        ImGui::Checkbox("Enable new instances automatically", &enableByDefault);
        ImGui::VerticalSpace(20.f);

        size_t numBeforeResize = globalObjectsToDraw.size();
        globalObjectsToDraw.resize(semantics.globalSemanticMap.size());
        if (enableByDefault)
        {
            for (size_t i = numBeforeResize; i < globalObjectsToDraw.size(); i++)
                globalObjectsToDraw.at(i) = true;
        }

        if (ImGui::Button("Toggle all"))
        {
            static bool set_to = false;
            set_to = !set_to;

            for (size_t i = 0; i < globalObjectsToDraw.size(); i++)
                if (i != 0)  // skip the background instance, it slows things down quite a bit
                    globalObjectsToDraw.at(i) = set_to;
        }

        for (size_t i = 0; i < semantics.globalSemanticMap.size(); i++)
        {
            if (semantics.globalSemanticMap.at(i).isStillValid())
            {
                ImGui::Checkbox(fmt::format("Object_{}", i).c_str(), (bool*)&globalObjectsToDraw[i]);
                ImGui::SameLine();
                ImGui::Text("%s", fmt::format("- {}", CategoryManager::getInstance().getCategoryName(semantics.globalSemanticMap.at(i).mostLikelyCategory())).c_str());
            }
            else
                globalObjectsToDraw[i] = false;
        }

        // render the selected instances
        {
            // add all the voxels in each of the selected instances to a pcl message, with the color and instanceID
            pcl::PointCloud<pcl::PointXYZRGBSemantics> pcl_cloud;
            auto add_point_to_pcl = [&](DataT& data, const Bonxai::Point3D& point) {
                InstanceID_t instanceID = data.getMostRepresentativeInstance();
                const SemanticObject& instance = semantics.globalSemanticMap.at(instanceID);
                voxeland::Color visualization_color;
                if (viewUnderSegmentationScore)
                    visualization_color = voxeland::valueToColor(instance.underSegmentScore / instance.numberObservations, 0, 1);
                else
                    visualization_color = data.toColor();
                std::uint32_t rgb = voxeland::serializeColor(visualization_color);
                pcl_cloud.emplace_back((float)point.x, (float)point.y, (float)point.z, *reinterpret_cast<float*>(&rgb), instanceID);
            };

            bool drawAny = std::any_of(globalObjectsToDraw.begin(), globalObjectsToDraw.end(), std::identity());
            if (drawAny)
            {
                std::vector<DataT> cell_data;
                std::vector<Bonxai::Point3D> cell_points;
                bonxai_->With<DataT>()->getOccupiedVoxels(cell_points, cell_data);
                for (size_t i = 0; i < cell_points.size(); i++)
                {
                    const auto& point = cell_points[i];

                    InstanceID_t instance = cell_data.at(i).getMostRepresentativeInstance();
                    if (globalObjectsToDraw[instance] && point.z >= occupancy_min_z_ && point.z <= occupancy_max_z_)
                    {
                        add_point_to_pcl(cell_data.at(i), point);
                    }
                }
            }

            PointCloud2 cloud;
            pcl::toROSMsg(pcl_cloud, cloud);

            cloud.header.frame_id = world_frame_id_;
            cloud.header.stamp = now();
            debugInstancesPub->publish(cloud);
        }

        // render segmentation clustering
        {
            static auto clusterPub = create_publisher<PointCloud2>("clusteringRefinement", 1);
            PointCloud2 cloud;
            cloud.data.clear();
            pcl::PointCloud<pcl::PointXYZRGBSemantics> pcl_cloud;
            if (showClusters)
            {
                for (size_t i = 0; i < semantics.debugInfo.mostRecentClusters.size(); i++)
                {
                    for (const auto& indices : semantics.debugInfo.mostRecentClusters.at(i))
                    {
                        const Bonxai::Point3D point = bonxai_->coordToPos(indices);

                        std::uint32_t rgb = semantics.indexToHexColor(i);
                        pcl_cloud.emplace_back((float)point.x, (float)point.y, (float)point.z, *reinterpret_cast<float*>(&rgb), -1);
                    }
                }
            }
            pcl::toROSMsg(pcl_cloud, cloud);
            cloud.header.frame_id = world_frame_id_;
            cloud.header.stamp = now();
            clusterPub->publish(cloud);
        }
    }

    void VoxelandServer::GetQueryPoint()
    {
        ImGui::Begin("Selected point");
        // this horrible thing is necessary because imgui does not let me work with doubles :(
        {
            float x = selectedCoordinates.x;
            float y = selectedCoordinates.y;
            float z = selectedCoordinates.z;
            ImGui::DragFloat("X", &x, 0.01f);
            ImGui::DragFloat("Y", &y, 0.01f);
            ImGui::DragFloat("Z", &z, 0.01f);
            selectedCoordinates.x = x;
            selectedCoordinates.y = y;
            selectedCoordinates.z = z;
        }

        static rclcpp::Publisher<Marker>::SharedPtr pub = create_publisher<Marker>("/voxeland/UIQueryPoint", 1);
        Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = now();
        marker.pose.position.x = selectedCoordinates.x;
        marker.pose.position.y = selectedCoordinates.y;
        marker.pose.position.z = selectedCoordinates.z;
        marker.type = Marker::SPHERE;
        marker.color.r = 1;
        marker.color.a = 1;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        pub->publish(marker);

        if (currentMode != voxeland::DataMode::Uninitialized)
            AUTO_TEMPLATE(currentMode, PrintVoxelInfo<DataT>(selectedCoordinates));
        ImGui::End();
    }

    template <typename DataT>
    void VoxelandServer::PrintVoxelInfo(const Bonxai::Point3D& point)
    {
        const Bonxai::IndicesT coord = bonxai_->posToCoord(point);
        Bonxai::ProbabilisticCell<DataT>* cell = BonxaiQuery<DataT>::getAccessor().value(coord);
        if (!cell)
        {
            ImGui::Text("No voxel has been created at position (%.2f, %.2f, %.2f)", point.x, point.y, point.z);
            return;
        }

        ImGui::Text("Voxel (%d, %d, %d):\n%s\n%s",
                    coord.x,
                    coord.y,
                    coord.z,
                    fmt::format("Probability occupied: {:.2f}", Bonxai::prob(cell->probability_log)).c_str(),
                    GetVoxelDescription(cell->data).c_str());
    }

    void VoxelandServer::PrintInstanceInfo()
    {
        ImGui::Begin("Instance Info");
        if (ImGui::Button("Trigger Global Refinement"))
            functionQueue.submit([&]() { doGlobalRefinement(); });

        if (semantics.globalSemanticMap.size() == 0)
        {
            ImGui::Text("There are no instances to show");
            ImGui::End();
            return;
        }
        static int itemSelectedIdx = 0;  // Here we store our selection data as an index.

        // sometimes instances disappear and we don't want a segfault
        if (itemSelectedIdx >= semantics.globalSemanticMap.size())
            itemSelectedIdx = 0;

        const SemanticObject& object = semantics.globalSemanticMap.at(itemSelectedIdx);

        if (ImGui::BeginCombo("Selected Instance", object.instanceName.c_str()))
        {
            for (size_t i = 0; i < semantics.globalSemanticMap.size(); i++)
            {
                if (!semantics.globalSemanticMap.at(i).isStillValid())
                    continue;
                if (ImGui::Selectable(semantics.globalSemanticMap.at(i).instanceName.c_str()))
                    itemSelectedIdx = i;
            }
            ImGui::EndCombo();
        }

        ImGui::Text("Alphas dirichlet:");
        ImGui::Indent(20.f);
        for (const auto& [categoryID, alpha] : object.alphaParamsCategories)
        {
            ImGui::Text("%s: %f",
                        CategoryManager::getInstance().getCategoryName(categoryID).c_str(),
                        alpha);
        }
        ImGui::Unindent(20.f);

        ImGui::Text("Num observations: %d", object.numberObservations);
        ImGui::Text("Under-segment score: %.2f", object.underSegmentScore);

        ImGui::End();
    }

    void VoxelandServer::PauseButton()
    {
        ImGui::Begin("PauseButton");
#if SEPARATE_RENDER_THREAD
        ImGui::Checkbox("Pause on integration", &debugging_utils::pause_on_integration);
        ImGui::Checkbox("Pause on fusion", &debugging_utils::pause_on_fusion);
        ImGui::Checkbox("Pause on splitting", &debugging_utils::pause_on_splitting);
        ImGui::SameLine();
        ImGui::Checkbox("Show clusters", &showClusters);
#endif
        if (debugging_utils::debug_paused)
        {
            paused = true;
            if (ImGui::Button("Continue Thread"))
            {
                debugging_utils::debug_paused = false;
                paused = false;
            }
        }
        else
        {
            std::string label = paused ? "Continue" : "Pause";
            if (ImGui::Button(label.c_str()))
                paused = !paused;
        }

        ImGui::End();
    }

    template <typename DataT>
    void VoxelandServer::ShowObservationPointCloud()
    {
        if (!paused)
        {
            ImGui::Text("Local geometry visualization is only possible while paused");
            return;
        }

        localObjectsToDraw.resize(semantics.lastLocalSemanticMap.size());

        if (ImGui::Button("Toggle all"))
        {
            static bool set_to = false;
            set_to = !set_to;

            for (size_t i = 0; i < localObjectsToDraw.size(); i++)
                localObjectsToDraw.at(i) = set_to;
        }

        for (size_t i = 0; i < localObjectsToDraw.size(); i++)
        {
            ImGui::Checkbox(fmt::format("Object_{}", i).c_str(), (bool*)&localObjectsToDraw[i]);
            ImGui::SameLine();
            ImGui::Text("%s", fmt::format("- {}", CategoryManager::getInstance().getCategoryName(semantics.lastLocalSemanticMap.at(i).mostLikelyCategory())).c_str());
        }

        using PointCloudType = typename DataT::PointCloudType;
        PointCloudType in_pc;
        pcl::fromROSMsg(mostRecentPointCloud->cloud, in_pc);

        pcl::PointCloud<pcl::PointXYZRGBSemantics> out_pcl;
        for (size_t i = 0; i < localObjectsToDraw.size(); i++)
        {
            if (localObjectsToDraw.at(i))
            {
                if (!semantics.lastLocalSemanticMap.at(i).localGeometry)
                {
                    ImGui::ScopedStyle textstyle(ImGuiCol_Text, ImColors::ErrorText);
                    ImGui::Text("Local object %zu has no localGeometry!", i);
                    continue;
                }

                for (const Bonxai::IndicesT& coord : semantics.lastLocalSemanticMap.at(i).localGeometry.value())
                {
                    const Bonxai::Point3D point = bonxai_->coordToPos(coord);
                    uint32_t rgb = semantics.indexToHexColor(i);
                    rgb &= 0x0000ffff;  // force the red channel to 0 to make the local geometry more visually distinct from the global one
                    out_pcl.emplace_back(point.x, point.y, point.z, *reinterpret_cast<float*>(&rgb), i);
                }
            }
        }

        PointCloud2 cloud;
        pcl::toROSMsg(out_pcl, cloud);

        cloud.header.frame_id = world_frame_id_;
        cloud.header.stamp = now();
        debugInputPub->publish(cloud);
    }

    void VoxelandServer::PrintCategoriesList()
    {
        ImGui::Begin("List of Categories");

        if (ImGui::CollapsingHeader("CatergoryManager"))
        {
            ImGui::TreePush("CatMan");
            CategoryManager& catMan = CategoryManager::getInstance();
            for (size_t i = 0; i < catMan.getNumCategories(); i++)
            {
                ImGui::Text("%zu - %s", i, catMan.getCategoryName(i).c_str());
            }
            ImGui::TreePop();
        }

        ImGui::End();
    }

}  // namespace voxeland_server
#endif
