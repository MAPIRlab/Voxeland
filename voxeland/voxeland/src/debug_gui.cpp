#include <imgui_gl/imgui_gl.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <imgui_gl/utils.hpp>
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
        ImguiGL::Setup(
            fmt::format("{}/resources/debug_gui.ini", ament_index_cpp::get_package_share_directory("voxeland")).c_str(),
            "voxeland_gui",
            800,
            900);
        renderTimer = create_wall_timer(std::chrono::milliseconds(30), std::bind(&VoxelandServer::RenderGUI, this));
        debugInstancesPub = create_publisher<PointCloud2>("/voxeland/debugInstances", 1);
        debugInputPub = create_publisher<PointCloud2>("/voxeland/debugInput", 1);

        clickedPointSub = create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 1, [this](const geometry_msgs::msg::PointStamped::SharedPtr point) {
                selectedCoordinates.x = point->point.x;
                selectedCoordinates.y = point->point.y;
                selectedCoordinates.z = point->point.z;
            });
    }

    void VoxelandServer::RenderGUI()
    {
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

        if (ImGui::Button("Toggle all"))
        {
            static bool set_to = false;
            set_to = !set_to;

            for (size_t i = 0; i < globalObjectsToDraw.size(); i++)
                if (i != 0)  // skip the background instance, it slows things down quite a bit
                    globalObjectsToDraw.at(i) = set_to;
        }

        globalObjectsToDraw.resize(semantics.globalSemanticMap.size());
        for (size_t i = 0; i < semantics.globalSemanticMap.size(); i++)
        {
            if (semantics.globalSemanticMap.at(i).isStillValid())
            {
                ImGui::Checkbox(fmt::format("Object_{}", i).c_str(), (bool*)&globalObjectsToDraw[i]);
                ImGui::SameLine();
                ImGui::Text("%s", fmt::format("- {}", semantics.default_categories.at(semantics.globalSemanticMap.at(i).mostLikelyCategory())).c_str());
            }
            else
                globalObjectsToDraw[i] = false;
        }

        // add all the voxels in each of the selected instances to a pcl message, with the color and instanceID
        pcl::PointCloud<pcl::PointXYZRGBSemantics> pcl_cloud;
        for (size_t i = 0; i < semantics.globalSemanticMap.size(); i++)
        {
            if (!globalObjectsToDraw[i])
                continue;

            std::vector<Bonxai::CoordT> coords;
            coords = semantics.listOfVoxelsInObject<DataT>(semantics.globalSemanticMap.at(i));

            for (size_t i = 0; i < coords.size(); i++)
            {
                const auto& coord = coords.at(i);
                const Bonxai::Point3D point = bonxai_->coordToPos(coord);

                Bonxai::ProbabilisticCell<DataT>* cell = SemanticMap::BonxaiQuery<DataT>::getAccessor().value(coord);

                if (cell->probability_log > bonxai_->options().occupancy_threshold_log  //
                    && point.z >= occupancy_min_z_ && point.z <= occupancy_max_z_)
                {
                    voxeland::Color visualization_color = cell->data.toColor();
                    std::uint32_t rgb = voxeland::serializeColor(visualization_color);
                    InstanceID_t instanceID = cell->data.getMostRepresentativeInstance();
                    pcl_cloud.emplace_back((float)point.x, (float)point.y, (float)point.z, *reinterpret_cast<float*>(&rgb), instanceID);
                }
            }
        }

        PointCloud2 cloud;
        pcl::toROSMsg(pcl_cloud, cloud);

        cloud.header.frame_id = world_frame_id_;
        cloud.header.stamp = now();
        debugInstancesPub->publish(cloud);
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
        const Bonxai::CoordT coord = bonxai_->posToCoord(point);
        Bonxai::ProbabilisticCell<DataT>* cell = SemanticMap::BonxaiQuery<DataT>::getAccessor().value(coord);
        if (!cell)
        {
            ImGui::Text("No voxel has been created at position (%.2f, %.2f, %.2f)", point.x, point.y, point.z);
            return;
        }

        ImGui::Text("Voxel (%d, %d, %d):\n%s\n%s", 
            coord.x, coord.y, coord.z, 
            fmt::format("Probability occupied: {:.2f}", Bonxai::prob(cell->probability_log)).c_str(),
            GetVoxelDescription(cell->data).c_str());
    }

    void VoxelandServer::PrintInstanceInfo()
    {
        ImGui::Begin("Instance Info");
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

        if (ImGui::BeginCombo("Selected Instance", semantics.globalSemanticMap.at(itemSelectedIdx).instanceName.c_str()))
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
        for (size_t i = 0; i < semantics.default_categories.size(); i++)
        {
            if (semantics.globalSemanticMap.at(itemSelectedIdx).alphaParamsCategories.at(i) > 0)
            {
                ImGui::Text("%s: %f",
                            semantics.default_categories.at(i).c_str(),
                            semantics.globalSemanticMap.at(itemSelectedIdx).alphaParamsCategories.at(i));
            }
        }

        ImGui::End();
    }

    void VoxelandServer::PauseButton()
    {
        ImGui::Begin("PauseButton");
        std::string label = paused ? "Continue" : "Pause";
        if (ImGui::Button(label.c_str()))
            paused = !paused;
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
            ImGui::Text("%s", fmt::format("- {}", semantics.default_categories.at(semantics.lastLocalSemanticMap.at(i).mostLikelyCategory())).c_str());
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

                for (const Bonxai::CoordT& coord : semantics.lastLocalSemanticMap.at(i).localGeometry.value())
                {
                    const Bonxai::Point3D point = bonxai_->coordToPos(coord);
                    out_pcl.emplace_back(point.x, point.y, point.z, semantics.indexToHexColor(i), i);
                }
            }
        }

        PointCloud2 cloud;
        pcl::toROSMsg(out_pcl, cloud);

        cloud.header.frame_id = world_frame_id_;
        cloud.header.stamp = now();
        debugInputPub->publish(cloud);
    }

}  // namespace voxeland_server