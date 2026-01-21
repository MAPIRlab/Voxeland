#include <imgui_gl/imgui_gl.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <voxeland_server.hpp>

using visualization_msgs::msg::Marker;
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
        debugMarkersPub = create_publisher<PointCloud2>("/voxeland/debug", 1);

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

        ImGui::Begin("Object Visualization");
        if (modeHas(DataMode::SemanticsInstances))
            AUTO_TEMPLATE_INSTANCES_ONLY(currentMode, SelectObjectsAndDraw<DataT>(););
        ImGui::End();

        GetQueryPoint();
        PrintInstanceInfo();

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

        ImGui::Text("Selected items: ");

        // add all the voxels in each of the selected instances to a pcl message, with the color and instanceID
        pcl::PointCloud<pcl::PointXYZRGBSemantics> pcl_cloud;
        for (size_t i = 0; i < semantics.globalSemanticMap.size(); i++)
        {
            if (!globalObjectsToDraw[i])
                continue;
            ImGui::Text("\t%s", fmt::format("{}\n", i).c_str());

            std::vector<Bonxai::CoordT> coords;
            coords = semantics.listOfVoxelsInObject<DataT>(semantics.globalSemanticMap.at(i));

            for (size_t i = 0; i < coords.size(); i++)
            {
                const auto& coord = coords.at(i);
                const Bonxai::Point3D point = bonxai_->coordToPos(coord);

                Bonxai::ProbabilisticCell<DataT>* cell = SemanticMap::BonxaiQuery<DataT>::getAccessor().value(coord);

                if (point.z >= occupancy_min_z_ && point.z <= occupancy_max_z_)
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
        debugMarkersPub->publish(cloud);
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

        ImGui::Text("Voxel (%d, %d, %d):\n%s", coord.x, coord.y, coord.z, GetVoxelDescription(cell->data).c_str());
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

}  // namespace voxeland_server