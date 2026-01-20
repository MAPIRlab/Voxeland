#include <imgui_gl/imgui_gl.h>

#include <voxeland_server.hpp>

namespace voxeland_server
{
    void VoxelandServer::SetupGUI()
    {
        ImguiGL::Setup(nullptr, "voxeland_gui");
        renderTimer = create_wall_timer(std::chrono::milliseconds(30), std::bind(&VoxelandServer::RenderGUI, this));
        debugMarkersPub = create_publisher<PointCloud2>("/voxeland/debug", 1);
    }

    void VoxelandServer::RenderGUI()
    {
        ImguiGL::StartFrame();

        // use whatever ImGui calls you want here, directly
        ImguiGL::SetNextWindowFullscreen();
        ImGui::Begin("Frame");
        if (modeHas(DataMode::SemanticsInstances))
            AUTO_TEMPLATE_INSTANCES_ONLY(currentMode, SelectObjectsAndDraw<DataT>(););
        ImGui::End();

        ImguiGL::Render();
    }

    template <typename DataT>
    void VoxelandServer::SelectObjectsAndDraw()
    {
        globalObjectsToDraw.resize(semantics.globalSemanticMap.size());
        for (size_t i = 0; i < semantics.globalSemanticMap.size(); i++)
        {
            if (semantics.globalSemanticMap.at(i).isStillValid())
                ImGui::Checkbox(fmt::format("Object_{}", i).c_str(), (bool*)&globalObjectsToDraw[i]);
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
}  // namespace voxeland_server