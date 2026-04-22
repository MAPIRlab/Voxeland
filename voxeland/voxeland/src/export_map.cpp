
#include <voxeland_server.hpp>

namespace voxeland_server
{
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

        for (const auto& [id, instance] : semantics.globalSemanticMap)
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
                if (instance.instanceName.length() > 3 && instance.instanceName.substr(0, 3) == "obj")
                {
                    instance_id = std::stoi(instance.instanceName.substr(3));
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
                                    center_x,
                                    center_y,
                                    center_z,
                                    static_cast<int>(r),
                                    static_cast<int>(g),
                                    static_cast<int>(b),
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

            // Get color from visualization (this calls updateCandidatesAndVotes() internally)
            voxeland::Color viz_color = cell_data[i].toColor();

            // Get instance ID using the same logic as toColor()/visualization:
            // - background demotion (prefers real instance over background when close)
            // - instance fusion redirect (follows pointsTo chain)
            // Note: updateCandidatesAndVotes() was already called by toColor() above
            InstanceID_t instanceID = cell_data[i].getMostRepresentativeInstance();

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
                for (const auto& [id, instance] : semantics.globalSemanticMap)
                {
                    if (instance.pointsTo != -1)
                        continue;

                    int instance_numeric_id = 0;
                    try
                    {
                        if (instance.instanceName.length() > 3 && instance.instanceName.substr(0, 3) == "obj")
                        {
                            instance_numeric_id = std::stoi(instance.instanceName.substr(3));
                        }
                    }
                    catch (...)
                    {
                        continue;
                    }

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
                                    voxel.x,
                                    voxel.y,
                                    voxel.z,
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

            for (const auto& [id, instance] : semantics.globalSemanticMap)
            {
                if (instance.pointsTo != -1)
                    continue;

                int instance_id = 0;
                try
                {
                    if (instance.instanceName.length() > 3 && instance.instanceName.substr(0, 3) == "obj")
                    {
                        instance_id = std::stoi(instance.instanceName.substr(3));
                    }
                    else
                    {
                        continue;
                    }
                }
                catch (const std::exception& e)
                {
                    VXL_WARN("Failed to parse instance ID {}: {}", instance.instanceID, e.what());
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
                                 categoryIndex,
                                 instance.instanceID,
                                 e.what());
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
                    bbox_json["center"] = { 0.0, 0.0, 0.0 };
                    bbox_json["size"] = { 0.1, 0.1, 0.1 };

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
                bbox_json["center"] = { centerX, centerY, centerZ };
                bbox_json["size"] = { sizeX, sizeY, sizeZ };

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

            // Save fusion history if enabled
            // if (semantics.enableFusionHistory && !semantics.getFusionHistory().empty())
            // {
            //     std::filesystem::path ply_p(output_ply_path_);
            //     std::string fusion_history_path = (ply_p.parent_path() / (std::string("voxeland_fusion_history_") + detector_name_ + "_" + scene_name_ + ".txt")).string();

            //     std::ofstream fusion_outfile(fusion_history_path);
            //     if (fusion_outfile.is_open())
            //     {
            //         fusion_outfile << semantics.fusionHistoryToString();
            //         fusion_outfile.close();
            //         VXL_INFO("Saved fusion history ({} fusions) to {}", semantics.getFusionHistory().size(), fusion_history_path);
            //     }
            //     else
            //     {
            //         VXL_ERROR("Failed to save fusion history: {}", fusion_history_path);
            //     }
            // }
        }
    }
}  // namespace voxeland_server
