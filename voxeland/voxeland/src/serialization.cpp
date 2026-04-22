
#include <happly.hpp>
#include <voxeland_server.hpp>

namespace voxeland_server
{
    // TODO change the serialization to use happly rather than raw strings
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

    template <typename DataT>
    void VoxelandServer::mapFromPLY(const std::filesystem::path& plyPath)
    {
        happly::PLYData ply(plyPath);

        std::vector<float> x = ply.getElement("vertex").getProperty<float>("x");
        std::vector<float> y = ply.getElement("vertex").getProperty<float>("y");
        std::vector<float> z = ply.getElement("vertex").getProperty<float>("z");

        std::vector<int> instanceid = ply.getElement("vertex").getProperty<int>("instanceid");
        std::vector<float> uncertainty_instances = ply.getElement("vertex").getProperty<float>("uncertainty_instances");
        std::vector<float> uncertainty_categories = ply.getElement("vertex").getProperty<float>("uncertainty_categories");

        for (size_t i = 0; i < x.size(); i++)
        {
            Bonxai::IndicesT indices = bonxai_->posToIndex({ x.at(i), y.at(i), z.at(i) });
            Bonxai::ProbabilisticCell<DataT>* cell = BonxaiQuery<DataT>::getAccessor().value(indices, true);
            cell->probability_log = 10000;

            // create a single vote for the winning instance
            InstanceID_t id = instanceid.at(i);
            if (!semantics.globalSemanticMap.contains(id))
            {
                VXL_WARN("Ignoring voxel at ({:.2f},{:.2f},{:.2f}) with instance id {}, which is not in the instance map.", x.at(i), y.at(i), z.at(i), id);
                cell->data.instances_candidates.push_back(0);
                cell->data.instances_votes.push_back(1);
            }
            else
            {
                // TODO can we somehow account for the uncertainty? Probably not, since we are not storing all the votes
                cell->data.instances_candidates.push_back(id);
                cell->data.instances_votes.push_back(1);
            }
        }
    }

}  // namespace voxeland_server
