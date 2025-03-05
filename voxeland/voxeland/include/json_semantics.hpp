
#include <sys/types.h>
#include <map>
#include <string>
#include <vector>
#include "voxeland_map/semantics.hpp"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

struct JsonSemanticObject{
    std::string InstanceID;
    BoundingBox3D bbox;
    uint32_t n_observations;
    std::map<std::string, double> results;
};

class JsonSemanticMap{
    public:
        static JsonSemanticMap load_map(const std::string& json_file);
        JsonSemanticObject* get_instance(const std::string& instanceID);
        std::vector<JsonSemanticObject>* get_instances();
        const std::string to_string();
    protected:
        std::vector<JsonSemanticObject> instances;
    private:
        static BoundingBox3D parse_bbox(json& bbox);
        static std::map<std::string, double> parse_results(json& results);
};