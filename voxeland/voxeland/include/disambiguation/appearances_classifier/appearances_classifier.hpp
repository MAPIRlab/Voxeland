#include <cstdint>
#include <string>
#include "disambiguation/json_semantics.hpp"

class AppearancesClassifier{
    public:
        virtual void classify_instance_appearances(UncertainInstance& instance, std::vector<std::string> categories, uint32_t max_appearances) = 0;
        std::string name;
    protected:
        std::vector<std::string> choose_selected_categories(std::map<std::string, double>& results);
};

class RandomAppearancesClassifier : public AppearancesClassifier{
    public:
        void classify_instance_appearances(UncertainInstance& instance, std::vector<std::string> categories, uint32_t max_appearances) override;
        std::string name = "RANDOM";
};

class SplitAppearancesClassifier : public AppearancesClassifier{
    public:
        void classify_instance_appearances(UncertainInstance& instance, std::vector<std::string> categories, uint32_t max_appearances) override;
        std::string name = "SPLIT";
};