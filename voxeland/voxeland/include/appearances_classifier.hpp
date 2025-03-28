#include <cstdint>
#include "json_semantics.hpp"

class AppearancesClassifier{
    public:
        AppearancesClassifier(uint32_t max_appearances) : max_appearances(max_appearances) {}
        virtual void classify_instance_appearances(UncertainInstance& instance) = 0;
    protected:
        std::vector<std::string> choose_selected_categories(std::map<std::string, double>& results); // The n_categories with the highest probabilities
        int32_t max_appearances;
        int32_t n_categories = 2;
};

class RandomAppearancesClassifier : public AppearancesClassifier{
    public:
        RandomAppearancesClassifier(uint32_t max_appearances) : AppearancesClassifier(max_appearances) {}
        void classify_instance_appearances(UncertainInstance& instance) override;
};

class SplitAppearancesClassifier : public AppearancesClassifier{
    public:
        SplitAppearancesClassifier(uint32_t max_appearances) : AppearancesClassifier(max_appearances) {}
        void classify_instance_appearances(UncertainInstance& instance) override;
};