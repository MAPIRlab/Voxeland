#pragma once

#include <cstdint>
#include <string>
#include "disambiguation/json_semantics.hpp"

class AppearancesClassifier{
    public:
        virtual ~AppearancesClassifier() = default;
        virtual void classify_instance_appearances(UncertainInstance& instance, std::vector<std::string> categories, uint32_t max_appearances) = 0;
        virtual std::string get_name() const = 0;
    protected:
        std::vector<std::string> choose_selected_categories(std::map<std::string, double>& results);
};

class RandomAppearancesClassifier : public AppearancesClassifier{
    public:
        void classify_instance_appearances(UncertainInstance& instance, std::vector<std::string> categories, uint32_t max_appearances) override;
        std::string get_name() const override;
    private:
        const std::string name = "random";
};

class SplitAppearancesClassifier : public AppearancesClassifier{
    public:
        void classify_instance_appearances(UncertainInstance& instance, std::vector<std::string> categories, uint32_t max_appearances) override;
        std::string get_name() const override;
    private:    
        const std::string name = "split";
};

class BBoxAppearancesClassifier : public AppearancesClassifier{
    public:
        void classify_instance_appearances(UncertainInstance& instance, std::vector<std::string> categories, uint32_t max_appearances) override;
        std::string get_name() const override;
    private:
        const std::string name = "bbox";
};

class SplitBboxAreaAppearancesClassifier : public AppearancesClassifier{
    public:
        void classify_instance_appearances(UncertainInstance& instance, std::vector<std::string> categories, uint32_t max_appearances) override;
        std::string get_name() const override;
    private:
        const std::string name = "split_bbox";
};