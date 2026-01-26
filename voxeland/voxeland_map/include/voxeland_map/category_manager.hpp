#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <mutex>

/**
 * @brief Dynamic category manager for open vocabulary semantic mapping
 * 
 * This class manages categories dynamically, allowing for open vocabulary
 * semantic mapping instead of being limited to a fixed set of categories.
 */
class CategoryManager
{
public:
    using CategoryIndex = size_t;
    static constexpr CategoryIndex INVALID_CATEGORY = SIZE_MAX;
    static constexpr CategoryIndex UNKNOWN_CATEGORY = 0;
    static constexpr CategoryIndex BACKGROUND_CATEGORY = 1;

    CategoryManager();
    ~CategoryManager() = default;

    // Singleton pattern for global access
    static CategoryManager& getInstance();

    /**
     * @brief Add a new category dynamically
     * @param categoryName Name of the category to add
     * @return Index of the category (existing or newly created)
     */
    CategoryIndex addCategory(const std::string& categoryName);

    /**
     * @brief Get the index of a category
     * @param categoryName Name of the category
     * @return Index of the category, or INVALID_CATEGORY if not found
     */
    CategoryIndex getCategoryIndex(const std::string& categoryName) const;

    /**
     * @brief Get the name of a category by index
     * @param index Index of the category
     * @return Name of the category, or empty string if invalid index
     */
    std::string getCategoryName(CategoryIndex index) const;

    /**
     * @brief Get all categories
     * @return Vector of all category names
     */
    std::vector<std::string> getAllCategories() const;

    /**
     * @brief Get the total number of categories
     * @return Number of categories
     */
    size_t getNumCategories() const;

    /**
     * @brief Check if a category exists
     * @param categoryName Name of the category
     * @return True if category exists, false otherwise
     */
    bool hasCategory(const std::string& categoryName) const;

    /**
     * @brief Initialize with a set of default categories
     * @param defaultCategories Vector of default category names
     */
    void initializeWithCategories(const std::vector<std::string>& defaultCategories);

    /**
     * @brief Reset all categories (useful for testing)
     */
    void reset();

    /**
     * @brief Get categories added since last check
     * @return Vector of newly added category names
     */
    std::vector<std::string> getNewCategories();

    /**
     * @brief Check if new categories have been added since last check
     * @return True if new categories exist
     */
    bool hasNewCategories() const;

private:
    std::vector<std::string> categories_;
    std::unordered_map<std::string, CategoryIndex> categoryToIndex_;
    std::unordered_set<std::string> newCategories_;
    bool initialized_;

    void addCategoryInternal(const std::string& categoryName);
};