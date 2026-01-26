#include <algorithm>
#include <stdexcept>
#include <voxeland_map/category_manager.hpp>

CategoryManager::CategoryManager()
{
    reset();
}

CategoryManager& CategoryManager::getInstance()
{
    static CategoryManager instance;
    return instance;
}

CategoryManager::CategoryIndex CategoryManager::addCategory(const std::string& categoryName)
{
    // Check if category already exists
    auto it = categoryToIndex_.find(categoryName);
    if (it != categoryToIndex_.end())
        return it->second;

    // Add new category
    addCategoryInternal(categoryName);
    newCategories_.insert(categoryName);

    return categories_.size() - 1;
}

void CategoryManager::addCategoryInternal(const std::string& categoryName)
{
    CategoryIndex newIndex = categories_.size();
    categories_.push_back(categoryName);
    categoryToIndex_[categoryName] = newIndex;
}

CategoryManager::CategoryIndex CategoryManager::getCategoryIndex(const std::string& categoryName) const
{
    auto it = categoryToIndex_.find(categoryName);
    if (it != categoryToIndex_.end())
        return it->second;

    return INVALID_CATEGORY;
}

std::string CategoryManager::getCategoryName(CategoryIndex index) const
{
    if (index < categories_.size())
        return categories_[index];

    return "";
}

std::vector<std::string> CategoryManager::getAllCategories() const
{
    return categories_;
}

size_t CategoryManager::getNumCategories() const
{
    return categories_.size();
}

bool CategoryManager::hasCategory(const std::string& categoryName) const
{
    return categoryToIndex_.find(categoryName) != categoryToIndex_.end();
}

void CategoryManager::initializeWithCategories(const std::vector<std::string>& defaultCategories)
{
    if (initialized_)
        throw std::runtime_error("CategoryManager already initialized. Use reset() first if needed.");

    // Add all default categories (unknown is already added in constructor)
    for (const auto& category : defaultCategories)
        if (category != "unknown")
            addCategoryInternal(category);

    initialized_ = true;
}

void CategoryManager::reset()
{
    categories_.clear();
    categoryToIndex_.clear();
    newCategories_.clear();
    initialized_ = false;

    // Re-add default categories
    addCategoryInternal("unknown");
}

std::vector<std::string> CategoryManager::getNewCategories()
{
    std::vector<std::string> result(newCategories_.begin(), newCategories_.end());
    newCategories_.clear();

    return result;
}

bool CategoryManager::hasNewCategories() const
{
    return !newCategories_.empty();
}