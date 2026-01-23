#include <voxeland_map/category_manager.hpp>
#include <algorithm>
#include <stdexcept>

CategoryManager::CategoryManager()
    : initialized_(false)
{
    // Always initialize with "unknown" and "background" as default categories
    categories_.reserve(100); // Reserve space for efficiency
    addCategoryInternal("unknown");
    addCategoryInternal("background");
}

CategoryManager& CategoryManager::getInstance()
{
    static CategoryManager instance;
    return instance;
}

CategoryManager::CategoryIndex CategoryManager::addCategory(const std::string& categoryName)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Check if category already exists
    auto it = categoryToIndex_.find(categoryName);
    if (it != categoryToIndex_.end())
    {
        return it->second;
    }

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
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = categoryToIndex_.find(categoryName);
    if (it != categoryToIndex_.end())
    {
        return it->second;
    }
    
    return INVALID_CATEGORY;
}

std::string CategoryManager::getCategoryName(CategoryIndex index) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (index < categories_.size())
    {
        return categories_[index];
    }
    
    return "";
}

std::vector<std::string> CategoryManager::getAllCategories() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return categories_;
}

size_t CategoryManager::getNumCategories() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return categories_.size();
}

bool CategoryManager::hasCategory(const std::string& categoryName) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return categoryToIndex_.find(categoryName) != categoryToIndex_.end();
}

void CategoryManager::initializeWithCategories(const std::vector<std::string>& defaultCategories)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (initialized_)
    {
        throw std::runtime_error("CategoryManager already initialized. Use reset() first if needed.");
    }
    
    // Add all default categories (unknown and background are already added in constructor)
    for (const auto& category : defaultCategories)
    {
        if (category != "unknown" && category != "background")
        {
            addCategoryInternal(category);
        }
    }
    
    initialized_ = true;
}

void CategoryManager::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    categories_.clear();
    categoryToIndex_.clear();
    newCategories_.clear();
    initialized_ = false;
    
    // Re-add default categories
    addCategoryInternal("unknown");
    addCategoryInternal("background");
}

std::vector<std::string> CategoryManager::getNewCategories()
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<std::string> result(newCategories_.begin(), newCategories_.end());
    newCategories_.clear();
    
    return result;
}

bool CategoryManager::hasNewCategories() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return !newCategories_.empty();
}