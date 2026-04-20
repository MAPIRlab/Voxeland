#pragma once
#include <cstdlib>
#include <set>
#include <vector>

namespace Utils
{
    inline bool approx(double a, double b, double epsilon = 1e-5)
    {
        return std::abs(a - b) < epsilon;
    }

    template <typename Collection, typename T>
    inline std::set<T> AsSet(const Collection& collection)
    {
        std::set<T> set;
        std::copy(collection.begin(), collection.end(), std::inserter(set, set.begin()));
        return set;
    }

    template <typename T>
    inline void Normalize(std::vector<T>& vec)
    {
        float sum = 0;
        for (size_t i = 0; i < vec.size(); i++)
            sum += vec.at(i);

        for (size_t i = 0; i < vec.size(); i++)
            vec.at(i) /= sum;
    }
}  // namespace Utils