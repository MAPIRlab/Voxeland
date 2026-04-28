#pragma once
#include <cmath>
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

    // vector must be a normalized distribution
    template <typename T>
    inline float Shannon_entropy(const std::vector<T>& probabilities)
    {
        float sum = 0;
        for (size_t i = 0; i < probabilities.size(); i++)
        {
            sum -= probabilities.at(i) * std::log(probabilities.at(i));
        }
        return sum;
    }
}  // namespace Utils