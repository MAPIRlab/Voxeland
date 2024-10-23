#include <cstdlib>
namespace Utils
{
    inline bool approx(double a, double b, double epsilon = 1e-5)
    {
        return std::abs(a - b) < epsilon;
    }
}  // namespace Utils