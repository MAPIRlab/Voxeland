#pragma once

#if defined TRACY_ENABLE
#include <tracy/Tracy.hpp>
#else

#define ZoneScoped
#define ZoneScopedN(name)

#endif

#include <chrono>
#include <voxeland_map/Utils/logging.hpp>

namespace voxeland
{
    class ScopedStopwatch
    {
        using TimePoint = std::chrono::_V2::system_clock::time_point;

    public:
        [[nodiscard]] ScopedStopwatch(const std::string& _name = "")
        {
            start = clock.now();
            name = _name;
        }

        ~ScopedStopwatch()
        {
            auto nanoseconds = (clock.now() - start).count();
            double seconds = nanoseconds / 1e9;
            VXL_INFO("{} - Ellapsed: {:.3f}s", name.c_str(), seconds);
        }

    private:
        std::chrono::system_clock clock;
        TimePoint start;
        std::string name;
    };
} // namespace gaden