#pragma once
#include <chrono>

struct Stopwatch
{
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::_V2::system_clock::duration Duration;
    typedef std::chrono::_V2::system_clock::time_point TimePoint;
    Clock clock;
    TimePoint start;

    Stopwatch() : clock(), start(clock.now())
    {
    }

    double ellapsed()
    {
        return toSeconds(clock.now() - start);
    }

    void restart()
    {
        start = clock.now();
    }
    
    double toSeconds(Duration duration)
    {
        constexpr double nanoToSec = 1e-9;
        return duration.count() * nanoToSec ;
    }
};