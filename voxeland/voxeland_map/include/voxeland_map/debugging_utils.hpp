#pragma once

#include <mutex>

namespace debugging_utils
{
    inline volatile bool debug_paused = false;
    inline bool pause_on_integration = false;
    inline bool pause_on_fusion = false;
    inline bool pause_on_splitting = false;

    inline std::mutex mutex;
}  // namespace debugging_utils

#if ENABLE_DEBUG_GUI
#define PAUSE_THREAD_UNTIL_GUI_CONTINUE(condition)                      \
    if (condition)                                                      \
    {                                                                   \
        fprintf(stderr, "THREAD PAUSED\n");                             \
        debugging_utils::debug_paused = true;                           \
        debugging_utils::mutex.unlock();                                \
        while (debugging_utils::debug_paused)                           \
            std::this_thread::sleep_for(std::chrono::milliseconds(15)); \
        debugging_utils::mutex.lock();                                  \
        fprintf(stderr, "THREAD EXECUTION RESUMED\n");                  \
    }
#else
#define PAUSE_THREAD_UNTIL_GUI_CONTINUE(condition)
#endif