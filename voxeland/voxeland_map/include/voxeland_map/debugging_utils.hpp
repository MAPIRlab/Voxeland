#pragma once

#include <mutex>

namespace debugging_utils
{
    inline bool debug_paused = false;
    inline bool debug_paused_enabled = false;

    inline std::mutex mutex;
}  // namespace debugging_utils

#if ENABLE_DEBUG_GUI
#define PAUSE_THREAD_UNTIL_GUI_CONTINUE                \
    if (debugging_utils::debug_paused_enabled)         \
    {                                                  \
        debugging_utils::mutex.unlock();               \
        fprintf(stderr, "THREAD PAUSED\n");            \
        debugging_utils::debug_paused = true;          \
        while (debugging_utils::debug_paused)          \
            ;                                          \
        debugging_utils::mutex.lock();                 \
        fprintf(stderr, "THREAD EXECUTION RESUMED\n"); \
    }
#else
#define PAUSE_THREAD_UNTIL_GUI_CONTINUE
#endif