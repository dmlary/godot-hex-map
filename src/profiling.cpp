#include "profiling.h"

#ifdef __APPLE__

os_log_t profiling_os_log = os_log_create("org.godot.gdextension.hexmap",
        OS_LOG_CATEGORY_POINTS_OF_INTEREST);

#endif // __APPLE__
