#pragma once

#include "godot_cpp/core/defs.hpp"

#ifdef __APPLE__

#include <os/log.h>
#include <os/signpost.h>
#include <cassert>

extern os_log_t profiling_os_log;

class ProfilingSignpost {
	os_log_t log;
	os_signpost_id_t signpost;

public:
	_FORCE_INLINE_ ProfilingSignpost(os_log_t log, os_signpost_id_t signpost) :
			log(log), signpost(signpost) {}
	_FORCE_INLINE_ ~ProfilingSignpost() {
		os_signpost_interval_end(log, signpost, "interval signpost");
	}
};

#define profiling_begin(...)                                                  \
	({                                                                        \
		os_signpost_id_t __signpost_id =                                      \
				os_signpost_id_generate(profiling_os_log);                    \
		assert(__signpost_id != OS_SIGNPOST_ID_INVALID);                      \
		os_signpost_interval_begin(profiling_os_log,                          \
				__signpost_id,                                                \
				"interval signpost",                                          \
				##__VA_ARGS__);                                               \
		ProfilingSignpost(profiling_os_log, __signpost_id);                   \
	})
#define profiling_emit(name, ...)                                             \
	os_signpost_event_emit(profiling_os_log,                                  \
			OS_SIGNPOST_ID_EXCLUSIVE + __COUNTER__,                           \
			name,                                                             \
			##__VA_ARGS__)

#else

#define profiling_begin(...) 0
#define profiling_emit(...)                                                   \
	do {                                                                      \
	} while (0)

#endif // __APPLE__
