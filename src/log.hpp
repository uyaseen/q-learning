#ifndef _LOGH_
#define _LOGH_

#include <cstdio>

#define LOG(...) static_cast<void>(std::printf(__VA_ARGS__))
#define ERROR(...) static_cast<void>(std::fprintf(stderr, __VA_ARGS__))

#endif
