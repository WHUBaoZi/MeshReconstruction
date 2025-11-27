#include "AlgoDebugIO.h"

#ifdef ENABLE_ALGO_DEBUG
std::string GAlgoDebugOutputDir = "./DebugOutput";

std::unordered_map<std::string, int64_t> timings;
std::chrono::time_point<std::chrono::steady_clock> start;
std::chrono::time_point<std::chrono::steady_clock> end;
#endif