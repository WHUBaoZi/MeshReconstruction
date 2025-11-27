#pragma once

#include "AlgoDebugConfig.h"
#include <string>
#include <iostream>
#include <unordered_map>
#include <boost/filesystem/operations.hpp>


#ifdef ENABLE_ALGO_DEBUG

extern std::string GAlgoDebugOutputDir;

extern std::chrono::steady_clock::time_point start;
extern std::chrono::steady_clock::time_point end;
extern std::unordered_map<std::string, long long> timings;

inline void SetAlgoDebugOutputDir(const std::string& dir)
{
    GAlgoDebugOutputDir = dir;

#ifdef ENABLE_ALGO_DEBUG
    namespace fs = boost::filesystem;
    if (GAlgoDebugOutputDir.empty())
        return;

    fs::path p(GAlgoDebugOutputDir);

    if (!fs::exists(p))
        fs::create_directories(p);

    const std::vector<std::string> subdirs = {
        "RansacResults",
        "RansacResults/Planes",
        "SegmentationResults/IndivisiblePolyhedrons/Useful",
        "SegmentationResults/IndivisiblePolyhedrons/Useless",
        "SegmentationResults/Progress",
        "RemeshResults",
        "TempOutput"
    };

    for (const auto& sub : subdirs)
    {
        fs::path subPath = p / sub;

        if (fs::exists(subPath))
        {
            boost::system::error_code ec;
            fs::remove_all(subPath, ec);
            if (ec)
            {
                std::cerr << "[Warning] Failed to delete: "
                    << subPath.string() << " : " << ec.message()
                    << std::endl;
                continue;
            }
            }
        fs::create_directories(subPath);
        }
#endif
}

inline long long GetTotalTime()
{
    long long time = 0;
    for (const auto& pair : timings)
    {
        time += pair.second;
    }
    return time;
}

#else

inline void SetAlgoDebugOutputDir(const std::string&) {}

inline void GetTotalTime() {}

#endif