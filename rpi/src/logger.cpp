#include "logger.h"
#include <unordered_map>
#include <filesystem>
#include <mutex>
#include <atomic>
#include <iostream>
#include <vector>

namespace
{
    struct Counters
    {
        std::unordered_map<std::string, std::atomic<int>> map;
        std::mutex mtx;
    };
    Counters counters;
    const std::string kOutDir = "vodom_dbg";
}

Logger::Logger()
{
    std::error_code ec;
    std::filesystem::create_directories(kOutDir, ec);
    if (ec)
    {
        std::cerr << "[Logger] Failed to create " << kOutDir
                  << ": " << ec.message() << std::endl;
        return;
    }

    try
    {
        for (auto &entry : std::filesystem::directory_iterator(kOutDir))
        {
            if (entry.is_regular_file())
            {
                std::filesystem::remove(entry.path());
            }
        }
        std::cout << "[Logger] Purged old files in " << kOutDir << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Logger] Purge failed: " << e.what() << std::endl;
    }
}

Logger &Logger::Get()
{
    static Logger instance; // guaranteed thread-safe in C++11+
    return instance;
}

void Logger::Write(const std::string &logName, const cv::Mat &mat)
{
    if (mat.empty())
        return;

    int n;
    {
        std::scoped_lock lock(counters.mtx);
        auto &c = counters.map[logName];
        n = c++;
    }

    char fname[256];
    std::snprintf(fname, sizeof(fname), "%s/%03d_%s.jpg",
                  kOutDir.c_str(), n, logName.c_str());

    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 95};
    try
    {
        cv::imwrite(fname, mat, params);
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "[Logger] Failed to write " << fname
                  << ": " << e.what() << std::endl;
    }
}
