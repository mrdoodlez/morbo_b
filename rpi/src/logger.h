#pragma once

#include <opencv2/opencv.hpp>
#include <string>

class Logger
{
public:
    static Logger &Get();

    void Write(const std::string &logName, const cv::Mat &mat);

private:
    Logger(); // private constructor

    ~Logger() = default;

    Logger(const Logger &) = delete;

    Logger &operator=(const Logger &) = delete;

    Logger(Logger &&) = delete;

    Logger &operator=(Logger &&) = delete;
};

#define vlog (Logger::Get())
