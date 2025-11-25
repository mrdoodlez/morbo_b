#pragma once

#include <cstdint>
#include <opencv2/core.hpp>
#include <string>

enum ParamPage
{
    ParamPage_Vodom,
    ParamPage_System,
    ParamPage_Mcu,
};

struct ParamsView
{
    const void *ptr;  // points to an immutable struct in RAM
    size_t size;      // sizeof(the struct)
    uint32_t version; // increments on each update of this page
};


int Controller_LoadParams(const std::string &config_path);

int Controller_GetParams(ParamPage page, ParamsView *out_view);

/*******************************************************************************/

struct VodomParams
{
    // Image
    int W = 640, H = 480, FPS = 30;

    float tag_size_m = 0.08f;

    // Intrinsics
    cv::Mat K;
    cv::Mat D;
    double fx = 0, fy = 0, cx = 0, cy = 0;

    // Extrinsics base<-camera
    cv::Mat Rbc;
    cv::Mat tbc;

    // Temporal filtering
    int confirm_hits = 2;
    int forget_miss = 3;

    // ROI
    float roi_expand = 2.0f;

    // Device
    std::string camera_dev = "/dev/video0";
};

struct SystemParams
{
    std::string mcu_dev = "/dev/ttyUSB0";
    std::string host_dev = "/dev/ttyUSB1";
};

struct McuParams
{
    // Dummy PID coeffs for now
    double angle_kp = 1.0, angle_ki = 0.1, angle_kd = 0.01;
    double rate_kp  = 0.8, rate_ki  = 0.05, rate_kd  = 0.005;
    double pos_kp   = 0.5, pos_ki   = 0.0,  pos_kd   = 0.0;
};