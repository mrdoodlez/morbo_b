#pragma once

#include <cstdint>
#include <opencv2/core.hpp>

enum ParamPage
{
    ParamPage_Vodom,
};

struct ParamsView
{
    const void *ptr;  // points to an immutable struct in RAM
    size_t size;      // sizeof(the struct)
    uint32_t version; // increments on each update of this page
};


int Controller_LoadParams();

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
};
