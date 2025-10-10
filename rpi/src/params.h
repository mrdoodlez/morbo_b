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

int Controller_GetParams(ParamPage page, ParamsView *out_view);

/*******************************************************************************/

struct VodomParams
{
    // Image
    int W = 640, H = 480, FPS = 30;

    // Target geometry
    float disc_D_m; // physical diameter, meters

    // Intrinsics (from YAML)
    cv::Mat K; // 3x3, CV_64F
    cv::Mat D; // 1x5 or 1x8, CV_64F
    // cached
    double fx = 0, fy = 0, cx = 0, cy = 0;

    // Extrinsics base<-camera (from YAML)
    cv::Mat Rbc; // 3x3, CV_64F
    cv::Mat tbc; // 3x1, CV_64F

    // HSV thresholds (red is split into two bands)
    // Red HSV (two bands, 0..180 hue)
    // band A ~ [0..10], band B ~ [170..180]
    int H1a = 0, H2a = 10, SminA = 90, VminA = 80;
    int H1b = 170, H2b = 180, SminB = 90, VminB = 80;

    // Morphology
    int morph_open = 3;
    int morph_close = 5;

    // Detection thresholds
    int dpx_min = 12;
    int dpx_max = 300;
    float max_axis_ratio = 1.20f;
    float min_color_fill = 0.65f;

    // Temporal filtering
    int confirm_hits = 2;
    int forget_miss = 3;

    // ROI
    float roi_expand = 2.0f;
};