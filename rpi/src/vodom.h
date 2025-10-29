#pragma once

#include <string>

typedef struct
{
    uint64_t t_us; // capture timestamp (monotonic/us)
    enum VodomStatus
    {
        VODOM_NOT_DETECTED = 0,
        VODOM_DETECTED_RED = 1,
        VODOM_DETECTED_GREEN = 2
    } status;

    float dx_m; // base_link
    float dy_m;
    float range_m;     // hypot(dx,dy)
    float bearing_rad; // atan2(dy,dx)
    float d_px;        // apparent diameter in pixels
    float quality;     // 0..1
} VodomMsg;

int Vodom_Start(const std::string &videoDev);