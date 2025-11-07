#pragma once

#include <string>
#include <cstdint>

struct VodomMsg
{
    uint64_t t_us; // capture timestamp (monotonic/us)

    enum class VodomStatus : uint8_t {
        DETECTED = 0,
        TARGET_LOST = 1
    } status;

    float dx_m; // base_link
    float dy_m;
    float range_m;     // hypot(dx,dy)
    float bearing_rad; // atan2(dy,dx)
    float d_px;        // apparent diameter in pixels
    float quality;     // 0..1
};

int Vodom_Start(const std::string &videoDev);