#pragma once

#include "host_interface_cmds.h"
#include "vodom.h"
#include <cstring>
#include <string>

struct ControllerMsgPayload
{
    union
    {
        VodomMsg vodom;
        HIP_Cmd_t rovData;
        uint8_t raw[128];
    };

    void clear() { std::memset(this, 0, sizeof(*this)); }
};

struct ControllerMsg
{
    uint64_t ts_ms;

    enum Type : uint16_t
    {
        TYPE_VODOM,
        TYPE_ROVER,
        TYPE_HOST,
    } type;

    ControllerMsgPayload payload;
};

int Controller_Start();

int Controller_PostMessage(const ControllerMsg &m);

uint64_t Controller_NowMs();