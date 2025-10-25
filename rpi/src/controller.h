#pragma once

#include "host_interface_cmds.h"
#include <string>

struct ControllerParams
{
    std::string videoDev;
};

int Controller_Start(const ControllerParams& params);