#include <thread>
#include "host_interface_cmds.h"
#include "host_interface.h"
#include "controller.h"
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <iostream>
#include <map>
#include "logger.h"

using HIP_CmdHandler = void(*)(const HIP_Cmd_t*);

static std::map<uint16_t, HIP_CmdHandler> g_handlTable;

extern "C" void _Host_NewCommand(const HIP_Cmd_t* cmd)
{
    HIP_CmdHandler handler = nullptr;

    // vlog.text << __func__ << ' ' << cmd->header.cmd << std::endl;

    auto it = g_handlTable.find(cmd->header.cmd);
    if (it != g_handlTable.end())
        handler = it->second;

    if (handler)
    {
        handler(cmd);
    }
    else
    {
        ControllerMsg hstMsg;
        hstMsg.ts_ms = Controller_NowMs();
        hstMsg.type = ControllerMsg::Type::TYPE_HOST;
        hstMsg.payload.rovData = *cmd;

        Controller_PostMessage(hstMsg);
    }
}

void Host_Start(int comm)
{
    std::thread([comm] {
        HostIface_Listen(comm, _Host_NewCommand);
    }).detach();
}