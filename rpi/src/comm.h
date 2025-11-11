#pragma once

#include <chrono>

void Comm_Start(int comm);

int WaitForAck(uint16_t cmdIdWaitingForAck, std::chrono::milliseconds timeout, bool& isAck);

#ifdef __cplusplus
extern "C"
{
#endif

void Controller_NewCommand(const HIP_Cmd_t* cmd);

#ifdef __cplusplus
}
#endif