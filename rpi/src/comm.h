#pragma once

#include <chrono>

void Comm_Start(int comm);

int WaitForAck(uint16_t cmdIdWaitingForAck, std::chrono::milliseconds timeout, bool& isAck);