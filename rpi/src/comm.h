#pragma once

#include <chrono>

void Comm_Start(int mcu, int hst);

int WaitForAck(uint16_t cmdIdWaitingForAck, std::chrono::milliseconds timeout, bool& isAck);