#include "system.h"

void System_Reset()
{
    NVIC_SystemReset();
}