#include <iostream>
#include "serial.h"
#include "controller.h"
#include <iostream>
#include <thread>
#include <chrono>

int main()
{
    std::cout << "vpos starts\n";
    int rc = 0;
    if ((rc = Serial_Init("/dev/ttyUSB0")) != 0)
    {
        std::cout << "serial open error: " << rc << std::endl;
        return -10;
    }

    ControllerParams params;
    if ((rc = Controller_Start(params)) != 0)
    {
        std::cout << "controller start error: " << rc << std::endl;
        return -20;
    }

    for (;;)
    {
        // do nothing — keep process alive
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
