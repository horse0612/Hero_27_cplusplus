#include "LED.hpp"

#include "libxr.hpp"
#include "ws2812.hpp"
#include "detect.hpp"
extern "C"
{
#include "app_main.h"
#include "cmsis_os.h"
}

extern "C" void OS_LedCallback(void *argument)
{
    (void)argument;
    while (App_IsInitialized() == 0)
    {
        LibXR::Thread::Sleep(1);
    }
    for (;;)
    {
        blue_led.SetBlue(64);
        LibXR::Thread::Sleep(500);
        blue_led.Off();
        LibXR::Thread::Sleep(500);
    }
}
