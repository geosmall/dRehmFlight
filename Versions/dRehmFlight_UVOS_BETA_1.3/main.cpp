#include "uvos_brd.h"

/** Declare setup() and loop() as external symbols
 *  so that they can be called from the main function
 *  to simulate Arduino behavior.
 */

/** This prevents us from having to type "uvos::" in front of a lot of things. */
using namespace uvos;

/** Publish global Hardware access */
extern UVOSboard hw;
extern void setup();
extern void loop();

uint8_t sumbuff[1024];

void UsbCallback(uint8_t* buf, uint32_t* len)
{
    for(size_t i = 0; i < *len; i++)
    {
        sumbuff[i] = buf[i];
    }
}

int main(void)
{
    /** Initialize the board hardware */
    hw.Init();
    hw.usb_handle.Init(UsbHandle::FS_INTERNAL);
    hw.usb_handle.SetReceiveCallback(UsbCallback, UsbHandle::FS_INTERNAL);

    setup();

    for (;;) {
        loop();
    }

    return 0;
}