#include "cyfx3device.h"

/**
 * @brief reset device into bootloader mode for updating SPI flash
 *
 */
void reset_into_bootloader()
{
    CyFx3BootDeviceReset();
}