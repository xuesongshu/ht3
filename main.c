#include "cyfx3usb.h"
#include "cyfx3device.h"
#include "cyfx3utils.h"
#include "cyfx3gpio.h"

#include "descr.h"
#include "usb_boot.h"
#include "uart.h"

int main()
{
    CyFx3BootErrorCode_t status;
    CyFx3BootIoMatrixConfig_t io_cfg;
    // CyFx3BootGpioSimpleConfig_t gpio_cfg;

    CyFx3BootDeviceInit(CyFalse);
    CyFx3BootMemSet((uint8_t *)&io_cfg, 0, sizeof(CyFx3BootIoMatrixConfig_t));
    //io_cfg.isDQ32Bit = CyFalse;
    // io_cfg.useUart = CyTrue;
    io_cfg.useI2C = CyTrue;
    // io_cfg.useI2S = CyFalse;
    // io_cfg.useSpi = CyFalse;
    // io_cfg.gpioSimpleEn[0] = 0;
    // io_cfg.gpioSimpleEn[1] = 0;
    status = CyFx3BootDeviceConfigureIOMatrix(&io_cfg);
    // if (status != CY_FX3_BOOT_SUCCESS) {
    //     CyFx3BootDeviceReset();
    //     return status;
    // }
    // CyFx3BootGpioOverride(GPIO_OUT_ID);
    // CyFx3BootGpioInit();
    // gpio_cfg.outValue = CyFalse;
    // gpio_cfg.driveLowEn = CyTrue;
    // gpio_cfg.driveHighEn = CyTrue;
    // gpio_cfg.inputEn = CyFalse;
    // gpio_cfg.intrMode = CY_FX3_BOOT_GPIO_NO_INTR;
    // status = CyFx3BootGpioSetSimpleConfig(GPIO_OUT_ID, &gpio_cfg);
    // if (status != CY_FX3_BOOT_SUCCESS) {
    //     CyFx3BootDeviceReset();
    //     return status;
    // }
    // CyFx3BootRetainGpioState();
    // uart_init();
    // usb_boot();
    // while (1) {
    //     CyFx3BootUsbHandleEvents();
    // }
    (void)status;
    CyFx3BootDeviceReset();
    return 0;
}
