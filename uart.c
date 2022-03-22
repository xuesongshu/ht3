#include "uart.h"

#include "cyfx3uart.h"
#include "cyfx3device.h"

void uart_init()
{
    CyFx3BootUartConfig_t uart_config;
    CyFx3BootErrorCode_t status;

    status = CyFx3BootUartInit();
    if (status != CY_FX3_BOOT_SUCCESS) {
        CyFx3BootDeviceReset();
        return;
    }
    uart_config.txEnable = CyTrue;
    uart_config.rxEnable = CyFalse;
    uart_config.flowCtrl = CyFalse;
    uart_config.isDma = CyFalse;
    uart_config.baudRate = CY_FX3_BOOT_UART_BAUDRATE_115200;
    uart_config.stopBit = CY_FX3_BOOT_UART_ONE_STOP_BIT;
    uart_config.parity = CY_FX3_BOOT_UART_NO_PARITY;
    status = CyFx3BootUartSetConfig(&uart_config);
    if (status != CY_FX3_BOOT_SUCCESS) {
        CyFx3BootDeviceReset();
        return;
    }
    CyFx3BootUartPrintMessage((uint8_t *)UART_ADDRESS, 256, "uart init.");
}