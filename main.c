//#include "cyfx3usb.h"
//#include "cyfx3device.h"
//#include "cyfx3utils.h"
//#include "cyfx3gpio.h"

#include "cyu3system.h"
#include "cyu3usb.h"
#include "cyu3os.h"
#include "cyu3error.h"
#include "cyu3uart.h"
#include "cyu3utils.h"
#include "stdint.h"

#include "descr.h"
#include "usb_boot.h"
#include "uart.h"
#include "app.h"

extern CyU3PThread g_app_thread;
CyBool_t g_is_app_active = CyFalse;

void CyFxApplicationDefine(void)
{
    void *ptr = NULL;
    uint32_t iret = CY_U3P_SUCCESS;
    ptr = CyU3PMemAlloc(THREAD_STACK);
    iret = CyU3PThreadCreate(&g_app_thread,
                             "ht3",
                             app_thread_entry,
                             0,
                             ptr,
                             THREAD_STACK,
                             THREAD_PRIORITY,
                             THREAD_PRIORITY,
                             CYU3P_NO_TIME_SLICE,
                             CYU3P_AUTO_START
                            );
    if (iret != 0) {
        while (1);
    }
}

int main()
{
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    status = CyU3PDeviceInit(0);
    if (status != CY_U3P_SUCCESS) {
        goto handle_fatal_error;
    }
    status = CyU3PDeviceCacheControl(CyTrue, CyFalse, CyFalse);
    if (status != CY_U3P_SUCCESS) {
        goto handle_fatal_error;
    }
    CyU3PMemSet((uint8_t *)&io_cfg, 0, sizeof(io_cfg));
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.useUart = CyTrue;
    io_cfg.useI2C = CyFalse;
    io_cfg.useI2S = CyFalse;
    io_cfg.useSpi = CyFalse;
    io_cfg.lppMode = CY_U3P_IO_MATRIX_LPP_UART_ONLY;
    io_cfg.gpioSimpleEn[0] = 0;
    io_cfg.gpioSimpleEn[1] = 0;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    status = CyU3PDeviceConfigureIOMatrix(&io_cfg);
    if (status != CY_U3P_SUCCESS) {
        goto handle_fatal_error;
    }
    CyU3PKernelEntry();
    return 0;
handle_fatal_error:
    while (1);
}
