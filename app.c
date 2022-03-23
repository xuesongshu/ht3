#include "app.h"

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3uart.h"
#include "cyu3gpif.h"
#include "cyu3pib.h"
#include "cyu3utils.h"
#include "pib_regs.h"
#include <cyu3gpio.h>

/* This file should be included only once as it contains
 * structure definitions. Including it in multiple places
 * can result in linker error. */
#include "cyfxgpif_syncsf.h"

CyU3PThread g_app_thread;            /* Slave FIFO application thread structure */
CyU3PDmaChannel g_dma_u2p;   /* DMA Channel handle for U2P transfer. */
CyU3PDmaChannel g_dma_p2u;   /* DMA Channel handle for P2U transfer. */

uint32_t g_dma_rx_count = 0;               /* Counter to track the number of buffers received from USB. */
uint32_t g_dma_tx_count = 0;               /* Counter to track the number of buffers sent to USB. */
CyBool_t g_is_app_active = CyFalse;      /* Whether the loopback application is active or not. */

/* Application Error Handler */
void
app_error_handler(
    CyU3PReturnStatus_t apiRetStatus    /* API return status */
)
{
    /* Application failed with the error code apiRetStatus */
    /* Add custom debug or recovery actions here */
    /* Loop Indefinitely */
    for (;;) {
        /* Thread sleep : 100 ms */
        CyU3PThreadSleep(100);
    }
}

/* This function initializes the debug module. The debug prints
 * are routed to the UART and can be seen using a UART console
 * running at 115200 baud rate. */
void
app_debug_init(void)
{
    CyU3PUartConfig_t uartConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    /* Initialize the UART for printing debug messages */
    apiRetStatus = CyU3PUartInit();
    if (apiRetStatus != CY_U3P_SUCCESS) {
        /* Error handling */
        app_error_handler(apiRetStatus);
    }
    /* Set UART configuration */
    CyU3PMemSet((uint8_t *)&uartConfig, 0, sizeof(uartConfig));
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity = CY_U3P_UART_NO_PARITY;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyFalse;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.isDma = CyTrue;
    apiRetStatus = CyU3PUartSetConfig(&uartConfig, NULL);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        app_error_handler(apiRetStatus);
    }
    /* Set the UART transfer to a really large value. */
    apiRetStatus = CyU3PUartTxSetBlockXfer(0xFFFFFFFF);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        app_error_handler(apiRetStatus);
    }
    /* Initialize the debug module. */
    apiRetStatus = CyU3PDebugInit(CY_U3P_LPP_SOCKET_UART_CONS, 8);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        app_error_handler(apiRetStatus);
    }
}

/* DMA callback function to handle the produce events for U to P transfers. */
void
u2p_dma_callback(
    CyU3PDmaChannel   *chHandle,
    CyU3PDmaCbType_t  type,
    CyU3PDmaCBInput_t *input
)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    if (type == CY_U3P_DMA_CB_PROD_EVENT) {
        status = CyU3PDmaChannelCommitBuffer(chHandle, input->buffer_p.count, 0);
        if (status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint(4, "CyU3PDmaChannelCommitBuffer failed, Error code = %d\n", status);
        }
        g_dma_rx_count++;
    }
}

/* DMA callback function to handle the produce events for P to U transfers. */
void
p2u_dma_callback(
    CyU3PDmaChannel   *chHandle,
    CyU3PDmaCbType_t  type,
    CyU3PDmaCBInput_t *input
)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    if (type == CY_U3P_DMA_CB_PROD_EVENT) {
        status = CyU3PDmaChannelCommitBuffer(chHandle, input->buffer_p.count, 0);
        if (status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint(4, "CyU3PDmaChannelCommitBuffer failed, Error code = %d\n", status);
        }
        /* Increment the counter. */
        g_dma_tx_count++;
    }
}

/* This function starts the slave FIFO loop application. This is called
 * when a SET_CONF event is received from the USB host. The endpoints
 * are configured and the DMA pipe is setup in this function. */
void
app_start(
    void)
{
    uint16_t size = 0;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();
    /* First identify the usb speed. Once that is identified,
     * create a DMA channel and start the transfer on this. */
    /* Based on the Bus Speed configure the endpoint packet size */
    switch (usbSpeed) {
    case CY_U3P_FULL_SPEED:
        size = 64;
        break;
    case CY_U3P_HIGH_SPEED:
        size = 512;
        break;
    case  CY_U3P_SUPER_SPEED:
        size = 1024;
        break;
    default:
        CyU3PDebugPrint(4, "Error! Invalid USB speed.\n");
        app_error_handler(CY_U3P_ERROR_FAILURE);
        break;
    }
    CyU3PMemSet((uint8_t *)&epCfg, 0, sizeof(epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.burstLen = 1;
    epCfg.streams = 0;
    epCfg.pcktSize = size;
    /* Producer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* Consumer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* Create a DMA MANUAL channel for U2P transfer.
     * DMA size is set based on the USB speed. */
    dmaCfg.size  = size;
    dmaCfg.count = CY_FX_SLFIFO_DMA_BUF_COUNT;
    dmaCfg.prodSckId = CY_FX_PRODUCER_USB_SOCKET;
    dmaCfg.consSckId = CY_FX_CONSUMER_PPORT_SOCKET;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    /* Enabling the callback for produce event. */
    dmaCfg.notification = CY_U3P_DMA_CB_PROD_EVENT;
    dmaCfg.cb = u2p_dma_callback;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;
    apiRetStatus = CyU3PDmaChannelCreate(&g_dma_u2p,
                                         CY_U3P_DMA_TYPE_MANUAL, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PDmaChannelCreate failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* Create a DMA MANUAL channel for P2U transfer. */
    dmaCfg.prodSckId = CY_FX_PRODUCER_PPORT_SOCKET;
    dmaCfg.consSckId = CY_FX_CONSUMER_USB_SOCKET;
    dmaCfg.cb = p2u_dma_callback;
    apiRetStatus = CyU3PDmaChannelCreate(&g_dma_p2u,
                                         CY_U3P_DMA_TYPE_MANUAL, &dmaCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PDmaChannelCreate failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* Flush the Endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
    /* Set DMA channel transfer size. */
    apiRetStatus = CyU3PDmaChannelSetXfer(&g_dma_u2p, CY_FX_SLFIFO_DMA_TX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    apiRetStatus = CyU3PDmaChannelSetXfer(&g_dma_p2u, CY_FX_SLFIFO_DMA_RX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* Update the status flag. */
    g_is_app_active = CyTrue;
    CyU3PGpioSetValue(59, CyFalse);
}

/* This function stops the slave FIFO loop application. This shall be called
 * whenever a RESET or DISCONNECT event is received from the USB host. The
 * endpoints are disabled and the DMA pipe is destroyed by this function. */
void
app_stop(
    void)
{
    CyU3PEpConfig_t epCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    /* Update the flag. */
    g_is_app_active = CyFalse;
    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
    /* Destroy the channel */
    CyU3PDmaChannelDestroy(&g_dma_u2p);
    CyU3PDmaChannelDestroy(&g_dma_p2u);
    /* Disable endpoints. */
    CyU3PMemSet((uint8_t *)&epCfg, 0, sizeof(epCfg));
    epCfg.enable = CyFalse;
    /* Producer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* Consumer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
}

/* Callback to handle the USB setup requests. */
CyBool_t
usb_setup_cb(
    uint32_t setupdat0,
    uint32_t setupdat1
)
{
    uint8_t  bRequest, bReqType;
    uint8_t  bType, bTarget;
    uint16_t wValue, wIndex;
    CyBool_t isHandled = CyFalse;

    bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
    bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
    bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
    if (bType == CY_U3P_USB_STANDARD_RQT) {
        if ((bTarget == CY_U3P_USB_TARGET_INTF) &&
                ((bRequest == CY_U3P_USB_SC_SET_FEATURE) ||
                 (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) &&
                (wValue == 0)) {
            if (g_is_app_active)
                CyU3PUsbAckSetup();
            else
                CyU3PUsbStall(0, CyTrue, CyFalse);
            isHandled = CyTrue;
        }
        if ((bTarget == CY_U3P_USB_TARGET_ENDPT) && (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)
                && (wValue == CY_U3P_USBX_FS_EP_HALT)) {
            if (g_is_app_active) {
                if (wIndex == CY_FX_EP_PRODUCER) {
                    CyU3PUsbSetEpNak(CY_FX_EP_PRODUCER, CyTrue);
                    CyU3PBusyWait(125);
                    CyU3PDmaChannelReset(&g_dma_u2p);
                    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
                    CyU3PUsbResetEp(CY_FX_EP_PRODUCER);
                    CyU3PDmaChannelSetXfer(&g_dma_u2p, CY_FX_SLFIFO_DMA_TX_SIZE);
                    CyU3PUsbSetEpNak(CY_FX_EP_PRODUCER, CyFalse);
                }
                if (wIndex == CY_FX_EP_CONSUMER) {
                    CyU3PUsbSetEpNak(CY_FX_EP_PRODUCER, CyTrue);
                    CyU3PBusyWait(125);
                    CyU3PDmaChannelReset(&g_dma_p2u);
                    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
                    CyU3PUsbResetEp(CY_FX_EP_CONSUMER);
                    CyU3PDmaChannelSetXfer(&g_dma_p2u, CY_FX_SLFIFO_DMA_RX_SIZE);
                    CyU3PUsbSetEpNak(CY_FX_EP_PRODUCER, CyFalse);
                }
                CyU3PUsbStall(wIndex, CyFalse, CyTrue);
                CyU3PUsbAckSetup();
                isHandled = CyTrue;
            }
        }
    }
    return isHandled;
}

/* This is the callback function to handle the USB events. */
void
usb_event_cb(
    CyU3PUsbEventType_t evtype,
    uint16_t            evdata
)
{
    switch (evtype) {
    case CY_U3P_USB_EVENT_SETCONF:

        CyU3PUsbLPMDisable();

        if (g_is_app_active) {
            app_stop();
        }

        app_start();
        break;
    case CY_U3P_USB_EVENT_RESET:
    case CY_U3P_USB_EVENT_DISCONNECT:

        if (g_is_app_active) {
            app_stop();
        }
        break;
    default:
        break;
    }
}

CyBool_t
app_lpm_rqt_cb(
    CyU3PUsbLinkPowerMode link_mode)
{
    return CyTrue;
}

void
app_init(void)
{
    CyU3PPibClock_t pibClock;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PGpioClock_t gpioClock;
    CyU3PGpioSimpleConfig_t gpioConfig;
    /* Initialize the p-port block. */
    pibClock.clkDiv = 2;
    pibClock.clkSrc = CY_U3P_SYS_CLK;
    pibClock.isHalfDiv = CyFalse;
    /* Disable DLL for sync GPIF */
    pibClock.isDllEnable = CyFalse;
    apiRetStatus = CyU3PPibInit(CyTrue, &pibClock);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "P-port Initialization failed, Error Code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* Load the GPIF configuration for Slave FIFO sync mode. */
    apiRetStatus = CyU3PGpifLoad(&CyFxGpifConfig);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PGpifLoad failed, Error Code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
#if (CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT == 1)
    CyU3PGpifSocketConfigure(0, CY_U3P_PIB_SOCKET_0, 6, CyFalse, 1);
    CyU3PGpifSocketConfigure(3, CY_U3P_PIB_SOCKET_3, 6, CyFalse, 1);
#else
    CyU3PGpifSocketConfigure(0, CY_U3P_PIB_SOCKET_0, 3, CyFalse, 1);
    CyU3PGpifSocketConfigure(3, CY_U3P_PIB_SOCKET_3, 3, CyFalse, 1);
#endif
    /* Start the state machine. */
    apiRetStatus = CyU3PGpifSMStart(RESET, ALPHA_RESET);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PGpifSMStart failed, Error Code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* Init the GPIO module */
    gpioClock.fastClkDiv = 2;
    gpioClock.slowClkDiv = 0;
    gpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    gpioClock.clkSrc = CY_U3P_SYS_CLK;
    gpioClock.halfDiv = 0;
    apiRetStatus = CyU3PGpioInit(&gpioClock, NULL);
    if (apiRetStatus != 0) {
        /* Error Handling */
        CyU3PDebugPrint(4, "CyU3PGpioInit failed, error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* Configure GPIO 59 as output */
    gpioConfig.outValue = CyTrue;
    gpioConfig.driveLowEn = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.inputEn = CyFalse;
    gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
    apiRetStatus = CyU3PGpioSetSimpleConfig(59, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        /* Error handling */
        CyU3PDebugPrint(4, "CyU3PGpioSetSimpleConfig failed, error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* Start the USB functionality. */
    apiRetStatus = CyU3PUsbStart();
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PUsbStart failed to Start, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }

    CyU3PUsbRegisterSetupCallback(usb_setup_cb, CyTrue);
    CyU3PUsbRegisterEventCallback(usb_event_cb);
    CyU3PUsbRegisterLPMRequestCallback(app_lpm_rqt_cb);

    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB30DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set device descriptor failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* High speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set device descriptor failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* BOS descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)CyFxUSBBOSDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set configuration descriptor failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* Device qualifier descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set device qualifier descriptor failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* Super speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set configuration descriptor failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* High speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB Set Other Speed Descriptor failed, Error Code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* Full speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB Set Configuration Descriptor failed, Error Code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* String descriptor 0 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* String descriptor 1 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* String descriptor 2 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
    /* Connect the USB Pins with super speed operation enabled. */
    apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB Connect failed, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }
}

/* Entry function for the g_app_thread. */
void
app_thread_entry(
    uint32_t input)
{
    /* Initialize the debug module */
    app_debug_init();
    /* Initialize the slave FIFO application */
    app_init();
    for (;;) {
        CyU3PThreadSleep(1000);
        if (g_is_app_active) {
            /* Print the number of buffers received so far from the USB host. */
            CyU3PDebugPrint(6, "Data tracker: buffers received: %d, buffers sent: %d.\n",
                            g_dma_rx_count, g_dma_tx_count);
        }
    }
}

/* Application define function which creates the threads. */
void
CyFxApplicationDefine(
    void)
{
    void *ptr = NULL;
    uint32_t retThrdCreate = CY_U3P_SUCCESS;
    /* Allocate the memory for the thread */
    ptr = CyU3PMemAlloc(CY_FX_SLFIFO_THREAD_STACK);
    /* Create the thread for the application */
    retThrdCreate = CyU3PThreadCreate(&g_app_thread,            /* Slave FIFO app thread structure */
                                      "Slave FIFO Synchronous",                    /* Thread ID and thread name */
                                      app_thread_entry,                   /* Slave FIFO app thread entry function */
                                      0,                                       /* No input parameter to thread */
                                      ptr,                                     /* Pointer to the allocated thread stack */
                                      CY_FX_SLFIFO_THREAD_STACK,               /* App Thread stack size */
                                      CY_FX_SLFIFO_THREAD_PRIORITY,            /* App Thread priority */
                                      CY_FX_SLFIFO_THREAD_PRIORITY,            /* App Thread pre-emption threshold */
                                      CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
                                      CYU3P_AUTO_START                         /* Start the thread immediately */
                                     );
    /* Check the return code */
    if (retThrdCreate != 0) {
        while (1);
    }
}

/*
 * Main function
 */
int
main(void)
{
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PSysClockConfig_t clockConfig;

#if (CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT == 0)
    clockConfig.setSysClk400  = CyFalse;
#else
    clockConfig.setSysClk400  = CyTrue;
#endif
    clockConfig.cpuClkDiv     = 2;
    clockConfig.dmaClkDiv     = 2;
    clockConfig.mmioClkDiv    = 2;
    clockConfig.useStandbyClk = CyFalse;
    clockConfig.clkSrc        = CY_U3P_SYS_CLK;
    status = CyU3PDeviceInit(&clockConfig);
    if (status != CY_U3P_SUCCESS) {
        goto handle_fatal_error;
    }
    /* Initialize the caches. Enable both Instruction and Data Caches. */
    status = CyU3PDeviceCacheControl(CyTrue, CyTrue, CyTrue);
    if (status != CY_U3P_SUCCESS) {
        goto handle_fatal_error;
    }

    io_cfg.s0Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.s1Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.useUart   = CyTrue;
    io_cfg.useI2C    = CyFalse;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyFalse;
#if (CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT == 0)
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_UART_ONLY;
#else
    io_cfg.isDQ32Bit = CyTrue;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
#endif
    /* No GPIOs are enabled. */
    io_cfg.gpioSimpleEn[0]  = 0;
    io_cfg.gpioSimpleEn[1]  = 0x08000000;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    status = CyU3PDeviceConfigureIOMatrix(&io_cfg);
    if (status != CY_U3P_SUCCESS) {
        goto handle_fatal_error;
    }
    /* This is a non returnable call for initializing the RTOS kernel */
    CyU3PKernelEntry();
    /* Dummy return to make the compiler happy */
    return 0;
handle_fatal_error:
    /* Cannot recover from this error. */
    while (1);
}

/* [ ] */

