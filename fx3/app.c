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
CyU3PDmaChannel g_dma_u2p;           /* DMA Channel handle for U2P transfer. */
CyU3PDmaChannel g_dma_p2u;           /* DMA Channel handle for P2U transfer. */

uint32_t g_dma_rx_count = 0;         /* Counter to track the number of buffers received from USB. */
uint32_t g_dma_tx_count = 0;         /* Counter to track the number of buffers sent to USB. */
CyBool_t g_is_app_active = CyFalse;  /* Whether the loopback application is active or not. */

/* Application Error Handler */
void app_error_handler(CyU3PReturnStatus_t iapi_ret_status)    /* API return status */
{
    /* Application failed with the error code iapi_ret_status */
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
void app_debug_init(void)
{
    CyU3PUartConfig_t uart_config;
    CyU3PReturnStatus_t iapi_ret_status = CY_U3P_SUCCESS;
    /* Initialize the UART for printing debug messages */
    iapi_ret_status = CyU3PUartInit();
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        /* Error handling */
        app_error_handler(iapi_ret_status);
    }
    /* Set UART configuration */
    CyU3PMemSet((uint8_t *)&uart_config, 0, sizeof(uart_config));
    uart_config.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uart_config.stopBit = CY_U3P_UART_ONE_STOP_BIT;
    uart_config.parity = CY_U3P_UART_NO_PARITY;
    uart_config.txEnable = CyTrue;
    uart_config.rxEnable = CyFalse;
    uart_config.flowCtrl = CyFalse;
    uart_config.isDma = CyTrue;
    iapi_ret_status = CyU3PUartSetConfig(&uart_config, NULL);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret_status);
    }
    /* Set the UART transfer to a really large value. */
    iapi_ret_status = CyU3PUartTxSetBlockXfer(0xFFFFFFFF);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret_status);
    }
    /* Initialize the debug module. */
    iapi_ret_status = CyU3PDebugInit(CY_U3P_LPP_SOCKET_UART_CONS, 8);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret_status);
    }
}

/* DMA callback function to handle the produce events for U to P transfers. */
void u2p_dma_callback(CyU3PDmaChannel *chHandle, CyU3PDmaCbType_t type, CyU3PDmaCBInput_t *input)
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
void p2u_dma_callback(CyU3PDmaChannel *chHandle, CyU3PDmaCbType_t type, CyU3PDmaCBInput_t *input)
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
void app_start(void)
{
    uint16_t size = 0;
    CyU3PEpConfig_t ep_cfg;
    CyU3PDmaChannelConfig_t dma_cfg;
    CyU3PReturnStatus_t iapi_ret_status = CY_U3P_SUCCESS;
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
    CyU3PMemSet((uint8_t *)&ep_cfg, 0, sizeof(ep_cfg));
    ep_cfg.enable = CyTrue;
    ep_cfg.epType = CY_U3P_USB_EP_BULK;
    ep_cfg.burstLen = 1;
    ep_cfg.streams = 0;
    ep_cfg.pcktSize = size;
    /* Producer endpoint configuration */
    iapi_ret_status = CyU3PSetEpConfig(PRODUCER1_EP, &ep_cfg);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* Consumer endpoint configuration */
    iapi_ret_status = CyU3PSetEpConfig(CONSUMER1_EP, &ep_cfg);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* Create a DMA MANUAL channel for U2P transfer.
     * DMA size is set based on the USB speed. */
    dma_cfg.size  = size;
    dma_cfg.count = CY_FX_SLFIFO_DMA_BUF_COUNT;
    dma_cfg.prodSckId = PRODUCER1_SOCKET;
    dma_cfg.consSckId = CONSUMER1_PPORT;
    dma_cfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    /* Enabling the callback for produce event. */
    dma_cfg.notification = CY_U3P_DMA_CB_PROD_EVENT;
    dma_cfg.cb = u2p_dma_callback;
    dma_cfg.prodHeader = 0;
    dma_cfg.prodFooter = 0;
    dma_cfg.consHeader = 0;
    dma_cfg.prodAvailCount = 0;
    iapi_ret_status = CyU3PDmaChannelCreate(&g_dma_u2p, CY_U3P_DMA_TYPE_MANUAL, &dma_cfg);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PDmaChannelCreate failed, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* Create a DMA MANUAL channel for P2U transfer. */
    dma_cfg.prodSckId = PRODUCER1_PPORT;
    dma_cfg.consSckId = CONSUMER1_SOCKET;
    dma_cfg.cb = p2u_dma_callback;
    iapi_ret_status = CyU3PDmaChannelCreate(&g_dma_p2u, CY_U3P_DMA_TYPE_MANUAL, &dma_cfg);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PDmaChannelCreate failed, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* Flush the Endpoint memory */
    CyU3PUsbFlushEp(PRODUCER1_EP);
    CyU3PUsbFlushEp(CONSUMER1_EP);
    /* Set DMA channel transfer size. */
    iapi_ret_status = CyU3PDmaChannelSetXfer(&g_dma_u2p, CY_FX_SLFIFO_DMA_TX_SIZE);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    iapi_ret_status = CyU3PDmaChannelSetXfer(&g_dma_p2u, CY_FX_SLFIFO_DMA_RX_SIZE);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* Update the status flag. */
    g_is_app_active = CyTrue;
    CyU3PGpioSetValue(59, CyFalse);
    CyU3PDebugPrint(4, "app started.");
}

/* This function stops the slave FIFO loop application. This shall be called
 * whenever a RESET or DISCONNECT event is received from the USB host. The
 * endpoints are disabled and the DMA pipe is destroyed by this function. */
void app_stop(void)
{
    CyU3PEpConfig_t ep_cfg;
    CyU3PReturnStatus_t iapi_ret_status = CY_U3P_SUCCESS;
    /* Update the flag. */
    g_is_app_active = CyFalse;
    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(PRODUCER1_EP);
    CyU3PUsbFlushEp(CONSUMER1_EP);
    /* Destroy the channel */
    CyU3PDmaChannelDestroy(&g_dma_u2p);
    CyU3PDmaChannelDestroy(&g_dma_p2u);
    /* Disable endpoints. */
    CyU3PMemSet((uint8_t *)&ep_cfg, 0, sizeof(ep_cfg));
    ep_cfg.enable = CyFalse;
    /* Producer endpoint configuration. */
    iapi_ret_status = CyU3PSetEpConfig(PRODUCER1_EP, &ep_cfg);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* Consumer endpoint configuration. */
    iapi_ret_status = CyU3PSetEpConfig(CONSUMER1_EP, &ep_cfg);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
}

/* Callback to handle the USB setup requests. */
CyBool_t usb_setup_cb(uint32_t setupdat0, uint32_t setupdat1)
{
    uint8_t  irequest, ireq_type;
    uint8_t  itype, itarget;
    uint16_t ivalue, iindex;
    CyBool_t bis_handled = CyFalse;

    CyU3PDebugPrint(6, "usb_setup_cb\n");
    ireq_type = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    itype    = (ireq_type & CY_U3P_USB_TYPE_MASK);
    itarget  = (ireq_type & CY_U3P_USB_TARGET_MASK);
    irequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    ivalue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    iindex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
    if (itype == CY_U3P_USB_STANDARD_RQT) {
        if ((itarget == CY_U3P_USB_TARGET_INTF) &&
                ((irequest == CY_U3P_USB_SC_SET_FEATURE) || (irequest == CY_U3P_USB_SC_CLEAR_FEATURE))
                && (ivalue == 0)) {
            if (g_is_app_active)
                CyU3PUsbAckSetup();
            else
                CyU3PUsbStall(0, CyTrue, CyFalse);
            bis_handled = CyTrue;
        }
        if ((itarget == CY_U3P_USB_TARGET_ENDPT) && (irequest == CY_U3P_USB_SC_CLEAR_FEATURE)
                && (ivalue == CY_U3P_USBX_FS_EP_HALT)) {
            if (g_is_app_active) {
                if (iindex == PRODUCER1_EP) {
                    CyU3PUsbSetEpNak(PRODUCER1_EP, CyTrue);
                    CyU3PBusyWait(125);
                    CyU3PDmaChannelReset(&g_dma_u2p);
                    CyU3PUsbFlushEp(PRODUCER1_EP);
                    CyU3PUsbResetEp(PRODUCER1_EP);
                    CyU3PDmaChannelSetXfer(&g_dma_u2p, CY_FX_SLFIFO_DMA_TX_SIZE);
                    CyU3PUsbSetEpNak(PRODUCER1_EP, CyFalse);
                }
                if (iindex == CONSUMER1_EP) {
                    CyU3PUsbSetEpNak(PRODUCER1_EP, CyTrue);
                    CyU3PBusyWait(125);
                    CyU3PDmaChannelReset(&g_dma_p2u);
                    CyU3PUsbFlushEp(CONSUMER1_EP);
                    CyU3PUsbResetEp(CONSUMER1_EP);
                    CyU3PDmaChannelSetXfer(&g_dma_p2u, CY_FX_SLFIFO_DMA_RX_SIZE);
                    CyU3PUsbSetEpNak(PRODUCER1_EP, CyFalse);
                }
                CyU3PUsbStall(iindex, CyFalse, CyTrue);
                CyU3PUsbAckSetup();
                bis_handled = CyTrue;
            }
        }
    }
    return bis_handled;
}

/* This is the callback function to handle the USB events. */
void usb_event_cb(CyU3PUsbEventType_t evtype, uint16_t evdata)
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

CyBool_t app_lpm_rqt_cb(CyU3PUsbLinkPowerMode link_mode)
{
    return CyTrue;
}

void app_init(void)
{
    CyU3PPibClock_t pib_clock;
    CyU3PReturnStatus_t iapi_ret_status = CY_U3P_SUCCESS;
    CyU3PGpioClock_t gpio_clock;
    CyU3PGpioSimpleConfig_t gpio_config;
    /* Initialize the p-port block. */
    pib_clock.clkDiv = 2;
    pib_clock.clkSrc = CY_U3P_SYS_CLK;
    pib_clock.isHalfDiv = CyFalse;
    /* Disable DLL for sync GPIF */
    pib_clock.isDllEnable = CyFalse;
    iapi_ret_status = CyU3PPibInit(CyTrue, &pib_clock);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "P-port Initialization failed, Error Code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* Load the GPIF configuration for Slave FIFO sync mode. */
    iapi_ret_status = CyU3PGpifLoad(&CyFxGpifConfig);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PGpifLoad failed, Error Code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
#if (CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT == 1)
    CyU3PGpifSocketConfigure(0, PRODUCER1_PPORT, 6, CyFalse, 1);
    CyU3PGpifSocketConfigure(3, CONSUMER1_PPORT, 6, CyFalse, 1);
#else
    CyU3PGpifSocketConfigure(0, PRODUCER1_PPORT, 3, CyFalse, 1);
    CyU3PGpifSocketConfigure(3, CONSUMER1_PPORT, 3, CyFalse, 1);
#endif
    /* Start the state machine. */
    iapi_ret_status = CyU3PGpifSMStart(RESET, ALPHA_RESET);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PGpifSMStart failed, Error Code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* Init the GPIO module */
    gpio_clock.fastClkDiv = 2;
    gpio_clock.slowClkDiv = 0;
    gpio_clock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    gpio_clock.clkSrc = CY_U3P_SYS_CLK;
    gpio_clock.halfDiv = 0;
    iapi_ret_status = CyU3PGpioInit(&gpio_clock, NULL);
    if (iapi_ret_status != 0) {
        /* Error Handling */
        CyU3PDebugPrint(4, "CyU3PGpioInit failed, error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* Configure GPIO 59 as output */
    gpio_config.outValue = CyTrue;
    gpio_config.driveLowEn = CyTrue;
    gpio_config.driveHighEn = CyTrue;
    gpio_config.inputEn = CyFalse;
    gpio_config.intrMode = CY_U3P_GPIO_NO_INTR;
    iapi_ret_status = CyU3PGpioSetSimpleConfig(59, &gpio_config);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        /* Error handling */
        CyU3PDebugPrint(4, "CyU3PGpioSetSimpleConfig failed, error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* Start the USB functionality. */
    iapi_ret_status = CyU3PUsbStart();
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "CyU3PUsbStart failed to Start, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }

    CyU3PUsbRegisterSetupCallback(usb_setup_cb, CyTrue);
    CyU3PUsbRegisterEventCallback(usb_event_cb);
    CyU3PUsbRegisterLPMRequestCallback(app_lpm_rqt_cb);

    iapi_ret_status = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)g_usb30_device);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set device descriptor failed, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* High speed device descriptor. */
    iapi_ret_status = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)g_usb20_device);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set device descriptor failed, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* BOS descriptor */
    iapi_ret_status = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)g_bos);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set configuration descriptor failed, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* Device qualifier descriptor */
    iapi_ret_status = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)g_device_qual);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set device qualifier descriptor failed, Error code = %d\n",
                        iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* Super speed configuration descriptor */
    iapi_ret_status = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)g_ss_config);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set configuration descriptor failed, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* High speed configuration descriptor */
    iapi_ret_status = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)g_hs_config);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB Set Other Speed Descriptor failed, Error Code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* Full speed configuration descriptor */
    iapi_ret_status = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)g_fs_config);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB Set Configuration Descriptor failed, Error Code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* String descriptor 0 */
    iapi_ret_status = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)g_lang_id);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set string descriptor failed, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* String descriptor 1 */
    iapi_ret_status = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)g_manufacture);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set string descriptor failed, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* String descriptor 2 */
    iapi_ret_status = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)g_product);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB set string descriptor failed, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
    /* Connect the USB Pins with super speed operation enabled. */
    iapi_ret_status = CyU3PConnectState(CyTrue, CyTrue);
    if (iapi_ret_status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint(4, "USB Connect failed, Error code = %d\n", iapi_ret_status);
        app_error_handler(iapi_ret_status);
    }
}

/* Entry function for the g_app_thread. */
void app_thread_entry(uint32_t input)
{
    /* Initialize the debug module */
    app_debug_init();
    /* Initialize the slave FIFO application */
    app_init();
    for (;;) {
        CyU3PThreadSleep(1000);
        /*if (g_is_app_active) {
         Print the number of buffers received so far from the USB host.
        CyU3PDebugPrint(6, "Data tracker: buffers received: %d, buffers sent: %d.\n",
                        g_dma_rx_count, g_dma_tx_count);
        }*/
    }
}

/* Application define function which creates the threads. */
void CyFxApplicationDefine(void)
{
    void *ptr = NULL;
    uint32_t ithrd_create = CY_U3P_SUCCESS;
    /* Allocate the memory for the thread */
    ptr = CyU3PMemAlloc(CY_FX_SLFIFO_THREAD_STACK);
    /* Create the thread for the application */
    ithrd_create = CyU3PThreadCreate(&g_app_thread,            /* Slave FIFO app thread structure */
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
    if (ithrd_create != 0) {
        while (1);
    }
}

/*
 * Main function
 */
int main(void)
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

