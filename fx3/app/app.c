/*
 ## Cypress USB 3.0 Platform source file (cyfxbulksrcsink.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2018,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/* This file illustrates the bulk source sink application example using the DMA MANUAL_IN
   and DMA MANUAL_OUT mode */

/*
   This example illustrates USB endpoint data source and data sink mechanism. The example
   comprises of vendor class USB enumeration descriptors with 2 bulk endpoints. A bulk OUT
   endpoint acts as the producer of data and acts as the sink to the host. A bulk IN endpoint
   acts as the consumer of data and acts as the source to the host.

   The data source and sink is achieved with the help of a DMA MANUAL IN channel and a DMA
   MANUAL OUT channel. A DMA MANUAL IN channel is created between the producer USB bulk
   endpoint and the CPU. A DMA MANUAL OUT channel is created between the CPU and the consumer
   USB bulk endpoint. Data is received in the IN channel DMA buffer from the host through the
   producer endpoint. CPU is signaled of the data reception using DMA callbacks. The CPU
   discards this buffer. This leads to the sink mechanism. A constant pattern data is loaded
   onto the OUT Channel DMA buffer whenever the buffer is available. CPU issues commit of
   the DMA data transfer to the consumer endpoint which then gets transferred to the host.
   This leads to a constant source mechanism.

   The DMA buffer size is defined based on the USB speed. 64 for full speed, 512 for high speed
   and 1024 for super speed. CY_FX_BULKSRCSINK_DMA_BUF_COUNT in the header file defines the
   number of DMA buffers.
   
   For performance optimizations refer the readme.txt
 */

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "app.h"
#include "cyu3usb.h"
#include "cyu3uart.h"
#include "cyu3gpio.h"
#include "cyu3utils.h"

CyU3PThread     g_app_thread;    /* Application thread structure */
CyU3PDmaChannel g_dma_in;      /* DMA MANUAL_IN channel handle.          */
CyU3PDmaChannel g_dma_out;       /* DMA MANUAL_OUT channel handle.         */

CyBool_t g_is_active = CyFalse;      /* Whether the source sink application is active or not. */
uint32_t g_rx_count = 0;               /* Counter to track the number of buffers received. */
uint32_t g_tx_count = 0;               /* Counter to track the number of buffers transmitted. */
CyBool_t g_is_transfer_started = CyFalse;   /* Whether DMA transfer has been started after enumeration. */
CyBool_t g_standby_mode_enable  = CyFalse;   /* Whether standby mode entry is enabled. */
CyBool_t g_trigger_standby_mode = CyFalse;   /* Request to initiate standby entry. */
CyBool_t g_force_link_u2      = CyFalse;   /* Whether the device should try to initiate U2 mode. */

volatile uint32_t g_ep0_stat_count = 0;           /* Number of EP0 status events received. */
uint8_t g_ep0_buffer[32] __attribute__ ((aligned (32))); /* Local buffer used for vendor command handling. */

/* Control request related variables. */
CyU3PEvent g_bulk_event;       /* Event group used to signal the thread that there is a pending request. */
uint32_t   g_setupdat0;        /* Variable that holds the setupdat0 value (bmRequestType, irequest and ivalue). */
uint32_t   g_setupdat1;        /* Variable that holds the setupdat1 value (iindex and ilength). */
#define CYFX_USB_CTRL_TASK      (1 << 0)        /* Event that indicates that there is a pending USB control request. */
#define CYFX_USB_HOSTWAKE_TASK  (1 << 1)        /* Event that indicates the a Remote Wake should be attempted. */

/* Buffer used for USB event logs. */
uint8_t *g_usb_log_buffer = NULL;
#define CYFX_USBLOG_SIZE        (0x1000)

/* Timer Instance */
CyU3PTimer g_lpm_timer;

/* GPIO used for testing IO state retention when switching from boot firmware to full firmware. */
#define FX3_GPIO_TEST_OUT               (50)
#define FX3_GPIO_TO_LOFLAG(gpio)        (1 << (gpio))
#define FX3_GPIO_TO_HIFLAG(gpio)        (1 << ((gpio) - 32))

static volatile CyBool_t g_src_ep_flush = CyFalse;

/* Application Error Handler */
void app_error_handler (
        CyU3PReturnStatus_t iapi_ret    /* API return status */
        )
{
    /* Application failed with the error code iapi_ret */

    /* Add custom debug or recovery actions here */

    /* Loop Indefinitely */
    for (;;)
    {
        /* Thread sleep : 100 ms */
        CyU3PThreadSleep (100);
    }
}

/* This function initializes the debug module. The debug prints
 * are routed to the UART and can be seen using a UART console
 * running at 115200 baud rate. */
void app_debug_init (void)
{
    CyU3PGpioClock_t  gpioClock;
    CyU3PUartConfig_t uart_config;
    CyU3PReturnStatus_t iapi_ret = CY_U3P_SUCCESS;

    /* Initialize the GPIO block. If we are transitioning from the boot app, we can verify whether the GPIO
       state is retained. */
    gpioClock.fastClkDiv = 2;
    gpioClock.slowClkDiv = 32;
    gpioClock.simpleDiv  = CY_U3P_GPIO_SIMPLE_DIV_BY_16;
    gpioClock.clkSrc     = CY_U3P_SYS_CLK_BY_2;
    gpioClock.halfDiv    = 0;
    iapi_ret = CyU3PGpioInit (&gpioClock, NULL);

    /* When FX3 is restarting from standby mode, the GPIO block would already be ON and need not be started
       again. */
    if ((iapi_ret != 0) && (iapi_ret != CY_U3P_ERROR_ALREADY_STARTED))
    {
        app_error_handler(iapi_ret);
    }
    else
    {
        /* Set the test GPIO as an output and update the value to 0. */
        CyU3PGpioSimpleConfig_t testConf = {CyFalse, CyTrue, CyTrue, CyFalse, CY_U3P_GPIO_NO_INTR};

        iapi_ret = CyU3PGpioSetSimpleConfig (FX3_GPIO_TEST_OUT, &testConf);
        if (iapi_ret != 0)
            app_error_handler (iapi_ret);
    }

    /* Initialize the UART for printing debug messages */
    iapi_ret = CyU3PUartInit();
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        /* Error handling */
        app_error_handler(iapi_ret);
    }

    /* Set UART configuration */
    CyU3PMemSet ((uint8_t *)&uart_config, 0, sizeof (uart_config));
    uart_config.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uart_config.stopBit = CY_U3P_UART_ONE_STOP_BIT;
    uart_config.parity = CY_U3P_UART_NO_PARITY;
    uart_config.txEnable = CyTrue;
    uart_config.rxEnable = CyFalse;
    uart_config.flowCtrl = CyFalse;
    uart_config.isDma = CyTrue;

    iapi_ret = CyU3PUartSetConfig (&uart_config, NULL);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        app_error_handler(iapi_ret);
    }

    /* Set the UART transfer to a really large value. */
    iapi_ret = CyU3PUartTxSetBlockXfer (0xFFFFFFFF);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        app_error_handler(iapi_ret);
    }

    /* Initialize the debug module. */
    iapi_ret = CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 8);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        app_error_handler(iapi_ret);
    }

    CyU3PDebugPreamble(CyFalse);
}


CyBool_t lpm_rqt_cb (CyU3PUsbLinkPowerMode link_mode)
{
    return CyTrue;
}

/* Callback funtion for the timer expiry notification. */
void timer_cb(void)
{
    /* Enable the low power mode transition on timer expiry */
    CyU3PUsbLPMEnable();
}

/* Callback funtion for the DMA event notification. */
void dma_cb (
        CyU3PDmaChannel   *chHandle, /* Handle to the DMA channel. */
        CyU3PDmaCbType_t  type,      /* Callback type.             */
        CyU3PDmaCBInput_t *input)    /* Callback status.           */
{
    CyU3PDmaBuffer_t buf_p;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    g_is_transfer_started = CyTrue;

    /** Start/restart the timer and disable LPM **/
    CyU3PUsbLPMDisable();
    CyU3PTimerStop (&g_lpm_timer);
    CyU3PTimerModify(&g_lpm_timer, 100, 0);
    CyU3PTimerStart(&g_lpm_timer);

    if (type == CY_U3P_DMA_CB_PROD_EVENT)
    {
        /* This is a produce event notification to the CPU. This notification is 
         * received upon reception of every buffer. We have to discard the buffer
         * as soon as it is received to implement the data sink. */
        status = CyU3PDmaChannelDiscardBuffer (chHandle);
        if (status != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (4, "CyU3PDmaChannelDiscardBuffer failed, Error code = %d\n", status);
        }

        /* Increment the counter. */
        g_rx_count++;
    }
    if (type == CY_U3P_DMA_CB_CONS_EVENT)
    {
        /* This is a consume event notification to the CPU. This notification is 
         * received when a buffer is sent out from the device. We have to commit
         * a new buffer as soon as a buffer is available to implement the data
         * source. The data is preloaded into the buffer at that start. So just
         * commit the buffer. */
        status = CyU3PDmaChannelGetBuffer (chHandle, &buf_p, CYU3P_NO_WAIT);
        if (status == CY_U3P_SUCCESS)
        {
            /* Commit the full buffer with default status. */
            status = CyU3PDmaChannelCommitBuffer (chHandle, buf_p.size, 0);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PDebugPrint (4, "CyU3PDmaChannelCommitBuffer failed, Error code = %d\n", status);
            }
        }
        else
        {
            CyU3PDebugPrint (4, "CyU3PDmaChannelGetBuffer failed, Error code = %d\n", status);
        }

        /* Increment the counter. */
        g_tx_count++;
    }
}

/*
 * Fill all DMA buffers on the IN endpoint with data. This gets data moving after an endpoint reset.
 */
static void fill_in_buffers (void)
{
    CyU3PReturnStatus_t stat;
    CyU3PDmaBuffer_t    buf_p;
    uint16_t            index = 0;

    /* Now preload all buffers in the MANUAL_OUT pipe with the required data. */
    for (index = 0; index < CY_FX_BULKSRCSINK_DMA_BUF_COUNT; index++)
    {
        stat = CyU3PDmaChannelGetBuffer (&g_dma_out, &buf_p, CYU3P_NO_WAIT);
        if (stat != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (4, "CyU3PDmaChannelGetBuffer failed, Error code = %d\n", stat);
            app_error_handler(stat);
        }

        CyU3PMemSet (buf_p.buffer, CY_FX_BULKSRCSINK_PATTERN, buf_p.size);
        stat = CyU3PDmaChannelCommitBuffer (&g_dma_out, buf_p.size, 0);
        if (stat != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (4, "CyU3PDmaChannelCommitBuffer failed, Error code = %d\n", stat);
            app_error_handler(stat);
        }
    }
}


void ep_evt_cb (
        CyU3PUsbEpEvtType evtype,
        CyU3PUSBSpeed_t   speed,
        uint8_t           epNum)
{
    /* Hit an endpoint retry case. Need to stall and flush the endpoint for recovery. */
    if (evtype == CYU3P_USBEP_SS_RETRY_EVT)
    {
        g_src_ep_flush = CyTrue;
    }
}

/* This function starts the application. This is called
 * when a SET_CONF event is received from the USB host. The endpoints
 * are configured and the DMA pipe is setup in this function. */
void app_start (void)
{
    uint16_t size = 0;
    CyU3PEpConfig_t ep_cfg;
    CyU3PDmaChannelConfig_t dma_cfg;
    CyU3PReturnStatus_t iapi_ret = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usb_speed = CyU3PUsbGetSpeed();

    /* First identify the usb speed. Once that is identified,
     * create a DMA channel and start the transfer on this. */

    /* Based on the Bus Speed configure the endpoint packet size */
    switch (usb_speed)
    {
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
        CyU3PDebugPrint (4, "Error! Invalid USB speed.\n");
        app_error_handler (CY_U3P_ERROR_FAILURE);
        break;
    }

    CyU3PMemSet ((uint8_t *)&ep_cfg, 0, sizeof (ep_cfg));
    ep_cfg.enable = CyTrue;
    ep_cfg.epType = CY_U3P_USB_EP_BULK;
    ep_cfg.burstLen = (usb_speed == CY_U3P_SUPER_SPEED) ?
        (CY_FX_EP_BURST_LENGTH) : 1;
    ep_cfg.streams = 0;
    ep_cfg.pcktSize = size;

    /* Producer endpoint configuration */
    iapi_ret = CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &ep_cfg);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", iapi_ret);
        app_error_handler (iapi_ret);
    }

    /* Consumer endpoint configuration */
    iapi_ret = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &ep_cfg);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", iapi_ret);
        app_error_handler (iapi_ret);
    }

    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

    /* Create a DMA MANUAL_IN channel for the producer socket. */
    CyU3PMemSet ((uint8_t *)&dma_cfg, 0, sizeof (dma_cfg));
    /* The buffer size will be same as packet size for the
     * full speed, high speed and super speed non-burst modes.
     * For super speed burst mode of operation, the buffers will be
     * 1024 * burst length so that a full burst can be completed.
     * This will mean that a buffer will be available only after it
     * has been filled or when a short packet is received. */
    dma_cfg.size  = (size * CY_FX_EP_BURST_LENGTH);
    /* Multiply the buffer size with the multiplier
     * for performance improvement. */
    dma_cfg.size *= CY_FX_DMA_SIZE_MULTIPLIER;
    dma_cfg.count = CY_FX_BULKSRCSINK_DMA_BUF_COUNT;
    dma_cfg.prodSckId = CY_FX_EP_PRODUCER_SOCKET;
    dma_cfg.consSckId = CY_U3P_CPU_SOCKET_CONS;
    dma_cfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dma_cfg.notification = CY_U3P_DMA_CB_PROD_EVENT;
    dma_cfg.cb = dma_cb;
    dma_cfg.prodHeader = 0;
    dma_cfg.prodFooter = 0;
    dma_cfg.consHeader = 0;
    dma_cfg.prodAvailCount = 0;

    iapi_ret = CyU3PDmaChannelCreate (&g_dma_in,
            CY_U3P_DMA_TYPE_MANUAL_IN, &dma_cfg);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PDmaChannelCreate failed, Error code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }

    /* Create a DMA MANUAL_OUT channel for the consumer socket. */
    dma_cfg.notification = CY_U3P_DMA_CB_CONS_EVENT;
    dma_cfg.prodSckId = CY_U3P_CPU_SOCKET_PROD;
    dma_cfg.consSckId = CY_FX_EP_CONSUMER_SOCKET;
    iapi_ret = CyU3PDmaChannelCreate (&g_dma_out,
            CY_U3P_DMA_TYPE_MANUAL_OUT, &dma_cfg);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PDmaChannelCreate failed, Error code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }

    /* Set DMA Channel transfer size */
    iapi_ret = CyU3PDmaChannelSetXfer (&g_dma_in, CY_FX_BULKSRCSINK_DMA_TX_SIZE);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PDmaChannelSetXfer failed, Error code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }

    iapi_ret = CyU3PDmaChannelSetXfer (&g_dma_out, CY_FX_BULKSRCSINK_DMA_TX_SIZE);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PDmaChannelSetXfer failed, Error code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }

    CyU3PUsbRegisterEpEvtCallback (ep_evt_cb, CYU3P_USBEP_SS_RETRY_EVT, 0x00, 0x02);
    fill_in_buffers ();

    /* Update the flag so that the application thread is notified of this. */
    g_is_active = CyTrue;
}

/* This function stops the application. This shall be called whenever a RESET
 * or DISCONNECT event is received from the USB host. The endpoints are
 * disabled and the DMA pipe is destroyed by this function. */
void app_stop (void)
{
    CyU3PEpConfig_t ep_cfg;
    CyU3PReturnStatus_t iapi_ret = CY_U3P_SUCCESS;

    /* Update the flag so that the application thread is notified of this. */
    g_is_active = CyFalse;

    /* Destroy the channels */
    CyU3PDmaChannelDestroy (&g_dma_in);
    CyU3PDmaChannelDestroy (&g_dma_out);

    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

    /* Disable endpoints. */
    CyU3PMemSet ((uint8_t *)&ep_cfg, 0, sizeof (ep_cfg));
    ep_cfg.enable = CyFalse;

    /* Producer endpoint configuration. */
    iapi_ret = CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &ep_cfg);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", iapi_ret);
        app_error_handler (iapi_ret);
    }

    /* Consumer endpoint configuration. */
    iapi_ret = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &ep_cfg);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", iapi_ret);
        app_error_handler (iapi_ret);
    }
}

/* Callback to handle the USB setup requests. */
CyBool_t usb_setup_cb (
        uint32_t setupdat0, /* SETUP Data 0 */
        uint32_t setupdat1  /* SETUP Data 1 */
    )
{
    /* Fast enumeration is used. Only requests addressed to the interface, class,
     * vendor and unknown control requests are received by this function.
     * This application does not support any class or vendor requests. */

    uint8_t  irequest, breq_type;
    uint8_t  itype, itarget;
    uint16_t ivalue, iindex;
    CyBool_t bhandled = CyFalse;

    /* Decode the fields from the setup request. */
    breq_type = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    itype    = (breq_type & CY_U3P_USB_TYPE_MASK);
    itarget  = (breq_type & CY_U3P_USB_TARGET_MASK);
    irequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    ivalue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    iindex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);

    if (itype == CY_U3P_USB_STANDARD_RQT)
    {
        /* Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND)
         * requests here. It should be allowed to pass if the device is in configured
         * state and failed otherwise. */
        if ((itarget == CY_U3P_USB_TARGET_INTF) && ((irequest == CY_U3P_USB_SC_SET_FEATURE)
                    || (irequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (ivalue == 0))
        {
            if (g_is_active)
            {
                CyU3PUsbAckSetup ();

                /* As we have only one interface, the link can be pushed into U2 state as soon as
                   this interface is suspended.
                 */
                if (irequest == CY_U3P_USB_SC_SET_FEATURE)
                {
                    g_is_transfer_started = CyFalse;
                    g_force_link_u2      = CyTrue;
                }
                else
                {
                    g_force_link_u2 = CyFalse;
                }
            }
            else
                CyU3PUsbStall (0, CyTrue, CyFalse);

            bhandled = CyTrue;
        }

        /* CLEAR_FEATURE request for endpoint is always passed to the setup callback
         * regardless of the enumeration model used. When a clear feature is received,
         * the previous transfer has to be flushed and cleaned up. This is done at the
         * protocol level. Since this is just a loopback operation, there is no higher
         * level protocol. So flush the EP memory and reset the DMA channel associated
         * with it. If there are more than one EP associated with the channel reset both
         * the EPs. The endpoint stall and toggle / sequence number is also expected to be
         * reset. Return CyFalse to make the library clear the stall and reset the endpoint
         * toggle. Or invoke the CyU3PUsbStall (ep, CyFalse, CyTrue) and return CyTrue.
         * Here we are clearing the stall. */
        if ((itarget == CY_U3P_USB_TARGET_ENDPT) && (irequest == CY_U3P_USB_SC_CLEAR_FEATURE)
                && (ivalue == CY_U3P_USBX_FS_EP_HALT))
        {
            if (g_is_active)
            {
                if (iindex == CY_FX_EP_PRODUCER)
                {
                    CyU3PUsbSetEpNak (CY_FX_EP_PRODUCER, CyTrue);
                    CyU3PBusyWait (125);

                    CyU3PDmaChannelReset (&g_dma_in);
                    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
                    CyU3PUsbResetEp (CY_FX_EP_PRODUCER);
                    CyU3PUsbSetEpNak (CY_FX_EP_PRODUCER, CyFalse);

                    CyU3PDmaChannelSetXfer (&g_dma_in, CY_FX_BULKSRCSINK_DMA_TX_SIZE);
                    CyU3PUsbStall (iindex, CyFalse, CyTrue);
                    bhandled = CyTrue;
                    CyU3PUsbAckSetup ();
                }

                if (iindex == CY_FX_EP_CONSUMER)
                {
                    CyU3PUsbSetEpNak (CY_FX_EP_CONSUMER, CyTrue);
                    CyU3PBusyWait (125);

                    CyU3PDmaChannelReset (&g_dma_out);
                    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
                    CyU3PUsbResetEp (CY_FX_EP_CONSUMER);
                    CyU3PUsbSetEpNak (CY_FX_EP_CONSUMER, CyFalse);

                    CyU3PDmaChannelSetXfer (&g_dma_out, CY_FX_BULKSRCSINK_DMA_TX_SIZE);
                    CyU3PUsbStall (iindex, CyFalse, CyTrue);
                    bhandled = CyTrue;
                    CyU3PUsbAckSetup ();

                    fill_in_buffers ();
                }
            }
        }
    }

    if ((itype == CY_U3P_USB_VENDOR_RQT) && (itarget == CY_U3P_USB_TARGET_DEVICE))
    {
        
        CyU3PDebugPrint(4, "request: %d\r\n", irequest);
        /* We set an event here and let the application thread below handle these requests.
         * bhandled needs to be set to True, so that the driver does not stall EP0. */
        bhandled = CyTrue;
        g_setupdat0 = setupdat0;
        g_setupdat1 = setupdat1;
        CyU3PEventSet (&g_bulk_event, CYFX_USB_CTRL_TASK, CYU3P_EVENT_OR);
    }

    return bhandled;
}

/* This is the callback function to handle the USB events. */
void usb_event_cb (
        CyU3PUsbEventType_t evtype, /* Event type */
        uint16_t            evdata  /* Event data */
        )
{
    CyU3PDebugPrint (2, "USB EVENT: %d %d\r\n", evtype, evdata);

    switch (evtype)
    {
    case CY_U3P_USB_EVENT_CONNECT:
        break;

    case CY_U3P_USB_EVENT_SETCONF:
        /* If the application is already active
         * stop it before re-enabling. */
        if (g_is_active)
        {
            app_stop ();
        }

        /* Start the source sink function. */
        app_start ();
        break;

    case CY_U3P_USB_EVENT_RESET:
    case CY_U3P_USB_EVENT_DISCONNECT:
        g_force_link_u2 = CyFalse;

        /* Stop the source sink function. */
        if (g_is_active)
        {
            app_stop ();
        }

        g_is_transfer_started = CyFalse;
        break;

    case CY_U3P_USB_EVENT_EP0_STAT_CPLT:
        g_ep0_stat_count++;
        break;

    case CY_U3P_USB_EVENT_VBUS_REMOVED:
        if (g_standby_mode_enable)
        {
            g_trigger_standby_mode = CyTrue;
            g_standby_mode_enable  = CyFalse;
        }
        break;

    default:
        break;
    }
}

/* Callback function to handle LPM requests from the USB 3.0 host. This function is invoked by the API
   whenever a state change from U0 -> U1 or U0 -> U2 happens. If we return CyTrue from this function, the
   FX3 device is retained in the low power state. If we return CyFalse, the FX3 device immediately tries
   to trigger an exit back to U0.

   This application does not have any state in which we should not allow U1/U2 transitions; and therefore
   the function always return CyTrue.
 */
CyBool_t link_mode_cb (CyU3PUsbLinkPowerMode link_mode)
{
    return CyTrue;
}

/* This function initializes the USB Module, sets the enumeration descriptors.
 * This function does not start the bulk streaming and this is done only when
 * SET_CONF event is received. */
void app_init (void)
{
    CyU3PReturnStatus_t iapi_ret = CY_U3P_SUCCESS;
    CyBool_t no_renum = CyFalse;

    /* The fast enumeration is the easiest way to setup a USB connection,
     * where all enumeration phase is handled by the library. Only the
     * class / vendor requests need to be handled by the application. */
    CyU3PUsbRegisterSetupCallback(usb_setup_cb, CyTrue);

    /* Setup the callback to handle the USB events. */
    CyU3PUsbRegisterEventCallback(usb_event_cb);

    /* Register a callback to handle LPM requests from the USB 3.0 host. */
    CyU3PUsbRegisterLPMRequestCallback(lpm_rqt_cb);

    /* Start the USB functionality. */
    iapi_ret = CyU3PUsbStart();
    if (iapi_ret == CY_U3P_ERROR_NO_REENUM_REQUIRED)
        no_renum = CyTrue;
    else if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PUsbStart failed to Start, Error code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }

    /* Change GPIO state again. */
    CyU3PGpioSimpleSetValue (FX3_GPIO_TEST_OUT, CyTrue);

    /* Set the USB Enumeration descriptors */

    /* Super speed device descriptor. */
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)g_usb30_device);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set device descriptor failed, Error code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }

    /* High speed device descriptor. */
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)g_usb20_device);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set device descriptor failed, Error code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }

    /* BOS descriptor */
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)g_bos);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set configuration descriptor failed, Error code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }

    /* Device qualifier descriptor */
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)g_device_qual);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set device qualifier descriptor failed, Error code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }

    /* Super speed configuration descriptor */
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)g_ss_config);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set configuration descriptor failed, Error code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }

    /* High speed configuration descriptor */
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)g_hs_config);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB Set Other Speed Descriptor failed, Error Code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }

    /* Full speed configuration descriptor */
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)g_fs_config);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB Set Configuration Descriptor failed, Error Code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }

    /* String descriptor 0 */
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)g_lang_id);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set string descriptor failed, Error code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }

    /* String descriptor 1 */
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)g_manufacture);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set string descriptor failed, Error code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }

    /* String descriptor 2 */
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)g_product);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set string descriptor failed, Error code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }

    /* Register a buffer into which the USB driver can log relevant events. */
    g_usb_log_buffer = (uint8_t *)CyU3PDmaBufferAlloc (CYFX_USBLOG_SIZE);
    if (g_usb_log_buffer)
        CyU3PUsbInitEventLog (g_usb_log_buffer, CYFX_USBLOG_SIZE);

    CyU3PDebugPrint (4, "About to connect to USB host\r\n");

    /* Connect the USB Pins with super speed operation enabled. */
    if (!no_renum) {

        iapi_ret = CyU3PConnectState(CyTrue, CyTrue);
        if (iapi_ret != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (4, "USB Connect failed, Error code = %d\n", iapi_ret);
            app_error_handler(iapi_ret);
        }
    }
    else
    {
        /* USB connection is already active. Configure the endpoints and DMA channels. */
        app_start ();
    }

    CyU3PDebugPrint (8, "app_init complete\r\n");
}

/*
 * De-initialize function for the USB block. Used to test USB Stop/Start functionality.
 */
static void app_deinit (void)
{
    if (g_is_active)
        app_stop ();

    CyU3PConnectState (CyFalse, CyTrue);
    CyU3PThreadSleep (1000);
    CyU3PUsbStop ();
    CyU3PThreadSleep (1000);
}

/* Entry function for the BulkSrcSinkAppThread. */
void app_thread_entry (uint32_t input)
{
    CyU3PReturnStatus_t ireturn_stat;
    uint32_t ievent_mask = CYFX_USB_CTRL_TASK | CYFX_USB_HOSTWAKE_TASK;   /* Events that we are interested in. */
    uint32_t ievent_stat;                                                 /* Current status of the events. */
    uint8_t  ivendor_rqt_cnt = 0;

    uint16_t prevUsbLogIndex = 0, tmp1, tmp2;
    CyU3PUsbLinkPowerMode curState;

    /* Initialize the debug module */
    app_debug_init();
    CyU3PDebugPrint (1, "\n\ndebug initialized\r\n");

    /* Initialize the application */
    app_init();

    /* Create a timer with 100 ms expiry to enable/disable LPM transitions */ 
    CyU3PTimerCreate (&g_lpm_timer, timer_cb, 0, 100, 100, CYU3P_NO_ACTIVATE);

    for (;;)
    {
        /* The following call will block until at least one of the events enabled in ievent_mask is received.
           The ievent_stat variable will hold the events that were active at the time of returning from this API.
           The CLEAR flag means that all events will be atomically cleared before this function returns.

           We cause this event wait to time out every 10 milli-seconds, so that we can periodically get the FX3
           device out of low power modes.
           */
        ireturn_stat = CyU3PEventGet (&g_bulk_event, ievent_mask, CYU3P_EVENT_OR_CLEAR, &ievent_stat, 10);
        if (ireturn_stat == CY_U3P_SUCCESS)
        {
            /* If the HOSTWAKE task is set, send a DEV_NOTIFICATION (FUNCTION_WAKE) or remote wakeup signalling
               based on the USB connection speed. */
            if (ievent_stat & CYFX_USB_HOSTWAKE_TASK)
            {
                CyU3PThreadSleep (1000);
                if (CyU3PUsbGetSpeed () == CY_U3P_SUPER_SPEED)
                    ireturn_stat = CyU3PUsbSendDevNotification (1, 0, 0);
                else
                    ireturn_stat = CyU3PUsbDoRemoteWakeup ();

                if (ireturn_stat != CY_U3P_SUCCESS)
                    CyU3PDebugPrint (2, "Remote wake attempt failed with code: %d\r\n", ireturn_stat);
            }

            /* If there is a pending control request, handle it here. */
            if (ievent_stat & CYFX_USB_CTRL_TASK)
            {
                uint8_t  irequest, itype;
                uint16_t ilength, temp;
                uint16_t ivalue, iindex;

                /* Decode the fields from the setup request. */
                itype = (g_setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
                irequest = ((g_setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
                ilength  = ((g_setupdat1 & CY_U3P_USB_LENGTH_MASK)  >> CY_U3P_USB_LENGTH_POS);
                ivalue   = ((g_setupdat0 & CY_U3P_USB_VALUE_MASK) >> CY_U3P_USB_VALUE_POS);
                iindex   = ((g_setupdat1 & CY_U3P_USB_INDEX_MASK) >> CY_U3P_USB_INDEX_POS);

                if ((itype & CY_U3P_USB_TYPE_MASK) == CY_U3P_USB_VENDOR_RQT)
                {
                    switch (irequest)
                    {
                    case 0x76:
                        g_ep0_buffer[0] = ivendor_rqt_cnt;
                        g_ep0_buffer[1] = ~ivendor_rqt_cnt;
                        g_ep0_buffer[2] = 1;
                        g_ep0_buffer[3] = 5;
                        CyU3PUsbSendEP0Data (ilength, g_ep0_buffer);
                        ivendor_rqt_cnt++;
                        break;

                    case 0x77:      /* Trigger remote wakeup. */
                        CyU3PUsbAckSetup ();
                        CyU3PEventSet (&g_bulk_event, CYFX_USB_HOSTWAKE_TASK, CYU3P_EVENT_OR);
                        break;

                    case 0x78:      /* Get count of EP0 status events received. */
                        CyU3PMemCopy ((uint8_t *)g_ep0_buffer, ((uint8_t *)&g_ep0_stat_count), 4);
                        CyU3PUsbSendEP0Data (4, g_ep0_buffer);
                        break;

                    case 0x79:      /* Request with no data phase. Insert a delay and then ACK the request. */
                        CyU3PThreadSleep (5);
                        CyU3PUsbAckSetup ();
                        break;

                    case 0x80:      /* Request with OUT data phase. Just get the data and ignore it for now. */
                        CyU3PUsbGetEP0Data (sizeof (g_ep0_buffer), (uint8_t *)g_ep0_buffer, &ilength);
                        break;

                    case 0x81:
                        /* Get the current event log index and send it to the host. */
                        if (ilength == 2)
                        {
                            temp = CyU3PUsbGetEventLogIndex ();
                            CyU3PMemCopy ((uint8_t *)g_ep0_buffer, (uint8_t *)&temp, 2);
                            CyU3PUsbSendEP0Data (2, g_ep0_buffer);
                        }
                        else
                            CyU3PUsbStall (0, CyTrue, CyFalse);
                        break;

                    case 0x82:
                        /* Send the USB event log buffer content to the host. */
                        if (ilength != 0)
                        {
                            if (ilength < CYFX_USBLOG_SIZE)
                                CyU3PUsbSendEP0Data (ilength, g_usb_log_buffer);
                            else
                                CyU3PUsbSendEP0Data (CYFX_USBLOG_SIZE, g_usb_log_buffer);
                        }
                        else
                            CyU3PUsbAckSetup ();
                        break;

                    case 0x83:
                        {
                            uint32_t addr = ((uint32_t)ivalue << 16) | (uint32_t)iindex;
                            CyU3PReadDeviceRegisters ((uvint32_t *)addr, 1, (uint32_t *)g_ep0_buffer);
                            CyU3PUsbSendEP0Data (4, g_ep0_buffer);
                        }
                        break;

                    case 0x84:
                        {
                            uint8_t major, minor, patch;

                            if (CyU3PUsbGetBooterVersion (&major, &minor, &patch) == CY_U3P_SUCCESS)
                            {
                                g_ep0_buffer[0] = major;
                                g_ep0_buffer[1] = minor;
                                g_ep0_buffer[2] = patch;
                                CyU3PUsbSendEP0Data (3, g_ep0_buffer);
                            }
                            else
                                CyU3PUsbStall (0, CyTrue, CyFalse);
                        }
                        break;

                    case 0x90:
                        /* Request to switch control back to the boot firmware. */

                        /* Complete the control request. */
                        CyU3PUsbAckSetup ();
                        CyU3PThreadSleep (10);

                        /* Get rid of the DMA channels and EP configuration. */
                        app_stop ();

                        /* De-initialize the Debug and UART modules. */
                        CyU3PDebugDeInit ();
                        CyU3PUartDeInit ();

                        /* Now jump back to the boot firmware image. */
                        CyU3PUsbSetBooterSwitch (CyTrue);
                        CyU3PUsbJumpBackToBooter (0x40078000);
                        while (1)
                            CyU3PThreadSleep (100);
                        break;

                    case 0xB1:
                        /* Switch to a USB 2.0 Connection. */
                        CyU3PUsbAckSetup ();
                        CyU3PThreadSleep (1000);
                        app_stop ();
                        CyU3PConnectState (CyFalse, CyTrue);
                        CyU3PThreadSleep (100);
                        CyU3PConnectState (CyTrue, CyFalse);
                        break;

                    case 0xB2:
                        /* Switch to a USB 3.0 connection. */
                        CyU3PUsbAckSetup ();
                        CyU3PThreadSleep (100);
                        app_stop ();
                        CyU3PConnectState (CyFalse, CyTrue);
                        CyU3PThreadSleep (10);
                        CyU3PConnectState (CyTrue, CyTrue);
                        break;

                    case 0xB3:
                        /* Stop and restart the USB block. */
                        CyU3PUsbAckSetup ();
                        CyU3PThreadSleep (100);
                        app_deinit ();
                        app_init ();
                        break;

                    case 0xE0:
                        /* Request to reset the FX3 device. */
                        CyU3PUsbAckSetup ();
                        CyU3PThreadSleep (2000);
                        CyU3PConnectState (CyFalse, CyTrue);
                        CyU3PThreadSleep (1000);
                        CyU3PDeviceReset (CyFalse);
                        CyU3PThreadSleep (1000);
                        break;

                    case 0xE1:
                        /* Request to place FX3 in standby when VBus is next disconnected. */
                        g_standby_mode_enable = CyTrue;
                        CyU3PUsbAckSetup ();
                        break;

                    default:        /* Unknown request. Stall EP0. */
                        CyU3PUsbStall (0, CyTrue, CyFalse);
                        break;
                    }
                }
                else
                {
                    /* Only vendor requests are to be handled here. */
                    CyU3PUsbStall (0, CyTrue, CyFalse);
                }
            }
        }

        if (g_src_ep_flush)
        {
            /* Stall the endpoint, so that the host can reset the pipe and continue. */
            g_src_ep_flush = CyFalse;
            CyU3PUsbStall (CY_FX_EP_CONSUMER, CyTrue, CyFalse);
        }

        /* Force the USB 3.0 link to U2. */
        if (g_force_link_u2)
        {
            ireturn_stat = CyU3PUsbGetLinkPowerState (&curState);
            while ((g_force_link_u2) && (ireturn_stat == CY_U3P_SUCCESS) && (curState == CyU3PUsbLPM_U0))
            {
                /* Repeatedly try to go into U2 state.*/
                CyU3PUsbSetLinkPowerState (CyU3PUsbLPM_U2);
                CyU3PThreadSleep (5);
                ireturn_stat = CyU3PUsbGetLinkPowerState (&curState);
            }
        }

        if (g_trigger_standby_mode)
        {
            g_trigger_standby_mode = CyFalse;

            CyU3PConnectState (CyFalse, CyTrue);
            CyU3PUsbStop ();
            CyU3PDebugDeInit ();
            CyU3PUartDeInit ();

            /* Add a delay to allow VBus to settle. */
            CyU3PThreadSleep (1000);

            /* VBus has been turned off. Go into standby mode and wait for VBus to be turned on again.
               The I-TCM content and GPIO register state will be backed up in the memory area starting
               at address 0x40060000. */
            ireturn_stat = CyU3PSysEnterStandbyMode (CY_U3P_SYS_USB_VBUS_WAKEUP_SRC, CY_U3P_SYS_USB_VBUS_WAKEUP_SRC,
                    (uint8_t *)0x40060000);
            if (ireturn_stat != CY_U3P_SUCCESS)
            {
                app_debug_init ();
                CyU3PDebugPrint (4, "Enter standby returned %d\r\n", ireturn_stat);
                app_error_handler (ireturn_stat);
            }

            /* If the entry into standby succeeds, the CyU3PSysEnterStandbyMode function never returns. The
               firmware application starts running again from the main entry point. Therefore, this code
               will never be executed. */
            app_error_handler (1);
        }
        else
        {
            /* Compare the current USB driver log index against the previous value. */
            tmp1 = CyU3PUsbGetEventLogIndex ();
            if (tmp1 != prevUsbLogIndex)
            {
                tmp2 = prevUsbLogIndex;
                while (tmp2 != tmp1)
                {
                    CyU3PDebugPrint (4, "USB LOG: %x\r\n", g_usb_log_buffer[tmp2]);
                    tmp2++;
                    if (tmp2 == CYFX_USBLOG_SIZE)
                        tmp2 = 0;
                }
            }

            /* Store the current log index. */
            prevUsbLogIndex = tmp1;
        }
    }
}

/* Application define function which creates the threads. */
void CyFxApplicationDefine (void)
{
    void *ptr = NULL;
    uint32_t ret = CY_U3P_SUCCESS;

    /* Create an event flag group that will be used for signalling the application thread. */
    ret = CyU3PEventCreate (&g_bulk_event);
    if (ret != 0)
    {
        /* Loop indefinitely */
        while (1);
    }

    /* Allocate the memory for the threads */
    ptr = CyU3PMemAlloc (CY_FX_BULKSRCSINK_THREAD_STACK);

    /* Create the thread for the application */
    ret = CyU3PThreadCreate (&g_app_thread,                /* App thread structure */
                          "21:Bulk_src_sink",                      /* Thread ID and thread name */
                          app_thread_entry,              /* App thread entry function */
                          0,                                       /* No input parameter to thread */
                          ptr,                                     /* Pointer to the allocated thread stack */
                          CY_FX_BULKSRCSINK_THREAD_STACK,          /* App thread stack size */
                          CY_FX_BULKSRCSINK_THREAD_PRIORITY,       /* App thread priority */
                          CY_FX_BULKSRCSINK_THREAD_PRIORITY,       /* App thread priority */
                          CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
                          CYU3P_AUTO_START                         /* Start the thread immediately */
                          );

    /* Check the return code */
    if (ret != 0)
    {
        /* Thread Creation failed with the error code retThrdCreate */

        /* Add custom recovery or debug actions here */

        /* Application cannot continue */
        /* Loop indefinitely */
        while(1);
    }
}

/*
 * Main function
 */
int main (void)
{
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize the device */
    CyU3PSysClockConfig_t clockConfig;
    clockConfig.setSysClk400  = CyFalse;
    clockConfig.cpuClkDiv     = 2;
    clockConfig.dmaClkDiv     = 2;
    clockConfig.mmioClkDiv    = 2;
    clockConfig.useStandbyClk = CyFalse;
    clockConfig.clkSrc         = CY_U3P_SYS_CLK;
    status = CyU3PDeviceInit (&clockConfig);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Initialize the caches. The D-Cache is kept disabled. Enabling this will cause performance to drop,
       as the driver will start doing a lot of un-necessary cache clean/flush operations.
       Enable the D-Cache only if there is a need to process the data being transferred by firmware code.
     */
    status = CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Configure the IO matrix for the device. On the FX3 DVK board, the COM port 
     * is connected to the IO(53:56). This means that either DQ32 mode should be
     * selected or lppMode should be set to UART_ONLY. Here we are choosing
     * UART_ONLY configuration. */
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.s0Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.s1Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.useUart   = CyTrue;
    io_cfg.useI2C    = CyFalse;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyFalse;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_UART_ONLY;

    /* Enable the GPIO which would have been setup by 2-stage booter. */
    io_cfg.gpioSimpleEn[0]  = 0;
    io_cfg.gpioSimpleEn[1]  = FX3_GPIO_TO_HIFLAG(FX3_GPIO_TEST_OUT);
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* This is a non returnable call for initializing the RTOS kernel */
    CyU3PKernelEntry ();

    /* Dummy return to make the compiler happy */
    return 0;

handle_fatal_error:

    /* Cannot recover from this error. */
    while (1);
}

/* [ ] */

