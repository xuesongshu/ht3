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
#include "cyu3usb.h"
#include "cyu3uart.h"
#include "cyu3gpio.h"
#include "cyu3utils.h"
#include "module.h"
#include "cyfxbulksrcsink.h"
#include "../usb_descr.h"

CyU3PThread     bulkSrcSinkAppThread;    /* Application thread structure */
CyU3PDmaChannel g_uvc_dma_prod;      /* DMA MANUAL_IN channel handle.          */
CyU3PDmaChannel g_uvc_dma_cons;       /* DMA MANUAL_OUT channel handle.         */
CyU3PDmaChannel g_gpu_dma_prod;      /* DMA MANUAL_IN channel handle.          */
CyU3PDmaChannel g_gpu_dma_cons;       /* DMA MANUAL_OUT channel handle.         */

CyBool_t glIsApplnActive = CyFalse;      /* Whether the source sink application is active or not. */
uint32_t glDMARxCount = 0;               /* Counter to track the number of buffers received. */
uint32_t glDMATxCount = 0;               /* Counter to track the number of buffers transmitted. */
CyBool_t glDataTransStarted = CyFalse;   /* Whether DMA transfer has been started after enumeration. */
CyBool_t StandbyModeEnable  = CyFalse;   /* Whether standby mode entry is enabled. */
CyBool_t TriggerStandbyMode = CyFalse;   /* Request to initiate standby entry. */
CyBool_t glForceLinkU2      = CyFalse;   /* Whether the device should try to initiate U2 mode. */

volatile uint32_t glEp0StatCount = 0;           /* Number of EP0 status events received. */
uint8_t glEp0Buffer[32] __attribute__ ((aligned (32))); /* Local buffer used for vendor command handling. */

/* Control request related variables. */
CyU3PEvent glBulkLpEvent;       /* Event group used to signal the thread that there is a pending request. */
uint32_t   gl_setupdat0;        /* Variable that holds the setupdat0 value (bmRequestType, bRequest and wValue). */
uint32_t   gl_setupdat1;        /* Variable that holds the setupdat1 value (wIndex and wLength). */

/* Buffer used for USB event logs. */
uint8_t *gl_UsbLogBuffer = NULL;

/* Timer Instance */
CyU3PTimer glLpmTimer;

/* This function initializes the debug module. The debug prints
 * are routed to the UART and can be seen using a UART console
 * running at 115200 baud rate. */
void
app_debug_init (void)
{
    CyU3PUartConfig_t uartConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Initialize the UART for printing debug messages */
    apiRetStatus = CyU3PUartInit();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        app_error_handler(apiRetStatus);
    }

    /* Set UART configuration */
    CyU3PMemSet ((uint8_t *)&uartConfig, 0, sizeof (uartConfig));
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity = CY_U3P_UART_NO_PARITY;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyFalse;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.isDma = CyTrue;

    apiRetStatus = CyU3PUartSetConfig (&uartConfig, NULL);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        app_error_handler(apiRetStatus);
    }

    /* Set the UART transfer to a really large value. */
    apiRetStatus = CyU3PUartTxSetBlockXfer (0xFFFFFFFF);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        app_error_handler(apiRetStatus);
    }

    /* Initialize the debug module. */
    apiRetStatus = CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 8);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        app_error_handler(apiRetStatus);
    }

    CyU3PDebugPreamble(CyFalse);
}


CyBool_t
lpm_request_cb (
        CyU3PUsbLinkPowerMode link_mode)
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
void
dma_cb (
        CyU3PDmaChannel   *chHandle, /* Handle to the DMA channel. */
        CyU3PDmaCbType_t  type,      /* Callback type.             */
        CyU3PDmaCBInput_t *input)    /* Callback status.           */
{
    CyU3PDmaBuffer_t buf_p;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    glDataTransStarted = CyTrue;

    /** Start/restart the timer and disable LPM **/
    CyU3PUsbLPMDisable();
    CyU3PTimerStop (&glLpmTimer);
    CyU3PTimerModify(&glLpmTimer, 100, 0);
    CyU3PTimerStart(&glLpmTimer);

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
        glDMARxCount++;
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
        glDMATxCount++;
    }
}

/*
 * Fill all DMA buffers on the IN endpoint with data. This gets data moving after an endpoint reset.
 */
static void
fill_in_buffer (
        void)
{
    CyU3PReturnStatus_t stat;
    CyU3PDmaBuffer_t    buf_p;
    uint16_t            index = 0;

    /* Now preload all buffers in the MANUAL_OUT pipe with the required data. */
    for (index = 0; index < CY_FX_BULKSRCSINK_DMA_BUF_COUNT; index++)
    {
        stat = CyU3PDmaChannelGetBuffer (&g_uvc_dma_cons, &buf_p, CYU3P_NO_WAIT);
        if (stat != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (4, "CyU3PDmaChannelGetBuffer failed, Error code = %d\n", stat);
            app_error_handler(stat);
        }

        CyU3PMemSet (buf_p.buffer, CY_FX_BULKSRCSINK_PATTERN, buf_p.size);
        stat = CyU3PDmaChannelCommitBuffer (&g_uvc_dma_cons, buf_p.size, 0);
        if (stat != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (4, "CyU3PDmaChannelCommitBuffer failed, Error code = %d\n", stat);
            app_error_handler(stat);
        }
    }
}

static volatile CyBool_t glSrcEpFlush = CyFalse;

void
ep_event_cb (
        CyU3PUsbEpEvtType evtype,
        CyU3PUSBSpeed_t   speed,
        uint8_t           epNum)
{
    /* Hit an endpoint retry case. Need to stall and flush the endpoint for recovery. */
    if (evtype == CYU3P_USBEP_SS_RETRY_EVT)
    {
        glSrcEpFlush = CyTrue;
    }
}

/* This function starts the application. This is called
 * when a SET_CONF event is received from the USB host. The endpoints
 * are configured and the DMA pipe is setup in this function. */
void
app_start (
        void)
{
    uint16_t size = get_buffer_size();

    config_endpoint(HT_BURST_LENGTH, size, HT_UVC_PROD);
    config_endpoint(HT_BURST_LENGTH, size, HT_UVC_CONS);
    config_endpoint(HT_BURST_LENGTH, size, HT_GPU_PROD);
    config_endpoint(HT_BURST_LENGTH, size, HT_GPU_CONS);

    config_dma(&g_uvc_dma_prod, size, HT_UVC_PROD_SOCKET, CY_U3P_CPU_SOCKET_CONS, CyTrue);
    config_dma(&g_uvc_dma_cons, size, CY_U3P_CPU_SOCKET_PROD, HT_UVC_CONS_SOCKET, CyFalse);
    config_dma(&g_gpu_dma_prod, size, HT_GPU_PROD_SOCKET, CY_U3P_UIB_SOCKET_CONS_4, CyTrue);
    config_dma(&g_gpu_dma_cons, size, CY_U3P_UIB_SOCKET_PROD_4, HT_GPU_CONS_SOCKET, CyFalse);
    
    CyU3PUsbRegisterEpEvtCallback (ep_event_cb, CYU3P_USBEP_SS_RETRY_EVT, 0x00, 0x02);
    fill_in_buffer ();

    /* Update the flag so that the application thread is notified of this. */
    glIsApplnActive = CyTrue;
}

/* This function stops the application. This shall be called whenever a RESET
 * or DISCONNECT event is received from the USB host. The endpoints are
 * disabled and the DMA pipe is destroyed by this function. */
void
app_stop (
        void)
{
    destroy_endpoint(&g_uvc_dma_prod, HT_UVC_PROD, HT_UVC_PROD_SOCKET);
    destroy_endpoint(&g_uvc_dma_cons, HT_UVC_CONS, HT_UVC_CONS_SOCKET);
    destroy_endpoint(&g_gpu_dma_prod, HT_GPU_PROD, HT_GPU_PROD_SOCKET);
    destroy_endpoint(&g_gpu_dma_cons, HT_GPU_CONS, HT_GPU_CONS_SOCKET);
}

/* Callback to handle the USB setup requests. */
CyBool_t
usb_setup_cb (
        uint32_t setupdat0, /* SETUP Data 0 */
        uint32_t setupdat1  /* SETUP Data 1 */
    )
{
    /* Fast enumeration is used. Only requests addressed to the interface, class,
     * vendor and unknown control requests are received by this function.
     * This application does not support any class or vendor requests. */

    uint8_t  bRequest, bReqType;
    uint8_t  bType, bTarget;
    uint16_t wValue, wIndex;
    CyBool_t isHandled = CyFalse;

    /* Decode the fields from the setup request. */
    bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
    bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
    bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);

    if (bType == CY_U3P_USB_STANDARD_RQT)
    {
        /* Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND)
         * requests here. It should be allowed to pass if the device is in configured
         * state and failed otherwise. */
        if ((bTarget == CY_U3P_USB_TARGET_INTF) && ((bRequest == CY_U3P_USB_SC_SET_FEATURE)
                    || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
        {
            if (glIsApplnActive)
            {
                CyU3PUsbAckSetup ();

                /* As we have only one interface, the link can be pushed into U2 state as soon as
                   this interface is suspended.
                 */
                if (bRequest == CY_U3P_USB_SC_SET_FEATURE)
                {
                    glDataTransStarted = CyFalse;
                    glForceLinkU2      = CyTrue;
                }
                else
                {
                    glForceLinkU2 = CyFalse;
                }
            }
            else
                CyU3PUsbStall (0, CyTrue, CyFalse);

            isHandled = CyTrue;
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
        if ((bTarget == CY_U3P_USB_TARGET_ENDPT) && (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)
                && (wValue == CY_U3P_USBX_FS_EP_HALT))
        {
            if (glIsApplnActive)
            {
                if (wIndex == HT_UVC_PROD)
                {
                    setup_cb_endpoint(wIndex, &g_uvc_dma_prod);
                    isHandled = CyTrue;
                }

                if (wIndex == HT_UVC_CONS)
                {
                    setup_cb_endpoint(wIndex, &g_uvc_dma_cons);
                    isHandled = CyTrue;
                    fill_in_buffer ();
                }

                if (wIndex == HT_GPU_PROD)
                {
                    setup_cb_endpoint(wIndex, &g_gpu_dma_prod);
                    isHandled = CyTrue;
                }

                if (wIndex == HT_GPU_CONS)
                {
                    setup_cb_endpoint(wIndex, &g_gpu_dma_cons);
                    isHandled = CyTrue;
                    fill_in_buffer ();
                }
            }
        }
    }

    if ((bType == CY_U3P_USB_VENDOR_RQT) && (bTarget == CY_U3P_USB_TARGET_DEVICE))
    {
        /* We set an event here and let the application thread below handle these requests.
         * isHandled needs to be set to True, so that the driver does not stall EP0. */
        isHandled = CyTrue;
        gl_setupdat0 = setupdat0;
        gl_setupdat1 = setupdat1;
        CyU3PEventSet (&glBulkLpEvent, CYFX_USB_CTRL_TASK, CYU3P_EVENT_OR);
    }

    return isHandled;
}

/* This is the callback function to handle the USB events. */
void
usb_event_cb (
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
        if (glIsApplnActive)
        {
            app_stop ();
        }

        /* Start the source sink function. */
        app_start ();
        break;

    case CY_U3P_USB_EVENT_RESET:
    case CY_U3P_USB_EVENT_DISCONNECT:
        glForceLinkU2 = CyFalse;

        /* Stop the source sink function. */
        if (glIsApplnActive)
        {
            app_stop ();
        }

        glDataTransStarted = CyFalse;
        break;

    case CY_U3P_USB_EVENT_EP0_STAT_CPLT:
        glEp0StatCount++;
        break;

    case CY_U3P_USB_EVENT_VBUS_REMOVED:
        if (StandbyModeEnable)
        {
            TriggerStandbyMode = CyTrue;
            StandbyModeEnable  = CyFalse;
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
CyBool_t
app_cb (
        CyU3PUsbLinkPowerMode link_mode)
{
    return CyTrue;
}

/* This function initializes the USB Module, sets the enumeration descriptors.
 * This function does not start the bulk streaming and this is done only when
 * SET_CONF event is received. */
void
app_init (void)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyBool_t no_renum = CyFalse;

    /* The fast enumeration is the easiest way to setup a USB connection,
     * where all enumeration phase is handled by the library. Only the
     * class / vendor requests need to be handled by the application. */
    CyU3PUsbRegisterSetupCallback(usb_setup_cb, CyTrue);

    /* Setup the callback to handle the USB events. */
    CyU3PUsbRegisterEventCallback(usb_event_cb);

    /* Register a callback to handle LPM requests from the USB 3.0 host. */
    CyU3PUsbRegisterLPMRequestCallback(lpm_request_cb);

    /* Start the USB functionality. */
    apiRetStatus = CyU3PUsbStart();
    if (apiRetStatus == CY_U3P_ERROR_NO_REENUM_REQUIRED)
        no_renum = CyTrue;
    else if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PUsbStart failed to Start, Error code = %d\n", apiRetStatus);
        app_error_handler(apiRetStatus);
    }

    //config_pib();
    /* Set the USB Enumeration descriptors */
    config_usb();


    gl_UsbLogBuffer = (uint8_t *)CyU3PDmaBufferAlloc (CYFX_USBLOG_SIZE);
    if (gl_UsbLogBuffer)
        CyU3PUsbInitEventLog (gl_UsbLogBuffer, CYFX_USBLOG_SIZE);

    CyU3PDebugPrint (4, "About to connect to USB host\r\n");

    /* Connect the USB Pins with super speed operation enabled. */
    if (!no_renum) {

        apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
        if (apiRetStatus != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (4, "USB Connect failed, Error code = %d\n", apiRetStatus);
            app_error_handler(apiRetStatus);
        }
    }
    else
    {
        /* USB connection is already active. Configure the endpoints and DMA channels. */
        app_start ();
    }

    CyU3PDebugPrint (8, "CyFxBulkSrcSinkApplnInit complete\r\n");
}

/*
 * De-initialize function for the USB block. Used to test USB Stop/Start functionality.
 */
void
app_deinit (
        void)
{
    if (glIsApplnActive)
        app_stop ();

    CyU3PConnectState (CyFalse, CyTrue);
    CyU3PThreadSleep (1000);
    CyU3PUsbStop ();
    CyU3PThreadSleep (1000);
}

/* Entry function for the BulkSrcSinkAppThread. */
void
app_entry (
        uint32_t input)
{
    CyU3PReturnStatus_t stat;
    uint32_t eventMask = CYFX_USB_CTRL_TASK | CYFX_USB_HOSTWAKE_TASK;   /* Events that we are interested in. */
    uint32_t eventStat;              

    uint16_t prevUsbLogIndex = 0, tmp1, tmp2;
    CyU3PUsbLinkPowerMode curState;

    /* Initialize the debug module */
    app_debug_init();
    CyU3PDebugPrint (1, "\n\ndebug initialized\r\n");

    /* Initialize the application */
    app_init();

    /* Create a timer with 100 ms expiry to enable/disable LPM transitions */ 
    CyU3PTimerCreate (&glLpmTimer, timer_cb, 0, 100, 100, CYU3P_NO_ACTIVATE);

    for (;;)
    {
        /* The following call will block until at least one of the events enabled in eventMask is received.
           The eventStat variable will hold the events that were active at the time of returning from this API.
           The CLEAR flag means that all events will be atomically cleared before this function returns.

           We cause this event wait to time out every 10 milli-seconds, so that we can periodically get the FX3
           device out of low power modes.
           */
        stat = CyU3PEventGet (&glBulkLpEvent, eventMask, CYU3P_EVENT_OR_CLEAR, &eventStat, 10);
        if (stat == CY_U3P_SUCCESS)
        {
            /* If the HOSTWAKE task is set, send a DEV_NOTIFICATION (FUNCTION_WAKE) or remote wakeup signalling
               based on the USB connection speed. */
            if (eventStat & CYFX_USB_HOSTWAKE_TASK)
            {
                CyU3PThreadSleep (1000);
                if (CyU3PUsbGetSpeed () == CY_U3P_SUPER_SPEED)
                    stat = CyU3PUsbSendDevNotification (1, 0, 0);
                else
                    stat = CyU3PUsbDoRemoteWakeup ();

                if (stat != CY_U3P_SUCCESS)
                    CyU3PDebugPrint (2, "Remote wake attempt failed with code: %d\r\n", stat);
            }

            /* If there is a pending control request, handle it here. */
            if (eventStat & CYFX_USB_CTRL_TASK)
            {
                process_command(glEp0Buffer, gl_setupdat0, gl_setupdat1, &glBulkLpEvent, gl_UsbLogBuffer, &glEp0StatCount, &StandbyModeEnable);
                
            }
        }

        if (glSrcEpFlush)
        {
            /* Stall the endpoint, so that the host can reset the pipe and continue. */
            glSrcEpFlush = CyFalse;
            CyU3PUsbStall (HT_UVC_CONS, CyTrue, CyFalse);
        }

        /* Force the USB 3.0 link to U2. */
        if (glForceLinkU2)
        {
            stat = CyU3PUsbGetLinkPowerState (&curState);
            while ((glForceLinkU2) && (stat == CY_U3P_SUCCESS) && (curState == CyU3PUsbLPM_U0))
            {
                /* Repeatedly try to go into U2 state.*/
                CyU3PUsbSetLinkPowerState (CyU3PUsbLPM_U2);
                CyU3PThreadSleep (5);
                stat = CyU3PUsbGetLinkPowerState (&curState);
            }
        }

        if (TriggerStandbyMode)
        {
            TriggerStandbyMode = CyFalse;

            CyU3PConnectState (CyFalse, CyTrue);
            CyU3PUsbStop ();
            CyU3PDebugDeInit ();
            CyU3PUartDeInit ();

            /* Add a delay to allow VBus to settle. */
            CyU3PThreadSleep (1000);

            /* VBus has been turned off. Go into standby mode and wait for VBus to be turned on again.
               The I-TCM content and GPIO register state will be backed up in the memory area starting
               at address 0x40060000. */
            stat = CyU3PSysEnterStandbyMode (CY_U3P_SYS_USB_VBUS_WAKEUP_SRC, CY_U3P_SYS_USB_VBUS_WAKEUP_SRC,
                    (uint8_t *)0x40060000);
            if (stat != CY_U3P_SUCCESS)
            {
                app_debug_init ();
                CyU3PDebugPrint (4, "Enter standby returned %d\r\n", stat);
                app_error_handler (stat);
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
                    CyU3PDebugPrint (4, "USB LOG: %x\r\n", gl_UsbLogBuffer[tmp2]);
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
void
CyFxApplicationDefine (
        void)
{
    void *ptr = NULL;
    uint32_t ret = CY_U3P_SUCCESS;

    /* Create an event flag group that will be used for signalling the application thread. */
    ret = CyU3PEventCreate (&glBulkLpEvent);
    if (ret != 0)
    {
        /* Loop indefinitely */
        while (1);
    }

    /* Allocate the memory for the threads */
    ptr = CyU3PMemAlloc (CY_FX_BULKSRCSINK_THREAD_STACK);

    /* Create the thread for the application */
    ret = CyU3PThreadCreate (&bulkSrcSinkAppThread,                /* App thread structure */
                          "21:Bulk_src_sink",                      /* Thread ID and thread name */
                          app_entry,              /* App thread entry function */
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
int
main (void)
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
    io_cfg.isDQ32Bit = CyTrue;
    io_cfg.s0Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.s1Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.useUart   = CyTrue;
    io_cfg.useI2C    = CyFalse;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyFalse;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;

    /* Enable the GPIO which would have been setup by 2-stage booter. */
    uint32_t GPIO_23 = (1 << 23);
    uint32_t GPIO_25 = (1 << 25);
    uint32_t GPIO_26 = (1 << 26);
    uint32_t GPIO_27 = (1 << 27);

    io_cfg.gpioSimpleEn[0]  = GPIO_23 | GPIO_25 | GPIO_26 | GPIO_27;
    io_cfg.gpioSimpleEn[1]  = FX3_GPIO_TO_HIFLAG(FX3_GPIO_TEST_OUT);
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    config_gpio();
    /* This is a non returnable call for initializing the RTOS kernel */
    CyU3PKernelEntry ();

    /* Dummy return to make the compiler happy */
    return 0;

handle_fatal_error:

    /* Cannot recover from this error. */
    while (1);
}

/* [ ] */

