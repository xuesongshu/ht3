//专用于模块化优化app.c
#include "module.h"
#include "../config.h"
#include "../usb_descr.h"

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3usb.h"
#include "cyu3uart.h"

void config_endpoint(uint16_t burst_len, uint16_t size, uint16_t ep_id)
{
    CyU3PReturnStatus_t iapi_ret = CY_U3P_SUCCESS;
    CyU3PEpConfig_t ep_cfg;

    CyU3PMemSet ((uint8_t *)&ep_cfg, 0, sizeof (ep_cfg));
    ep_cfg.enable = CyTrue;
    ep_cfg.epType = CY_U3P_USB_EP_BULK;
    ep_cfg.burstLen = burst_len;
    ep_cfg.streams = 0;
    ep_cfg.pcktSize = size;
    iapi_ret = CyU3PSetEpConfig(ep_id, &ep_cfg);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", iapi_ret);
        app_error_handler (iapi_ret);
    }
    CyU3PUsbFlushEp(ep_id);
}

void config_dma(CyU3PDmaChannel* dma, uint16_t size, uint16_t socket_id, CyBool_t is_producer)
{
    CyU3PReturnStatus_t iapi_ret = CY_U3P_SUCCESS;
    CyU3PDmaChannelConfig_t dma_cfg;

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
    if(is_producer)
    {
        dma_cfg.notification = CY_U3P_DMA_CB_PROD_EVENT;
        dma_cfg.prodSckId = socket_id;
        dma_cfg.consSckId = CY_U3P_CPU_SOCKET_CONS;
    }
    else
    {
        dma_cfg.notification = CY_U3P_DMA_CB_CONS_EVENT;
        dma_cfg.prodSckId = CY_U3P_CPU_SOCKET_PROD;
        dma_cfg.consSckId = socket_id;
    }
    
    dma_cfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dma_cfg.cb = dma_cb;
    dma_cfg.prodHeader = 0;
    dma_cfg.prodFooter = 0;
    dma_cfg.consHeader = 0;
    dma_cfg.prodAvailCount = 0;

    if(is_producer)
    {
        iapi_ret = CyU3PDmaChannelCreate (dma,CY_U3P_DMA_TYPE_MANUAL_IN, &dma_cfg);
    }
    else
    {
        iapi_ret = CyU3PDmaChannelCreate (dma,CY_U3P_DMA_TYPE_MANUAL_OUT, &dma_cfg);
    }
    
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PDmaChannelCreate failed, Error code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }
    /* Set DMA Channel transfer size */
    iapi_ret = CyU3PDmaChannelSetXfer (dma, CY_FX_BULKSRCSINK_DMA_TX_SIZE);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PDmaChannelSetXfer failed, Error code = %d\n", iapi_ret);
        app_error_handler(iapi_ret);
    }
}

uint16_t get_buffer_size()
{
    uint16_t size = 0;
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
    return size;
}

void destroy_endpoint(CyU3PDmaChannel *dma, uint16_t ep_id, uint16_t socket_id)
{
    CyU3PEpConfig_t ep_cfg;
    CyU3PReturnStatus_t iapi_ret = CY_U3P_SUCCESS;

    CyU3PDmaChannelDestroy (dma);
    CyU3PUsbFlushEp(ep_id);

    CyU3PMemSet ((uint8_t *)&ep_cfg, 0, sizeof (ep_cfg));
    ep_cfg.enable = CyFalse;
    iapi_ret = CyU3PSetEpConfig(ep_id, &ep_cfg);
    if (iapi_ret != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", iapi_ret);
        app_error_handler (iapi_ret);
    }
}

void setup_cb_endpoint(uint16_t ep_id, CyU3PDmaChannel *dma)
{
    CyU3PUsbSetEpNak (ep_id, CyTrue);
    CyU3PBusyWait (125);

    CyU3PDmaChannelReset (dma);
    CyU3PUsbFlushEp(ep_id);
    CyU3PUsbResetEp (ep_id);
    CyU3PUsbSetEpNak (ep_id, CyFalse);

    CyU3PDmaChannelSetXfer (dma, CY_FX_BULKSRCSINK_DMA_TX_SIZE);
    CyU3PUsbStall (ep_id, CyFalse, CyTrue);
    CyU3PUsbAckSetup ();
}

void process_command(uint8_t *ep0_buffer, 
    uint32_t setupdat0, 
    uint32_t setupdat1, 
    CyU3PEvent *bulk_event,
    uint8_t *usb_log_buffer,
    volatile uint32_t* ep0_stat_count,
    CyBool_t* standby_mode_enable
)
{   
    uint8_t  ivendor_rqt_cnt = 0;
    uint8_t  irequest, itype;
    uint16_t ilength, temp;
    uint16_t ivalue, iindex;

    /* Decode the fields from the setup request. */
    itype = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    irequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    ilength  = ((setupdat1 & CY_U3P_USB_LENGTH_MASK)  >> CY_U3P_USB_LENGTH_POS);
    ivalue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK) >> CY_U3P_USB_VALUE_POS);
    iindex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK) >> CY_U3P_USB_INDEX_POS);

    if ((itype & CY_U3P_USB_TYPE_MASK) == CY_U3P_USB_VENDOR_RQT)
    {
        switch (irequest)
        {
        case 0x76:
            ep0_buffer[0] = ivendor_rqt_cnt;
            ep0_buffer[1] = ~ivendor_rqt_cnt;
            ep0_buffer[2] = 1;
            ep0_buffer[3] = 5;
            CyU3PUsbSendEP0Data (ilength, ep0_buffer);
            ivendor_rqt_cnt++;
            break;

        case 0x77:      /* Trigger remote wakeup. */
            CyU3PUsbAckSetup ();
            CyU3PEventSet (bulk_event, CYFX_USB_HOSTWAKE_TASK, CYU3P_EVENT_OR);
            break;

        case 0x78:      /* Get count of EP0 status events received. */
            CyU3PMemCopy ((uint8_t *)ep0_buffer, ((uint8_t *)&ep0_stat_count), 4);
            CyU3PUsbSendEP0Data (4, ep0_buffer);
            break;

        case 0x79:      /* Request with no data phase. Insert a delay and then ACK the request. */
            CyU3PThreadSleep (5);
            CyU3PUsbAckSetup ();
            break;

        case 0x80:      /* Request with OUT data phase. Just get the data and ignore it for now. */
            CyU3PUsbGetEP0Data (sizeof (ep0_buffer), (uint8_t *)ep0_buffer, &ilength);
            break;

        case 0x81:
            /* Get the current event log index and send it to the host. */
            if (ilength == 2)
            {
                temp = CyU3PUsbGetEventLogIndex ();
                CyU3PMemCopy ((uint8_t *)ep0_buffer, (uint8_t *)&temp, 2);
                CyU3PUsbSendEP0Data (2, ep0_buffer);
            }
            else
                CyU3PUsbStall (0, CyTrue, CyFalse);
            break;

        case 0x82:
            /* Send the USB event log buffer content to the host. */
            if (ilength != 0)
            {
                if (ilength < CYFX_USBLOG_SIZE)
                    CyU3PUsbSendEP0Data (ilength, usb_log_buffer);
                else
                    CyU3PUsbSendEP0Data (CYFX_USBLOG_SIZE, usb_log_buffer);
            }
            else
                CyU3PUsbAckSetup ();
            break;

        case 0x83:
            {
                uint32_t addr = ((uint32_t)ivalue << 16) | (uint32_t)iindex;
                CyU3PReadDeviceRegisters ((uvint32_t *)addr, 1, (uint32_t *)ep0_buffer);
                CyU3PUsbSendEP0Data (4, ep0_buffer);
            }
            break;

        case 0x84:
            {
                uint8_t major, minor, patch;

                if (CyU3PUsbGetBooterVersion (&major, &minor, &patch) == CY_U3P_SUCCESS)
                {
                    ep0_buffer[0] = major;
                    ep0_buffer[1] = minor;
                    ep0_buffer[2] = patch;
                    CyU3PUsbSendEP0Data (3, ep0_buffer);
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
            *standby_mode_enable = CyTrue;
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
