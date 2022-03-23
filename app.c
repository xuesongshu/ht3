#include "app.h"
#include "main.h"
#include "descr.h"

#include "cyu3system.h"
#include "cyu3usb.h"
#include "cyu3os.h"
#include "cyu3error.h"
#include "cyu3uart.h"
#include "cyu3utils.h"
#include "uart_regs.h"

CyU3PThread g_app_thread;
CyU3PDmaChannel g_usb_to_uart;
CyU3PDmaChannel g_uart_to_usb;
CyU3PUartConfig_t g_uart_config;
volatile uint16_t g_pkts_pending = 0;

void app_init(void);
void app_stop(void);
void app_start(void);
void app_error_handler(CyU3PReturnStatus_t ierror);
void dma_callback(CyU3PDmaChannel *handle, CyU3PDmaCbType_t itype, CyU3PDmaCBInput_t *input);
void app_usb_event_callback(CyU3PUsbEventType_t ievtype, uint16_t ievdata);
CyBool_t app_usb_setup_callback(uint32_t idat0, uint32_t idat1);
CyBool_t app_lpm_rqt_callback(CyU3PUsbLinkPowerMode ilink_mode);

void app_thread_entry(uint32_t iinput)
{
    uint32_t ireg_en = 0, ireg_ds = 0;
    CyU3PMemSet((uint8_t *)&g_uart_config, 0, sizeof(CyU3PUartConfig_t));
    app_init();
    ireg_en = UART->lpp_uart_config;
    ireg_ds = UART->lpp_uart_config & (~(CY_U3P_LPP_UART_RTS | CY_U3P_LPP_UART_RX_ENABLE));
    for (;;) {
        if (g_is_app_active) {
            if (g_pkts_pending == 0) {
                UART->lpp_uart_config = ireg_ds;
                CyU3PDmaChannelSetWrapUp(&g_uart_to_usb);
                UART->lpp_uart_config = ireg_en;
            }
            g_pkts_pending = 0;
        }
        CyU3PThreadSleep(50);
    }
}

void app_init(void)
{
    CyU3PReturnStatus_t iapi_ret = CY_U3P_SUCCESS;
    iapi_ret = CyU3PUsbStart();
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
    iapi_ret = CyU3PUartInit();
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
    CyU3PMemSet((uint8_t *)&g_uart_config, 0, sizeof(g_uart_config));
    g_uart_config.baudRate = CY_U3P_UART_BAUDRATE_115200;
    g_uart_config.stopBit = CY_U3P_UART_ONE_STOP_BIT;
    g_uart_config.parity = CY_U3P_UART_NO_PARITY;
    g_uart_config.flowCtrl = CyFalse;
    g_uart_config.txEnable = CyTrue;
    g_uart_config.rxEnable = CyTrue;
    g_uart_config.isDma = CyTrue;
    iapi_ret = CyU3PUartSetConfig(&g_uart_config, NULL);
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
    CyU3PUsbRegisterSetupCallback(app_usb_setup_callback, CyTrue);
    CyU3PUsbRegisterEventCallback(app_usb_event_callback);
    CyU3PUsbRegisterLPMRequestCallback(app_lpm_rqt_callback);
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)g_usb30_device);
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)g_usb20_device);
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)g_bos);
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)g_device_qual);
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)g_ss_config);
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)g_hs_config);
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)g_fs_config);
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)g_string_lang);
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)g_manufacture);
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
    iapi_ret = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)g_product);
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
    iapi_ret = CyU3PConnectState(CyTrue, CyTrue);
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
}

void app_stop(void)
{
    CyU3PEpConfig_t ep_cfg;
    CyU3PReturnStatus_t iapi_ret = CY_U3P_SUCCESS;
    g_is_app_active = CyFalse;
    CyU3PUsbFlushEp(EP_PRODUCER);
    CyU3PUsbFlushEp(EP_CONSUMER);
    CyU3PUsbFlushEp(EP_INTERRUPT);
    CyU3PDmaChannelDestroy(&g_usb_to_uart);
    CyU3PDmaChannelDestroy(&g_uart_to_usb);
    CyU3PMemSet((uint8_t *)&ep_cfg, 0, sizeof(ep_cfg));
    ep_cfg.enable = CyFalse;
    iapi_ret = CyU3PSetEpConfig(EP_PRODUCER, &ep_cfg);
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
    iapi_ret = CyU3PSetEpConfig(EP_CONSUMER, &ep_cfg);
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
    iapi_ret = CyU3PSetEpConfig(EP_INTERRUPT, &ep_cfg);
    if (iapi_ret != CY_U3P_SUCCESS) {
        app_error_handler(iapi_ret);
    }
}

void app_start(void)
{
    uint16_t size = 0;
    CyU3PEpConfig_t ep_cfg;
    CyU3PDmaChannelConfig_t dma_cfg;
    CyU3PReturnStatus_t iret = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t iusb_speed = CyU3PUsbGetSpeed();
    switch (iusb_speed) {
    case CY_U3P_FULL_SPEED:
        size = 64;
        break;
    case CY_U3P_HIGH_SPEED:
        size = 512;
        break;
    case  CY_U3P_SUPER_SPEED:
        CyU3PUsbLPMDisable();
        size = 1024;
        break;
    default:
        app_error_handler(CY_U3P_ERROR_FAILURE);
        break;
    }
    CyU3PMemSet((uint8_t *)&ep_cfg, 0, sizeof(ep_cfg));
    ep_cfg.enable = CyTrue;
    ep_cfg.epType = CY_U3P_USB_EP_BULK;
    ep_cfg.burstLen = 16;
    ep_cfg.streams = 0;
    ep_cfg.pcktSize = size;
    iret = CyU3PSetEpConfig(EP_PRODUCER, &ep_cfg);
    if (iret != CY_U3P_SUCCESS) {
        app_error_handler(iret);
    }
    iret = CyU3PSetEpConfig(EP_CONSUMER, &ep_cfg);
    if (iret != CY_U3P_SUCCESS) {
        app_error_handler(iret);
    }
    ep_cfg.epType = CY_U3P_USB_EP_INTR;
    ep_cfg.pcktSize = 64;
    ep_cfg.isoPkts = 1;
    iret = CyU3PSetEpConfig(EP_INTERRUPT, &ep_cfg);
    if (iret != CY_U3P_SUCCESS) {
        app_error_handler(iret);
    }
    dma_cfg.size = size;
    dma_cfg.count = DMA_BUF_COUNT;
    dma_cfg.prodSckId = EP_PRODUCER1_SOCKET;
    dma_cfg.consSckId = EP_CONSUMER1_SOCKET;
    dma_cfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dma_cfg.notification = 0;
    dma_cfg.cb = NULL;
    dma_cfg.prodHeader = 0;
    dma_cfg.prodFooter = 0;
    dma_cfg.consHeader = 0;
    dma_cfg.prodAvailCount = 0;
    iret = CyU3PDmaChannelCreate(&g_usb_to_uart,
                                 CY_U3P_DMA_TYPE_AUTO, &dma_cfg);
    if (iret != CY_U3P_SUCCESS) {
        app_error_handler(iret);
    }
    dma_cfg.size         = 32;
    dma_cfg.prodSckId    = EP_PRODUCER2_SOCKET;
    dma_cfg.consSckId    = EP_CONSUMER2_SOCKET;
    dma_cfg.notification = CY_U3P_DMA_CB_PROD_EVENT;
    dma_cfg.cb           = dma_callback;
    iret = CyU3PDmaChannelCreate(&g_uart_to_usb,
                                 CY_U3P_DMA_TYPE_MANUAL, &dma_cfg);
    if (iret != CY_U3P_SUCCESS) {
        app_error_handler(iret);
    }
    iret = CyU3PDmaChannelSetXfer(&g_usb_to_uart, 0);
    if (iret != CY_U3P_SUCCESS) {
        app_error_handler(iret);
    }
    iret = CyU3PDmaChannelSetXfer(&g_uart_to_usb, 0);
    if (iret != CY_U3P_SUCCESS) {
        app_error_handler(iret);
    }
    g_is_app_active = CyTrue;
}

void dma_callback(CyU3PDmaChannel *handle, CyU3PDmaCbType_t itype, CyU3PDmaCBInput_t *input)
{
    if (itype == CY_U3P_DMA_CB_PROD_EVENT) {
        CyU3PDmaChannelCommitBuffer(&g_uart_to_usb, input->buffer_p.count, 0);
        g_pkts_pending++;
    }
}

void app_error_handler(CyU3PReturnStatus_t ierror)
{
    while (1) {
        CyU3PThreadSleep(100);
    }
}

CyBool_t app_usb_setup_callback(uint32_t idat0, uint32_t idat1)
{
    uint16_t readCount = 0;
    uint8_t  brequest, ireq_type;
    uint8_t  itype, itarget;
    uint16_t ivalue;
    uint8_t config_data[7];
    CyBool_t bis_handled = CyFalse;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PReturnStatus_t iapi_ret = CY_U3P_SUCCESS;
    CyU3PUartConfig_t uart_config;
    ireq_type = (idat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    itype    = (ireq_type & CY_U3P_USB_TYPE_MASK);
    itarget  = (ireq_type & CY_U3P_USB_TARGET_MASK);
    brequest = ((idat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    ivalue   = ((idat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    if (itype == CY_U3P_USB_STANDARD_RQT) {
        if ((itarget == CY_U3P_USB_TARGET_INTF) && ((brequest == CY_U3P_USB_SC_SET_FEATURE)
                                                    || (brequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (ivalue == 0)) {
            if (g_is_app_active)
                CyU3PUsbAckSetup();
            else
                CyU3PUsbStall(0, CyTrue, CyFalse);
            bis_handled = CyTrue;
        }
    }
    if (itype == CY_U3P_USB_CLASS_RQT) {
        bis_handled = CyTrue;
        if (brequest == SET_LINE_CODING) {
            status = CyU3PUsbGetEP0Data(0x07, config_data, &readCount);
            if (status != CY_U3P_SUCCESS) {
                app_error_handler(status);
            }
            if (readCount != 0x07) {
                app_error_handler(CY_U3P_ERROR_BAD_SIZE);
            } else {
                CyU3PMemSet((uint8_t *)&uart_config, 0, sizeof(uart_config));
                uart_config.baudRate = (CyU3PUartBaudrate_t)(config_data[0] | (config_data[1] << 8) |
                                                             (config_data[2] << 16) | (config_data[3] << 24));
                if (config_data[4] == 0) {
                    uart_config.stopBit = CY_U3P_UART_ONE_STOP_BIT;
                } else if (config_data[4] == 2) {
                    uart_config.stopBit = CY_U3P_UART_TWO_STOP_BIT;
                } else {
                    uart_config.stopBit = (CyU3PUartStopBit_t)0;
                }
                if (config_data[5] == 1) {
                    uart_config.parity = CY_U3P_UART_ODD_PARITY;
                } else if (config_data[5] == 2) {
                    uart_config.parity = CY_U3P_UART_EVEN_PARITY;
                } else {
                    uart_config.parity = CY_U3P_UART_NO_PARITY;
                }
                uart_config.txEnable = CyTrue;
                uart_config.rxEnable = CyTrue;
                uart_config.flowCtrl = CyFalse;
                uart_config.isDma = CyTrue;
                iapi_ret = CyU3PUartSetConfig(&uart_config, NULL);
                if (iapi_ret == CY_U3P_SUCCESS) {
                    CyU3PMemCopy((uint8_t *)&g_uart_config, (uint8_t *)&uart_config,
                                 sizeof(CyU3PUartConfig_t));
                }
            }
        } else if (brequest == GET_LINE_CODING) {
            config_data[0] = g_uart_config.baudRate & (0x000000FF);
            config_data[1] = ((g_uart_config.baudRate & (0x0000FF00)) >> 8);
            config_data[2] = ((g_uart_config.baudRate & (0x00FF0000)) >> 16);
            config_data[3] = ((g_uart_config.baudRate & (0xFF000000)) >> 24);
            if (g_uart_config.stopBit == CY_U3P_UART_ONE_STOP_BIT) {
                config_data[4] = 0;
            } else {
                config_data[4] = 2;
            }
            if (g_uart_config.parity == CY_U3P_UART_EVEN_PARITY) {
                config_data[5] = 2;
            } else if (g_uart_config.parity == CY_U3P_UART_ODD_PARITY) {
                config_data[5] = 1;
            } else {
                config_data[5] = 0;
            }
            config_data[6] =  0x08;
            status = CyU3PUsbSendEP0Data(0x07, config_data);
            if (status != CY_U3P_SUCCESS) {
                app_error_handler(status);
            }
        } else if (brequest == SET_CONTROL_LINE_STATE) {
            if (g_is_app_active) {
                CyU3PUsbAckSetup();
            } else
                CyU3PUsbStall(0, CyTrue, CyFalse);
        } else {
            status = CY_U3P_ERROR_FAILURE;
        }
        if (status != CY_U3P_SUCCESS) {
            bis_handled = CyFalse;
        }
    }
    return bis_handled;
}

void app_usb_event_callback(CyU3PUsbEventType_t ievtype, uint16_t ievdata)
{
    switch (ievtype) {
    case CY_U3P_USB_EVENT_SETCONF:
        if (g_is_app_active) {
            app_stop();
        }
        app_start();
        break;
    case CY_U3P_USB_EVENT_RESET:
    case CY_U3P_USB_EVENT_CONNECT:
    case CY_U3P_USB_EVENT_DISCONNECT:
        if (g_is_app_active) {
            CyU3PUsbLPMEnable();
            app_stop();
        }
        break;
    default:
        break;
    }
}

CyBool_t app_lpm_rqt_callback(CyU3PUsbLinkPowerMode ilink_mode)
{
    return CyTrue;
}