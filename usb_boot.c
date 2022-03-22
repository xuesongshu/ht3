#include "cyfx3usb.h"
#include "cyfx3device.h"
#include "cyfx3utils.h"
#include "cyfx3gpio.h"

#include "usb_boot.h"
#include "descr.h"

static CyU3PUsbDescrPtrs *g_usb_descr = 0;
static CyFx3BootUsbEp0Pkt_t g_ep0;
uint8_t  g_usb_state = 0;
uint8_t  g_config = 0;
uint8_t  g_alt_setting = 0;
uint8_t  g_usb_dev_status = 0;
uint8_t  g_check_for_disconnect = 0;
uint8_t  g_in_compliance = 0;
uint16_t g_dev_status __attribute__ ((aligned (4))) = 0;

typedef enum {
    US_STALL = 0,
    US_DATA_IN,
    US_DATA_OUT,
    US_STATUS
} UsbStage_t;

typedef int (*PFI)();

static int get_status(void)
{
    CyBool_t bstatus;

    g_dev_status = 0;

    switch (g_ep0.bmReqType & USB_REQ_MASK) {
    case USB_REQ_DEV:
        if (CyFx3BootUsbGetSpeed () == CY_FX3_BOOT_SUPER_SPEED) {
            g_dev_status  = (g_usb_descr->usbSSConfigDesc_p[7] & 0x40) ? 1 : 0;
            g_dev_status |= g_usb_dev_status;
        } else {
            g_dev_status  = (g_usb_descr->usbHSConfigDesc_p[7] & 0x40) ? 1 : 0;
            g_dev_status |= g_usb_dev_status;
        }
        break;

    case USB_REQ_INTERFACE:
        if (!g_config) {
            return US_STALL;
        }
        break;

    case USB_REQ_ENDPOINT:
        if (CyFx3BootUsbGetEpCfg (g_ep0.bIdx0, 0, &bstatus) != 0) {
            return US_STALL;
        }
        if (bstatus)
            g_dev_status = 1;
        break;
    default:
        return US_STALL;
    }

    g_ep0.pData[0] = (uint8_t)(g_dev_status & 0xFF);
    g_ep0.pData[1] = (uint8_t)(g_dev_status >> 8);
    return US_DATA_IN;
}

int clear_feature (void)
{
    if (CyFx3BootUsbSetClrFeature (0, (CyBool_t)g_usb_state, &g_ep0) != 0) {
        return US_STALL;
    }

    if (g_ep0.bmReqType == USB_REQ_DEV) {
        if (CyFx3BootUsbGetSpeed () == CY_FX3_BOOT_SUPER_SPEED) {
            switch (g_ep0.bVal0) {
            case 48:
                g_usb_dev_status &= ~U1_ENABLE;
                break;
            case 49:
                g_usb_dev_status &= ~U2_ENABLE;
                break;
            case 50:
                g_usb_dev_status &= ~LTM_ENABLE;
                break;
            default:
                break;
            }
        } else {
            if (g_ep0.bVal0 == 1) {
                g_usb_dev_status &= ~REMOTE_WAKEUP;
            }
        }
    }

    return US_STATUS;
}

int get_stall(void)
{
    return US_STALL;
}

int set_address(void)
{
    return US_STATUS;
}

int set_feature(void)
{
    if (CyFx3BootUsbSetClrFeature (1, (CyBool_t)g_usb_state, &g_ep0) != 0) {
        return US_STALL;
    }

    if (g_ep0.bmReqType == USB_REQ_DEV) {
        if (CyFx3BootUsbGetSpeed () == CY_FX3_BOOT_SUPER_SPEED) {
            switch (g_ep0.bVal0) {
            case 48:
                g_usb_dev_status |= U1_ENABLE;
                break;
            case 49:
                g_usb_dev_status |= U2_ENABLE;
                break;
            case 50:
                g_usb_dev_status |= LTM_ENABLE;
                break;
            default:
                break;
            }
        } else {
            if (g_ep0.bVal0 == 1) {
                g_usb_dev_status |= REMOTE_WAKEUP;
            }
        }
    }
    return US_STATUS;
}

int get_descriptor(void)
{
    uint32_t len = 0;
    uint8_t *p = 0;
    uint8_t *cfg_p = 0;
    uint8_t iusb_speed;

    iusb_speed = CyFx3BootUsbGetSpeed ();

    g_usb_descr->usbHSConfigDesc_p[1] = CY_U3P_USB_CONFIG_DESCR;
    g_usb_descr->usbFSConfigDesc_p[1] = CY_U3P_USB_CONFIG_DESCR;

    if (iusb_speed == CY_FX3_BOOT_HIGH_SPEED) {
        cfg_p = (uint8_t *)g_usb_descr->usbHSConfigDesc_p;
        len = ((g_usb_descr->usbHSConfigDesc_p[3] << 8) | g_usb_descr->usbHSConfigDesc_p[2]);
    } else if (iusb_speed == CY_FX3_BOOT_SUPER_SPEED) {
        cfg_p = (uint8_t *)g_usb_descr->usbSSConfigDesc_p;
        len = ((g_usb_descr->usbSSConfigDesc_p[3] << 8) | g_usb_descr->usbSSConfigDesc_p[2]);
    } else if (iusb_speed == CY_FX3_BOOT_FULL_SPEED) {
        cfg_p = (uint8_t *)g_usb_descr->usbFSConfigDesc_p;
        len = ((g_usb_descr->usbFSConfigDesc_p[3] << 8) | g_usb_descr->usbFSConfigDesc_p[2]);
    }

    switch (g_ep0.bVal1) {
    case CY_U3P_USB_DEVICE_DESCR: {
        if ((iusb_speed == CY_FX3_BOOT_HIGH_SPEED) || (iusb_speed == CY_FX3_BOOT_FULL_SPEED)) {
            p = (uint8_t *)g_usb_descr->usbDevDesc_p;
            len = g_usb_descr->usbDevDesc_p[0];
        } else if (iusb_speed == CY_FX3_BOOT_SUPER_SPEED) {
            p = (uint8_t *)g_usb_descr->usbSSDevDesc_p;
            len = g_usb_descr->usbSSDevDesc_p[0];
        }
        break;
    }
    case CY_U3P_BOS_DESCR: {
        p = (uint8_t *)g_usb_descr->usbSSBOSDesc_p;
        len = (g_usb_descr->usbSSBOSDesc_p[3] << 8) | g_usb_descr->usbSSBOSDesc_p[2];
        break;
    }
    case CY_U3P_USB_CONFIG_DESCR: {
        p = cfg_p;
        break;
    }
    case CY_U3P_USB_DEVQUAL_DESCR: {
        if ((iusb_speed == CY_FX3_BOOT_HIGH_SPEED)  || (iusb_speed == CY_FX3_BOOT_FULL_SPEED)) {
            p = (uint8_t *)g_usb_descr->usbDevQualDesc_p;
            len = g_usb_descr->usbDevQualDesc_p[0];
            break;
        }
        return US_STALL;
    }
    case CY_U3P_USB_STRING_DESCR: {
        if (g_ep0.bVal0 < CY_FX3_USB_MAX_STRING_DESC_INDEX) {
            p = (uint8_t *)g_usb_descr->usbStringDesc_p[g_ep0.bVal0];
            if (p != 0)
                len = p[0];
        } else
            return US_STALL;
        break;
    }
    case CY_U3P_USB_OTHERSPEED_DESCR: {
        if (iusb_speed == CY_FX3_BOOT_HIGH_SPEED) {
            g_usb_descr->usbFSConfigDesc_p[1] = CY_U3P_USB_OTHERSPEED_DESCR;
            p = (uint8_t *)g_usb_descr->usbFSConfigDesc_p;

            len = ((g_usb_descr->usbFSConfigDesc_p[3] < 8) | g_usb_descr->usbFSConfigDesc_p[2]);

            if (len > g_ep0.wLen) {
                len = g_ep0.wLen;
            }
        } else if (iusb_speed == CY_FX3_BOOT_FULL_SPEED) {
            g_usb_descr->usbHSConfigDesc_p[1] = CY_U3P_USB_OTHERSPEED_DESCR;
            p = (uint8_t *)g_usb_descr->usbHSConfigDesc_p;
            len = ((g_usb_descr->usbHSConfigDesc_p[3] < 8) | g_usb_descr->usbHSConfigDesc_p[2]);

            if (len > g_ep0.wLen) {
                len = g_ep0.wLen;
            }
        }
    }
    break;
    default: {
        return US_STALL;
    }
    }

    if (p != 0) {
        CyFx3BootMemCopy ((uint8_t *)USB_ADDRESS, p, len);
        if (g_ep0.wLen > len) {
            g_ep0.wLen = len;
        }

        return US_DATA_IN;
    } else
        return US_STALL;
}

int get_config (void)
{
    g_ep0.pData = (uint8_t *)&g_config;
    return US_DATA_IN;
}


int set_config (void)
{
    uint8_t iusb_speed = 0;
    uint32_t iret_val  = 0;
    CyFx3BootUsbEpConfig_t ep_cfg;

    if ((g_ep0.bVal0 == 0) || (g_ep0.bVal0 == 1)) {
        g_usb_state = g_ep0.bVal0;
        g_config = g_ep0.bVal0;
        iusb_speed = CyFx3BootUsbGetSpeed();
        ep_cfg.pcktSize = 512;
        if (iusb_speed == CY_FX3_BOOT_HIGH_SPEED) {
            ep_cfg.pcktSize = 512;
        }
        if (iusb_speed == CY_FX3_BOOT_SUPER_SPEED) {
            ep_cfg.pcktSize = 1024;
        }
        if (iusb_speed == CY_FX3_BOOT_FULL_SPEED) {
            ep_cfg.pcktSize = 64;
        }

        ep_cfg.enable = 1;
        ep_cfg.epType = CY_FX3_BOOT_USB_EP_BULK;
        ep_cfg.burstLen = 1;
        ep_cfg.streams = 0;
        ep_cfg.isoPkts = 0;

        iret_val = CyFx3BootUsbSetEpConfig(PRODUCER1, &ep_cfg);
        if (iret_val != 0) {
            return US_STALL;
        }

        ep_cfg.enable = 1;
        ep_cfg.epType = CY_FX3_BOOT_USB_EP_BULK;
        ep_cfg.burstLen = 1;
        ep_cfg.streams = 0;
        ep_cfg.isoPkts = 0;

        iret_val = CyFx3BootUsbSetEpConfig (CONSUMER1, &ep_cfg);
        if (iret_val != 0) {
            return US_STALL;
        }

        return US_STATUS;
    }

    return US_STALL;
}

int get_interface (void)
{
    if (g_config == 0) {
        return US_STALL;
    }

    g_ep0.pData = (uint8_t *)&g_alt_setting;
    return US_DATA_IN;
}

int set_interface (void)
{
    g_alt_setting = g_ep0.bVal0;
    return US_STATUS;
}

const PFI chapter9_cmds[] = {
    get_status,           /* USB_GET_STATUS         0 */
    clear_feature,        /* USB_CLEAR_FEATURE      1 */
    get_stall,            /* Reserved               2 */
    set_feature,          /* USB_SET_FEATURE        3 */
    get_stall,            /* Reserved               4 */
    set_address,          /* USB_SET_ADDRESS        5 */
    get_descriptor,       /* USB_GET_DESCRIPTOR     6 */
    get_stall,            /* USB_SET_DESCRIPTOR     7 */
    get_config,           /* USB_GET_CONFIGURATION  8 */
    set_config,           /* USB_SET_CONFIGURATION  9 */
    get_interface,        /* USB_GET_INTERFACE     10 */
    set_interface,        /* USB_SET_INTERFACE     11 */
};

int check_address (uint32_t address, uint32_t len)
{
    if (address & 3) {
        return -1;
    }
    len += address;
    if ((address >= CY_FX3_BOOT_SYSMEM_BASE1) && (len <= CY_FX3_BOOT_SYSMEM_END)) {
        return 0;
    }
    if (len <= CY_FX3_BOOT_ITCM_END) {
        return 0;
    }

    return -1;
}

void vendor_cmd_handler (void)
{
    int stage;
    int status;
    uint32_t address  = ((g_ep0.bIdx1 << 24) |
                         (g_ep0.bIdx0 << 16) |
                         (g_ep0.bVal1 << 8) |
                         (g_ep0.bVal0));
    uint16_t len  = g_ep0.wLen;
    uint16_t breq = g_ep0.bReq;
    uint16_t dir  = g_ep0.bmReqType & USB_SETUP_DIR;

    if (len > USB_DATA_BUF_SIZE) {
        CyFx3BootUsbStall (0, CyTrue, CyFalse);
        return;
    }

    if (dir) {
        stage = US_DATA_IN;
    } else {
        stage = US_DATA_OUT;
    }

    if (breq == 0xA0) {
        if (address == 0xE600) {
            CyFx3BootUsbStall (0, CyTrue, CyFalse);
            return;
        }

        status = check_address (address, len);
        if (len == 0) {
            CyFx3BootUsbConnect (CyFalse, CyTrue);
            CyFx3BootGpioSetValue (50, CyTrue);
            CyFx3BootJumpToProgramEntry (address);
            return;
        }

        if (status < 0) {
            CyFx3BootUsbStall (0, CyTrue, CyFalse);
            return;
        }
        if ((address >= CY_FX3_BOOT_SYSMEM_BASE1) && (address < CY_FX3_BOOT_SYSMEM_END)) {
            g_ep0.pData = (uint8_t *)address;
        }
        CyFx3BootUsbAckSetup ();

        if (US_DATA_IN == stage) {
            if ((address + g_ep0.wLen) <= CY_FX3_BOOT_ITCM_END) {
                CyFx3BootMemCopy(g_ep0.pData, (uint8_t *)address, len);
            }

            status = CyFx3BootUsbDmaXferData (
                         0x80,
                         (uint32_t)g_ep0.pData,
                         g_ep0.wLen,
                         CY_FX3_BOOT_WAIT_FOREVER);
            if (status != CY_FX3_BOOT_SUCCESS) {
                CyFx3BootUsbStall (0, CyTrue, CyFalse);
                return;
            }
        } else if (stage == US_DATA_OUT) {
            status = CyFx3BootUsbDmaXferData (
                         0x00,
                         (uint32_t)g_ep0.pData,
                         g_ep0.wLen,
                         CY_FX3_BOOT_WAIT_FOREVER);
            if (status != CY_FX3_BOOT_SUCCESS) {
                CyFx3BootUsbStall (0, CyTrue, CyFalse);
                return;
            }
            if ((address + g_ep0.wLen) <= CY_FX3_BOOT_ITCM_END) {
                if (address < 0xFF) {
                    g_ep0.pData += 0xFF - address;
                    g_ep0.wLen -= 0xFF - address;
                    address = 0xFF;
                }
                CyFx3BootMemCopy((uint8_t *)address, g_ep0.pData, g_ep0.wLen);
            }
        }
        return;
    }
    CyFx3BootUsbStall (0, CyTrue, CyFalse);
    return;
}

void setup_handler(uint32_t idata0, uint32_t idata1)
{
    uint32_t *p;
    int status = US_STALL;

    p = (uint32_t *)&g_ep0;
    p[0] = idata0;
    p[1] = idata1;

    g_ep0.pData = (uint8_t *)USB_ADDRESS;

    switch (g_ep0.bmReqType & USB_SETUP_MASK) {
    case USB_STANDARD_REQUEST:
        if (g_ep0.bReq <= USB_SET_INTERFACE) {
            status = (*chapter9_cmds[g_ep0.bReq])();
        } else {
            if (g_ep0.bReq == USB_SC_SET_SEL) {
                if ((CyFx3BootUsbGetSpeed () == CY_FX3_BOOT_SUPER_SPEED) &&
                        (g_ep0.bIdx0 == 0) &&
                        (g_ep0.bIdx1 == 0) &&
                        (g_ep0.bVal0 == 0) &&
                        (g_ep0.bVal1 == 0) &&
                        (g_ep0.wLen == 6)) {
                    g_ep0.wLen = 32;
                    status = US_DATA_OUT;
                } else {
                    status = US_STALL;
                }
            } else if (g_ep0.bReq == USB_SC_SET_ISOC_DELAY) {
                status = US_STATUS;
                if ((CyFx3BootUsbGetSpeed () != CY_FX3_BOOT_SUPER_SPEED) ||
                        (g_ep0.bIdx0 != 0) ||
                        (g_ep0.bIdx1 != 0) ||
                        (g_ep0.wLen != 0)) {
                    status = US_STALL;
                }
            } else {
                status = US_STALL;
            }
        }
        break;

    case USB_VENDOR_REQUEST:
        vendor_cmd_handler();
        return;
    }

    switch (status) {
    case US_DATA_IN:
        CyFx3BootUsbAckSetup ();
        status = CyFx3BootUsbDmaXferData (0x80, (uint32_t)g_ep0.pData, g_ep0.wLen, 1000);
        if (status != CY_FX3_BOOT_SUCCESS) {
            CyFx3BootUsbStall (0, CyTrue, CyFalse);
        }
        break;
    case US_DATA_OUT:
        CyFx3BootUsbAckSetup ();
        status = CyFx3BootUsbDmaXferData (
                     0x00,
                     (uint32_t)g_ep0.pData,
                     g_ep0.wLen,
                     CY_FX3_BOOT_WAIT_FOREVER);
        if (status != CY_FX3_BOOT_SUCCESS) {
            CyFx3BootUsbStall (0, CyTrue, CyFalse);
        }
        break;
    case US_STATUS:
        CyFx3BootUsbAckSetup ();
        break;
    default:
        CyFx3BootUsbStall (0, CyTrue, CyFalse);
        return;
    }

    return;
}

void event_handler(CyFx3BootUsbEventType_t ievent)
{
    if (ievent == CY_FX3_BOOT_USB_RESET) {
        g_config = 0;
        g_alt_setting = 0;
        g_usb_dev_status = 0;
        g_usb_state = 0;
        g_in_compliance = 0;
    }

    if ((ievent == CY_FX3_BOOT_USB_CONNECT) || (ievent == CY_FX3_BOOT_USB_DISCONNECT)) {
        g_usb_state    = 0;
        g_usb_dev_status = 0;
    }

    if (ievent == CY_FX3_BOOT_USB_IN_SS_DISCONNECT) {
        g_check_for_disconnect = CyTrue;
    }

    if (ievent == CY_FX3_BOOT_USB_COMPLIANCE) {
        g_in_compliance = CyTrue;
    }
}

UsbStage_t endpoint_config()
{
    CyFx3BootUsbSpeed_t ispeed;
    CyFx3BootUsbEpConfig_t ep_config;
    CyFx3BootErrorCode_t ierror;

    CyFx3BootMemSet((uint8_t *)&ep_config, 0, sizeof(CyFx3BootUsbEpConfig_t));
    if (g_ep0.bVal0 == 0 || g_ep0.bVal0 == 1) {
        g_usb_state = g_ep0.bVal0;
        g_config = g_ep0.bVal0;
        ispeed = CyFx3BootUsbGetSpeed();
        if (ispeed == CY_FX3_BOOT_HIGH_SPEED) {
            ep_config.pcktSize = 512;
            ep_config.burstLen = 2;
        } else if (ispeed == CY_FX3_BOOT_SUPER_SPEED) {
            ep_config.pcktSize = 1024;
            ep_config.burstLen = 16;
        } else {
            ep_config.pcktSize = 64;
            ep_config.burstLen = 1;
        }
        ep_config.enable = 1;
        ep_config.epType = CY_FX3_BOOT_USB_EP_BULK;
        ierror = CyFx3BootUsbSetEpConfig(PRODUCER1, &ep_config);
        if (ierror != CY_FX3_BOOT_SUCCESS)
            return US_STALL;
        CyFx3BootUsbSetEpConfig(CONSUMER1, &ep_config);
        if (ierror != CY_FX3_BOOT_SUCCESS)
            return US_STALL;
        return US_STATUS;
    }
    return US_STALL;
}


extern unsigned char g_usb30_device[];
extern unsigned char g_usb20_device[];
extern unsigned char g_bos[];
extern unsigned char g_device_qual[];
extern unsigned char g_ss_config[];
extern unsigned char g_hs_config[];
extern unsigned char g_fs_config[];
extern unsigned char g_string_lang[];
extern unsigned char g_manufacture[];
extern unsigned char g_product[];
extern unsigned char g_descriptor_align[];
extern unsigned char g_serial_number[];

void usb_boot()
{
    CyFx3BootErrorCode_t apiRetStatus;
    CyBool_t no_renum = CyFalse;

    g_config = 0;
    g_alt_setting = 0;
    g_usb_dev_status = 0;
    g_usb_state = 0;
    g_check_for_disconnect = 0;
    g_in_compliance = 0;

    apiRetStatus = CyFx3BootUsbStart (CyTrue, event_handler);
    if (apiRetStatus == CY_FX3_BOOT_ERROR_NO_REENUM_REQUIRED)
        no_renum = CyTrue;

    CyFx3BootRegisterSetupCallback (setup_handler);

    if (!no_renum) {
        CyFx3BootUsbSetDesc (CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, g_usb30_device);
        CyFx3BootUsbSetDesc (CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, g_fs_config);
        CyFx3BootUsbSetDesc (CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, g_ss_config);
        CyFx3BootUsbSetDesc (CY_U3P_USB_SET_SS_BOS_DESCR, 0, g_bos);

        CyFx3BootUsbSetDesc (CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, g_usb20_device);
        CyFx3BootUsbSetDesc (CY_U3P_USB_SET_DEVQUAL_DESCR, 0, g_device_qual);
        CyFx3BootUsbSetDesc (CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, g_hs_config);

        CyFx3BootUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 0, g_string_lang);
        CyFx3BootUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 1, g_manufacture);
        CyFx3BootUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 2, g_product);
        CyFx3BootUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 3, g_serial_number);

        g_usb_descr = CyFx3BootUsbGetDesc ();
        CyFx3BootUsbConnect (CyTrue, CyTrue);
    } else {
        g_usb_descr = CyFx3BootUsbGetDesc ();
    }
    g_ep0.bVal0 = 1;
    set_config ();
}
