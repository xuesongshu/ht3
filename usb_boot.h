#ifndef __USB_BOOT_H__
#define __USB_BOOT_H__

#define USB_ADDRESS           0x40077800
#define USB_SETUP_DIR         0x80
#define USB_SETUP_MASK        0x60
#define USB_STANDARD_REQUEST  0x00
#define USB_VENDOR_REQUEST    0x40
#define USB_REQ_MASK          0x3F
#define USB_REQ_DEV           0
#define USB_REQ_INTERFACE     1
#define USB_REQ_ENDPOINT      2
#define USB_SET_INTERFACE     11
#define USB_SC_SET_SEL        0x30
#define USB_SC_SET_ISOC_DELAY 0x31
#define USB_DATA_BUF_SIZE     1024*4

#define SELF_POWERED          0x01
#define REMOTE_WAKEUP         0x02
#define U1_ENABLE             0x04
#define U2_ENABLE             0x08
#define LTM_ENABLE            0x10

void usb_boot();

#endif