#ifndef __USB_DSCR_H__
#define __USB_DSCR_H__

#include "cyu3types.h"
#include "cyu3usbconst.h"

#define MAJOR_VERSION 0x01
#define MINOR_VERSION 0x02

/* Extern definitions for the USB Descriptors */
extern const unsigned char g_usb20_device[];
extern const unsigned char g_usb30_device[];
extern const unsigned char g_device_qual[];
extern const unsigned char g_fs_config[];
extern const unsigned char g_hs_config[];
extern const unsigned char g_bos[];
extern const unsigned char g_ss_config[];
extern const unsigned char g_lang_id[];
extern const unsigned char g_manufacture[];
extern const unsigned char g_product[];


/* Endpoint and socket definitions for the Slave FIFO application */

/* To change the Producer and Consumer EP enter the appropriate EP numbers for the #defines.
 * In the case of IN endpoints enter EP number along with the direction bit.
 * For eg. EP 6 IN endpoint is 0x86
 *     and EP 6 OUT endpoint is 0x06.
 * To change sockets mention the appropriate socket number in the #defines. */

/* Note: For USB 2.0 the endpoints and corresponding sockets are one-to-one mapped
         i.e. EP 1 is mapped to UIB socket 1 and EP 2 to socket 2 so on */

#define PRODUCER1_EP               0x01    /* EP 1 OUT */
#define CONSUMER1_EP               0x81    /* EP 1 IN */

#define PRODUCER1_SOCKET    CY_U3P_UIB_SOCKET_PROD_1    /* USB Socket 1 is producer */
#define CONSUMER1_SOCKET    CY_U3P_UIB_SOCKET_CONS_1    /* USB Socket 1 is consumer */

#endif