#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "cyu3types.h"
#include "cyu3usbconst.h"
#include "cyu3externcstart.h"

#define MAJOR_NUMBER 0x01
#define MINOR_NUMBER 0x00
/* Endpoint and socket definitions for the bulk source sink application */

/* To change the producer and consumer EP enter the appropriate EP numbers for the #defines.
 * In the case of IN endpoints enter EP number along with the direction bit.
 * For eg. EP 6 IN endpoint is 0x86
 *     and EP 6 OUT endpoint is 0x06.
 * To change sockets mention the appropriate socket number in the #defines. */

/* Note: For USB 2.0 the endpoints and corresponding sockets are one-to-one mapped
         i.e. EP 1 is mapped to UIB socket 1 and EP 2 to socket 2 so on */

#define CY_FX_EP_PRODUCER               0x01    /* EP 1 OUT */
#define CY_FX_EP_CONSUMER               0x81    /* EP 1 IN */

/* Burst mode definitions: Only for super speed operation. The maximum burst mode 
 * supported is limited by the USB hosts available. The maximum value for this is 16
 * and the minimum (no-burst) is 1. */

#ifdef CYMEM_256K

/*
   As we have only 32 KB total DMA buffers available on the CYUSB3011/CYUSB3012 parts, the buffering
   needs to be reduced.
 */

/* Burst length in 1 KB packets. Only applicable to USB 3.0. */
#define CY_FX_EP_BURST_LENGTH                   (4)

/* Multiplication factor used when allocating DMA buffers to reduce DMA callback frequency. */
#define CY_FX_DMA_SIZE_MULTIPLIER               (1)

/* Number of DMA buffers to be used. More buffers can give better throughput. */
#define CY_FX_BULKSRCSINK_DMA_BUF_COUNT         (2)

#else

/* Burst length in 1 KB packets. Only applicable to USB 3.0. */
#define CY_FX_EP_BURST_LENGTH                   (16)

/* Multiplication factor used when allocating DMA buffers to reduce DMA callback frequency. */
#define CY_FX_DMA_SIZE_MULTIPLIER               (2)

/* Number of DMA buffers to be used. More buffers can give better throughput. */
#define CY_FX_BULKSRCSINK_DMA_BUF_COUNT         (3)

#endif

#define CY_FX_EP_PRODUCER_SOCKET        CY_U3P_UIB_SOCKET_PROD_1    /* Socket 1 is producer */
#define CY_FX_EP_CONSUMER_SOCKET        CY_U3P_UIB_SOCKET_CONS_1    /* Socket 1 is consumer */

#define CY_FX_BULKSRCSINK_DMA_TX_SIZE        (0)        /* DMA transfer size is set to infinite */
#define CY_FX_BULKSRCSINK_THREAD_STACK       (0x1000)   /* Bulk loop application thread stack size */
#define CY_FX_BULKSRCSINK_THREAD_PRIORITY    (8)        /* Bulk loop application thread priority */

/* Byte value that is filled into the source buffers that FX3 sends out. */
#define CY_FX_BULKSRCSINK_PATTERN            (0xAA)
#define CYFX_USB_CTRL_TASK      (1 << 0)        /* Event that indicates that there is a pending USB control request. */
#define CYFX_USB_HOSTWAKE_TASK  (1 << 1)        /* Event that indicates the a Remote Wake should be attempted. */

#define CYFX_USBLOG_SIZE        (0x1000)

/* GPIO used for testing IO state retention when switching from boot firmware to full firmware. */
#define FX3_GPIO_TEST_OUT               (50)
#define FX3_GPIO_TO_LOFLAG(gpio)        (1 << (gpio))
#define FX3_GPIO_TO_HIFLAG(gpio)        (1 << ((gpio) - 32))

#endif