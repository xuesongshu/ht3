#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "cyu3types.h"
#include "cyu3usbconst.h"
#include "cyu3externcstart.h"

#define MAJOR_NUMBER 0x01
#define MINOR_NUMBER 0x00

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

#endif