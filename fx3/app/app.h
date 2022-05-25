/*
 ## Cypress USB 3.0 Platform header file (cyfxslfifosync.h)
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

/* This file contains the constants and definitions used by the Slave FIFO application example */

#ifndef _INCLUDED_CYFXSLFIFOASYNC_H_
#define _INCLUDED_CYFXSLFIFOASYNC_H_

#include "../usb_dscr.h"
#include "cyu3externcstart.h"

/* 16/32 bit GPIF Configuration select */
/* Set CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT = 0 for 16 bit GPIF data bus.
 * Set CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT = 1 for 32 bit GPIF data bus.
 */
#define CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT (1)

#define CY_FX_SLFIFO_DMA_BUF_COUNT      (2)                       /* Slave FIFO channel buffer count */
#define CY_FX_SLFIFO_DMA_TX_SIZE        (0)                   /* DMA transfer size is set to infinite */
#define CY_FX_SLFIFO_DMA_RX_SIZE        (0)                   /* DMA transfer size is set to infinite */
#define CY_FX_SLFIFO_THREAD_STACK       (0x0400)                  /* Slave FIFO application thread stack size */
#define CY_FX_SLFIFO_THREAD_PRIORITY    (8)                       /* Slave FIFO application thread priority */


/* Used with FX3 Silicon. */
#define PRODUCER1_PPORT    CY_U3P_PIB_SOCKET_0    /* P-port Socket 0 is producer */
#define CONSUMER1_PPORT    CY_U3P_PIB_SOCKET_3    /* P-port Socket 3 is consumer */

#define FX3_GPIO_TEST_OUT               (50)
#define FX3_GPIO_TO_LOFLAG(gpio)        (1 << (gpio))
#define FX3_GPIO_TO_HIFLAG(gpio)        (1 << ((gpio) - 32))

#include "cyu3externcend.h"

#endif /* _INCLUDED_CYFXSLFIFOASYNC_H_ */

/*[]*/
