#ifndef __THREAD_H__
#define __THREAD_H__

#define  THREAD_STACK        (1000)
#define  THREAD_PRIORITY     (8)

#define EP_PRODUCER               0x02
#define EP_CONSUMER               0x82
#define EP_INTERRUPT              0x81

#define EP_PRODUCER1_SOCKET        CY_U3P_UIB_SOCKET_PROD_2
#define EP_CONSUMER1_SOCKET        CY_U3P_LPP_SOCKET_UART_CONS
#define EP_PRODUCER2_SOCKET        CY_U3P_LPP_SOCKET_UART_PROD
#define EP_CONSUMER2_SOCKET        CY_U3P_UIB_SOCKET_CONS_2
#define INTR_CONSUMER1_SOCKET   CY_U3P_UIB_SOCKET_CONS_1

#define DMA_BUF_COUNT       (8)

#define SET_LINE_CODING        0x20
#define GET_LINE_CODING        0x21
#define SET_CONTROL_LINE_STATE 0x22


#include "stdint.h"

void app_thread_entry(uint32_t iinput);
#endif
