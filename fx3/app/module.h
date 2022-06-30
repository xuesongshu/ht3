#ifndef __MODULE_H__
#define __MODULE_H__

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3uart.h"
#include "cyu3gpio.h"
#include "cyu3utils.h"

void config_dma_consumer(CyU3PDmaChannel *dma, uint16_t size, uint16_t socket_id);
void config_dma_producer(CyU3PDmaChannel *dma, uint16_t size, uint16_t socket_id);
void config_endpoint(uint16_t burst_len, uint16_t size, uint16_t ep_id);
void destroy_endpoint(CyU3PDmaChannel *dma, uint16_t ep_id, uint16_t socket_id);
void setup_cb_producer(uint16_t ep_id, CyU3PDmaChannel *dma);
void setup_cb_consumer(uint16_t ep_id, CyU3PDmaChannel *dma);
uint16_t get_buffer_size();
void dma_cb(CyU3PDmaChannel *chHandle, CyU3PDmaCbType_t type, CyU3PDmaCBInput_t *input);
void process_command(uint8_t *ep0_buffer, 
    uint32_t setupdat0, 
    uint32_t setupdat1, 
    CyU3PEvent *bulk_event,
    uint8_t *usb_log_buffer,
    uint32_t* ep0_stat_count,
    CyBool_t* standby_mode_enable
);
void app_error_handler(CyU3PReturnStatus_t iapi_ret);
void app_stop(void);
void app_deinit (void);
void app_init (void);
#endif