#ifndef __MODULE_H__
#define __MODULE_H__

#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3utils.h"

void config_dma(CyU3PDmaChannel *dma, 
    uint16_t size, 
    uint16_t producer_id, 
    uint16_t consumer_id, 
    CyBool_t is_for_producer
);
void config_endpoint(uint16_t burst_len, uint16_t size, uint16_t ep_id);
void destroy_endpoint(CyU3PDmaChannel *dma, uint16_t ep_id, uint16_t socket_id);
void setup_cb_endpoint(uint16_t ep_id, CyU3PDmaChannel *dma);
uint16_t get_buffer_size();
void dma_cb(CyU3PDmaChannel *chHandle, CyU3PDmaCbType_t type, CyU3PDmaCBInput_t *input);
void process_command(uint8_t *ep0_buffer, 
    uint32_t setupdat0, 
    uint32_t setupdat1, 
    CyU3PEvent *bulk_event,
    uint8_t *usb_log_buffer,
    volatile uint32_t* ep0_stat_count,
    CyBool_t* standby_mode_enable
);
void config_pib(void);
void app_stop(void);
void app_deinit (void);
void app_init (void);
void config_gpio();
void setup_gpio(uint16_t pin);
#endif