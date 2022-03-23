#ifndef __DESCR_H__
#define __DESCR_H__

#define MAJOR_VERSION 1
#define MINOR_VERSION 2
#define PRODUCER1     0x01
#define CONSUMER1     0x81
#define PRODUCER2     0x02
#define CONSUMER2     0x82
#define ENDPOINT_COUNT 0x02

#define GPIO_OUT_ID  50

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

#endif

