/*
 ## Cypress USB 3.0 Platform header file (cyfxbulksrcsink.h)
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

/* This file contains the constants used by the bulk source sink application example */

#ifndef _INCLUDED_CYFXBULKSRCSINK_H_
#define _INCLUDED_CYFXBULKSRCSINK_H_

#include "../config.h"

/* Extern definitions for the USB Descriptors */
extern const uint8_t g_usb20_device[];
extern const uint8_t g_usb30_device[];
extern const uint8_t g_device_qual[];
extern const uint8_t g_fs_config[];
extern const uint8_t g_hs_config[];
extern const uint8_t g_bos[];
extern const uint8_t g_ss_config[];
extern const uint8_t g_lang_id[];
extern const uint8_t g_manufacture[];
extern const uint8_t g_product[];
extern const uint8_t g_usb_os[];

#include <cyu3externcend.h>

#endif /* _INCLUDED_CYFXBULKSRCSINK_H_ */

/*[]*/
