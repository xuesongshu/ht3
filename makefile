## Copyright Cypress Semiconductor Corporation, 2010-2018,
## All Rights Reserved
## UNPUBLISHED, LICENSED SOFTWARE.
##
## CONFIDENTIAL AND PROPRIETARY INFORMATION
## WHICH IS THE PROPERTY OF CYPRESS.
##
## Use of this file is governed
## by the license agreement included in the file
##
##      <install>/license/license.txt
##
## where <install> is the Cypress software
## installation root directory path.
##

MODULE = ht3
FX3FWROOT = E:/in_use/FX3
ARMGCC_INSTALL_PATH = E:/in_use/FX3/ARM_GCC

# include ${FX3FWROOT}/fw_build/boot_fw/fx3_build_config.mak
include ${FX3FWROOT}/fw_build/fx3_fw/fx3_build_config.mak

Include += -I$(FX3FWROOT)/boot_lib/$(CYSDKVERSION)/include
LDLIBS += -L $(FX3FWROOT)/boot_lib/$(CYSDKVERSION)/lib -lcyfx3boot

ifeq ($(CYFXBUILD), arm)
APP_ASM_SOURCE = 
else
APP_ASM_SOURCE = ./cyfx_gcc_startup.S
endif

APP_SOURCE = 	    \
	     main.c		\
	     app.c		\
	     descr.c 	\
	     usb_boot.c	\
	     uart.c		\
		 cyfxtx.c    

APP_OBJECT=$(APP_SOURCE:%.c=./%.o)
APP_ASM_OBJECT=$(APP_ASM_SOURCE:%.S=./%.o)

EXES = $(MODULE).$(EXEEXT)

all:compile

$(APP_ASM_OBJECT) : %.o : %.S 
	$(ASSEMBLE)

$(APP_OBJECT) : %.o : %.c 
	$(COMPILE)

$(MODULE).$(EXEEXT): $(APP_OBJECT) $(APP_ASM_OBJECT) 
	$(LINK)
	elf2img -i $(MODULE).elf -o $(MODULE).img -v

clean:
	rm -f ./$(MODULE).$(EXEEXT)
	rm -f ./$(MODULE).map
	rm -f ./*.o
	rm -f ./$(MODULE).img

compile: $(APP_OBJECT) $(APP_ASM_OBJECT) $(EXES)

#[]#
