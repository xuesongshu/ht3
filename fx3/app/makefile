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

FX3FWROOT=E:/in_use/FX3

all:compile

include $(FX3FWROOT)/fw_build/fx3_fw/fx3_build_config.mak

Include += -I$(FX3FWROOT)/boot_lib/$(CYSDKVERSION)/include
# LDLIBS += -L $(FX3FWROOT)/boot_lib/$(CYSDKVERSION)/lib -lcyfx3boot

MODULE = app

SOURCE= cyfxbulksrcsink.c     \
	../usb_descr.c	   \
	cyfxtx.c \
	module.c

ifeq ($(CYFXBUILD),arm)
SOURCE_ASM=cyfx_startup.S
else
SOURCE_ASM=cyfx_gcc_startup.S
endif

C_OBJECT=$(SOURCE:%.c=./%.o)
A_OBJECT=$(SOURCE_ASM:%.S=./%.o)

EXES = $(MODULE).$(EXEEXT)

$(MODULE).$(EXEEXT): $(A_OBJECT) $(C_OBJECT)
	$(LINK)
	[ -f $(MODULE).img ] && rm -f $(MODULE).img && echo remove old image. || true
#	elf2img -i $(MODULE).elf -o $(MODULE).img -v -i2cconf 0x0E
	elf2img -i $(MODULE).elf -o $(MODULE).img -v

cyfxtx.c:
	cp $(FX3FWROOT)/fw_build/fx3_fw/cyfxtx.c .

cyfx_startup.S:
	cp $(FX3FWROOT)/fw_build/fx3_fw/cyfx_startup.S .

cyfx_gcc_startup.S:
	cp $(FX3FWROOT)/fw_build/fx3_fw/cyfx_gcc_startup.S .

$(C_OBJECT) : %.o : %.c
	$(COMPILE)

$(A_OBJECT) : %.o : %.S
	$(ASSEMBLE)

clean:
	rm -f ./$(MODULE).$(EXEEXT)
	rm -f ./$(MODULE).map
	rm -f ./*.o
	rm -f cyfxtx.c cyfx_startup.S cyfx_gcc_startup.S
	rm -f $(MODULE).img

compile: $(C_OBJECT) $(A_OBJECT) $(EXES)
	rm -f ./$(MODULE).map
	rm -f ./*.o
	
#[]#
