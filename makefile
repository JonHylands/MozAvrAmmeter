#
#             LUFA Library
#     Copyright (C) Dean Camera, 2013.
#
#  dean [at] fourwalledcubicle [dot] com
#           www.lufa-lib.org
#
# --------------------------------------
#         LUFA Project Makefile.
# --------------------------------------

# Run "make help" for target help.

MCU          = atmega32u4
ARCH         = AVR8
BOARD        = ADAFRUITU4
F_CPU        = 16000000
F_USB        = $(F_CPU)
OPTIMIZATION = s
TARGET       = MozAvrAmmeter
OBJDIR        = ./obj

PROJ	     = ../projects
PROJ_SRC     = $(PROJ)/common/avr/adc.c \
	       $(PROJ)/common/avr/UART.c \
	       $(PROJ)/common/avr/Delay.c

SRC          = $(TARGET).c Descriptors.c PacketParser.c $(PROJ_SRC) $(LUFA_SRC_USB) $(LUFA_SRC_USBCLASS)
LUFA_PATH    = ../LUFA-130901/LUFA
CC_FLAGS     = -DUSE_LUFA_CONFIG_HEADER -IConfig/ -I$(PROJ)/common -I$(PROJ)/common/avr -DCFG_CPU_CLOCK=$(F_CPU)
LD_FLAGS     =

# Default target
all:

# Include LUFA build script makefiles
include $(LUFA_PATH)/Build/lufa_core.mk
include $(LUFA_PATH)/Build/lufa_sources.mk
include $(LUFA_PATH)/Build/lufa_build.mk
include $(LUFA_PATH)/Build/lufa_cppcheck.mk
include $(LUFA_PATH)/Build/lufa_doxygen.mk
include $(LUFA_PATH)/Build/lufa_dfu.mk
include $(LUFA_PATH)/Build/lufa_hid.mk
include $(LUFA_PATH)/Build/lufa_avrdude.mk
include $(LUFA_PATH)/Build/lufa_atprogram.mk

d: all
	avrdude -p m32u4 -c avr109 -P /dev/ttyACM0 -U flash:w:$(TARGET).hex
