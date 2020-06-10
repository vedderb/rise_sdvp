##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O2 -ggdb -fomit-frame-pointer -falign-functions=16 -std=gnu99
  USE_OPT += -DBOARD_OTG_NOVBUSSENS $(build_args)
  USE_OPT += -fsingle-precision-constant -Wdouble-promotion
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = 
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = yes
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# Enables the use of FPU on Cortex-M4 (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

# Enable this if you really want to use the STM FWLib.
ifeq ($(USE_FWLIB),)
  USE_FWLIB = yes
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = rc_controller

# Imported source files and paths
CHIBIOS = ChibiOS_3.0.2
# Startup files.
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F4xx/platform.mk
include $(CHIBIOS)/os/hal/boards/ST_STM32F4_DISCOVERY/board.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/rt/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
# Other files (optional).
include cc2520/cc2520.mk
include cc1120/cc1120.mk
include imu/imu.mk

# Define linker script file here
LDSCRIPT= ld_eeprom_emu.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(STARTUPSRC) \
       $(KERNSRC) \
       $(PORTSRC) \
       $(OSALSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(CHIBIOS)/os/hal/lib/streams/chprintf.c \
       $(CHIBIOS)/os/various/syscalls.c \
       $(CC2520SRC) \
       $(CC1120SRC) \
       $(IMUSRC) \
       main.c \
       comm_usb_serial.c \
       utils.c \
       comm_cc2520.c \
       led.c \
       buffer.c \
       ext_cb.c \
       crc.c \
       adconv.c \
       comm_usb.c \
       packet.c \
       commands.c \
       conf_general.c \
       pos.c \
       terminal.c \
       comm_can.c \
       bldc_interface.c \
       servo_simple.c \
       autopilot.c \
       eeprom.c \
       timeout.c \
       log.c \
       digital_filter.c \
       comm_cc1120.c \
       ublox.c \
       rtcm3_simple.c \
       srf10.c \
       pwm_esc.c \
       mr_control.c \
       actuator.c \
       motor_sim.c \
       m8t_base.c \
       pos_uwb.c \
       fi.c \
       servo_vesc.c \
       hydraulic.c \
       i2c_bb.c \
       timer.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC =

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(STARTUPASM) $(PORTASM) $(OSALASM)

INCDIR = $(STARTUPINC) $(KERNINC) $(PORTINC) $(OSALINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) $(TESTINC) \
         $(CHIBIOS)/os/various \
         $(CHIBIOS)/os/hal/lib/streams \
         $(CC2520INC) $(CC1120INC) $(IMUINC)

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra -Wundef

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = -lm

#
# End of user defines
##############################################################################

ifeq ($(USE_FWLIB),yes)
  include $(CHIBIOS)/ext/stdperiph_stm32f4/stm32lib.mk
  CSRC += $(STM32SRC)
  INCDIR += $(STM32INC)
  USE_OPT += -DUSE_STDPERIPH_DRIVER
endif

build/$(PROJECT).bin: build/$(PROJECT).elf 
	$(BIN) build/$(PROJECT).elf build/$(PROJECT).bin

.PHONY: upload upload-olimex debug-start upload_remote

# Program
upload: build/$(PROJECT).bin
	openocd -f board/stm32f4discovery.cfg -c "reset_config trst_only combined" -c "program build/$(PROJECT).elf verify reset exit" # For openocd 0.9

#program with olimex arm-usb-tiny-h and jtag-swd adapter board. needs openocd>=0.9
upload-olimex: build/$(PROJECT).bin
	openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f interface/ftdi/olimex-arm-jtag-swd.cfg -c "set WORKAREASIZE 0x2000" -f target/stm32f4x.cfg -c "program build/$(PROJECT).elf verify reset"

debug-start:
	openocd -f stm32-bv_openocd.cfg
	
upload_remote: build/$(PROJECT).bin
#	./upload_remote build/$(PROJECT).bin pi 10.1.151.19 22 pi_gpio
	./upload_remote build/$(PROJECT).bin pi 127.0.0.1 2230 stlink

touch_conf_general:
	touch ./conf_general.h

vcc: override PROJECT = fw_vcc
vcc: USE_OPT += -DMAIN_MODE=MAIN_MODE_CAR -DUBLOX_EN=0
vcc: touch_conf_general build/fw_vcc.elf build/fw_vcc.bin

tero: override PROJECT = fw_tero
tero: USE_OPT += -DMAIN_MODE=MAIN_MODE_CAR -DCAR_TERO -DIS_F9_BOARD=1
tero: touch_conf_general build/fw_tero.elf build/fw_tero.bin

rise_ubx: override PROJECT = fw_rise_ubx
rise_ubx: USE_OPT += -DMAIN_MODE=MAIN_MODE_CAR -DUBLOX_EN=1 -DBOARD_YAW_ROT=-90.0
rise_ubx: touch_conf_general build/fw_rise_ubx.elf build/fw_rise_ubx.bin

rise_f9: override PROJECT = fw_rise_f9
rise_f9: USE_OPT += -DMAIN_MODE=MAIN_MODE_CAR -DIS_F9_BOARD=1
rise_f9: touch_conf_general build/fw_rise_f9.elf build/fw_rise_f9.bin

rise_f9_copter: override PROJECT = fw_rise_f9_copter
rise_f9_copter: USE_OPT += -DMAIN_MODE=MAIN_MODE_MULTIROTOR -DIS_F9_BOARD=1
rise_f9_copter: touch_conf_general build/fw_rise_f9_copter.elf build/fw_rise_f9_copter.bin

drangen: override PROJECT = fw_drangen
drangen: USE_OPT += -DMAIN_MODE=MAIN_MODE_CAR -DUBLOX_EN=1 -DBOARD_YAW_ROT=180.0 -DHAS_HYDRAULIC_DRIVE=1 -DSERVO_VESC_ID=255
drangen: touch_conf_general build/fw_drangen.elf build/fw_drangen.bin

mactrac: override PROJECT = fw_mactrac
mactrac: USE_OPT += -DMAIN_MODE=MAIN_MODE_CAR -DIS_MACTRAC
mactrac: touch_conf_general build/fw_mactrac.elf build/fw_mactrac.bin

RULESPATH = $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC
include $(RULESPATH)/rules.mk
