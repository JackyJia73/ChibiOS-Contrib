# Required platform files.
PLATFORMSRC := ${CHIBIOS}/os/hal/ports/common/ARMCMx/nvic.c \
                       ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32L23x/gd32_isr.c \
                       ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32L23x/hal_lld.c

# Required include directories.
PLATFORMINC := $(CHIBIOS)/os/hal/ports/common/ARMCMx \
                       $(CHIBIOS_CONTRIB)/os/hal/ports/GD/GD32L23x

# Optional platform files.
ifeq ($(USE_SMART_BUILD),yes)

# Configuration files directory
ifeq ($(HALCONFDIR),)
  ifeq ($(CONFDIR),)
    HALCONFDIR = 
  else
    HALCONFDIR := $(CONFDIR)
  endif
endif

HALCONF := $(strip $(shell cat $(HALCONFDIR)/halconf.h | egrep -e "\#define"))

endif

# Drivers compatible with the platform.
include ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32L23x/GPIOv1/driver.mk
include ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32L23x/FMCv1/driver.mk
include ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32L23x/RCUv1/driver.mk
include ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32L23x/SYSTICKv1/driver.mk
include ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32L23x/USBDv1/driver.mk

# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
