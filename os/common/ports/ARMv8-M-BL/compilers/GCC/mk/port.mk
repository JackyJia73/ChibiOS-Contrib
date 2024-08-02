# List of the ChibiOS ARMv8M-BaseLine generic port files.
PORTSRC = $(CHIBIOS_CONTRIB)/os/common/ports/ARMv8-M-BL/chcore.c
          
PORTASM = $(CHIBIOS_CONTRIB)/os/common/ports/ARMv8-M-BL/compilers/GCC/chcoreasm.S

PORTINC = $(CHIBIOS)/os/common/portability/GCC \
          $(CHIBIOS)/os/common/ports/ARM-common \
          $(CHIBIOS_CONTRIB)/os/common/ports/ARMv8-M-BL

# Shared variables
ALLXASMSRC += $(PORTASM)
ALLCSRC    += $(PORTSRC)
ALLINC     += $(PORTINC)
