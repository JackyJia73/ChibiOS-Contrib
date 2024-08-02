# List of all the board related files.
BOARDSRC = ${CHIBIOS_CONTRIB}/os/hal/boards/GD_GD32L235R_EVAL/board.c

# Required include directories
BOARDINC = ${CHIBIOS_CONTRIB}/os/hal/boards/GD_GD32L235R_EVAL

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
