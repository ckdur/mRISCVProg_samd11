# Path to top level ASF directory relative to this project directory.
PRJ_PATH = .

# Target CPU architecture: cortex-m3, cortex-m4
ARCH = cortex-m0plus

# Target part: none, sam3n4 or sam4l4aa
PART = samd11c14a

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET_FLASH = mRISCVprog_flash.elf
TARGET_SRAM = mRISCVprog_sram.elf

# List of C source files.
CSRCS = \
       xdk-asf/interrupt_sam_nvic.c \
       xdk-asf/startup_samd11.c \
       xdk-asf/system_samd11.c \
       xdk-asf/spi.c \
       xdk-asf/sercom.c \
       xdk-asf/gclk.c \
       xdk-asf/clock.c \
       xdk-asf/pinmux.c \
       xdk-asf/system.c \
       xdk-asf/rtc_count.c \
       main.c \
       gpio.c \
       greset.c

# List of assembler source files.
ASSRCS = test5.S

# List of include paths.
INC_PATH = \
       /opt/xdk-asf-3.45.0/common/utils                                       \
       /opt/xdk-asf-3.45.0/common/utils/interrupt/                            \
       /opt/xdk-asf-3.45.0/sam0/drivers/port                                  \
       /opt/xdk-asf-3.45.0/sam0/drivers/sercom                                \
       /opt/xdk-asf-3.45.0/sam0/drivers/sercom/spi                            \
       /opt/xdk-asf-3.45.0/sam0/drivers/system                                \
       /opt/xdk-asf-3.45.0/sam0/drivers/system/clock                          \
       /opt/xdk-asf-3.45.0/sam0/drivers/system/clock/clock_samd09_d10_d11     \
       /opt/xdk-asf-3.45.0/sam0/drivers/system/interrupt                      \
       /opt/xdk-asf-3.45.0/sam0/drivers/system/interrupt/system_interrupt_samd10_d11 \
       /opt/xdk-asf-3.45.0/sam0/drivers/system/pinmux                         \
       /opt/xdk-asf-3.45.0/sam0/drivers/system/power                          \
       /opt/xdk-asf-3.45.0/sam0/drivers/system/power/power_sam_d_r_h          \
       /opt/xdk-asf-3.45.0/sam0/drivers/system/reset                          \
       /opt/xdk-asf-3.45.0/sam0/drivers/system/reset/reset_sam_d_r_h          \
       /opt/xdk-asf-3.45.0/sam0/drivers/rtc/                                       \
       /opt/xdk-asf-3.45.0/sam0/utils                                         \
       /opt/xdk-asf-3.45.0/sam0/utils/cmsis/samd11/include                    \
       /opt/xdk-asf-3.45.0/sam0/utils/cmsis/samd11/source                     \
       /opt/xdk-asf-3.45.0/sam0/utils/header_files                            \
       /opt/xdk-asf-3.45.0/sam0/utils/preprocessor                            \
       /opt/xdk-asf-3.45.0/sam0/utils/cmsis/samd11/include/pio/               \
       /opt/xdk-asf-3.45.0/thirdparty/CMSIS/Include                           \
       /opt/xdk-asf-3.45.0/thirdparty/CMSIS/Lib/GCC                           \
			.

# Additional search paths for libraries.
LIB_PATH =  \
       thirdparty/CMSIS/Lib/GCC                          

# List of libraries to use during linking.
LIBS =                              

# Path relative to top level directory pointing to a linker script.
LINKER_SCRIPT_FLASH = xdk-asf/samd11c14a_flash.ld
LINKER_SCRIPT_SRAM  = xdk-asf/samd11c14a_sram.ld

# Path relative to top level directory pointing to a linker script.
DEBUG_SCRIPT_FLASH = xdk-asf/samd11c14a_flash.gdb
DEBUG_SCRIPT_SRAM  = xdk-asf/samd11c14a_sram.gdb

# Project type parameter: all, sram or flash
PROJECT_TYPE        = flash

# Additional options for debugging. By default the common Makefile.in will
# add -g3.
DBGFLAGS = 

# Application optimization used during compilation and linking:
# -O0, -O1, -O2, -O3 or -Os
OPTIMIZATION = -O1

# Extra flags to use when archiving.
ARFLAGS = 

# Extra flags to use when assembling.
ASFLAGS = 

# Extra flags to use when compiling.
CFLAGS = 

# Extra flags to use when preprocessing.
#
# Preprocessor symbol definitions
#   To add a definition use the format "-D name[=definition]".
#   To cancel a definition use the format "-U name".
#
# The most relevant symbols to define for the preprocessor are:
#   BOARD      Target board in use, see boards/board.h for a list.
#   EXT_BOARD  Optional extension board in use, see boards/board.h for a list.
CPPFLAGS = \
       -D ARM_MATH_CM0PLUS=false                           \
       -D USART_CALLBACK_MODE=false                        \
       -D SPI_CALLBACK_MODE=false                         \
       -D RTC_COUNT_ASYNC=false                           \
       -D __SAMD11C14A__

# Extra flags to use when linking
LDFLAGS = \
                                                          \
       -Wl,--defsym,STACK_SIZE=0x400                      \
       -Wl,--defsym,__stack_size__=0x400                  \
       test5.o

# Pre- and post-build commands
PREBUILD_CMD = 
POSTBUILD_CMD = 
