# Command-line build of oepl_CC1310 with TI ARM-CGT, mirroring the CCS
# project (.cproject). Runs in WSL/Linux:
#
#   make [CONFIG=Debug|Proxy|EpdTest|Chroma74H]   ->  build/<CONFIG>/oepl_CC1310.out + .map
#
# EpdTest is a Chroma 74H bring-up build: it draws the stock screen stored in
# SPI flash at 0x15000 using the panel's factory script, then idles.
#   make clean [CONFIG=...]
#
# SDK and compiler locations can be overridden on the command line.

SDK ?= $(HOME)/ti/simplelink_cc13x0_sdk_4_20_02_07
CGT ?= $(HOME)/ti/ti-cgt-arm_18.12.8.LTS
CONFIG ?= Debug

CC = $(CGT)/bin/armcl
BUILD = build/$(CONFIG)
NAME = oepl_CC1310

DEFINES = -DDeviceFamily_CC13X0 -DCCFG_FORCE_VDDR_HH=0 \
          -DSUPPORT_PHY_CUSTOM -DSUPPORT_PHY_50KBPS2GFSK \
          -DSUPPORT_PHY_625BPSLRM -DSUPPORT_PHY_5KBPSSLLR

# Files the CCS Debug/Proxy configurations exclude from ../common
COMMON_EXCLUDE = syncedproto.c RFQueue.c radio.c powermgt.c

ifeq ($(CONFIG),Debug)
DEFINES += -DSERIAL_LOG=1 -DDEBUG_LOG=0 -DCC1310_128K -D_CUSTOM_IO_=oepl_io.inl
else ifeq ($(CONFIG),Proxy)
DEFINES += -DSERIAL_LOG=1 -DDEBUG_LOG=0 -DCC1310_128K -D_CUSTOM_IO_=oepl_io.inl \
           -DCHROMA_PROXY
else ifeq ($(CONFIG),EpdTest)
DEFINES += -DSERIAL_LOG=1 -DDEBUG_LOG=0 -DCHROMA_74H -D_CUSTOM_IO_=oepl_io.inl \
           -DEPD_TEST
else ifeq ($(CONFIG),Chroma74H)
# The OEPL tag firmware: main in oepl_74h.c, no EasyLink
# EPD_SPI= (empty) on the make command line: bit-banged EPD SPI instead of SSI1
EPD_SPI ?= -DEPD_HW_SPI
DEFINES += -DSERIAL_LOG=1 -DDEBUG_LOG=0 -DCHROMA_74H -DOEPL_74H $(EPD_SPI)
# common/ OEPL code is written for C99 (the Aeon's compiler default).
# Logging needs no float printf. Flash used, 2026-11-16: 52.5 KB without
# either, 43.9 KB with both (--opt_level=4 would save another 1.3 KB).
CFLAGS_EXTRA = --c99 --printf_support=nofloat
OPT ?= --opt_level=2 --opt_for_speed=0
COMMON_EXCLUDE = proxy.c CobsFraming.c
LOCAL_EXCLUDE = oepl_CC1310.c easylink/EasyLink_nortos.c easylink/easylink_config.c \
                smartrf_settings/smartrf_settings_predefined.c ccfg.c
# Linked at 0x1000 behind ../bootloader_CC1310, which carries the CCFG
LINK_DEFINES = --define=OEPL_APP
# G5 and zlib image decoders (G5_SUPPORT, ZLIB_SUPPORT in common/oepl_board.h)
EXTRA_SOURCES = ../g5/g5.c ../uzlib/src/tinflate.c ../uzlib/src/tinfzlib.c \
                ../uzlib/src/adler32.c ../uzlib/src/crc32.c
else
$(error CONFIG must be Debug, Proxy, EpdTest or Chroma74H (the CCS project's Release has no board define))
endif

# One-off test switches, e.g. make CONFIG=Chroma74H EXTRA_DEFINES=-DWDT_TEST -B
DEFINES += $(EXTRA_DEFINES)

SOURCES = $(filter-out $(LOCAL_EXCLUDE),$(wildcard *.c easylink/*.c smartrf_settings/*.c)) \
          $(filter-out $(addprefix ../common/,$(COMMON_EXCLUDE)),$(wildcard ../common/*.c)) \
          $(EXTRA_SOURCES)
OBJECTS = $(addprefix $(BUILD)/,$(notdir $(SOURCES:.c=.obj)))
vpath %.c . easylink smartrf_settings ../common ../g5 ../uzlib/src

CFLAGS = --silicon_version=7M3 --code_state=16 --little_endian \
         --float_support=none --symdebug:dwarf --gen_func_subsections=on \
         --diag_warning=225 --diag_warning=255 --diag_wrap=off \
         --display_error_number $(OPT) $(CFLAGS_EXTRA) $(DEFINES) \
         -I. -I../common -I../shared -I../g5 -I../uzlib/src -I../bb_epaper/src \
         -I$(SDK)/source -I$(SDK)/kernel/nortos -I$(SDK)/kernel/nortos/posix \
         -I$(CGT)/include

LFLAGS = -i$(SDK)/source -i$(SDK)/kernel/nortos -i$(CGT)/lib \
         -lti/display/lib/display.aem3 \
         -lti/drivers/rf/lib/rf_multiMode_cc13x0.aem3 \
         -lti/drivers/lib/drivers_cc13x0.aem3 \
         -llib/nortos_cc13x0.aem3 \
         -lti/devices/cc13x0/driverlib/bin/ccs/driverlib.lib \
         -llibc.a \
         --rom_model --warn_sections --display_error_number --diag_wrap=off \
         -m$(BUILD)/$(NAME).map

all: $(BUILD)/$(NAME).out

$(BUILD)/%.obj: %.c | $(BUILD)
	@echo "CC $<"
	@$(CC) $(CFLAGS) --preproc_with_compile \
	   --preproc_dependency=$(@:.obj=.d) $< --output_file=$@

$(BUILD)/$(NAME).out: $(OBJECTS) Chroma_CC1310.cmd
	@echo "LINK $@"
	@$(CC) $(CFLAGS) -z $(LINK_DEFINES) $(OBJECTS) Chroma_CC1310.cmd $(LFLAGS) -o $@

$(BUILD):
	@mkdir -p $@

clean:
	rm -rf $(BUILD)

.PHONY: all clean

# armcl --preproc_dependency lists headers without the empty rules gcc -MP
# adds: a header that was renamed or deleted would stop make with "No rule to
# make target". These rules let make rebuild the object instead.
%.h %.inl: ;

-include $(OBJECTS:.obj=.d)
