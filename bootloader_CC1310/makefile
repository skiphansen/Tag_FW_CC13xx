# Chroma 74H bootloader (flash sector 0 + CCFG), TI ARM-CGT. Runs in WSL:
#
#   make   ->  build/boot.out + .map
#
# Flash it together with the app (oepl_CC1310 CONFIG=Chroma74H); OTA never
# rewrites it. See ../oepl_CC1310/ota_74h.h.

SDK ?= $(HOME)/ti/simplelink_cc13x0_sdk_4_20_02_07
CGT ?= $(HOME)/ti/ti-cgt-arm_18.12.8.LTS

CC = $(CGT)/bin/armcl
BUILD = build
NAME = boot

# ccfg.c is the app's (identical to stock: DC/DC off, ROM bootloader off,
# CPU DAP on)
SOURCES = boot.c ../oepl_CC1310/ccfg.c
OBJECTS = $(addprefix $(BUILD)/,$(notdir $(SOURCES:.c=.obj)))
vpath %.c . ../oepl_CC1310

CFLAGS = --silicon_version=7M3 --code_state=16 --little_endian \
         --float_support=none --symdebug:dwarf --gen_func_subsections=on \
         --opt_level=2 --opt_for_speed=0 \
         --diag_warning=225 --diag_warning=255 --diag_wrap=off \
         --display_error_number -DDeviceFamily_CC13X0 -DCCFG_FORCE_VDDR_HH=0 \
         -I. -I../oepl_CC1310 -I$(SDK)/source -I$(CGT)/include

LFLAGS = -i$(SDK)/source -i$(CGT)/lib \
         -lti/devices/cc13x0/driverlib/bin/ccs/driverlib.lib \
         -llibc.a \
         --rom_model --warn_sections --display_error_number --diag_wrap=off \
         -m$(BUILD)/$(NAME).map

all: $(BUILD)/$(NAME).out

$(BUILD)/%.obj: %.c | $(BUILD)
	@echo "CC $<"
	@$(CC) $(CFLAGS) --preproc_with_compile \
	   --preproc_dependency=$(@:.obj=.d) $< --output_file=$@

$(BUILD)/$(NAME).out: $(OBJECTS) boot.cmd
	@echo "LINK $@"
	@$(CC) $(CFLAGS) -z $(OBJECTS) boot.cmd $(LFLAGS) -o $@

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
