##
## @file Makefile
## @brief Main makefile
##

# Check if any target selected
-include Makefile.user
ifeq "$(TARGET_NAME)" ""
$(error Please specify target by running "make TARGET_NAME=<value> <TARGET>")
endif
ifeq ("$(wildcard target/$(TARGET_NAME)/Makefile.mk)","")
$(error Target '$(TARGET_NAME)' not found)
endif

# Initial variable setup
BUILD_DIR:=build
SOURCE_DIRS=app
GLOBAL_DEPS=Makefile
LINKER_SCRIPT=target/$(TARGET_NAME)/linkerscript.ld
INCLUDE_PATH=api app modules $(BUILD_DIR)/$(TARGET_NAME)/generated_files

# Required peripherals (gpio) and communication api
PERIPHERALS=gpio comm

# Include target Makefile
-include target/$(TARGET_NAME)/Makefile.mk

# Check if target is NOT abstract
ifneq "$(IS_ABSTRACT)" "no"
$(error This target is abstract only, so it can not be compiled)
endif

# Export all source files from source directories
SOURCES=$(wildcard $(addsuffix /*.c,$(SOURCE_DIRS)))
SOURCES_CPP=$(wildcard $(addsuffix /*.cpp,$(SOURCE_DIRS)))
HEADERS=$(wildcard $(addsuffix /*.h,$(INCLUDE_PATH)))

# Export source files for supported peripherals
SOURCES+=$(PERIPHERALS:%=api/%.c)
SOURCES_CPP+=$(PERIPHERALS_CPP:%=api/%.cpp)
PERIPH_MACROS=$(shell echo $(PERIPHERALS) | tr a-z A-Z)
PERIPH_MACROS+=$(shell echo $(PERIPHERALS_CPP) | tr a-z A-Z)

# Export define macros for supported peripherals
C_MACROS+=$(PERIPH_MACROS:%=%_PERIPH_ENABLED)
C_MACROS+=TARGET_NAME=\"$(TARGET_NAME)\"

# Export source files for supported modules
#SOURCES+=$(MODULES:%=modules/%.c)
SOURCES_CPP+=$(MODULES:%=modules/%.cpp)

# Toolchain setup
CC=$(TOOL_PREFIX)gcc
CCX=$(TOOL_PREFIX)g++
LD=$(TOOL_PREFIX)ld
OBJCOPY=$(TOOL_PREFIX)objcopy
GDB=$(TOOL_PREFIX)gdb

# Host compiler used for building PC tools
HOST_CC=g++

# Export compiler options: optimization, include path and C macros
CCOPTS:=$(OPTS)
OPTS+=-Wall -g -Os -nostartfiles -ffunction-sections
OPTS+=$(INCLUDE_PATH:%=-I%)
OPTS+=$(C_MACROS:%=-D%)

CCXOPTS:=-std=c++11 -fno-exceptions -fno-rtti $(OPTS)
OPTS+=-fgnu89-inline

# Create path for build output
TARGET_DIR=$(BUILD_DIR)/$(TARGET_NAME)
OBJECTS=$(SOURCES:%.c=$(TARGET_DIR)/%.c.o)
OBJECTS+=$(SOURCES_CPP:%.cpp=$(TARGET_DIR)/%.cpp.o)
STATS=$(SOURCES:%.c=$(TARGET_DIR)/stats/%.c.stats)
STATS=$(SOURCES_CPP:%.cpp=$(TARGET_DIR)/stats/%.cpp.stats)
ELF_FILE=$(TARGET_DIR)/build.elf
BIN_FILE=$(TARGET_DIR)/$(TARGET_NAME).bin
DFU_FILE=$(TARGET_DIR)/$(TARGET_NAME).dfu
HEX_FILE=$(TARGET_DIR)/$(TARGET_NAME).hex

# Configuration of GDB server
GDB_SERVER=openocd -f target/$(TARGET_NAME)/openocd.cfg

.PHONY: debug flash elf doc

elf: $(ELF_FILE)

bin: $(BIN_FILE)

hex: $(HEX_FILE)

dfu: $(DFU_FILE)

$(BIN_FILE): $(ELF_FILE)
	$(OBJCOPY) -O binary $(ELF_FILE) $(TARGET_DIR)/$(TARGET_NAME).bin

$(HEX_FILE): $(ELF_FILE)
	$(OBJCOPY) -O ihex $(ELF_FILE) $(TARGET_DIR)/$(TARGET_NAME).hex

$(DFU_FILE): $(BIN_FILE)
	python ./tools/dfu.py -b 0x08000000:$(BIN_FILE) $(DFU_FILE)

dfu-load: $(DFU_FILE)
	dfu-util -a 0 -D $(DFU_FILE)

# Include generated dependency files
# This allows recompiling only necessary modules
# *.d files are generated during compilation
-include $(OBJECTS:.o=.d)

# Include stuff related to testing
# Testing targets not working at the moment
# TODO: make it work
-include tests/Makefile.mk

$(TARGET_DIR)/build.bin: $(ELF_FILE)
	$(OBJCOPY) -O binary $(ELF_FILE) $(TARGET_DIR)/build.bin

$(ELF_FILE): $(OBJECTS) $(LINKER_SCRIPT)
	@echo "Linking..."
	@$(CC) $(OPTS) -Wl,-M -T $(LINKER_SCRIPT) $(OBJECTS) -o $(ELF_FILE) > $(TARGET_DIR)/build.map
	@echo "Program '$(TARGET_NAME)' builded"
	@echo "Clear binary outputs"
	@rm -f $(HEX_FILE) $(BIN_FILE) $(DFU_FILE)

$(TARGET_DIR)/%.c.o: %.c $(GLOBAL_DEPS)
	@mkdir -p ${@D}
	@echo "Compiling $*.c ..."
	@$(CC) -c $(OPTS) $*.c -o $(TARGET_DIR)/$*.c.o
	@$(CC) -MM -MP -MT $@ $(OPTS) $*.c > $(TARGET_DIR)/$*.c.d

$(TARGET_DIR)/%.cpp.o: %.cpp $(GLOBAL_DEPS)
	@mkdir -p ${@D}
	@echo "Compiling $*.cpp ..."
	@$(CCX) -c $(CCXOPTS) $*.cpp -o $(TARGET_DIR)/$*.cpp.o
	@$(CCX) -MM -MP -MT $@ $(CCXOPTS) $*.cpp > $(TARGET_DIR)/$*.cpp.d

build_tools/%: tools/%.cpp $(GLOBAL_DEPS)
	@mkdir -p ${@D}
	@echo "Compiling tool '$*'"
	@$(HOST_CC) tools/$*.cpp -o build_tools/$*

clean:
	rm -rf $(TARGET_DIR)/*

# Debug targets

# Program device without debugging
flash: $(ELF_FILE)
	$(GDB_SERVER) -c "program $(ELF_FILE) verify reset"

# Run the server (step1 of debuggin from command line)
server:
	$(GDB_SERVER)

# Reun the GDB (step2, gdb server must be run first)
debug: $(ELF_FILE)
	$(GDB) $(ELF_FILE) -x startup.gdb

# Internal tests for selecting proper targets inside build.sh script
test_abstract:
	@test "$(IS_ABSTRACT)" = "no"

test_dfu:
	@test "$(IS_DFU)" = "yes"

# Documentation utilities
print_notes:
	@for f in $(SOURCES) $(HEADERS); do \
		t=`awk '$$0 ~ str{gsub(/^[ \t]+|[ \t]+$$/, "", $$0);print}{b=$$0}' str="NOTE: " $$f`; \
		if [ -n "$$t" ]; then echo "$$f"; echo "$$t"; fi; \
	done

print_todos:
	@for f in $(SOURCES) $(HEADERS); do \
		t=`awk '$$0 ~ str{gsub(/^[ \t]+|[ \t]+$$/, "", $$0);print}{b=$$0}' str="TODO: " $$f`; \
		if [ -n "$$t" ]; then echo "$$f"; echo "$$t"; fi; \
	done

doc:
	doxygen config.doxygen
	./tools/target_doc.py doc/html/Targets.html > doc/help_tagets.html

vars:
	@echo 'TOOL_PREFIX="$(TOOL_PREFIX)"'
	@echo 'TOOL_PREFIX="$(TARGET_NAME)"'
	@echo '$(OS)'

# Generation of Keil uVision and SCIDE projects
# Not tested and maintained
# Some variables needs to be defined in the target:
# - MDK_CPU
# - MDK_SVD
# - MDK_DEVICE
# - KEIL_SCT -> should point to scatter file (Keil linkerscript)
gen_project: build_proj/$(TARGET_NAME)/$(TARGET_NAME).uvproj build_proj/$(TARGET_NAME)/nbproject/configurations.xml

build_proj/$(TARGET_NAME)/$(TARGET_NAME).uvproj: build_proj/$(TARGET_NAME).list tools/mdk_proj.sh
	@mkdir -p ${@D}
	@echo "Generating uV project for '$(TARGET_NAME)' ..."
	@./tools/mdk_proj.sh build_proj/$(TARGET_NAME).list build_proj/$(TARGET_NAME)/$(TARGET_NAME).uvproj "$(TARGET_NAME)" "$(MDK_CPU)" "$(MDK_SVD)" "$(MDK_DEVICE)"

build_proj/$(TARGET_NAME)/nbproject/configurations.xml: build_proj/$(TARGET_NAME).list tools/scide_proj.sh
	@echo "Generating SCIDE project for '$(TARGET_NAME)' ..."
	@./tools/scide_proj.sh build_proj/$(TARGET_NAME).list build_proj/$(TARGET_NAME)/nbproject "$(TARGET_NAME)" "$(MDK_CPU)" "$(MDK_SVD)"

build_proj/$(TARGET_NAME).list: $(GLOBAL_DEPS)
	@mkdir -p ${@D}
	@echo "Generating file list for '$(TARGET_NAME)' ..."
	@echo "SRC FOLDERS:" > build_proj/$(TARGET_NAME).list
	@echo $(SOURCE_DIRS) >> build_proj/$(TARGET_NAME).list
	@echo "INC PATH:" >> build_proj/$(TARGET_NAME).list
	@echo $(INCLUDE_PATH) >> build_proj/$(TARGET_NAME).list
	@echo "DEFINES:" >> build_proj/$(TARGET_NAME).list
	@echo $(C_MACROS) >> build_proj/$(TARGET_NAME).list
	@echo "MODULES:" >> build_proj/$(TARGET_NAME).list
	@echo $(MODULES:%=%.c) >> build_proj/$(TARGET_NAME).list
	@echo "API:" >> build_proj/$(TARGET_NAME).list
	@echo $(PERIPHERALS:%=%.c) >> build_proj/$(TARGET_NAME).list
	@echo "OPTS:" >> build_proj/$(TARGET_NAME).list
	@echo $(CCOPTS) >> build_proj/$(TARGET_NAME).list
	@echo "KEIL_SCT:" >> build_proj/$(TARGET_NAME).list
	@echo $(KEIL_SCT) >> build_proj/$(TARGET_NAME).list
	@echo "KEIL_S:" >> build_proj/$(TARGET_NAME).list
	@echo $(KEIL_S) >> build_proj/$(TARGET_NAME).list

# Statistics computation, should show RAM and ROM usage per module/driver
# Not working at the moment
# TODO: make this work
statistics: $(TARGET_DIR)/stats/parsed.ram
	cd $(TARGET_DIR) && gnuplot -p ../../gnuplot_statistics

$(TARGET_DIR)/stats/parsed.ram: $(TARGET_DIR)/stats/global.gstats $(STATS) build_tools/statistics
	./build_tools/statistics $(STATS) $(TARGET_DIR)/stats/global.gstats $(TARGET_DIR)/stats/parsed

$(TARGET_DIR)/stats/global.gstats: $(ELF_FILE) $(GLOBAL_DEPS)
	@mkdir -p ${@D}
	$(TOOL_PREFIX)nm --size-sort -f posix -S $(ELF_FILE) >$(TARGET_DIR)/stats/global.gstats

$(TARGET_DIR)/stats/%.stats: $(TARGET_DIR)/%.o $(GLOBAL_DEPS)
	@mkdir -p ${@D}
	$(TOOL_PREFIX)nm --defined-only $(TARGET_DIR)/$*.o >$(TARGET_DIR)/stats/$*.stats