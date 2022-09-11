# Stuff related to testing
# Testing targets not working at the moment
# TODO: make it work
# TODO: missing print output (probably removed during optimization)

TEST_SOURCE_DIRS=$(SOURCE_DIRS:%=tests/%)
TEST_SOURCES=$(wildcard $(addsuffix /*.c,$(TEST_SOURCE_DIRS)))

-include $(TEST_SOURCES:%.c=$(TARGET_DIR)/%.d)

TEST_SOURCES+=$(SOURCES)
TEST_SOURCES+=tests/tests_core.c
TEST_SOURCES:=$(filter-out app/main.c,$(TEST_SOURCES))

TEST_OBJECTS=$(TEST_SOURCES:%.c=$(TARGET_DIR)/%.c.o)

GLOBAL_TEST_DEPS:=$(GLOBAL_DEPS)
GLOBAL_TEST_DEPS+=tests/Makefile.mk

test_flash: $(TARGET_DIR)/test_build.elf
	$(GDB_SERVER) -c "program $(TARGET_DIR)/test_build.elf verify reset"

test: $(TARGET_DIR)/generated_files/tests_list_generated.h $(TARGET_DIR)/test_build.bin

$(TARGET_DIR)/test_build.bin: $(TARGET_DIR)/test_build.elf $(GLOBAL_TEST_DEPS)
	$(OBJCOPY) -O binary $(TARGET_DIR)/test_build.elf $(TARGET_DIR)/test_build.bin

$(TARGET_DIR)/test_build.elf: $(TEST_OBJECTS) target/$(TARGET_NAME)/linkerscript.ld $(GLOBAL_TEST_DEPS)
	@echo "Linking test program..."
	@$(CC) $(OPTS) -Wl,-M -T $(LINKER_SCRIPT) $(TEST_OBJECTS) -o $(TARGET_DIR)/test_build.elf > $(TARGET_DIR)/test_build.map

$(TARGET_DIR)/generated_files/tests_list_generated.h: ./tools/test_gen.sh $(TEST_SOURCES) $(GLOBAL_TEST_DEPS)
	@mkdir -p ${@D}
	@echo "Generating test list..."
	@./tools/test_gen.sh $(TEST_SOURCE_DIRS) > $(TARGET_DIR)/generated_files/tests_list_generated.h
