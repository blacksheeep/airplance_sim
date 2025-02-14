# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -I./include -g
LDFLAGS = -lm -lpthread -ljson-c

# Directories
SRC_DIR = src
BUILD_DIR = build
INCLUDE_DIR = include

# Source directories
CORE_DIR = $(SRC_DIR)/core
COMPONENTS_DIR = $(SRC_DIR)/components
EXTERNAL_DIR = $(SRC_DIR)/external

# Find all source files
CORE_SRCS = $(wildcard $(CORE_DIR)/*.c)
COMPONENT_SRCS = $(wildcard $(COMPONENTS_DIR)/*.c)
EXTERNAL_SRCS = $(wildcard $(EXTERNAL_DIR)/*.c)
MAIN_SRC = $(SRC_DIR)/main.c

# Generate object file names
CORE_OBJS = $(CORE_SRCS:$(CORE_DIR)/%.c=$(BUILD_DIR)/core/%.o)
COMPONENT_OBJS = $(COMPONENT_SRCS:$(COMPONENTS_DIR)/%.c=$(BUILD_DIR)/components/%.o)
EXTERNAL_OBJS = $(EXTERNAL_SRCS:$(EXTERNAL_DIR)/%.c=$(BUILD_DIR)/external/%.o)
MAIN_OBJ = $(MAIN_SRC:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)

# External component executables
GPS_SENDER = $(BUILD_DIR)/external/gps_sender
LANDING_RADIO_SENDER = $(BUILD_DIR)/external/landing_radio_sender
SAT_COM_SENDER = $(BUILD_DIR)/external/sat_com_sender

# Main executable
MAIN_EXE = $(BUILD_DIR)/airplane_sim

# All executables
EXECUTABLES = $(MAIN_EXE) $(GPS_SENDER) $(LANDING_RADIO_SENDER) $(SAT_COM_SENDER)

# Default target
all: directories $(EXECUTABLES)

# Create build directories
directories:
	@mkdir -p $(BUILD_DIR)/core
	@mkdir -p $(BUILD_DIR)/components
	@mkdir -p $(BUILD_DIR)/external

# Main executable
$(MAIN_EXE): $(CORE_OBJS) $(COMPONENT_OBJS) $(MAIN_OBJ)
	$(CC) $^ -o $@ $(LDFLAGS)

# External components
$(GPS_SENDER): $(BUILD_DIR)/external/gps_sender.o
	$(CC) $^ -o $@ $(LDFLAGS)

$(LANDING_RADIO_SENDER): $(BUILD_DIR)/external/landing_radio_sender.o
	$(CC) $^ -o $@ $(LDFLAGS)

$(SAT_COM_SENDER): $(BUILD_DIR)/external/sat_com_sender.o
	$(CC) $^ -o $@ $(LDFLAGS)

# Compile core source files
$(BUILD_DIR)/core/%.o: $(CORE_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Compile component source files
$(BUILD_DIR)/components/%.o: $(COMPONENTS_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Compile external source files
$(BUILD_DIR)/external/%.o: $(EXTERNAL_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Compile main source file
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean build files
clean:
	rm -rf $(BUILD_DIR)

# Install dependencies (Ubuntu/Debian)
deps:
	sudo apt-get update
	sudo apt-get install -y build-essential libjson-c-dev

# Run the simulation
run: all
	./start_simulation.sh

# Generate compile_commands.json for IDE integration
compile_commands: clean
	bear -- make all

# Debug build with address sanitizer
debug: CFLAGS += -fsanitize=address -fno-omit-frame-pointer
debug: clean all

# Release build with optimizations
release: CFLAGS += -O2 -DNDEBUG
release: clean all

# Check for memory leaks using valgrind
memcheck: all
	valgrind --leak-check=full --show-leak-kinds=all ./$(MAIN_EXE)

# Format source code using clang-format
format:
	find . -name "*.c" -o -name "*.h" | xargs clang-format -i

# Phony targets
.PHONY: all clean deps run compile_commands debug release memcheck format directories

# Header file dependencies
-include $(CORE_OBJS:.o=.d)
-include $(COMPONENT_OBJS:.o=.d)
-include $(EXTERNAL_OBJS:.o=.d)
-include $(MAIN_OBJ:.o=.d)
